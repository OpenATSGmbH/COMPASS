/*
 * This file is part of OpenATS COMPASS.
 *
 * COMPASS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * COMPASS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "livecontroller.h"
#include "viewmanager.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataengine.h"
#include "dbcontent/livedatafeed.h"
#include "dbcontent/loadrequest.h"
#include "dbcontent/loadoperation.h"
#include "dbcontent/variable/variableset.h"
#include "filtermanager.h"
#include "dbfiltercondition.h"
#include "filterclause.h"
#include "idbvariableresolver.h"
#include "global.h"
#include "util/timeconv.h"
#include "logger.h"

#include <QApplication>
#include <QCoreApplication>
#include <QMessageBox>

using namespace Utils;

/**
 */
LiveController::LiveController(COMPASS& compass, ViewManager& view_manager, QObject* parent)
    : QObject(parent), compass_(compass), view_manager_(view_manager)
{
    // the feed lives for the whole session lifetime (cleared, not recreated, on stop);
    // DBContentManager + its engine already exist when ViewManager builds us
    feed_ = std::make_shared<LiveDataFeed>(compass_.dbContentManager());

    feed_->setReadSetProvider([this](const std::string& name) -> dbContent::VariableSet {
        auto& dbcm = compass_.dbContentManager();
        dbContent::VariableSet read_set = view_manager_.getReadSet(name);
        dbcm.addStandardVariables(name, read_set);

        if (name == "CAT063") // live sensor-status vars
        {
            traced_assert(dbcm.canGetVariable(name, dbcontent_vars::var_cat063_con_));
            traced_assert(dbcm.canGetVariable(name, dbcontent_vars::var_cat063_sensor_sac_));
            traced_assert(dbcm.canGetVariable(name, dbcontent_vars::var_cat063_sensor_sic_));

            read_set.add(dbcm.getVariable(name, dbcontent_vars::var_cat063_con_));
            read_set.add(dbcm.getVariable(name, dbcontent_vars::var_cat063_sensor_sac_));
            read_set.add(dbcm.getVariable(name, dbcontent_vars::var_cat063_sensor_sic_));
        }

        return read_set;
    });

    connect(&compass_.dbContentManager().dataEngine(), &DBContentDataEngine::insertedDataSignal,
            this, &LiveController::insertedDataSlot);
}

/**
 */
LiveController::~LiveController() = default;

/**
 */
LiveDataFeed& LiveController::feed()
{
    return *feed_;
}

/**
 */
std::shared_ptr<LiveDataFeed> LiveController::feedPtr()
{
    return feed_;
}

/**
 */
void LiveController::clearFeed()
{
    feed_->clear(); // full stop drops the live cache; next entry starts fresh
}

/**
 * Fresh entry (Offline -> Live): the feed was cleared on the last stop, so the session
 * starts blank. Just arm ticks. Guarded against a repeated entry (a doubled app-mode signal
 * would otherwise re-arm from an unexpected state).
 */
void LiveController::startSession()
{
    if (state_ != State::Stopped)
    {
        logwrn << "startSession: session already active (state " << (int) state_ << "), ignoring";
        return;
    }
    state_ = State::Running;
}

/**
 * Pause (Live -> LivePaused): freeze the display. The DB keeps ingesting (the engine emits
 * insertedDataSignal only while Running, so the feed sees no new inserts); Paused suppresses
 * ticks so the on-screen window stays put. No-op if not currently Running.
 */
void LiveController::pauseSession()
{
    if (state_ != State::Running)
    {
        logwrn << "pauseSession: not running (state " << (int) state_ << "), ignoring";
        return;
    }
    state_ = State::Paused;
}

/**
 * Resume (LivePaused -> Live): the display froze while the DB kept accumulating. Catch up on
 * the pause window from the DB, THEN arm ticks - reloadWindow runs while still Paused so a
 * pump-fired tick can't overlap it (see reloadWindow). No-op if not currently Paused, which
 * also guards the expensive reloadWindow from running twice.
 */
void LiveController::resumeSession()
{
    if (state_ != State::Paused)
    {
        logwrn << "resumeSession: not paused (state " << (int) state_ << "), ignoring";
        return;
    }
    reloadWindow();
    state_ = State::Running;
}

/**
 * Full stop (Live -> Offline): disarm ticks and drop the live cache; the next entry starts
 * fresh. No-op if already stopped (avoids a redundant clearFeed).
 */
void LiveController::stopSession()
{
    if (state_ == State::Stopped)
    {
        logwrn << "stopSession: already stopped, ignoring";
        return;
    }
    state_ = State::Stopped;
    clearFeed();
}

/**
 * Resume catch-up: harvest-load the current window from the DB (which kept accumulating
 * while the display was frozen) and replace the feed with it. A private, blocking data
 * fetch - the op is never made the display source, so it raises no bookend/dialog.
 */
void LiveController::reloadWindow()
{
    using namespace boost::posix_time;

    // Modal wait dialog around the blocking load below. That load pumps events (op->wait),
    // and unlike an offline load it shows no progress dialog - without a modal the user could
    // click Pause/Resume mid-load and re-enter the app-mode transition. The modal blocks that
    // and tells them a resume load is running.
    QMessageBox msg_box(QApplication::activeWindow());
    msg_box.setWindowTitle("Resuming Live Mode");
    msg_box.setText("Loading data ...");
    msg_box.setStandardButtons(QMessageBox::NoButton);
    msg_box.setWindowModality(Qt::ApplicationModal);
    msg_box.show();
    QCoreApplication::processEvents(); // paint before the load busies the main thread

    ptime min_ts = Time::currentUTCTime() - minutes(compass_.maxLiveDataAgeCache());

    // live-prime window as a per-content clause via the shared toolkit (timestamp >= min);
    // resolver is a long-lived filter member, the clause runs synchronously inside load() below
    IDBVariableResolver* resolver = &compass_.filterManager().variableResolver();
    long min_ts_long = Time::toLong(min_ts);

    LoadRequest req;
    req.dbcontents_ = {"*"};
    req.show_status_ = false;
    req.custom_filter_clause_ = [resolver, min_ts_long](const std::string& name) -> std::string {
        // sqlFor returns an empty clause for contents without a timestamp variable
        return DBFilterCondition::sqlFor(*resolver, name, dbcontent_vars::meta_var_timestamp_.name(),
                                         META_OBJECT_NAME, filter_op::greater_equal,
                                         std::to_string(min_ts_long)).sql;
    };

    auto op = std::make_shared<LoadOperation>(compass_.dbContentManager(), req);

    // the modal blocks user input, but not the engine's insertedDataSignal (fired Direct as
    // continued ingestion completes during the pump); state_ stays Paused across this load
    // (resumeSession only flips to Running afterwards), so a pump-fired tick can't issue an
    // overlapping DB delete or process (then lose) inserts the seedFrom is about to replace.
    // Inserts still stage and are merged on the next tick.
    compass_.dbContentManager().dataEngine().load(op);
    op->wait();
    feed_->seedFrom(op->buffers()); // replace the frozen feed with the current DB window
}

/**
 */
bool LiveController::hasMaxLatency() const
{
    return feed_->hasLatency() && !feed_->latency().is_not_a_date_time();
}

/**
 */
boost::posix_time::time_duration LiveController::maxLatency() const
{
    return feed_->latency();
}

/**
 * Fresh inserts from the engine: stage into the feed, then run a full tick.
 */
void LiveController::insertedDataSlot(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    // the engine announces every insert; gate here (the live/offline decision belongs to the
    // session, not the engine). Match the old engine-side condition exactly - appMode, NOT
    // running(): during the resume reloadWindow pump appMode is already LiveRunning while the
    // state is still Paused, and inserts arriving then must still stage (processLiveModeSlot's
    // running() guard suppresses the tick, so they merge on the first tick after resume).
    if (compass_.appMode() != AppMode::LiveRunning)
        return;

    feed().addInserted(std::move(buffers));
    processLiveModeSlot();
}

/**
 * One full live tick: bound the on-disk cache via the manager's DB delete, then run the
 * in-memory tick + distribution on the feed.
 */
void LiveController::processLiveModeSlot()
{
    if (!running()) // Paused (incl. resume catch-up load) / Stopped suppress ticks
        return;

    using namespace boost::posix_time;

    // DB write (engine-owned, requested via the manager - a view-side object never writes
    // to the DB itself): drop rows older than the live cache window. A previous bound is
    // still draining (its queued doneSlot hasn't cleared it): skip this tick's bound. The
    // DB delete is independent of the display cut (cutCachedData bounds what is shown), so
    // skipping one is harmless - the next tick re-bounds.
    if (compass_.dbContentManager().hasActiveDeleteJob())
        logwrn << "skipping DB bound, a delete is still in flight";
    else
    {
        ptime old_time = Time::currentUTCTime() - minutes(compass_.maxLiveDataAgeDb());
        compass_.dbContentManager().deleteDBContentData(old_time);
    }

    processTick();
}

/**
 * Live-entry refresh: trim the feed to the window + distribute, without the DB delete (the
 * every-tick DB bound resumes on the next real tick, so no delete overlaps here).
 */
void LiveController::refreshDisplay()
{
    processTick();
}

/**
 * In-memory merge/trim/filter on the feed. The feed's own dataChangedSignal drives
 * ViewManager::sourceDataChangedSlot (distribution + counts + dataDistributedSignal); only
 * the raw-buffer views need an explicit clear when the feed empties (their loadedData path
 * is skipped on the empty-names event).
 */
void LiveController::processTick()
{
    bool had_data = !feed_->buffers().empty();

    feed_->processTick();

    if (feed_->buffers().empty() && had_data)
        view_manager_.clearDataInViews();
}
