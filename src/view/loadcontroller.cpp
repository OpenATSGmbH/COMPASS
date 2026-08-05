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

#include "loadcontroller.h"
#include "viewmanager.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataengine.h"
#include "dbcontent/dbcontentdataset.h"
#include "dbcontent/loadoperation.h"
#include "logger.h"

#include <QApplication>
#include <QCoreApplication>
#include <QProgressDialog>

/**
 */
LoadController::LoadController(COMPASS& compass, QObject* parent)
    : QObject(parent), compass_(compass)
{
}

/**
 */
LoadController::~LoadController() = default;

/**
 * Show the dialog + wait cursor for the running load. Idempotent: any prior cycle is
 * ended first (a superseded load), so exactly one dialog is up and the cursor is balanced.
 */
void LoadController::begin(const LoadOperation& op)
{
    end(/*drain=*/false); // no pumping here: begin runs inside the op's startedSignal

    op_ = &op;

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    cursor_active_ = true;

    value_      = 0.0;
    load_total_ = static_cast<unsigned int>(
        compass_.dbContentManager().dataEngine().resolveTargetSet(op.spec()).size());
    view_total_ = 0;

    // advance the load phase directly off the op (un-guarded, once per content) - not via
    // ViewManager's deferrable sourceDataChangedSlot, which could re-run and corrupt the bar
    op_conn_ = connect(&op, &DBContentDataSet::dataChangedSignal,
                       this, &LoadController::opDataChangedSlot);

    // tie the UX to the op itself: an abandoned op (source swapped away) never reaches
    // ViewManager's end(), since its signals are disconnected there
    op_fin_conn_ = connect(&op, &LoadOperation::finishedSignal,
                           this, &LoadController::opFinishedSlot);

    if (load_total_ == 0)
        return; // nothing to load: wait cursor only

    dialog_.reset(new QProgressDialog("Loading data...", "Cancel", 0, 100,
                                      QApplication::activeWindow()));
    dialog_->setWindowModality(Qt::ApplicationModal);
    dialog_->setMinimumDuration(0);
    dialog_->setAutoClose(false);
    dialog_->setAutoReset(false);
    dialog_->setWindowTitle("Loading Data");

    connect(dialog_.get(), &QProgressDialog::canceled, this, &LoadController::canceledSlot);

    // paint before the main thread gets busy submitting jobs / processing arrivals
    dialog_->show();
    QCoreApplication::processEvents();
}

/**
 * Per non-empty content arrival (direct off the op): advance the load phase (0..50).
 * The denominator counts target contents, the numerator only those returning rows, so the
 * phase can end below 50 (beginViewPhase then jumps there) - cosmetic.
 * @TODO: drive this off an engine progress signal counting finished contents; that also
 * retires the resolveTargetSet recompute in begin().
 */
void LoadController::opDataChangedSlot(const std::vector<std::string>& names, bool /*reset*/, bool /*last*/)
{
    if (names.empty() || !dialog_ || load_total_ == 0)
        return;

    value_ += 50.0 / static_cast<double>(load_total_);
    if (value_ > 50.0)
        value_ = 50.0;
    dialog_->setValue(static_cast<int>(value_));
}

/**
 */
void LoadController::beginViewPhase(unsigned int num_views)
{
    if (!dialog_)
        return;

    view_total_ = num_views;

    if (num_views == 0)
        value_ = 100.0; // no views: jump to 100, leave the label
    else
    {
        value_ = 50.0;
        dialog_->setLabelText("Updating views...");
    }

    dialog_->setValue(static_cast<int>(value_));
    // synchronous paint, not processEvents() - pumping queued events here lets queued RT
    // commands fire mid view-loop and break UI-test injection
    dialog_->repaint();
}

/**
 */
void LoadController::advanceViewPhase()
{
    if (!dialog_ || view_total_ == 0)
        return;

    value_ += 50.0 / static_cast<double>(view_total_);
    if (value_ > 100.0)
        value_ = 100.0;
    dialog_->setValue(static_cast<int>(value_));
    dialog_->repaint(); // see beginViewPhase
}

/**
 * The op reached a terminal state. If it is still the displayed source - including a load the
 * user cancelled - ViewManager drives the completion path (view point, view phase, end()), so
 * the UX must stay up for it. Only an abandoned op ends here: its source was swapped away, its
 * signals to ViewManager are disconnected, and nothing else would ever close the dialog. No
 * drain, since this runs inside the op's own emit.
 */
void LoadController::opFinishedSlot()
{
    if (op_ && compass_.viewManager().currentSource().get() != op_)
        end(/*drain=*/false);
}

/**
 */
void LoadController::end(bool drain)
{
    op_ = nullptr;

    if (op_conn_)
    {
        disconnect(op_conn_);
        op_conn_ = {};
    }

    if (op_fin_conn_)
    {
        disconnect(op_fin_conn_);
        op_fin_conn_ = {};
    }

    if (dialog_)
    {
        if (drain)
            QCoreApplication::processEvents();
        dialog_->close();
        dialog_.reset();
    }

    if (cursor_active_)
    {
        QApplication::restoreOverrideCursor();
        cursor_active_ = false;
    }
}

/**
 */
void LoadController::canceledSlot()
{
    compass_.dbContentManager().dataEngine().cancelLoad();
}
