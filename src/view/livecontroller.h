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

#pragma once

#include <QObject>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>
#include <memory>
#include <string>

class COMPASS;
class ViewManager;
class LiveDataFeed;
class Buffer;

/**
 * Owns the live in-memory session: the LiveDataFeed plus the per-tick orchestration
 * (bound the on-disk cache, run the feed's in-memory tick). A ViewManager collaborator -
 * ViewManager owns current_source_ (pointed at the feed during a session) and the
 * distribution dispatch; this drives the feed and the DB bound.
 *
 * The session runs an explicit state machine (Stopped/Running/Paused) driven by
 * ViewManager::appModeSwitchSlot through start/pause/resume/stopSession(). Ticks only run
 * while Running; Paused (incl. the resume catch-up load) and Stopped suppress them.
 *
 * The feed is fed by the engine's insertedDataSignal (live inserts) and ticked on each
 * insert completion plus the ASTERIX watchdog. Distribution side-effects (counts,
 * dataDistributedSignal) ride the feed's own dataChangedSignal into
 * ViewManager::sourceDataChangedSlot; only the raw-buffer empty-clear is handled here.
 */
class LiveController : public QObject
{
    Q_OBJECT

public:
    LiveController(COMPASS& compass, ViewManager& view_manager, QObject* parent = nullptr);
    virtual ~LiveController();

    enum class State { Stopped, Running, Paused };

    // session transitions (driven by ViewManager::appModeSwitchSlot):
    void startSession();                  // fresh entry: feed is empty -> Running
    void pauseSession();                  // freeze the display -> Paused (DB keeps ingesting)
    void resumeSession();                 // catch up on the pause window, then -> Running
    void stopSession();                   // full stop -> Stopped, drop the live cache

    State state() const { return state_; }
    bool running() const { return state_ == State::Running; }

    std::shared_ptr<LiveDataFeed> feedPtr(); // the feed, made ViewManager's current_source_

    bool hasMaxLatency() const;
    boost::posix_time::time_duration maxLatency() const;

    // entering a live session: trim the feed to the current window + distribute, WITHOUT a
    // DB delete (the every-tick DB bound resumes on the next real tick). Fresh entry ->
    // empty feed -> no-op; resume -> shows the reloaded window (staged inserts merged in).
    void refreshDisplay();

public slots:
    // one full live tick (watchdog + insert completion): DB bound + in-memory tick
    void processLiveModeSlot();

private slots:
    // fresh inserts from the engine (live): stage into the feed, then run a tick
    void insertedDataSlot(std::map<std::string, std::shared_ptr<Buffer>> buffers);

private:
    LiveDataFeed& feed();                 // the live feed (lives for the session lifetime)

    void clearFeed();                     // drop the live cache on session stop
    // resume catch-up: harvest-load the current time window from the DB (which kept
    // accumulating while paused) and replace the frozen feed with it. No display source /
    // bookend / dialog - a private data fetch, like a batch consumer.
    void reloadWindow();

    void processTick();                   // in-memory tick + raw-buffer empty-clear

    COMPASS&     compass_;
    ViewManager& view_manager_;

    std::shared_ptr<LiveDataFeed> feed_;
    State state_ {State::Stopped};        // Running gates ticks; resume load stays Paused
};
