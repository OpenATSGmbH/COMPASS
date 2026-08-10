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

#include "dbcontentdataset.h"
#include "loadspec.h"
#include "result.h"

class DBContentManager;

// One-shot load: constraints in (LoadSpec), buffers out. Single-use, issuer-owned.
// The DBContentDataEngine drives it (fills buffers, advances state); observers
// connect to startedSignal/finishedSignal and read buffers()/index() when done.
class LoadOperation : public DBContentDataSet
{
    Q_OBJECT

signals:
    void startedSignal();
    void finishedSignal();   // fired once on Done | Cancelled | Failed
    void cancelRequestedSignal();

public:
    enum class State { Created, Running, Done, Cancelled, Failed };

    LoadOperation(DBContentManager& dbcont_man, LoadSpec spec);
    virtual ~LoadOperation();

    const LoadSpec& spec() const { return spec_; }

    State state() const { return state_; }
    bool isRunning() const { return state_ == State::Running; }
    bool isFinished() const; // any terminal state

    // terminal outcome (set by the engine at finish): ok() on Done/Cancelled,
    // failed() with the message on Failed. Meaningful once finishedSignal has fired.
    const Result& result() const { return result_; }

    void cancel();                            // cooperative; engine obsoletes in-flight jobs
    void wait(unsigned int sleep_msecs = 1u); // event-pumping wait until finished

private:
    friend class DBContentDataEngine;

    void setState(State state); // engine-driven; emits started/finished at transitions

    LoadSpec spec_;                     // load configuration
    State    state_ {State::Created};   // current load operation state
    bool     cancel_requested_ {false}; // cancel flag
    Result   result_;                   // terminal outcome (engine-set at finish)
};
