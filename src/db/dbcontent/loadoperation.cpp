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

#include "loadoperation.h"
#include "logger.h"
#include "util/async.h"

/**
 */
LoadOperation::LoadOperation(DBContentManager& dbcont_man, LoadSpec spec)
:   DBContentDataSet(dbcont_man)
,   spec_           (std::move(spec))
{
}

/**
 */
LoadOperation::~LoadOperation() = default;

/**
 */
bool LoadOperation::isFinished() const
{
    return state_ == State::Done || state_ == State::Cancelled || state_ == State::Failed;
}

/**
 * Cooperative cancel: flags the request and signals the engine, which obsoletes
 * the in-flight read jobs. The op reaches Cancelled when they drain.
 */
void LoadOperation::cancel()
{
    if (isFinished())
        return;

    cancel_requested_ = true;
    emit cancelRequestedSignal();
}

/**
 * Blocks the caller until the op is finished, pumping the event loop so the
 * engine's queued read-job callbacks can run (loads execute on a worker thread).
 */
void LoadOperation::wait(unsigned int sleep_msecs)
{
    // blocking issuers are modal (progress dialog / their own message box), so user input
    // may be processed - see the ordering rules in readme_loading.md
    Utils::Async::pumpUntil([this] { return isFinished(); }, /*process_user_input=*/true,
                            sleep_msecs);
}

/**
 * Engine-driven state transition; emits startedSignal on entering Running and
 * finishedSignal on reaching a terminal state - each at most once. Same-state
 * calls are ignored and terminal states are sticky, so no signal fires twice.
 */
void LoadOperation::setState(State state)
{
    if (state_ == state)
        return;

    if (isFinished())
    {
        logwrn << "LoadOperation: ignoring state change from a terminal state";
        return;
    }

    state_ = state;

    if (state_ == State::Running)
        emit startedSignal();
    else if (isFinished())
        emit finishedSignal();
}
