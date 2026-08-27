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

#include "rtcommand_runner_stash.h"
#include "ui_test_find.h"
#include "rtcommand.h"
#include "rtcommand_wait_condition.h"
#include "compass.h"
#include "viewmanager.h"
#include "logger.h"

#include <QSignalSpy>
#include <QTimer>

namespace rtcommand
{

namespace
{
    /**
     * Commands execute on a quiet view state: interactive redraws and reloads process
     * their data asynchronously, and a command mutating view or manager state while a
     * worker still reads it corrupts the computation (or worse).
     *
     * This only REPORTS whether now is a good time - the waiting belongs to the runner
     * thread. Waiting here, in the main thread, deadlocks in practice: a command can be
     * dispatched from a nested event pump belonging to the very dispatch that would make
     * the views quiet. QProgressDialog::setValue() pumps events for a modal dialog, and
     * LoadController::beginViewPhase() does that from inside ViewManager::loadingDoneSlot,
     * before the view loop has even run - so pumping for a quiet state there waits for a
     * stack frame below itself (one command stalled 2 h 58 min in CI, 2026-08-19).
     *
     * A ViewManager dispatch in flight is exactly that situation: its guard is only set
     * while the dispatch runs, so seeing it set means our event came out of its pump.
     */
    bool viewsBusy(COMPASS& compass)
    {
        //a shutdown tears the views down anyway, and its event loop is going away -
        //deferring against it would leave the runner thread retrying forever
        if (compass.isShutDown())
            return false;

        ViewManager& view_man = compass.viewManager();

        return view_man.hasPendingViewProcessing() || view_man.isProcessingData();
    }
}

/**
*/
RTCommandRunnerStash::RTCommandRunnerStash(COMPASS& compass)
:   compass_(compass)
{
}

/**
*/
RTCommandRunnerStash::~RTCommandRunnerStash() = default;

/**
 * Checks if the signal of the currently installed QSignalSpy has already been received.
*/
bool RTCommandRunnerStash::spySignalReceived() const
{
    return (spy_ && spy_->count() > 0);
}

/**
 * Install a QSignalSpy for a signal in the given QObject's subtree.
*/
bool RTCommandRunnerStash::spyForSignal(const QString& obj_name, const QString& signal_name)
{
    spy_.reset(WaitConditionSignal::createSpy(obj_name, signal_name, compass_));
    return (spy_ != nullptr);
}

/**
 * Remove the currently installed QSignalSpy.
*/
void RTCommandRunnerStash::removeSpy()
{
    spy_.reset();
}

/**
 * Execute the given runtime command. Returns an ExecResult: on a busy view state nothing
 * is run and the runner is told to retry, so the main thread returns to its event loop
 * instead of waiting for a state only the frames below it can reach.
*/
int RTCommandRunnerStash::executeCommand(RTCommandMetaTypeWrapper wrapper) const
{
    if (!wrapper.command)
        return static_cast<int>(ExecResult::Failed);

    if (viewsBusy(compass_))
        return static_cast<int>(ExecResult::Deferred);

    wrapper.command->compass_ = &compass_;

    return static_cast<int>(wrapper.command->run() ? ExecResult::Succeeded
                                                   : ExecResult::Failed);
}

/**
 * Execute the given runtime command async.
*/
void RTCommandRunnerStash::executeCommandAsync(RTCommandMetaTypeWrapper wrapper) const
{
    if (!wrapper.command)
        return;

    if (viewsBusy(compass_))
    {
        //no return value to defer with, and no runner thread waiting on us: re-arm
        //through the event loop. A timer, not a queued invocation - a re-post could be
        //dispatched again by the same pump we are running in, spinning inside it
        QTimer::singleShot(ExecRetryMSecs, const_cast<RTCommandRunnerStash*>(this),
                           [ this, wrapper ] () { executeCommandAsync(wrapper); });
        return;
    }

    wrapper.command->compass_ = &compass_;

    //the runner does not track state for async commands, since execution
    //happens after the command reply has already been sent
    if (wrapper.command->run())
        wrapper.command->setState(CmdState::Finished);
}

/**
 * Run result check on the given command.
*/
bool RTCommandRunnerStash::postCheckCommand(RTCommandMetaTypeWrapper wrapper) const
{
    if (!wrapper.command)
        return false;

    wrapper.command->compass_ = &compass_;
    return wrapper.command->checkResult();
}

} // namespace rtcommand

