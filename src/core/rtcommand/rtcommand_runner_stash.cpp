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
#include "async.h"
#include "logger.h"

#include <QSignalSpy>

namespace rtcommand
{

namespace
{
    /**
     * Commands execute on a quiet view state: interactive redraws and reloads process
     * their data asynchronously, and a command mutating view or manager state while a
     * worker still reads it corrupts the computation (or worse). Pump until all views
     * committed - the workers finish through queued commits, so pumping guarantees
     * progress; user input stays excluded.
     */
    void waitForViewProcessing(COMPASS& compass)
    {
        ViewManager& view_man = compass.viewManager();

        if (!view_man.hasPendingViewProcessing())
            return;

        loginf << "waiting for pending view processing before command execution";

        Utils::Async::pumpUntil([ &view_man ] { return !view_man.hasPendingViewProcessing(); });
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
 * Execute the given runtime command.
*/
bool RTCommandRunnerStash::executeCommand(RTCommandMetaTypeWrapper wrapper) const
{
    if (!wrapper.command)
        return false;

    waitForViewProcessing(compass_);

    wrapper.command->compass_ = &compass_;
    return wrapper.command->run();
}

/**
 * Execute the given runtime command async.
*/
void RTCommandRunnerStash::executeCommandAsync(RTCommandMetaTypeWrapper wrapper) const
{
    if (wrapper.command)
    {
        waitForViewProcessing(compass_);

        wrapper.command->compass_ = &compass_;

        //the runner does not track state for async commands, since execution
        //happens after the command reply has already been sent
        if (wrapper.command->run())
            wrapper.command->setState(CmdState::Finished);
    }
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

