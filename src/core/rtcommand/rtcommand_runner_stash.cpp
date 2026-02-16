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

#include <QSignalSpy>

namespace rtcommand
{

/**
*/
RTCommandRunnerStash::RTCommandRunnerStash() = default;

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
    spy_.reset(WaitConditionSignal::createSpy(obj_name, signal_name));
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

    return wrapper.command->run();
}

/**
 * Execute the given runtime command async.
*/
void RTCommandRunnerStash::executeCommandAsync(RTCommandMetaTypeWrapper wrapper) const
{
    if (wrapper.command)
        wrapper.command->run();
}

/**
 * Run result check on the given command.
*/
bool RTCommandRunnerStash::postCheckCommand(RTCommandMetaTypeWrapper wrapper) const
{
    if (!wrapper.command)
        return false;

    return wrapper.command->checkResult();
}

} // namespace rtcommand
