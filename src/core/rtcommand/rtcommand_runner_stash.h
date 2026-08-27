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

#include <memory>

#include <QObject>

class COMPASS;

class QSignalSpy;

namespace rtcommand
{
    struct RTCommand;
}

/**
*/
struct RTCommandMetaTypeWrapper
{
    std::shared_ptr<rtcommand::RTCommand> command;
};

namespace rtcommand
{

/**
 * Outcome of one attempt to run a command in the main thread. Deferred means the main
 * thread was not in a state to run it and the runner has to try again - see viewsBusy()
 * in the implementation. Passed as int through the queued invocation, since moc only
 * marshals registered types across threads.
 */
enum class ExecResult
{
    Failed = 0, // the command ran and reported failure
    Succeeded,  // the command ran and reported success
    Deferred    // not run at all, retry
};

// pause between attempts after a Deferred answer. Kept coarse on purpose: a deferral can
// last for the length of a load, and every attempt costs the main thread one dispatched
// event
constexpr int ExecRetryMSecs = 10;

// log a warning once a single command has been deferred for this long - a view that
// never goes quiet would otherwise stall silently
constexpr int ExecDeferWarnSecs = 30;

/**
 * Obtains data structures and calls for the command runner 
 * needed to reside in the main thread.
 */
class RTCommandRunnerStash : public QObject
{
    Q_OBJECT
public:
    RTCommandRunnerStash(COMPASS& compass);
    virtual ~RTCommandRunnerStash();

private slots:
    bool spyForSignal(const QString& obj_name, const QString& signal_name);
    void removeSpy();
    // returns an ExecResult - the caller must handle Deferred by retrying
    int executeCommand(RTCommandMetaTypeWrapper wrapper) const;
    void executeCommandAsync(RTCommandMetaTypeWrapper wrapper) const;
    bool postCheckCommand(RTCommandMetaTypeWrapper wrapper) const;
    // slot: must run in the main thread - the QSignalSpy list is appended there
    // on signal delivery, reading it from the runner thread is a data race
    bool spySignalReceived() const;

private:
    friend class RTCommandRunner;

    COMPASS& compass_;
    std::unique_ptr<QSignalSpy> spy_;
};

} // namespace rtcommand

Q_DECLARE_METATYPE(RTCommandMetaTypeWrapper)
