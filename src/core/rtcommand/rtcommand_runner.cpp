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

#include "rtcommand_runner.h"
#include "rtcommand.h"
#include "rtcommand_chain.h"
#include "rtcommand_wait_condition.h"
#include "rtcommand_runner_stash.h"
#include "rtcommand_response.h"
#include "compass.h"

#include "logger.h"

#include <QApplication>
#include <QThread>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace rtcommand
{

/**
 */
RTCommandRunner::RTCommandRunner(COMPASS& compass)
    :   stash_(new RTCommandRunnerStash(compass))
{
}

/**
 */
RTCommandRunner::~RTCommandRunner() = default;

/**
 * Asynchronously run the given command and return a future containing the execution state.
 */
std::future<RTCommandRunner::Results> RTCommandRunner::runCommand(std::unique_ptr<RTCommand>&& cmd)
{
    RTCommandChain c;
    c.append(std::move(cmd));

    return runCommands(std::move(c));
}

/**
 * Asynchronously run the given commands and return a future containing the execution states.
 */
std::future<RTCommandRunner::Results> RTCommandRunner::runCommands(RTCommandChain&& cmds)
{
    RTCommandRunnerStash* s = stash_.get();

    auto runCommand = [ s ] (RTCommandChain&& cmds_to_run) mutable
    {
        try
        {
            Results results;

            while (auto cmd = cmds_to_run.pop())
            {
                //from here on we use shared pointer to be able to run commands asynchronously on the main thread
                std::shared_ptr<RTCommand> cmd_ptr(cmd.release());

                RTCommandRunner::runCommand(cmd_ptr, s);
                results.push_back(cmd_ptr->result());
            }

            return results;
        }
        catch (const std::exception& e)
        {
            logerr << "exception '" << e.what() << "'";
            throw e;
        }
    };

    return std::async(std::launch::async, runCommand, std::move(cmds));

}

/**
 * Inits the wait condition before running the command.
 */
bool RTCommandRunner::initWaitCondition(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!cmd || !stash)    
        throw std::runtime_error("RTCommandRunner::initWaitCondition: Bad init");

    auto& c = cmd->condition;

    if (c.type == RTCommandWaitCondition::Type::Signal)
    {
        //register signal spy in main thread and block until finished
        bool ok      = false;
        bool invoked = QMetaObject::invokeMethod(stash, "spyForSignal", Qt::BlockingQueuedConnection,
                                                 Q_RETURN_ARG(bool, ok),
                                                 Q_ARG(QString, c.signal_obj),
                                                 Q_ARG(QString, c.signal_name));

        if (!invoked)
        {
            cmd->setError(CmdErrorCode::WaitCond_InvokeFailed);
            return false;
        }

        if (!ok)
        {
            cmd->setError(CmdErrorCode::WaitCond_BadInit);
            return false;
        }   
    }

    return true;
}

/**
 * Executes the wait condition after running the command.
 */
bool RTCommandRunner::execWaitCondition(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!cmd || !stash)
        throw std::runtime_error("RTCommandRunner::execWaitCondition: Bad init");

    bool ok = true;

    const auto& c = cmd->condition;

    if (c.type == RTCommandWaitCondition::Type::Signal)
    {
        // the QSignalSpy lives in the main thread and its list is appended there on
        // signal delivery - polling it directly from this thread is a data race, so
        // each poll is a blocking queued invoke executing the check in the main thread
        ok = waitForCondition([ = ] ()
        {
            bool received = false;
            if (!QMetaObject::invokeMethod(stash, "spySignalReceived",
                                           Qt::BlockingQueuedConnection,
                                           Q_RETURN_ARG(bool, received)))
                return false;
            return received;
        }, c.signal_timeout_ms);
    }
    else if (c.type == RTCommandWaitCondition::Type::Delay)
    {
        ok = waitForCondition(WaitConditionDelay(c.delay_ms));
    }

    if (!ok)
        cmd->setError(CmdErrorCode::WaitCond_Timeout);

    return ok;
}

/**
 * Cleans up after wait condition.
 */
bool RTCommandRunner::cleanupWaitCondition(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!cmd || !stash)
        throw std::runtime_error("RTCommandRunner::cleanupWaitCondition: Bad init");

    const auto& c = cmd->condition;

    if (c.type == RTCommandWaitCondition::Type::Signal)
    {
        //remove signal spy in main thread and block until finished
        if (!QMetaObject::invokeMethod(stash, "removeSpy", Qt::BlockingQueuedConnection))
            return false;
    }

    return true;
}

/**
 * Executes the given command in the main thread.
 */
bool RTCommandRunner::executeCommand(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!cmd || !stash)
        throw std::runtime_error("RTCommandRunner::executeCommand: Bad init");

    qRegisterMetaType<RTCommandMetaTypeWrapper>();

    logMsg("Executing...", cmd.get());

    RTCommandMetaTypeWrapper wrapper;
    wrapper.command = cmd;

    loginf << "executeCommand: posting to main thread, async " << cmd->execute_async
           << " cmd " << cmd->name().toStdString();

    //execute command in main thread and block until finished
    //@TODO: handle thread cleanup
    bool ok      = true;
    bool invoked = false;

    if (cmd->execute_async)
    {
        invoked = QMetaObject::invokeMethod(stash, "executeCommandAsync",
                                            Qt::QueuedConnection,
                                            Q_ARG(RTCommandMetaTypeWrapper, wrapper));
    }
    else
    {
        //A command needs a quiet view state, and the main thread cannot wait for that
        //itself - it may be running the command out of a nested event pump belonging to
        //the very dispatch that would reach that state. So the stash answers Deferred and
        //the waiting happens here, in this thread: the main thread returns to its event
        //loop between attempts and can actually get there.
        int  res      = static_cast<int>(ExecResult::Deferred);
        auto deferred_since = boost::posix_time::microsec_clock::local_time();
        bool reported = false;

        while (true)
        {
            invoked = QMetaObject::invokeMethod(stash, "executeCommand",
                                                Qt::BlockingQueuedConnection,
                                                Q_RETURN_ARG(int, res),
                                                Q_ARG(RTCommandMetaTypeWrapper, wrapper));

            if (!invoked || res != static_cast<int>(ExecResult::Deferred))
                break;

            //a deferral outlasting a normal load points at a view that never goes quiet
            auto waited_s = (boost::posix_time::microsec_clock::local_time()
                             - deferred_since).total_seconds();
            if (!reported && waited_s >= ExecDeferWarnSecs)
            {
                reported = true;
                logwrn << "waiting for quiet views for " << waited_s << "s, cmd "
                       << cmd->name().toStdString();
            }

            QThread::msleep(ExecRetryMSecs);
        }

        ok = (res == static_cast<int>(ExecResult::Succeeded));
    }

    loginf << "executeCommand: main thread returned invoked " << invoked << " ok " << ok;
    bool succeeded = (ok && invoked);

    //if invoking the execution failed, we set the commands state to failed
    if (!invoked)
        cmd->setError(CmdErrorCode::Exec_InvokeFailed);

    //async commands transition state in the deferred main thread execution -
    //setting the state here would race the queued invocation, whose run()
    //requires the configured state
    if (succeeded && !cmd->execute_async)
        cmd->setState(CmdState::Executed);

    logMsg(std::string("[") + (succeeded ? "Succeeded" : "Failed") + "]", cmd.get());

    return succeeded;
}

/**
 * Checks the given command's result in the main thread.
 */
bool RTCommandRunner::postCheckCommand(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!cmd || !stash)
        throw std::runtime_error("RTCommandRunner::postCheckCommand: Bad init");

    //asynchronous commands will not check their result - their state is
    //handled by the deferred execution in the main thread
    if (cmd->execute_async)
        return true;

    qRegisterMetaType<RTCommandMetaTypeWrapper>();

    logMsg("Checking result...", cmd.get());

    RTCommandMetaTypeWrapper wrapper;
    wrapper.command = cmd;

    //check command result in main thread and block until finished
    bool ok      = true;
    bool invoked = QMetaObject::invokeMethod(stash, "postCheckCommand", 
                                             Qt::BlockingQueuedConnection,
                                             Q_RETURN_ARG(bool, ok),
                                             Q_ARG(RTCommandMetaTypeWrapper, wrapper));
    bool succeeded = (ok && invoked);

    //if invoking the execution failed, we set the commands state to failed
    if (!invoked)
        cmd->setError(CmdErrorCode::ResultCheck_InvokeFailed);

    if (succeeded)
        cmd->setState(CmdState::Finished);

    logMsg(std::string("[") + (succeeded ? "Succeeded" : "Failed") + "]", cmd.get());

    return succeeded;
}

/**
 */
void RTCommandRunner::logMsg(const std::string& msg, RTCommand* cmd)
{
    std::string prefix = (cmd ? "Command '" + cmd->name().toStdString() + "': " : "");

    loginf << " ---------------------------------------------------------------";
    loginf << "| " << prefix << msg;
    loginf << " ---------------------------------------------------------------";
}

/**
 * Runs the given command in the main thread and handles any set wait conditions.
 */
void RTCommandRunner::runCommand(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash)
{
    if (!stash)
        throw std::runtime_error("RTCommandRunner: run: No stash");

    //reset result state
    cmd->resetResult();

    //if not yet configured, try to configure
    if (!cmd->isConfigured() && !cmd->checkConfiguration())
        return;

    //init wait condition
    if (initWaitCondition(cmd, stash))
    {
        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

        //execute command
        if (executeCommand(cmd, stash))
        {
            // if execution went well -> execute wait condition
            if (execWaitCondition(cmd, stash))
            {
                //post check result 
                postCheckCommand(cmd, stash);
            }
        }

        boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time();
        cmd->result_.runtime = end_time - start_time;
    }

    //always try to clean up wait condition
    cleanupWaitCondition(cmd, stash);

    logMsg(RTCommandResponse(*cmd).errorToString(), cmd.get());
}

} // namespace rtcommand
