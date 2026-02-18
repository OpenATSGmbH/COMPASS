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

#include "logger.h"

#include <QApplication>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace rtcommand
{

/**
 */
RTCommandRunner::RTCommandRunner()
    :   stash_(new RTCommandRunnerStash)
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
        ok = waitForCondition([ = ] () { return stash->spySignalReceived(); }, c.signal_timeout_ms);
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

    //execute command in main thread and block until finished
    //@TODO: handle thread cleanup
    bool ok      = true;
    bool invoked = cmd->execute_async ? QMetaObject::invokeMethod(stash, "executeCommandAsync", 
                                                                  Qt::QueuedConnection,
                                                                  Q_ARG(RTCommandMetaTypeWrapper, wrapper)) :
                       QMetaObject::invokeMethod(stash, "executeCommand",
                                                 Qt::BlockingQueuedConnection,
                                                 Q_RETURN_ARG(bool, ok),
                                                 Q_ARG(RTCommandMetaTypeWrapper, wrapper));
    bool succeeded = (ok && invoked);

    //if invoking the execution failed, we set the commands state to failed
    if (!invoked)
        cmd->setError(CmdErrorCode::Exec_InvokeFailed);

    if (succeeded)
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

    //asynchronous commands will not check their result
    if (cmd->execute_async)
    {
        cmd->setState(CmdState::Finished);
        return true;
    }

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
