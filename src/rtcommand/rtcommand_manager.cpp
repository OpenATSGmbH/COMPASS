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

#include "rtcommand_manager.h"
#include "rtcommand_string.h"
#include "rtcommand_runner.h"
#include "tcpserver.h"
#include "stringconv.h"
#include "rtcommand.h"
#include "rtcommand_result.h"
#include "rtcommand_response.h"
#include "compass.h"

#include "logger.h"

#include <QCoreApplication>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace Utils;

bool RTCommandManager::open_port_ {false};

RTCommandManager::CommandId RTCommandManager::command_count_ = 0;

const std::string RTCommandManager::PingName = "ping";

/**
 */
RTCommandManager::RTCommandManager()
    : Configurable("RTCommandManager", "RTCommandManager0", 0, "rtcommand.json")
{
    logdbg << "start";

    registerParameter("port_num", &port_num_, 27960u);
    registerParameter("db_file_list", &command_backlog_, nlohmann::json::array());
}

/**
 */
RTCommandManager::~RTCommandManager() 
{
     logdbg << "start"; 
}

/**
 */
void RTCommandManager::run()
{
    loginf << "starting io context";

    boost::asio::io_context io_context;

    boost::thread t;

    if (open_port_)
    {
        loginf << "running io context for rt command port";

        server_.reset(new TCPServer (io_context, port_num_));
        server_->start();

        t = boost::thread (boost::bind(&boost::asio::io_context::run, &io_context));
        t.detach();
    }

    loginf << "starting loop";

    while (1)
    {
        if (stop_requested_) //  && !hasAnyJobs()
        {
            loginf << "stop requested, breaking";
            break;
        }

        if (started_)
        {
            if (open_port_ && server_ && server_->hasSession() && server_->hasStrData())
            {
                logdbg << "getting commands";


                std::vector<std::string> cmds = server_->getStrData();

                loginf << "got " << cmds.size() << " commands '"
                  << String::compress(cmds, ';') << "'";

                // create commands from strings
                for (const auto& cmd_str : cmds)
                {
                    auto issue_info = addCommand(cmd_str, Source::Server);
                    rtcommand::RTCommandResponse cmd_response(issue_info);

                    if (issue_info.issued)
                        loginf << "added command " << cmd_str << " to queue, size " << command_queue_.size();

                    server_->sendStrData(cmd_response.toJSONString());
                }
            }

            // do commands


            if (command_queue_.size())
            {
                loginf << "issuing command";

                rtcommand::RTCommandRunner& cmd_runner = COMPASS::instance().rtCmdRunner();

                std::future<std::vector<rtcommand::RTCommandResult>> current_result;

                Source source;
                CommandId id;
                {
                    boost::mutex::scoped_lock lock(command_queue_mutex_);

                    auto cmd = std::move(command_queue_.front());
                    source = cmd.source;
                    id = cmd.id;

                    current_result = cmd_runner.runCommand(move(cmd.command));
                    command_queue_.pop();
                }

 
                 loginf << "waiting for result";

                current_result.wait();

                std::vector<rtcommand::RTCommandResult> results = current_result.get();
                traced_assert(results.size() == 1);

                rtcommand::RTCommandResult cmd_result = results.at(0);
                rtcommand::RTCommandResponse cmd_response(cmd_result);

                loginf << "result wait done, success " << cmd_response.isOk();
                loginf << "response = ";
                loginf << cmd_response.toJSONString();

                if (source == Source::Application)
                {
                    std::string msg  = cmd_response.errorToString();
                    std::string data = cmd_response.stringifiedReply();
                    emit commandProcessed(id, msg, data, cmd_response.error.hasError());
                }
                else if (source == Source::Shell)
                {
                    std::string msg  = cmd_response.errorToString();
                    std::string data = cmd_response.stringifiedReply();
                    emit shellCommandProcessed(QString::fromStdString(msg), 
                                            QString::fromStdString(data), 
                                            cmd_response.error.hasError());
                }
                else if (source == Source::Server)
                {
                    if (open_port_ && server_->hasSession())
                    {
                        traced_assert(server_);
                        //@TODO: get state from command result and compile reply
                        server_->sendStrData(cmd_response.toJSONString());
                    }
                }
            }
        }

        msleep(1);
    }

    if (open_port_)
    {
        io_context.stop();
        traced_assert(io_context.stopped());

        t.timed_join(100);
    }

    stopped_ = true;
    logdbg << "stopped";
}

void RTCommandManager::startCommandProcessing()
{
    loginf;
    started_ = true;
}

/**
 */
void RTCommandManager::shutdown()
{
    loginf;

    stop_requested_ = true;

    if (!started_)
        stopped_ = true;

    //    if (active_db_job_)
    //        active_db_job_->setObsolete();

    //    for (auto job_it = queued_db_jobs_.unsafe_begin(); job_it != queued_db_jobs_.unsafe_end();
    //         ++job_it)
    //        (*job_it)->setObsolete();

    //    while (hasAnyJobs())
    //    {
    //        loginf << "waiting on jobs to finish: db " << hasDBJobs()
    //               << " blocking " << hasBlockingJobs() << " non-locking " << hasNonBlockingJobs();

    //        msleep(1000);
    //    }

    msleep(100);

    while (!stopped_)
    {
        if (!started_)
            stopped_ = true;

        loginf << "waiting on run stop";

        msleep(1000);
    }

    started_ = false;

    logdbg << "done";
}

/**
 */
rtcommand::IssueInfo RTCommandManager::addCommand(const std::string& cmd_str, CommandId* id)
{
    return addCommand(cmd_str, Source::Application, id);
}

void RTCommandManager::addCommandFromConsole(const std::string& cmd_str)
{
    rtcommand::IssueInfo info = addCommand(cmd_str);

    if (info.error.hasError())
    {
        logerr << "command '" << cmd_str << "' resulted in error "
               << rtcommand::RTCommandResponse::errCode2String(info.error.code) 
               << ": '" << info.error.message << "', aborting";

        throw runtime_error(
            "RTCommandManager: addCommandFromConsole: incorrect command from console");
    }
}

/**
 */
rtcommand::IssueInfo RTCommandManager::addCommand(const std::string& cmd_str, Source source, CommandId* id)
{
    if (id)
        *id = -1;

    //handle pings 
    if (cmd_str == PingName)
    {
        rtcommand::IssueInfo info;
        info.issued        = false;
        info.error.code    = rtcommand::CmdErrorCode::NoError;
        info.error.message = "";
        info.command       = "ping";

        return info;
    }

    //commands from the server are only added when app is properly running
    if (source == Source::Server && COMPASS::instance().appState() != AppState::Running)
    {
        rtcommand::IssueInfo info;
        info.issued        = false;
        info.error.code    = rtcommand::CmdErrorCode::Issue_NotReady;
        info.error.message = rtcommand::RTCommandResponse::errCode2String(info.error.code);
        info.command       = ""; //can be left empty?

        return info;
    }

    //add command string to backlog
    if (!cmd_str.empty())
        addToBacklog(cmd_str);

    rtcommand::RTCommandString rt_command_string(QString(cmd_str.c_str())); // todo change to std str

    //issue command for command string
    auto issue_result = rt_command_string.issue();

    rtcommand::RTCommandResponse response(issue_result.second);

    loginf << "response = ";
    loginf << response.toJSONString();

    //if issue went well push to command queue
    if (issue_result.second.issued)
    {
        boost::mutex::scoped_lock lock(command_queue_mutex_);

        QueuedCommand q_cmd;
        q_cmd.id      = command_count_++;
        q_cmd.command = std::move(issue_result.first);
        q_cmd.source  = source;

        if (id)
            *id = q_cmd.id;

        command_queue_.push(std::move(q_cmd));
    }

    return issue_result.second;
}

/**
*/
void RTCommandManager::addToBacklog(const std::string& cmd)
{
    command_backlog_.push_back(cmd);

    if (command_backlog_.size() > BacklogSize)
    {
        size_t offs = command_backlog_.size() - BacklogSize;
        command_backlog_.erase(command_backlog_.begin(), command_backlog_.begin() + offs);
    }
}

/**
*/
void RTCommandManager::clearBacklog()
{
    command_backlog_.clear();
}

/**
*/
std::vector<std::string> RTCommandManager::commandBacklog() const
{
    return command_backlog_.get<std::vector<std::string>>();
}
