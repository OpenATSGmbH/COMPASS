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

#include "configurable.h"
#include "singleton.h"
//#include "logger.h"
#include "rtcommand_defs.h"
#include "json_fwd.hpp"

#include <QThread>

#include <boost/thread/mutex.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <queue>
#include <future>

// netcat call for console
// netcat 127.0.0.1 27960

namespace rtcommand
{
    struct RTCommand;
    struct RTCommandResponse;
    class  RTCommandShell;
}

class TCPServer;

/**
 * Class for listening for and processing string commands at runtime,
 * either coming externally from a TCP port or internally from the application itself.
 */
class RTCommandManager : public QThread, public Singleton, public Configurable
{
    Q_OBJECT
public:
    typedef std::unique_ptr<rtcommand::RTCommand>     CommandPtr;
    typedef uint64_t                                  CommandId;
    typedef std::pair<CommandId,rtcommand::ErrorInfo> AddInfo;

    enum class Source
    {
        Server = 0,
        Application,
        Shell
    };

    struct QueuedCommand
    {
        Source     source;
        CommandId  id;
        CommandPtr command;
    };

    static bool open_port_;

    virtual ~RTCommandManager();

    void startCommandProcessing(); // only process command after start has been called
    void shutdown();

    static RTCommandManager& instance()
    {
        static RTCommandManager instance;
        return instance;
    }

    rtcommand::IssueInfo addCommand(const std::string& cmd_str, CommandId* id = nullptr);
    void addCommandFromConsole(const std::string& cmd_str); // throws on failure

    void clearBacklog();
    std::vector<std::string> commandBacklog() const;

signals:
    void commandProcessed(CommandId id, std::string msg, std::string data, bool is_error);
    void shellCommandProcessed(const QString& msg, const QString& data, bool is_error);

protected:
    volatile bool started_ {false};
    volatile bool stop_requested_ {false};
    volatile bool stopped_ {false};

    std::unique_ptr<TCPServer> server_;
    unsigned int port_num_ {27960};

    std::queue<QueuedCommand> command_queue_;
    boost::mutex command_queue_mutex_;

//    bool command_active_ {false};
//    std::future<rtcommand::RTCommandResult> current_result_;

    RTCommandManager();

private:
    friend class rtcommand::RTCommandShell;

    static const std::string PingName;

    static const size_t BacklogSize = 100;

    static CommandId command_count_;

    void run();

    void addToBacklog(const std::string& cmd);
    rtcommand::IssueInfo addCommand(const std::string& cmd_str, Source source, CommandId* id = nullptr);

    nlohmann::json command_backlog_;
};
