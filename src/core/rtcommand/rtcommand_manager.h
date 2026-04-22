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

// forward declaration keeps the heavy boost/asio header out of this .h
namespace boost { namespace asio { class io_context; } }

class TCPServer;
class COMPASS;

/**
 * Class for listening for and processing string commands at runtime,
 * either coming externally from a TCP port or internally from the application itself.
 */
class RTCommandManager : public QThread, public Configurable
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

    RTCommandManager(nlohmann::json& config, COMPASS* parent);
    virtual ~RTCommandManager();

    void startCommandProcessing(); // only process command after start has been called
    void shutdown();

    rtcommand::IssueInfo addCommand(const std::string& cmd_str, CommandId* id = nullptr);
    void addCommandFromConsole(const std::string& cmd_str); // throws on failure

    void clearBacklog();
    std::vector<std::string> commandBacklog() const;

signals:
    void commandProcessed(CommandId id, std::string msg, std::string data, bool is_error);
    void shellCommandProcessed(const QString& msg, const QString& data, bool is_error);

protected:
    COMPASS& compass_;

    volatile bool started_ {false};
    volatile bool stop_requested_ {false};
    volatile bool stopped_ {false};

    // io_context_ must be declared BEFORE server_ so that on destruction
    // members are torn down in the correct order: server_ (and its TCPSession
    // sockets) first, then io_context_. Reversing this order caused a
    // heap-use-after-free when ~TCPSession's basic_stream_socket destructor
    // tried to touch the reactive_socket_service after its parent io_context
    // had already been destroyed. held via unique_ptr + forward-decl so the
    // heavy boost/asio header stays out of this public interface.
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::unique_ptr<TCPServer> server_;
    unsigned int port_num_ {27960};

    std::queue<QueuedCommand> command_queue_;
    boost::mutex command_queue_mutex_;

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
