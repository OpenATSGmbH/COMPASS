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

#include "message.h"
#include "logstream.h"
#include "logger.h"
#include "json.hpp"

#include <boost/stacktrace.hpp>
#include <boost/thread/mutex.hpp>

#define GET_MACRO_01(_1,NAME,...) NAME
#define GET_MACRO_012(_1,_2,NAME,...) NAME
#define GET_MACRO_0123(_1,_2,_3,NAME,...) NAME

#define GET_COMPONENT(Component) (std::string(#Component).empty() ? FORMAT_FUNC_NAME() : #Component)
#define GET_STACKTRACE() (std::stringstream() << boost::stacktrace::stacktrace()).str()

#define sloginf0() \
    msghandler::MessageHandler::getStream((loginf.getStream()), msghandler::Severity::Info, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", false)
#define sloginf1(Component) \
    msghandler::MessageHandler::getStream((loginf.getStream()), msghandler::Severity::Info, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", false)
#define sloginf(...) GET_MACRO_01(__VA_ARGS__, sloginf1, sloginf0)(__VA_ARGS__)

#define slogwrn0() \
    msghandler::MessageHandler::getStream((logwrn.getStream()), msghandler::Severity::Warning, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", false)
#define slogwrn1(Component) \
    msghandler::MessageHandler::getStream((logwrn.getStream()), msghandler::Severity::Warning, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", false)
#define slogwrn(...) GET_MACRO_01(__VA_ARGS__, slogwrn1, slogwrn0)(__VA_ARGS__)

#define slogerr0() \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", false)
#define slogerr1(Component) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", false)
#define slogerr2(Component, ErrCode) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, {}, "", false)
#define slogerr(...) GET_MACRO_012(__VA_ARGS__, slogerr2, slogerr1, slogerr0)(__VA_ARGS__)

#define slogcrit0() \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", false)
#define slogcrit1(Component) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", false)
#define slogcrit2(Component, ErrCode) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, {}, "", false)
#define slogcrit(...) GET_MACRO_012(__VA_ARGS__, slogcrit2, slogcrit1, slogcrit0)(__VA_ARGS__)

#define uloginf0() \
    msghandler::MessageHandler::getStream((loginf.getStream()), msghandler::Severity::Info, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", true)
#define uloginf1(Component) \
    msghandler::MessageHandler::getStream((loginf.getStream()), msghandler::Severity::Info, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", true)
#define uloginf2(Component, JSONInfo) \
    msghandler::MessageHandler::getStream((loginf.getStream()), msghandler::Severity::Info, GET_COMPONENT(Component), -1, __FILE__, __LINE__, JSONInfo, "", true)
#define uloginf(...) GET_MACRO_012(__VA_ARGS__, uloginf2, uloginf1, uloginf0)(__VA_ARGS__)

#define ulogwrn0() \
    msghandler::MessageHandler::getStream((logwrn.getStream()), msghandler::Severity::Warning, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", true)
#define ulogwrn1(Component) \
    msghandler::MessageHandler::getStream((logwrn.getStream()), msghandler::Severity::Warning, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", true)
#define ulogwrn2(Component, JSONInfo) \
    msghandler::MessageHandler::getStream((logwrn.getStream()), msghandler::Severity::Warning, GET_COMPONENT(Component), -1, __FILE__, __LINE__, JSONInfo, "", true)
#define ulogwrn(...) GET_MACRO_012(__VA_ARGS__, ulogwrn2, ulogwrn1, ulogwrn0)(__VA_ARGS__)

#define ulogerr0() \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", true)
#define ulogerr1(Component) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", true)
#define ulogerr2(Component, ErrCode) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, {}, "", true)
#define ulogerr3(Component, ErrCode, JSONInfo) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Error, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, JSONInfo, "", true)
#define ulogerr(...) GET_MACRO_0123(__VA_ARGS__, ulogerr3, ulogerr2, ulogerr1, ulogerr0)(__VA_ARGS__)

#define ulogcrit0() \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, FORMAT_FUNC_NAME(), -1, __FILE__, __LINE__, {}, "", true)
#define ulogcrit1(Component) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, GET_COMPONENT(Component), -1, __FILE__, __LINE__, {}, "", true)
#define ulogcrit2(Component, ErrCode) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, {}, "", true)
#define ulogcrit3(Component, ErrCode, JSONInfo) \
    msghandler::MessageHandler::getStream((logerr.getStream()), msghandler::Severity::Critical, GET_COMPONENT(Component), ErrCode, __FILE__, __LINE__, JSONInfo, "", true)
#define ulogcrit(...) GET_MACRO_0123(__VA_ARGS__, ulogcrit3, ulogcrit2, ulogcrit1, ulogcrit0)(__VA_ARGS__)

namespace msghandler
{

/**
 */
class MessageHandler
{
public:
    static std::string severityToString(Severity severity);

    static LogStream getStream(log4cpp::CategoryStream& strm,
                               Severity severity,
                               const std::string& component,
                               int err_code,
                               const std::string& file,
                               int line,
                               const nlohmann::json& info,
                               const std::string& stack_trace,
                               bool user_level);
    static void commit(log4cpp::CategoryStream& strm,
                       const Message& msg);
    
    static void log(log4cpp::CategoryStream& strm,
                    const Message& msg);

    static void reportCriticalError(const Message& msg);
    static bool hasCriticalError();
    static Message criticalError();
    static bool handleCriticalError();

private:
    static void logMessageFancy(log4cpp::CategoryStream& strm,
                                const Message& msg);
    static void handleSystemLevelMessage(log4cpp::CategoryStream& strm,
                                         const Message& msg);
    static void handleUserLevelMessage(log4cpp::CategoryStream& strm,
                                       const Message& msg);

    static void addToTaskLog(const Message& msg);
    static bool compileBugReport(const Message& msg);

    static bool showMessage(const Message& msg);
    static bool showAbortMessage(const Message& msg);

    static boost::mutex critical_error_mutex_;
    static Message      critical_error_msg_;
    static bool         critical_error_msg_set_;   
};

} // namespace msghandler
