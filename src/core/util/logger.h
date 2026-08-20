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

#include "singleton.h"

#include "log4cpp/Appender.hh"
#include "log4cpp/Category.hh"

#include <cstdint>
#include <string>

/**
 * Reduces a __PRETTY_FUNCTION__ signature to 'Class: function' or 'function'.
 */
std::string formatFuncName(const char* pretty_function);

/**
 * The signature is a compile time constant per function, so it is reduced once per call site
 * and returned by reference afterwards. Every macro expansion creates its own closure type,
 * and therefore its own static.
 * __PRETTY_FUNCTION__ has to be expanded here and not inside a nested lambda, otherwise it
 * names the lambda instead of the enclosing function.
 */
#define FORMAT_FUNC_NAME() []() -> const std::string& { \
    static const std::string func_name = formatFuncName(__PRETTY_FUNCTION__); \
    return func_name; \
}()

class LogHelper {
public:
    LogHelper(log4cpp::CategoryStream&& stream, const std::string& func_name)
        : stream_(std::move(stream)), used_with_stream_(false) {
        stream_ << func_name;
    }
    
    ~LogHelper() {
        if (!used_with_stream_) {
            // Was used standalone, don't add colon
        } else {
            // Was used with stream operator, colon already added
        }
    }
    
    template<typename T>
    log4cpp::CategoryStream& operator<<(const T& value) {
        if (!used_with_stream_) {
            stream_ << ": ";
            used_with_stream_ = true;
        }
            log4cpp::CategoryStream& result = stream_ << value;
    
        // Check if this is an error stream and flush immediately
        if (stream_.getPriority() == log4cpp::Priority::ERROR) {
            stream_.flush();
        }
        
        return result;
    }
    
    // For cases where the stream needs to be passed to other functions
    log4cpp::CategoryStream& getStream() {
        if (!used_with_stream_) {
            stream_ << ": ";
            used_with_stream_ = true;
        }
        return stream_;
    }
    
private:
    log4cpp::CategoryStream stream_;
    bool used_with_stream_;
};

/**
 * Turns a log expression into void so it can be used as one arm of a conditional operator.
 * operator& binds looser than operator<<, so 'LogVoidify() & LogHelper(...) << a << b' groups
 * as 'LogVoidify() & (LogHelper(...) << a << b)'. Using a plain if instead would make a
 * trailing else of the caller bind to the macro's if.
 */
class LogVoidify {
public:
    void operator&(log4cpp::CategoryStream&) {}
    void operator&(LogHelper&&) {} //logdbg used standalone, without streaming anything
};

#define logerr LogHelper(log4cpp::Category::getRoot().errorStream(), FORMAT_FUNC_NAME())
#define logwrn LogHelper(log4cpp::Category::getRoot().warnStream(), FORMAT_FUNC_NAME())
#define loginf LogHelper(log4cpp::Category::getRoot().infoStream(), FORMAT_FUNC_NAME())

/**
 * Nothing right of the ':' is evaluated while DEBUG is disabled, neither the function name nor
 * the streamed arguments. The check is done at runtime, so the level stays configurable through
 * log4cpp.properties (log4cpp.rootCategory=DEBUG, ...).
 */
#define logdbg \
    !log4cpp::Category::getRoot().isPriorityEnabled(log4cpp::Priority::DEBUG) \
        ? (void)0 \
        : LogVoidify() & LogHelper(log4cpp::Category::getRoot().debugStream(), FORMAT_FUNC_NAME())
#define logdbg1 \
    if (false) \
    log4cpp::Category::getRoot().debugStream()  // for improved performance
#define logdbg2 \
    if (false) \
    log4cpp::Category::getRoot().debugStream()  // for improved performance    

namespace logger
{
    class EventLog;
}

/**
 * @brief Thread-safe logger
 *
 * Uses log4cpp.
 */
class Logger : public Singleton
{
  public:
    struct Event
    {
        bool consume()
        {
            if (!fresh)
                return false;
            fresh = false;
            return true;
        }

        bool        fresh = true;
        uint32_t    id;
        int         timestamp;
        std::string message;
    };

    typedef std::map<int, std::vector<Event>> Events;

    const logger::EventLog* getEventLog() const;

  protected:
    static Logger* log_instance_;
    log4cpp::Appender* console_appender_;
    log4cpp::Appender* file_appender_;

    logger::EventLog* event_log_ = nullptr;

    Logger();

  public:
    static Logger& getInstance()
    {
        static Logger instance;
        return instance;
    }

    void init(const std::string& log_config_filename, bool enable_event_log = false);

    virtual ~Logger();
};

