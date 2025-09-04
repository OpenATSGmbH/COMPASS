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

#define FORMAT_FUNC_NAME() [&]() -> std::string { \
    std::string func_str(__PRETTY_FUNCTION__); \
    size_t end_pos = func_str.find('('); \
    if (end_pos == std::string::npos) end_pos = func_str.length(); \
    size_t start_pos = func_str.rfind(' ', end_pos) + 1; \
    size_t class_end = func_str.rfind("::", end_pos); \
    if (class_end != std::string::npos && class_end > start_pos) { \
        size_t class_start = func_str.rfind(' ', class_end - 1); \
        if (class_start == std::string::npos) class_start = 0; else class_start++; \
        std::string class_name = func_str.substr(class_start, class_end - class_start); \
        std::string func_name = func_str.substr(class_end + 2, end_pos - class_end - 2); \
        return class_name + ": " + func_name; \
    } else { \
        std::string func_name = func_str.substr(start_pos, end_pos - start_pos); \
        return func_name; \
    } \
}()

class LogHelper {
public:
    LogHelper(log4cpp::CategoryStream&& stream, const std::string& func_name) 
        : stream_(std::move(stream)), used_with_stream_(false), func_name_(func_name) {
        stream_ << func_name_;
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
        return stream_ << value;
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
    std::string func_name_;
};

#define logerr LogHelper(log4cpp::Category::getRoot().errorStream(), FORMAT_FUNC_NAME())
#define logwrn LogHelper(log4cpp::Category::getRoot().warnStream(), FORMAT_FUNC_NAME())
#define loginf LogHelper(log4cpp::Category::getRoot().infoStream(), FORMAT_FUNC_NAME())
#define logdbg LogHelper(log4cpp::Category::getRoot().isPriorityEnabled(log4cpp::Priority::DEBUG) ? \
    log4cpp::Category::getRoot().debugStream() : log4cpp::Category::getRoot().debugStream(), FORMAT_FUNC_NAME())
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

