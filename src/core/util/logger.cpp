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
#include "logger.h"
#include "event_log.h"

#include "log4cpp/PropertyConfigurator.hh"

#include <boost/thread.hpp>

#define MAX_EVENTS_PER_CATEGORY 1000

std::string formatFuncName(const char* pretty_function)
{
    std::string func_str(pretty_function);

    size_t end_pos = func_str.find('(');
    if (end_pos == std::string::npos) end_pos = func_str.length();

    size_t start_pos = func_str.rfind(' ', end_pos) + 1;
    size_t class_end = func_str.rfind("::", end_pos);

    if (class_end != std::string::npos && class_end > start_pos)
    {
        size_t class_start = func_str.rfind(' ', class_end - 1);
        if (class_start == std::string::npos) class_start = 0; else class_start++;

        std::string class_name = func_str.substr(class_start, class_end - class_start);
        std::string func_name  = func_str.substr(class_end + 2, end_pos - class_end - 2);

        return class_name + ": " + func_name;
    }
    else
    {
        return func_str.substr(start_pos, end_pos - start_pos);
    }
}

Logger::Logger() : console_appender_(0), file_appender_(0), event_log_(new logger::EventLog(MAX_EVENTS_PER_CATEGORY)) {}

void Logger::init(const std::string& log_config_filename, bool enable_event_log)
{
    log4cpp::PropertyConfigurator::configure(log_config_filename);

    if (enable_event_log)
    {
        log4cpp::Category::getRoot().addAppender(new logger::EventAppender("events", event_log_));
    }
}

Logger::~Logger()
{
    if (console_appender_)
    {
        delete console_appender_;
        console_appender_ = 0;
    }
    if (file_appender_)
    {
        delete file_appender_;
        file_appender_ = 0;
    }

    delete event_log_;
    event_log_ = nullptr;
}

/**
 * Retuns the event log.
 */
const logger::EventLog* Logger::getEventLog() const
{
    return event_log_;
}
