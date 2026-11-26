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
#include "log4cpp/PatternLayout.hh"
#include "log4cpp/RollingFileAppender.hh"

#include <boost/thread.hpp>

#include <QCoreApplication>

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <vector>

#define MAX_EVENTS_PER_CATEGORY 1000

namespace logger
{
    bool debug_enabled = false;

    log4cpp::Category& rootCategory()
    {
        static log4cpp::Category& root = log4cpp::Category::getRoot();

        return root;
    }

    void refreshLevels()
    {
        debug_enabled = rootCategory().isPriorityEnabled(log4cpp::Priority::DEBUG);
    }
}

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

namespace
{
    /// start time of the process for the log file name, e.g. 2026-11-23_19-42-10
    std::string startTimeString()
    {
        std::time_t now = std::time(nullptr);
        std::tm tm_local {};
#ifdef _WIN32
        localtime_s(&tm_local, &now);
#else
        localtime_r(&now, &tm_local);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tm_local);
        return buf;
    }

    /// keeps the newest keep log files of earlier starts (with their backups), deletes the others
    void removeOldLogFiles(const std::filesystem::path& dir, size_t keep)
    {
        std::error_code ec;
        std::vector<std::filesystem::path> logs;

        for (const auto& entry : std::filesystem::directory_iterator(dir, ec))
        {
            const std::string name = entry.path().filename().string();
            if (name.rfind(Logger::LogFilePrefix, 0) == 0 && entry.path().extension() == ".log")
                logs.push_back(entry.path());
        }

        if (logs.size() <= keep)
            return;

        //the names start with the start time, so the name order is the time order
        std::sort(logs.begin(), logs.end());

        for (size_t i = 0; i + keep < logs.size(); ++i)
        {
            std::filesystem::remove(logs[ i ], ec);

            for (int b = 1; b <= Logger::MaxLogFileBackups; ++b)
                std::filesystem::remove(logs[ i ].string() + "." + std::to_string(b), ec);
        }
    }
}

/**
 */
const std::string& Logger::logFilePath(const std::string& log_dir)
{
    //computed once: the crash handlers (main.cpp) and the logger use the same file
    static const std::string path = log_dir + LogFilePrefix + startTimeString() + "_"
                                    + std::to_string(QCoreApplication::applicationPid()) + ".log";
    return path;
}

Logger::Logger() : console_appender_(0), file_appender_(0), event_log_(new logger::EventLog(MAX_EVENTS_PER_CATEGORY)) {}

/**
 * Notice: defined here and not in the header, a function-local static in a header-defined
 * function exists once per DLL on Windows.
 */
Logger& Logger::getInstance()
{
    static Logger instance;
    return instance;
}

void Logger::init(const std::string& log_config_filename, bool enable_event_log, const std::string& log_file)
{
    log4cpp::PropertyConfigurator::configure(log_config_filename);

    //the log file is added here and not in the properties file: it needs the absolute path in the
    //configuration folder and a name per start, and existing copies of the properties file stay valid
    if (!log_file.empty())
    {
        log_file_ = log_file;

        const std::filesystem::path dir = std::filesystem::path(log_file).parent_path();

        std::error_code ec;
        std::filesystem::create_directories(dir, ec);

        //the file of this start is not there yet: keep one less of the earlier starts
        removeOldLogFiles(dir, MaxLogFiles - 1);

        auto layout = new log4cpp::PatternLayout();
        layout->setConversionPattern("%d{%Y-%m-%d %H:%M:%S.%l} [%p] %m%n");

        auto appender = new log4cpp::RollingFileAppender("logFile", log_file, MaxLogFileSize, MaxLogFileBackups);
        appender->setLayout(layout);

        logger::rootCategory().addAppender(appender); //the category owns the appender
    }

    logger::refreshLevels(); //the configured priority is cached, not queried per log call

    if (enable_event_log)
    {
        logger::rootCategory().addAppender(new logger::EventAppender("events", event_log_));
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
