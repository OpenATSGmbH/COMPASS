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

#include "catch.hpp"
#include "logger.h"

#include <log4cpp/Category.hh>
#include <log4cpp/StringQueueAppender.hh>
#include <log4cpp/PropertyConfigurator.hh>
#include <log4cpp/Priority.hh>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
    /**
     * Attaches a capturing appender to the root category and restores the previous
     * priority and appenders on destruction.
     */
    class LogCapture
    {
    public:
        LogCapture(log4cpp::Priority::Value priority)
        :   root_             (log4cpp::Category::getRoot())
        ,   previous_priority_(log4cpp::Category::getRoot().getPriority())
        {
            appender_ = new log4cpp::StringQueueAppender("unit_test_capture");

            root_.addAppender(appender_); //category takes ownership
            root_.setPriority(priority);
        }

        ~LogCapture()
        {
            root_.removeAppender(appender_); //deletes the appender
            root_.setPriority(previous_priority_);
        }

        size_t count() const { return appender_->queueSize(); }

        std::string joined() const
        {
            std::string all;

            auto queue = appender_->getQueue(); //copy, popping the original would consume it

            while (!queue.empty())
            {
                all += queue.front();
                queue.pop();
            }

            return all;
        }

    private:
        log4cpp::Category&              root_;
        log4cpp::Priority::Value        previous_priority_;
        log4cpp::StringQueueAppender*   appender_ {nullptr};
    };

    //returns whether the argument was evaluated, used to prove short circuiting
    bool sideEffect(bool& flag)
    {
        flag = true;
        return true;
    }

    //named on purpose, the emitted line has to carry this function's name
    void emitDebugMarker()
    {
        logdbg << "unit_test_debug_marker " << 42;
    }
}

/**
 * The function name is reduced once per call site and reused afterwards, so it has to stay
 * correct and stable across calls.
 */
TEST_CASE("formatFuncName reduces a signature", "[logger]")
{
    REQUIRE(formatFuncName("void MyClass::myFunc()") == "MyClass: myFunc");
    REQUIRE(formatFuncName("int MyClass::myFunc(int, double) const") == "MyClass: myFunc");
    REQUIRE(formatFuncName("void freeFunc()") == "freeFunc");
    REQUIRE(formatFuncName("int main(int, char**)") == "main");
}

/**
 * logdbg must remain switchable through the log4cpp priority, which is what
 * log4cpp.properties configures via log4cpp.rootCategory.
 */
TEST_CASE("logdbg follows the configured priority", "[logger]")
{
    SECTION("enabled at DEBUG priority")
    {
        LogCapture capture (log4cpp::Priority::DEBUG);

        REQUIRE(log4cpp::Category::getRoot().isPriorityEnabled(log4cpp::Priority::DEBUG));

        emitDebugMarker();

        REQUIRE(capture.count() == 1);

        std::string out = capture.joined();

        INFO("captured: " << out);

        REQUIRE(out.find("unit_test_debug_marker 42") != std::string::npos);

        //the name of the enclosing function is prepended by LogHelper
        REQUIRE(out.find("emitDebugMarker") != std::string::npos);
    }

    SECTION("the cached function name stays correct across calls")
    {
        LogCapture capture (log4cpp::Priority::DEBUG);

        //the name is reduced once per call site and reused, so a second call must not differ
        emitDebugMarker();
        emitDebugMarker();

        REQUIRE(capture.count() == 2);

        std::string out = capture.joined();

        INFO("captured: " << out);

        size_t first  = out.find("emitDebugMarker");
        size_t second = out.find("emitDebugMarker", first + 1);

        REQUIRE(first  != std::string::npos);
        REQUIRE(second != std::string::npos);
    }

    SECTION("silent at INFO priority")
    {
        LogCapture capture (log4cpp::Priority::INFO);

        REQUIRE(!log4cpp::Category::getRoot().isPriorityEnabled(log4cpp::Priority::DEBUG));

        logdbg << "unit_test_debug_marker_off";

        REQUIRE(capture.count() == 0);
    }

    SECTION("loginf is unaffected")
    {
        LogCapture capture (log4cpp::Priority::INFO);

        loginf << "unit_test_info_marker";

        REQUIRE(capture.count() == 1);
        REQUIRE(capture.joined().find("unit_test_info_marker") != std::string::npos);
    }
}

/**
 * The whole point of the macro shape: while DEBUG is off nothing right of the condition runs,
 * so streamed expressions must not be evaluated.
 */
TEST_CASE("logdbg does not evaluate its arguments when disabled", "[logger]")
{
    SECTION("disabled")
    {
        LogCapture capture (log4cpp::Priority::INFO);

        bool evaluated = false;

        logdbg << "value " << sideEffect(evaluated);

        REQUIRE(!evaluated);
    }

    SECTION("enabled")
    {
        LogCapture capture (log4cpp::Priority::DEBUG);

        bool evaluated = false;

        logdbg << "value " << sideEffect(evaluated);

        REQUIRE(evaluated);
    }
}

/**
 * End to end check of the path a user actually configures: the log4cpp properties file sets
 * log4cpp.rootCategory, PropertyConfigurator applies it, and logdbg has to follow.
 * This is the same mechanism Client uses through Logger::init.
 */
TEST_CASE("logdbg can be switched on through a log4cpp properties file", "[logger]")
{
    namespace fs = std::filesystem;

    auto& root = log4cpp::Category::getRoot();

    const log4cpp::Priority::Value previous_priority = root.getPriority();

    fs::path dir           = fs::temp_directory_path();
    fs::path properties_fn = dir / "compass_test_log4cpp.properties";
    fs::path output_fn     = dir / "compass_test_log4cpp_output.log";

    auto writeProperties = [&](const std::string& level)
    {
        std::ofstream out (properties_fn.string(), std::ios::trunc);

        out << "log4cpp.rootCategory=" << level << ", testFileAppender\n"
            << "log4cpp.appender.testFileAppender=FileAppender\n"
            << "log4cpp.appender.testFileAppender.fileName=" << output_fn.string() << "\n"
            << "log4cpp.appender.testFileAppender.append=false\n"
            << "log4cpp.appender.testFileAppender.layout=PatternLayout\n"
            << "log4cpp.appender.testFileAppender.layout.ConversionPattern=[%p] %m%n\n";
    };

    auto readOutput = [&]() -> std::string
    {
        root.removeAllAppenders(); //closes and flushes the file appender

        std::ifstream in (output_fn.string());
        std::stringstream buffer;
        buffer << in.rdbuf();

        return buffer.str();
    };

    SECTION("DEBUG in the properties file enables logdbg")
    {
        writeProperties("DEBUG");
        log4cpp::PropertyConfigurator::configure(properties_fn.string());

        REQUIRE(root.isPriorityEnabled(log4cpp::Priority::DEBUG));

        emitDebugMarker();

        std::string out = readOutput();

        INFO("captured: " << out);

        REQUIRE(out.find("unit_test_debug_marker 42") != std::string::npos);
        REQUIRE(out.find("emitDebugMarker") != std::string::npos);
    }

    SECTION("INFO in the properties file silences logdbg")
    {
        writeProperties("INFO");
        log4cpp::PropertyConfigurator::configure(properties_fn.string());

        REQUIRE(!root.isPriorityEnabled(log4cpp::Priority::DEBUG));

        emitDebugMarker();
        loginf << "unit_test_info_still_logged";

        std::string out = readOutput();

        INFO("captured: " << out);

        REQUIRE(out.find("unit_test_debug_marker") == std::string::npos);

        //info level must keep working, this is not a blanket mute
        REQUIRE(out.find("unit_test_info_still_logged") != std::string::npos);
    }

    //leave the root category as the rest of the suite expects it
    root.removeAllAppenders();
    root.setPriority(previous_priority);

    std::error_code ec;
    fs::remove(properties_fn, ec);
    fs::remove(output_fn, ec);
}

/**
 * The macro expands to a conditional expression, so it must not swallow a trailing else
 * and must be usable without streaming anything.
 */
TEST_CASE("logdbg composes with surrounding control flow", "[logger]")
{
    LogCapture capture (log4cpp::Priority::INFO);

    bool else_taken = false;

    if (false)
        logdbg << "not reached";
    else
        else_taken = true;

    REQUIRE(else_taken);

    //standalone use, as in some existing call sites
    logdbg;

    REQUIRE(capture.count() == 0);
}
