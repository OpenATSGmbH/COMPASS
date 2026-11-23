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
#include "stacktrace_format.h"

#include <string>
#include <utility>

namespace stacktrace_test
{
    /// the trace of this function only, with the line of the call
    __attribute__((noinline)) std::pair<std::string, int> traceHere()
    {
        const int line = __LINE__ + 1;
        std::string trace = stacktrace_format::current(1);

        return { trace, line };
    }
}

TEST_CASE("a stack trace frame has function, file, line and module", "[util][stacktrace]")
{
    // the build keeps function names and line tables (-g1), on Windows in the DLLs as well
    auto [ trace, line ] = stacktrace_test::traceHere();

    INFO(trace);

    CHECK(trace.find("stacktrace_test::traceHere") != std::string::npos);
    CHECK(trace.find("test_stacktrace_format.cpp:" + std::to_string(line)) != std::string::npos);
    CHECK(trace.find(" in compass_tests") != std::string::npos);
}

TEST_CASE("a stack trace frame in a library names the library", "[util][stacktrace]")
{
    // an address inside stacktrace_format::current (compass_util): the start of the function plus one,
    // resolve() looks up the byte before a return address
    const void* address = (const char*)(void*)&stacktrace_format::current + 1;

    auto frame = stacktrace_format::resolve(address);

    INFO(stacktrace_format::toString(frame, address));

    CHECK(frame.function.find("stacktrace_format::current") != std::string::npos);
    CHECK(frame.file.find("stacktrace_format.cpp") != std::string::npos);
    CHECK(frame.module.find("compass_util") != std::string::npos);
    CHECK(frame.offset > 0);
}
