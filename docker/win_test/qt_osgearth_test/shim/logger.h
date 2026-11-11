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

// Test replacement for the COMPASS logger: same stream macros, output to stdout.

#pragma once

#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>

class TestLogLine
{
public:
    TestLogLine(const char* level, const char* func, bool enabled)
        : level_(level), func_(func), enabled_(enabled) {}

    ~TestLogLine()
    {
        if (!enabled_)
            return;
        std::printf("[%s] %s: %s\n", level_, func_, stream_.str().c_str());
        std::fflush(stdout);
    }

    template <typename T>
    TestLogLine& operator<<(const T& value)
    {
        if (enabled_)
            stream_ << value;
        return *this;
    }

private:
    const char*        level_;
    const char*        func_;
    bool               enabled_;
    std::ostringstream stream_;
};

#define logerr TestLogLine("ERROR", __func__, true)
#define logwrn TestLogLine("WARN", __func__, true)
#define loginf TestLogLine("INFO", __func__, true)
#define logdbg TestLogLine("DEBUG", __func__, false)

#define traced_assert(expr)                                                              \
    do                                                                                   \
    {                                                                                    \
        if (!(expr))                                                                     \
        {                                                                                \
            std::printf("ASSERT failed: %s at %s:%d\n", #expr, __FILE__, __LINE__);      \
            std::fflush(stdout);                                                         \
            std::abort();                                                                \
        }                                                                                \
    } while (0)
