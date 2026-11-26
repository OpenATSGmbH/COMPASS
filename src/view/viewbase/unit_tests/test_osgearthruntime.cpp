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
#include "osgearthruntime.h"
#include "traced_assert.h"

#include <osgEarth/weejobs.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace
{
    /// starts osgEarth and runs one job, so the threads of the job pool exist
    void startJobPool()
    {
        osgearth_runtime::initialize();

        std::atomic<bool> ran { false };
        jobs::dispatch([ &ran ] () { ran = true; });

        while (!ran)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

TEST_CASE("osgEarth jobs run after the initialization", "[view][osgearthruntime]")
{
    startJobPool();

    // a second initialization does nothing
    osgearth_runtime::initialize();

    std::atomic<bool> ran { false };
    jobs::dispatch([ &ran ] () { ran = true; });

    for (int i = 0; i < 500 && !ran; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    CHECK(ran);
}

// Ends the process. Run it alone: the process has to end with the exit code of abort() within a
// few seconds. On Windows it waited forever before the exit hooks: abort() unloads the osgEarth DLL,
// whose static destructor waited for the job pool threads the process exit had already ended.
TEST_CASE("an assertion ends the process while osgEarth jobs exist", "[.][osgearthexitabort]")
{
    startJobPool();

    traced_assert(false);
}
