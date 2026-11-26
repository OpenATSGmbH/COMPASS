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

#include "liveoverloaddetector.h"

TEST_CASE("LiveOverloadDetector backlog", "[live][overload]")
{
    LiveOverloadDetector detector;
    detector.settings().max_packets_in_processing = 4;

    SECTION("a backlog up to the limit is no overload")
    {
        REQUIRE(!detector.update(0, {}));
        REQUIRE(!detector.update(4, {}));
        REQUIRE(!detector.isOverloaded());
    }

    SECTION("an overload holds backlog + extra tick attempts in total, then checks again")
    {
        REQUIRE(detector.update(6, {}));
        REQUIRE(detector.isOverloaded());

        const unsigned int hold_ticks = 6 + LiveOverloadDetector::PacketHoldExtraTicks;
        REQUIRE(detector.holdTicks() == hold_ticks);

        //held for the remaining attempts, independent of the current backlog
        for (unsigned int i = 1; i < hold_ticks; ++i)
            REQUIRE(detector.update(0, {}));

        //re-check: backlog gone
        REQUIRE(!detector.update(0, {}));
        REQUIRE(!detector.isOverloaded());
    }

    SECTION("an overload still present at the re-check starts a new hold from the current backlog")
    {
        REQUIRE(detector.update(5, {}));

        for (unsigned int i = 1; i < detector.holdTicks(); ++i)
            REQUIRE(detector.update(0, {}));

        REQUIRE(detector.update(8, {}));
        REQUIRE(detector.isOverloaded());
        REQUIRE(detector.holdTicks() == 8 + LiveOverloadDetector::PacketHoldExtraTicks);
    }

    SECTION("reset ends the overload")
    {
        REQUIRE(detector.update(10, {}));

        detector.reset();

        REQUIRE(!detector.isOverloaded());
        REQUIRE(!detector.update(0, {}));
    }
}

TEST_CASE("LiveOverloadDetector forced overload", "[live][overload]")
{
    LiveOverloadDetector detector;
    detector.settings().max_packets_in_processing = 4;

    SECTION("held while forced, regardless of the inputs")
    {
        detector.settings().force = true;

        for (unsigned int i = 0; i < 20; ++i)
        {
            REQUIRE(detector.update(0, {}));
            REQUIRE(detector.isOverloaded());
            REQUIRE(detector.heldTicks() == i);
        }
    }

    SECTION("switching the force off ends the hold with the next attempt")
    {
        detector.settings().force = true;

        REQUIRE(detector.update(0, {}));
        REQUIRE(detector.update(0, {}));

        detector.settings().force = false;

        REQUIRE(!detector.update(0, {}));
        REQUIRE(!detector.isOverloaded());
    }

    SECTION("a real overload after the force starts a normal hold")
    {
        detector.settings().force = true;
        REQUIRE(detector.update(0, {}));

        detector.settings().force = false;

        REQUIRE(detector.update(6, {}));
        REQUIRE(detector.heldTicks() == 0);
        REQUIRE(detector.holdTicks() == 6 + LiveOverloadDetector::PacketHoldExtraTicks);
    }
}

TEST_CASE("LiveOverloadDetector latency", "[live][overload]")
{
    LiveOverloadDetector detector;
    detector.settings().max_packets_in_processing = 4;

    SECTION("latency is not checked by default")
    {
        REQUIRE(!detector.update(0, 1000.0));
    }

    SECTION("a latency above the limit holds a fixed number of attempts")
    {
        detector.settings().max_latency_s = 2.0;

        REQUIRE(!detector.update(0, {}));
        REQUIRE(!detector.update(0, 2.0));

        REQUIRE(detector.update(0, 3.0));
        REQUIRE(detector.holdTicks() == LiveOverloadDetector::LatencyHoldTicks);

        for (unsigned int i = 1; i < LiveOverloadDetector::LatencyHoldTicks; ++i)
            REQUIRE(detector.update(0, {}));

        REQUIRE(!detector.update(0, {}));
    }

    SECTION("a backlog overload takes precedence")
    {
        detector.settings().max_latency_s = 2.0;

        REQUIRE(detector.update(10, 3.0));
        REQUIRE(detector.holdTicks() == 10 + LiveOverloadDetector::PacketHoldExtraTicks);
    }
}
