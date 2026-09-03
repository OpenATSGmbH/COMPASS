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

#include "task/analyze/scancyclewalk.h"
#include "eval/requirement/detection/detection_pd_helpers.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using analysis::CycleEvent;
using analysis::evaluateCyclesInPeriod;
using analysis::evaluateNominalSlotsInPeriod;
using EvaluationRequirement::PDHelpers::RefPeriod;

using boost::posix_time::ptime;
using boost::posix_time::seconds;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{
ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

RefPeriod periodFromSeconds(int begin_s, int end_s)
{
    return RefPeriod{ t0() + seconds(begin_s), t0() + seconds(end_s) };
}
}

TEST_CASE("evaluateCyclesInPeriod - report counts per cycle window",
          "[scan_cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> cycles{
        t0() + seconds(0),  // window [0, 1)
        t0() + seconds(1),  // window [1, 2)
        t0() + seconds(2),  // window [2, 3)
        t0() + seconds(3),  // window [3, 10)
    };

    std::vector<ptime> tst{
        t0() + milliseconds(100),               // cycle 0
        t0() + milliseconds(600),               // cycle 0 (split)
        // cycle 1: none
        t0() + seconds(2) + milliseconds(500),  // cycle 2
        t0() + seconds(3),                      // cycle 3 (at window start)
        t0() + seconds(5),                      // cycle 3
        t0() + seconds(9) + milliseconds(999),  // cycle 3
    };

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 4);

    REQUIRE(out[0].num_reports == 2);
    REQUIRE_FALSE(out[0].is_miss);

    REQUIRE(out[1].num_reports == 0);
    REQUIRE(out[1].is_miss);

    REQUIRE(out[2].num_reports == 1);
    REQUIRE_FALSE(out[2].is_miss);

    REQUIRE(out[3].num_reports == 3);
    REQUIRE_FALSE(out[3].is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - report at the window end belongs to the next cycle",
          "[scan_cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);
    std::vector<ptime> cycles{ t0() + seconds(0), t0() + seconds(1) };
    std::vector<ptime> tst{ t0() + seconds(1) };  // exactly at c_1

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 2);
    REQUIRE(out[0].num_reports == 0);
    REQUIRE(out[1].num_reports == 1);
}

TEST_CASE("evaluateNominalSlotsInPeriod - slots and counts",
          "[scan_cycle_walk]")
{
    // 10 s period, 1 s slots -> 10 slots [0,1) ... [9,10)
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> tst{
        t0() + milliseconds(200),               // slot 0
        t0() + milliseconds(700),               // slot 0
        t0() + seconds(4) + milliseconds(1),    // slot 4
        t0() + seconds(9) + milliseconds(999),  // slot 9
    };

    auto out = evaluateNominalSlotsInPeriod(period, 1.0, tst);
    REQUIRE(out.size() == 10);

    REQUIRE(out[0].t_cycle == t0());
    REQUIRE(out[0].num_reports == 2);
    REQUIRE_FALSE(out[0].is_miss);

    for (std::size_t i = 1; i < 10; ++i)
    {
        if (i == 4 || i == 9)
        {
            REQUIRE(out[i].num_reports == 1);
            REQUIRE_FALSE(out[i].is_miss);
        }
        else
        {
            REQUIRE(out[i].num_reports == 0);
            REQUIRE(out[i].is_miss);
        }
    }
}

TEST_CASE("evaluateNominalSlotsInPeriod - partial last slot is dropped",
          "[scan_cycle_walk]")
{
    // 10.5 s period, 1 s slots -> 10 full slots, the trailing 0.5 s is dropped
    RefPeriod period{ t0(), t0() + seconds(10) + milliseconds(500) };
    std::vector<ptime> tst;

    auto out = evaluateNominalSlotsInPeriod(period, 1.0, tst);
    REQUIRE(out.size() == 10);
    for (const auto& ev : out)
        REQUIRE(ev.is_miss);
}

TEST_CASE("evaluateNominalSlotsInPeriod - degenerate inputs",
          "[scan_cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);
    std::vector<ptime> tst;

    SECTION("non-positive period")
    {
        REQUIRE(evaluateNominalSlotsInPeriod(period, 0.0, tst).empty());
        REQUIRE(evaluateNominalSlotsInPeriod(period, -1.0, tst).empty());
    }

    SECTION("empty reference period")
    {
        RefPeriod empty{ t0(), t0() };
        REQUIRE(evaluateNominalSlotsInPeriod(empty, 1.0, tst).empty());
    }

    SECTION("slot longer than period")
    {
        REQUIRE(evaluateNominalSlotsInPeriod(period, 20.0, tst).empty());
    }
}
