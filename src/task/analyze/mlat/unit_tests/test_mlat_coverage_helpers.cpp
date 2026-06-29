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

#include "task/analyze/mlat/mlatcoveragehelpers.h"
#include "task/analyze/movementui.h"
#include "eval/requirement/detection/detection_pd_helpers.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using mlatcoverage_internal::CycleEvent;
using mlatcoverage_internal::evaluateCyclesInPeriod;
using EvaluationRequirement::PDHelpers::RefPeriod;
using analysis::MovementUI;
using analysis::SpeedSamples;

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

// Constant-speed reference samples spanning [0, span_s] at 1 s spacing, so
// MovementUI can classify any cycle time as standing/moving.
SpeedSamples constantSpeed(double speed_mps, int span_s)
{
    SpeedSamples s;
    for (int i = 0; i <= span_s; ++i)
    {
        s.ts.push_back(t0() + seconds(i));
        s.sp.push_back(speed_mps);
    }
    return s;
}
}

TEST_CASE("evaluateCyclesInPeriod - no cycles in period",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 100);
    std::vector<ptime> cycles;  // none
    std::vector<ptime> tst;

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.empty());
}

TEST_CASE("evaluateCyclesInPeriod - cycles before period are excluded",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(10, 20);

    std::vector<ptime> cycles{
        t0() + seconds(1),
        t0() + seconds(5),
        t0() + seconds(9),
    };
    std::vector<ptime> tst;

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.empty());
}

TEST_CASE("evaluateCyclesInPeriod - cycles after period are excluded",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> cycles{
        t0() + seconds(11),
        t0() + seconds(20),
    };
    std::vector<ptime> tst;

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.empty());
}

TEST_CASE("evaluateCyclesInPeriod - all cycles in period, all hits (test in every cycle)",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> cycles{
        t0() + seconds(0),
        t0() + seconds(1),
        t0() + seconds(2),
        t0() + seconds(3),
    };

    // one test report inside each cycle window
    std::vector<ptime> tst{
        t0() + milliseconds(100),
        t0() + seconds(1) + milliseconds(200),
        t0() + seconds(2) + milliseconds(300),
        t0() + seconds(3) + milliseconds(400),
    };

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 4);
    for (const auto& ev : out)
        REQUIRE_FALSE(ev.is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - all cycles in period, all misses (no test)",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> cycles{
        t0() + seconds(0),
        t0() + seconds(1),
        t0() + seconds(2),
    };
    std::vector<ptime> tst;  // empty

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 3);
    for (const auto& ev : out)
        REQUIRE(ev.is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - mixed misses and hits",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 10);

    std::vector<ptime> cycles{
        t0() + seconds(0),  // window [0, 1)
        t0() + seconds(1),  // window [1, 2)
        t0() + seconds(2),  // window [2, 3)
        t0() + seconds(3),  // window [3, period.end=10)
    };

    std::vector<ptime> tst{
        t0() + milliseconds(500),       // hits cycle 0
        // cycle 1: no test -> miss
        t0() + seconds(2) + milliseconds(750), // hits cycle 2
        t0() + seconds(4),              // hits cycle 3 (last cycle, window [3,10))
    };

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 4);
    REQUIRE_FALSE(out[0].is_miss);
    REQUIRE      (out[1].is_miss);
    REQUIRE_FALSE(out[2].is_miss);
    REQUIRE_FALSE(out[3].is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - last cycle window extends to period.end",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 100);

    std::vector<ptime> cycles{
        t0() + seconds(10),  // last cycle in period -> window [10, 100)
    };

    SECTION("test inside extended last window: hit")
    {
        std::vector<ptime> tst{ t0() + seconds(80) };
        auto out = evaluateCyclesInPeriod(period, cycles, tst);
        REQUIRE(out.size() == 1);
        REQUIRE_FALSE(out[0].is_miss);
    }

    SECTION("test outside extended last window: miss")
    {
        std::vector<ptime> tst{ t0() + seconds(100) + milliseconds(1) };
        auto out = evaluateCyclesInPeriod(period, cycles, tst);
        REQUIRE(out.size() == 1);
        REQUIRE(out[0].is_miss);
    }
}

TEST_CASE("evaluateCyclesInPeriod - test report before first cycle does not hit",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 100);

    std::vector<ptime> cycles{
        t0() + seconds(10),
        t0() + seconds(20),
    };

    // test report falls before any cycle starts
    std::vector<ptime> tst{ t0() + seconds(5) };

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 2);
    REQUIRE(out[0].is_miss);
    REQUIRE(out[1].is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - cycle at period.begin is kept",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(10, 20);
    std::vector<ptime> cycles{ t0() + seconds(10) };
    std::vector<ptime> tst;
    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 1);
    REQUIRE(out[0].t_cycle == t0() + seconds(10));
}

TEST_CASE("evaluateCyclesInPeriod - degenerate last-cycle window is skipped",
          "[mlat_coverage][cycle_walk]")
{
    // Single cycle exactly at period.end -> window [period.end, period.end) is
    // empty; the implementation should skip it.
    auto period = periodFromSeconds(0, 10);
    std::vector<ptime> cycles{ t0() + seconds(10) };
    std::vector<ptime> tst;

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.empty());
}

TEST_CASE("evaluateCyclesInPeriod - moving target with MovementUI keeps per-cycle expectation",
          "[mlat_coverage][cycle_walk]")
{
    // A moving target (speed above threshold) must behave exactly like the
    // null-mv case: every in-period cycle is an expected opportunity.
    auto period = periodFromSeconds(0, 10);
    std::vector<ptime> cycles{
        t0() + seconds(0), t0() + seconds(1),
        t0() + seconds(2), t0() + seconds(3),
    };
    std::vector<ptime> tst;  // all misses

    SpeedSamples ref = constantSpeed(50.0, 10);  // moving
    MovementUI mv;
    mv.ref         = &ref;
    mv.standing_max = 0.5;
    mv.ui_moving   = 1.0;
    mv.ui_standing = 5.0;

    auto out = evaluateCyclesInPeriod(period, cycles, tst, &mv);
    REQUIRE(out.size() == 4);  // one event per cycle
    for (const auto& ev : out)
        REQUIRE(ev.is_miss);
}

TEST_CASE("evaluateCyclesInPeriod - standing target is expected once per standing UI",
          "[mlat_coverage][cycle_walk]")
{
    // Cycles every 1 s for 10 s; target standing; standing UI 5 s. The target
    // should be expected only at the 5 s window boundaries, not every cycle.
    auto period = periodFromSeconds(0, 10);
    std::vector<ptime> cycles;
    for (int i = 0; i < 10; ++i)
        cycles.push_back(t0() + seconds(i));

    SpeedSamples ref = constantSpeed(0.0, 10);  // standing
    MovementUI mv;
    mv.ref         = &ref;
    mv.standing_max = 0.5;
    mv.ui_moving   = 1.0;
    mv.ui_standing = 5.0;
    mv.window_s    = 30.0;

    SECTION("one report per standing window -> all hits, no false misses")
    {
        // windows [0,5) and [5,10): a report in each
        std::vector<ptime> tst{ t0() + seconds(1), t0() + seconds(6) };
        auto out = evaluateCyclesInPeriod(period, cycles, tst, &mv);
        REQUIRE(out.size() == 2);                 // not 10
        for (const auto& ev : out)
            REQUIRE_FALSE(ev.is_miss);
    }

    SECTION("no reports -> two expected windows, both misses")
    {
        std::vector<ptime> tst;
        auto out = evaluateCyclesInPeriod(period, cycles, tst, &mv);
        REQUIRE(out.size() == 2);                 // not 10 false misses
        for (const auto& ev : out)
            REQUIRE(ev.is_miss);
    }
}

TEST_CASE("evaluateCyclesInPeriod - irregular cycle cadence",
          "[mlat_coverage][cycle_walk]")
{
    auto period = periodFromSeconds(0, 100);

    // Non-uniform spacing: 0, 3, 5, 12 -> windows [0,3), [3,5), [5,12), [12,100)
    std::vector<ptime> cycles{
        t0() + seconds(0),
        t0() + seconds(3),
        t0() + seconds(5),
        t0() + seconds(12),
    };
    std::vector<ptime> tst{
        t0() + seconds(2),    // hits cycle 0  [0,3)
        // cycle 1 [3,5): miss
        t0() + seconds(5),    // hits cycle 2  [5,12)
        t0() + seconds(50),   // hits cycle 3  [12,100)
    };

    auto out = evaluateCyclesInPeriod(period, cycles, tst);
    REQUIRE(out.size() == 4);
    REQUIRE_FALSE(out[0].is_miss);
    REQUIRE      (out[1].is_miss);
    REQUIRE_FALSE(out[2].is_miss);
    REQUIRE_FALSE(out[3].is_miss);
}
