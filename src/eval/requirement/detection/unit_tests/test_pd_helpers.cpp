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

#include "eval/requirement/detection/detection_pd_helpers.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <set>

using EvaluationRequirement::PDHelpers::MissTestParams;
using EvaluationRequirement::PDHelpers::isMiss;
using EvaluationRequirement::PDHelpers::numMisses;
using EvaluationRequirement::PDHelpers::missDuration;
using EvaluationRequirement::PDHelpers::buildReferencePeriods;

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using boost::posix_time::seconds;
using boost::posix_time::milliseconds;
using boost::posix_time::time_from_string;

namespace
{
ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }
}

TEST_CASE("PDHelpers::isMiss - update interval only", "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s = 1.0f;

    REQUIRE_FALSE(isMiss(0.0f,  p));
    REQUIRE_FALSE(isMiss(0.5f,  p));
    REQUIRE_FALSE(isMiss(1.0f,  p));  // strictly greater required
    REQUIRE      (isMiss(1.001f, p));
    REQUIRE      (isMiss(2.5f,  p));
}

TEST_CASE("PDHelpers::numMisses - update interval only", "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s = 1.0f;

    REQUIRE(numMisses(0.5f, p) == 0);     // not a miss -> 0
    REQUIRE(numMisses(1.0f, p) == 0);     // boundary, not a miss
    REQUIRE(numMisses(1.5f, p) == 1);
    REQUIRE(numMisses(3.0f, p) == 3);
    REQUIRE(numMisses(3.999f, p) == 3);   // floor
    REQUIRE(numMisses(4.0f, p) == 4);
}

TEST_CASE("PDHelpers::isMiss - miss tolerance", "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.2f;

    // adjusted = gap - tolerance
    REQUIRE_FALSE(isMiss(1.0f,  p));    // adj = 0.8 -> not > 1
    REQUIRE_FALSE(isMiss(1.19f, p));    // adj = 0.99
    REQUIRE      (isMiss(1.21f, p));    // adj = 1.01

    REQUIRE(numMisses(1.21f, p) == 1);
    REQUIRE(numMisses(2.21f, p) == 2);
}

TEST_CASE("PDHelpers::isMiss - min gap length suppresses small gaps",
          "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_min_gap_length = true;
    p.min_gap_length_s   = 1.5f;

    REQUIRE_FALSE(isMiss(1.2f, p));  // > UI but < min: ignored
    REQUIRE_FALSE(isMiss(1.4f, p));
    REQUIRE      (isMiss(1.6f, p));
    REQUIRE      (isMiss(3.0f, p));
}

TEST_CASE("PDHelpers::isMiss - max gap length suppresses out-of-coverage gaps",
          "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_max_gap_length = true;
    p.max_gap_length_s   = 30.0f;

    REQUIRE      (isMiss(5.0f,  p));
    REQUIRE      (isMiss(30.0f, p));
    REQUIRE_FALSE(isMiss(30.1f, p));  // beyond max -> out of coverage
    REQUIRE_FALSE(isMiss(120.0f, p));
}

TEST_CASE("PDHelpers::isMiss - tolerance combined with min/max",
          "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.5f;
    p.use_min_gap_length = true;
    p.min_gap_length_s   = 2.0f;
    p.use_max_gap_length = true;
    p.max_gap_length_s   = 10.0f;

    // adj = gap - 0.5
    REQUIRE_FALSE(isMiss(2.49f, p));  // adj = 1.99 -> below min
    REQUIRE      (isMiss(2.55f, p));  // adj = 2.05 -> miss
    REQUIRE      (isMiss(10.5f, p));  // adj = 10.0 -> at max boundary
    REQUIRE_FALSE(isMiss(10.51f, p)); // adj = 10.01 -> above max
}

TEST_CASE("PDHelpers::numMisses - returns 0 for non-misses",
          "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_max_gap_length = true;
    p.max_gap_length_s   = 5.0f;

    REQUIRE(numMisses(0.4f,   p) == 0);   // sub-UI
    REQUIRE(numMisses(100.0f, p) == 0);   // suppressed by max gap
}

TEST_CASE("PDHelpers::numMisses - update_interval_s == 0 returns 0",
          "[pd_helpers][misstest]")
{
    MissTestParams p;
    p.update_interval_s = 0.0f;
    REQUIRE(numMisses(5.0f, p) == 0);
}

TEST_CASE("PDHelpers::missDuration - update interval only",
          "[pd_helpers][misstest][timeratio]")
{
    // ED-129C Appendix C Equation 2-2: contribution max(G - UI, 0)
    MissTestParams p;
    p.update_interval_s = 1.0f;

    REQUIRE(missDuration(0.5f, p) == Approx(0.0f));   // not a miss -> 0
    REQUIRE(missDuration(1.0f, p) == Approx(0.0f));   // boundary, not a miss
    REQUIRE(missDuration(1.5f, p) == Approx(0.5f));
    REQUIRE(missDuration(4.0f, p) == Approx(3.0f));
}

TEST_CASE("PDHelpers::missDuration - miss tolerance",
          "[pd_helpers][misstest][timeratio]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.2f;

    REQUIRE(missDuration(1.1f, p) == Approx(0.0f));           // adj = 0.9, not a miss
    REQUIRE(missDuration(2.2f, p) == Approx(1.0f));           // adj = 2.0
}

TEST_CASE("PDHelpers::missDuration - long gap configuration",
          "[pd_helpers][misstest][timeratio]")
{
    // ED-129C REQ 18 setup for the 3NM service: UI 5s,
    // min gap length 3 x UI x 1.1 = 16.5s; contribution G - UI
    // for long gaps only (Appendix C Equation 4-2)
    MissTestParams p;
    p.update_interval_s  = 5.0f;
    p.use_min_gap_length = true;
    p.min_gap_length_s   = 16.5f;

    REQUIRE(missDuration(10.0f, p) == Approx(0.0f));          // miss, but not a long gap
    REQUIRE(missDuration(16.4f, p) == Approx(0.0f));          // just below long gap threshold
    REQUIRE(missDuration(20.0f, p) == Approx(15.0f));         // long gap: 20 - 5
}

TEST_CASE("PDHelpers::missDuration - max gap length suppresses out-of-coverage gaps",
          "[pd_helpers][misstest][timeratio]")
{
    MissTestParams p;
    p.update_interval_s  = 1.0f;
    p.use_max_gap_length = true;
    p.max_gap_length_s   = 5.0f;

    REQUIRE(missDuration(3.0f,   p) == Approx(2.0f));
    REQUIRE(missDuration(100.0f, p) == Approx(0.0f));         // suppressed by max gap
}

TEST_CASE("PDHelpers::buildReferencePeriods - empty input",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.empty());
}

TEST_CASE("PDHelpers::buildReferencePeriods - single timestamp dropped by min duration",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts{t0()};
    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.empty());  // degenerate period (begin==end) < min
}

TEST_CASE("PDHelpers::buildReferencePeriods - one contiguous period",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    for (int i = 0; i < 10; ++i)
        ts.insert(t0() + seconds(i));

    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.size() == 1);
    REQUIRE(periods[0].begin == t0());
    REQUIRE(periods[0].end   == t0() + seconds(9));
}

TEST_CASE("PDHelpers::buildReferencePeriods - split on large gap",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    // period A: 0..5 s
    for (int i = 0; i <= 5; ++i)
        ts.insert(t0() + seconds(i));
    // 10 s gap -- exceeds max_gap=4s -> new period
    // period B: 15..20 s
    for (int i = 15; i <= 20; ++i)
        ts.insert(t0() + seconds(i));

    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.size() == 2);
    REQUIRE(periods[0].begin == t0());
    REQUIRE(periods[0].end   == t0() + seconds(5));
    REQUIRE(periods[1].begin == t0() + seconds(15));
    REQUIRE(periods[1].end   == t0() + seconds(20));
}

TEST_CASE("PDHelpers::buildReferencePeriods - gap exactly at threshold extends period",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    ts.insert(t0());
    ts.insert(t0() + seconds(4));   // gap == max_gap -> extends
    ts.insert(t0() + seconds(8));

    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.size() == 1);
    REQUIRE(periods[0].begin == t0());
    REQUIRE(periods[0].end   == t0() + seconds(8));
}

TEST_CASE("PDHelpers::buildReferencePeriods - short periods are dropped",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    // long period
    for (int i = 0; i <= 10; ++i)
        ts.insert(t0() + seconds(i));
    // gap
    // short period (only 0.5 s wide) -- dropped by min_duration=1s
    ts.insert(t0() + seconds(20));
    ts.insert(t0() + seconds(20) + milliseconds(500));

    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.size() == 1);
    REQUIRE(periods[0].begin == t0());
    REQUIRE(periods[0].end   == t0() + seconds(10));
}

TEST_CASE("PDHelpers::buildReferencePeriods - multiple periods all kept",
          "[pd_helpers][periods]")
{
    std::set<ptime> ts;
    for (int i = 0; i <= 3; ++i)  ts.insert(t0() + seconds(i));        // A
    for (int i = 0; i <= 3; ++i)  ts.insert(t0() + seconds(10 + i));   // B
    for (int i = 0; i <= 3; ++i)  ts.insert(t0() + seconds(100 + i));  // C

    auto periods = buildReferencePeriods(ts, seconds(4), seconds(1));
    REQUIRE(periods.size() == 3);
    REQUIRE(periods[0].end - periods[0].begin == seconds(3));
    REQUIRE(periods[1].end - periods[1].begin == seconds(3));
    REQUIRE(periods[2].end - periods[2].begin == seconds(3));
}
