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

#include "coveragepdwalk.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using analysis::MovementUI;
using analysis::PDWalkParams;
using analysis::SpeedSamples;
using analysis::walkReferencePeriodTimeDifference;

using EvaluationRequirement::PDHelpers::RefPeriod;

using boost::posix_time::ptime;
using boost::posix_time::time_from_string;

namespace
{
ptime t0() { return time_from_string("2026-01-01 00:00:00.000"); }

ptime at(double s)
{
    return t0() + boost::posix_time::microseconds(static_cast<long long>(s * 1.0e6));
}

RefPeriod period(double begin_s, double end_s)
{
    return RefPeriod{at(begin_s), at(end_s)};
}

// movement selector with a constant update interval (no speed samples: unknown
// speed counts as moving, so `ui_moving` applies everywhere)
MovementUI constantUI(double ui_s)
{
    MovementUI mv;
    mv.ui_moving   = ui_s;
    mv.ui_standing = ui_s;
    return mv;
}

struct WalkResult
{
    std::vector<double> expected; // slot offsets in seconds from t0
    std::vector<double> missed;
};

WalkResult walk(const RefPeriod& p,
                const std::vector<ptime>& tst,
                const PDWalkParams& params)
{
    WalkResult r;

    auto offset = [ ] (const ptime& t) {
        return static_cast<double>((t - t0()).total_microseconds()) / 1.0e6;
    };

    walkReferencePeriodTimeDifference(
        p, tst, params,
        [ & ] (const ptime& t) { r.expected.push_back(offset(t)); },
        [ & ] (const ptime& t) { r.missed.push_back(offset(t)); });

    return r;
}
}

TEST_CASE("PDWalk - expected slots step by the update interval", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    // 5 s period, 1 s interval: slots at 0, 1, 2, 3, 4 (period end excluded)
    auto r = walk(period(0.0, 5.0), { }, p);

    REQUIRE(r.expected.size() == 5);
    REQUIRE(r.expected.front() == Approx(0.0));
    REQUIRE(r.expected.back()  == Approx(4.0));
}

TEST_CASE("PDWalk - period without any test report misses every slot", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto r = walk(period(0.0, 5.0), { }, p);

    // the whole period is one gap: misses at 1, 2, 3, 4 (the slot at the gap
    // start is the first expected update, its miss falls at the next step)
    REQUIRE(r.missed.size() == 4);
    REQUIRE(r.missed.front() == Approx(1.0));
    REQUIRE(r.missed.back()  == Approx(4.0));
}

TEST_CASE("PDWalk - reports within the update interval produce no miss", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto r = walk(period(0.0, 4.0), { at(1.0), at(2.0), at(3.0) }, p);

    REQUIRE(r.expected.size() == 4);
    REQUIRE(r.missed.empty());
}

TEST_CASE("PDWalk - gap in the middle of a period", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    // reports at 1 s and 5 s, so the 4 s gap between them holds 3 missed slots
    auto r = walk(period(0.0, 6.0), { at(1.0), at(5.0) }, p);

    REQUIRE(r.missed.size() == 3);
    REQUIRE(r.missed[ 0 ] == Approx(2.0));
    REQUIRE(r.missed[ 1 ] == Approx(3.0));
    REQUIRE(r.missed[ 2 ] == Approx(4.0));
}

TEST_CASE("PDWalk - miss tolerance suppresses the slot at the end of a gap", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    // gap of 2.05 s: without tolerance the slot at 2.0 s counts as missed
    auto without = walk(period(0.0, 2.05), { at(2.05) }, p);
    REQUIRE(without.missed.size() == 2);

    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.1f;

    // with a 0.1 s tolerance the slot at 2.0 s is within tolerance of the report
    auto with = walk(period(0.0, 2.05), { at(2.05) }, p);
    REQUIRE(with.missed.size() == 1);
    REQUIRE(with.missed.front() == Approx(1.0));
}

TEST_CASE("PDWalk - standing target uses the slower update interval", "[pd_walk][movement]")
{
    // speed samples: moving until 10 s, standing afterwards
    SpeedSamples spd;
    for (int i = 0; i <= 60; ++i)
    {
        spd.ts.push_back(at(i));
        spd.sp.push_back(i < 10 ? 20.0 : 0.0);
    }

    MovementUI mv;
    mv.test         = &spd;
    mv.window_s     = 10.0;
    mv.standing_max = 0.5;
    mv.ui_moving    = 1.0;
    mv.ui_standing  = 5.0;

    PDWalkParams p;
    p.mv = &mv;

    // 60 s period without any test report: the first 10 s are expected at 1 s,
    // the standing remainder at 5 s. Freezing the cadence at the gap start
    // would expect all 60 s at the moving rate.
    auto r = walk(period(0.0, 60.0), { }, p);

    REQUIRE(r.expected.size() > 10);
    REQUIRE(r.expected.size() < 30);   // far below the 60 slots of a frozen 1 s cadence

    // missed slots never exceed the expected ones, so the probability of
    // detection cannot become negative
    REQUIRE(r.missed.size() <= r.expected.size());
}

TEST_CASE("PDWalk - empty period and missing movement selector are safe", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto empty = walk(period(5.0, 5.0), { at(5.0) }, p);
    REQUIRE(empty.expected.empty());
    REQUIRE(empty.missed.empty());

    PDWalkParams no_mv;
    auto none = walk(period(0.0, 5.0), { }, no_mv);
    REQUIRE(none.expected.empty());
    REQUIRE(none.missed.empty());
}
