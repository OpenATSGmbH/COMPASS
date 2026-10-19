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

#include <cmath>
#include <map>

using analysis::MovementUI;
using analysis::PDWalkParams;
using analysis::SpeedSamples;
using analysis::samplePiece;
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

double offset(const ptime& t)
{
    return static_cast<double>((t - t0()).total_microseconds()) / 1.0e6;
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

struct Piece
{
    double begin    = 0.0;
    double end      = 0.0;
    double expected = 0.0;
    bool   miss     = false;
};

struct WalkResult
{
    std::vector<Piece>  pieces;
    std::vector<double> missed;   // begin of each missed piece = miss time
    double expected = 0.0;        // sum of the expected weights
    double num_missed = 0.0;      // sum of the missed weights
};

WalkResult walk(const RefPeriod& p,
                const std::vector<ptime>& tst,
                const PDWalkParams& params)
{
    WalkResult r;

    walkReferencePeriodTimeDifference(
        p, tst, params,
        [ & ] (const ptime& b, const ptime& e, double expected, bool is_miss)
        {
            r.pieces.push_back(Piece{offset(b), offset(e), expected, is_miss});
            r.expected += expected;
            if (is_miss)
            {
                r.missed.push_back(offset(b));
                r.num_missed += expected;
            }
        });

    return r;
}

// Splits all pieces of a walk into 1D cells along a straight track, position
// x(t) in meters, and returns per cell (#EUI, #MUI).
std::map<long, std::pair<double, double>> cellCounts(const RefPeriod& p,
                                                     const std::vector<ptime>& tst,
                                                     const PDWalkParams& params,
                                                     const std::function<double(double)>& x_of_t,
                                                     double cell_m)
{
    std::map<long, std::pair<double, double>> cells;

    walkReferencePeriodTimeDifference(
        p, tst, params,
        [ & ] (const ptime& b, const ptime& e, double expected, bool is_miss)
        {
            samplePiece(b, e, params.sample_step_s, [ & ] (const ptime& t, double frac)
            {
                const long cell = static_cast<long>(std::floor(x_of_t(offset(t)) / cell_m));
                cells[cell].first += expected * frac;
                if (is_miss)
                    cells[cell].second += expected * frac;
            });
        });

    return cells;
}
}

TEST_CASE("PDWalk - a period carries its length in update intervals", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto r = walk(period(0.0, 5.0), { }, p);

    REQUIRE(r.expected == Approx(5.0));
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
    REQUIRE(r.num_missed == Approx(4.0));
    REQUIRE(r.expected   == Approx(5.0));
}

TEST_CASE("PDWalk - reports within the update interval produce no miss", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto r = walk(period(0.0, 4.0), { at(1.0), at(2.0), at(3.0) }, p);

    REQUIRE(r.expected == Approx(4.0));
    REQUIRE(r.missed.empty());
}

TEST_CASE("PDWalk - reports closer than the update interval add less than one slot", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    // ten reports in 2 s: still 2 expected slots, not 10
    std::vector<ptime> tst;
    for (int i = 1; i < 10; ++i)
        tst.push_back(at(0.2 * i));

    auto r = walk(period(0.0, 2.0), tst, p);

    REQUIRE(r.expected == Approx(2.0));
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
    REQUIRE(r.expected == Approx(6.0));
}

TEST_CASE("PDWalk - miss tolerance suppresses the slot at the end of a gap", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    // gap of 2.05 s: without tolerance the slot at 2.0 s counts as missed
    auto without = walk(period(0.0, 2.05), { at(2.05) }, p);
    REQUIRE(without.missed.size() == 2);
    REQUIRE(without.expected == Approx(2.05));

    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.1f;

    // with a 0.1 s tolerance the slot at 2.0 s is within tolerance of the report
    auto with = walk(period(0.0, 2.05), { at(2.05) }, p);
    REQUIRE(with.missed.size() == 1);
    REQUIRE(with.missed.front() == Approx(1.0));
    REQUIRE(with.expected == Approx(2.05));
}

TEST_CASE("PDWalk - pieces cover the period without overlap", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto r = walk(period(0.0, 10.0), { at(0.4), at(3.7), at(4.1), at(8.95) }, p);

    REQUIRE_FALSE(r.pieces.empty());
    REQUIRE(r.pieces.front().begin == Approx(0.0));
    REQUIRE(r.pieces.back().end    == Approx(10.0));

    for (std::size_t i = 0; i < r.pieces.size(); ++i)
    {
        REQUIRE(r.pieces[ i ].end > r.pieces[ i ].begin);
        REQUIRE(r.pieces[ i ].expected >= 0.0);
        if (r.pieces[ i ].miss)
            REQUIRE(r.pieces[ i ].expected == Approx(1.0));
        if (i > 0)
            REQUIRE(r.pieces[ i ].begin == Approx(r.pieces[ i - 1 ].end));
    }

    REQUIRE(r.expected == Approx(10.0));
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

    REQUIRE(r.expected > 10.0);
    REQUIRE(r.expected < 30.0);   // far below the 60 slots of a frozen 1 s cadence

    // missed slots never exceed the expected ones, so the probability of
    // detection cannot become negative
    REQUIRE(r.num_missed <= r.expected);
}

TEST_CASE("PDWalk - samplePiece splits a piece by time", "[pd_walk]")
{
    std::vector<double> times, fracs;
    samplePiece(at(0.0), at(1.0), 0.2, [ & ] (const ptime& t, double f)
    {
        times.push_back(offset(t));
        fracs.push_back(f);
    });

    REQUIRE(times.size() == 5);
    REQUIRE(times.front() == Approx(0.1));
    REQUIRE(times.back()  == Approx(0.9));

    double sum = 0.0;
    for (double f : fracs)
        sum += f;
    REQUIRE(sum == Approx(1.0));
}

TEST_CASE("PDWalk - no cell gets more missed than expected slots", "[pd_walk][cells]")
{
    PDWalkParams p;
    p.use_miss_tolerance = true;
    p.miss_tolerance_s   = 0.1f;

    // irregular reports with gaps of different length
    const std::vector<ptime> tst = { at(0.3), at(1.4), at(2.1), at(5.65), at(6.2),
                                     at(9.9), at(10.6), at(17.05), at(17.9) };

    SECTION("slow target crossing cell borders inside gaps")
    {
        auto mv = constantUI(1.0);
        p.mv = &mv;

        // 7 m/s along a straight line, 20 m cells
        auto cells = cellCounts(period(0.0, 20.0), tst, p,
                                [] (double t) { return 7.0 * t; }, 20.0);

        double eui = 0.0, mui = 0.0;
        for (const auto& kv : cells)
        {
            REQUIRE(kv.second.second <= kv.second.first + 1e-9);
            eui += kv.second.first;
            mui += kv.second.second;
        }
        REQUIRE(eui == Approx(20.0));
        REQUIRE(mui > 0.0);
    }

    SECTION("fast target crossing several cells per slot")
    {
        auto mv = constantUI(1.0);
        p.mv = &mv;

        // 70 m/s (runway), 20 m cells: one slot spans 3 - 4 cells
        auto cells = cellCounts(period(0.0, 20.0), tst, p,
                                [] (double t) { return 70.0 * t; }, 20.0);

        REQUIRE(cells.size() > 50);
        for (const auto& kv : cells)
            REQUIRE(kv.second.second <= kv.second.first + 1e-9);
    }

    SECTION("speed around the standing limit")
    {
        // speed alternates around the standing limit every 0.5 s, so the
        // update interval switches between 1 s and 2 s inside the gaps
        SpeedSamples spd;
        for (int i = 0; i <= 40; ++i)
        {
            spd.ts.push_back(at(0.5 * i));
            spd.sp.push_back(i % 2 ? 0.6 : 0.4);
        }

        MovementUI mv;
        mv.test         = &spd;
        mv.window_s     = 0.5;
        mv.standing_max = 0.5;
        mv.ui_moving    = 1.0;
        mv.ui_standing  = 2.0;
        p.mv = &mv;

        // slow creep on a stand, 5 m cells
        auto cells = cellCounts(period(0.0, 20.0), tst, p,
                                [] (double t) { return 0.5 * t; }, 5.0);

        for (const auto& kv : cells)
            REQUIRE(kv.second.second <= kv.second.first + 1e-9);
    }
}

TEST_CASE("PDWalk - empty period and missing movement selector are safe", "[pd_walk]")
{
    auto mv = constantUI(1.0);

    PDWalkParams p;
    p.mv = &mv;

    auto empty = walk(period(5.0, 5.0), { at(5.0) }, p);
    REQUIRE(empty.pieces.empty());

    PDWalkParams no_mv;
    auto none = walk(period(0.0, 5.0), { }, no_mv);
    REQUIRE(none.pieces.empty());
}
