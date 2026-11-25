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

#include "timeperiod.h"

#include <cmath>
#include <set>
#include <utility>
#include <vector>

using namespace boost::posix_time;
using dbContent::TargetReport::DataID;
using dbContent::TargetReport::Index;
using dbContent::TargetReport::IndexMap;

namespace
{

const ptime T0(boost::gregorian::date(2026, 1, 1), hours(12));

ptime at(double seconds)
{
    return T0 + milliseconds(std::llround(seconds * 1000.0));
}

IndexMap makeIndexes(const std::vector<double>& seconds)
{
    IndexMap indexes;

    for (unsigned int i = 0; i < seconds.size(); ++i)
        indexes.insert({at(seconds[i]), Index(100 + i, i)});

    return indexes;
}

// all updates usable except the given internal indexes
TimePeriodCollection::RefUpdateFilter usableExcept(const std::set<unsigned int>& not_usable)
{
    return [ not_usable ] (const DataID& id) { return not_usable.count(id.index().idx_internal) == 0; };
}

std::vector<std::pair<ptime, ptime>> periodsOf(const TimePeriodCollection& periods)
{
    std::vector<std::pair<ptime, ptime>> ret;

    for (unsigned int i = 0; i < periods.size(); ++i)
    {
        REQUIRE(periods.period(i).type() == TimePeriod::Type::InsideSector);
        ret.emplace_back(periods.period(i).begin(), periods.period(i).end());
    }

    return ret;
}

using Periods = std::vector<std::pair<ptime, ptime>>;

} // namespace

TEST_CASE("TimePeriodCollection createFromReference", "[time_period_collection]")
{
    const time_duration max_gap = seconds(5);

    TimePeriodCollection periods;

    SECTION("no reference updates: no period")
    {
        periods.createFromReference(IndexMap(), max_gap, usableExcept({}));
        REQUIRE(periods.size() == 0);
    }

    SECTION("regular usable updates: one period from the first to the last")
    {
        periods.createFromReference(makeIndexes({0, 1, 2, 3, 4}), max_gap, usableExcept({}));
        REQUIRE(periodsOf(periods) == Periods{{at(0), at(4)}});
    }

    SECTION("a single usable update: a period of length 0")
    {
        periods.createFromReference(makeIndexes({7}), max_gap, usableExcept({}));
        REQUIRE(periodsOf(periods) == Periods{{at(7), at(7)}});
    }

    SECTION("a gap longer than max_ref_time_diff splits, a gap equal to it does not")
    {
        periods.createFromReference(makeIndexes({0, 1, 6, 12, 13}), max_gap, usableExcept({}));
        REQUIRE(periodsOf(periods) == Periods{{at(0), at(6)}, {at(12), at(13)}});
    }

    SECTION("an unusable update ends the period, whatever the gap")
    {
        periods.createFromReference(makeIndexes({0, 1, 2, 3, 4}), max_gap, usableExcept({2}));
        REQUIRE(periodsOf(periods) == Periods{{at(0), at(1)}, {at(3), at(4)}});
    }

    SECTION("several unusable updates in a row: one break")
    {
        periods.createFromReference(makeIndexes({0, 1, 2, 3, 4, 5}), max_gap, usableExcept({2, 3}));
        REQUIRE(periodsOf(periods) == Periods{{at(0), at(1)}, {at(4), at(5)}});
    }

    SECTION("leading and trailing unusable updates are left out")
    {
        periods.createFromReference(makeIndexes({0, 1, 2, 3, 4}), max_gap, usableExcept({0, 4}));
        REQUIRE(periodsOf(periods) == Periods{{at(1), at(3)}});
    }

    SECTION("an unusable update between two usable single updates: two periods of length 0")
    {
        periods.createFromReference(makeIndexes({0, 1, 2}), max_gap, usableExcept({1}));
        REQUIRE(periodsOf(periods) == Periods{{at(0), at(0)}, {at(2), at(2)}});
    }

    SECTION("no usable update: no period")
    {
        periods.createFromReference(makeIndexes({0, 1, 2}), max_gap, usableExcept({0, 1, 2}));
        REQUIRE(periods.size() == 0);
    }

    SECTION("the predicate gets the update with its timestamp and index")
    {
        std::vector<std::pair<ptime, unsigned int>> seen;

        periods.createFromReference(makeIndexes({0, 2}), max_gap,
                                    [ & ] (const DataID& id)
                                    {
                                        seen.emplace_back(id.timestamp(), id.index().idx_external);
                                        return true;
                                    });

        REQUIRE(seen == std::vector<std::pair<ptime, unsigned int>>{{at(0), 100}, {at(2), 101}});
    }

    SECTION("a new run replaces the previous periods")
    {
        periods.createFromReference(makeIndexes({0, 1}), max_gap, usableExcept({}));
        periods.createFromReference(makeIndexes({10, 11}), max_gap, usableExcept({}));
        REQUIRE(periodsOf(periods) == Periods{{at(10), at(11)}});
    }
}
