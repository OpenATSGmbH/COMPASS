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
 *
 * You should have received a copy of the GNU General Public License
 * along with COMPASS. If not, see <http://www.gnu.org/licenses/>.
 */

#include "catch.hpp"
#include "reconstructortarget.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

using namespace dbContent;
using namespace boost::posix_time;
using namespace boost::gregorian;

namespace
{
const ptime t0(date(2026, 7, 3), time_duration(12, 0, 0));

ptime at(int seconds_offset)
{
    return t0 + seconds(seconds_offset);
}
}

TEST_CASE("IdentityValueInfo observe", "[reconstructor][identity_values]")
{
    IdentityValueInfo info;

    SECTION("initially unset")
    {
        REQUIRE(info.first_seen_.is_not_a_date_time());
        REQUIRE(info.last_seen_.is_not_a_date_time());
    }

    SECTION("single observation sets both")
    {
        info.observe(at(0));

        REQUIRE(info.first_seen_ == at(0));
        REQUIRE(info.last_seen_ == at(0));
    }

    SECTION("in-order observations extend last_seen only")
    {
        info.observe(at(0));
        info.observe(at(10));

        REQUIRE(info.first_seen_ == at(0));
        REQUIRE(info.last_seen_ == at(10));
    }

    SECTION("out-of-order observations are order-independent")
    {
        info.observe(at(10));
        info.observe(at(0));

        REQUIRE(info.first_seen_ == at(0));
        REQUIRE(info.last_seen_ == at(10));
    }

    SECTION("re-observing the same timestamps is idempotent (rebuild case)")
    {
        info.observe(at(0));
        info.observe(at(10));

        IdentityValueInfo before = info;

        // rebuild paths re-add the retained reports
        info.observe(at(0));
        info.observe(at(5));
        info.observe(at(10));

        REQUIRE(info.first_seen_ == before.first_seen_);
        REQUIRE(info.last_seen_ == before.last_seen_);
    }
}

TEST_CASE("compareStoredIdentityValue", "[reconstructor][identity_values]")
{
    const time_duration max_age = seconds(60);

    std::map<unsigned int, IdentityValueInfo> infos;

    SECTION("empty map is unknown")
    {
        REQUIRE(compareStoredIdentityValue(infos, 512u, at(0), max_age)
                == ComparisonResult::UNKNOWN);
    }

    SECTION("current same value confirms")
    {
        infos[512u].observe(at(0));

        REQUIRE(compareStoredIdentityValue(infos, 512u, at(30), max_age)
                == ComparisonResult::SAME);
    }

    SECTION("current other value contradicts")
    {
        infos[512u].observe(at(0));

        REQUIRE(compareStoredIdentityValue(infos, 2242u, at(30), max_age)
                == ComparisonResult::DIFFERENT);
    }

    SECTION("stale value neither confirms nor contradicts")
    {
        infos[512u].observe(at(0));

        // beyond max_age: same value no longer confirms
        REQUIRE(compareStoredIdentityValue(infos, 512u, at(61), max_age)
                == ComparisonResult::UNKNOWN);

        // beyond max_age: other value no longer contradicts
        REQUIRE(compareStoredIdentityValue(infos, 2242u, at(61), max_age)
                == ComparisonResult::UNKNOWN);
    }

    SECTION("staleness boundary is inclusive")
    {
        infos[512u].observe(at(0));

        // exactly max_age old is still current
        REQUIRE(compareStoredIdentityValue(infos, 512u, at(60), max_age)
                == ComparisonResult::SAME);
    }

    SECTION("same wins over different during transition periods")
    {
        // both codes current, e.g. recode transition
        infos[512u].observe(at(0));
        infos[2242u].observe(at(10));

        REQUIRE(compareStoredIdentityValue(infos, 512u, at(30), max_age)
                == ComparisonResult::SAME);
        REQUIRE(compareStoredIdentityValue(infos, 2242u, at(30), max_age)
                == ComparisonResult::SAME);
    }

    SECTION("stale old value plus current new value contradicts the old one")
    {
        // swap signature: old code stale, new code current
        infos[512u].observe(at(0));
        infos[2242u].observe(at(100));

        REQUIRE(compareStoredIdentityValue(infos, 512u, at(120), max_age)
                == ComparisonResult::DIFFERENT);
        REQUIRE(compareStoredIdentityValue(infos, 2242u, at(120), max_age)
                == ComparisonResult::SAME);
    }

    SECTION("observation slightly after evaluation time is current")
    {
        // out-of-order batch processing: negative age must not be treated as stale
        infos[512u].observe(at(5));

        REQUIRE(compareStoredIdentityValue(infos, 512u, at(0), max_age)
                == ComparisonResult::SAME);
    }

    SECTION("string values work the same way")
    {
        std::map<std::string, IdentityValueInfo> acid_infos;

        acid_infos["SWR3EC"].observe(at(0));

        REQUIRE(compareStoredIdentityValue(acid_infos, std::string("SWR3EC"), at(30), max_age)
                == ComparisonResult::SAME);
        REQUIRE(compareStoredIdentityValue(acid_infos, std::string("SWR7VB"), at(30), max_age)
                == ComparisonResult::DIFFERENT);
        REQUIRE(compareStoredIdentityValue(acid_infos, std::string("SWR7VB"), at(120), max_age)
                == ComparisonResult::UNKNOWN);
    }
}
