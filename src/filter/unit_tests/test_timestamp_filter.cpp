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
#include "mock_variable_resolver.h"
#include "timestampfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"
#include "util/timeconv.h"

using namespace dbContent;

TEST_CASE("TimestampFilter construction", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Timestamp");
    CHECK_FALSE(filter.getActive());
}

TEST_CASE("TimestampFilter filters applicability", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT010"));
    CHECK(filter.filters("CAT020"));
    CHECK(filter.filters("CAT021"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
    CHECK_FALSE(filter.filters("Unknown"));
}

TEST_CASE("TimestampFilter getConditionString", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"active", true},
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    SECTION("active filter produces SQL")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK_FALSE(first);
        CHECK(sql.find("timestamp >=") != std::string::npos);
        CHECK(sql.find("timestamp <=") != std::string::npos);
    }

    SECTION("first=false prepends AND")
    {
        VariableSet read_set;
        bool first = false;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK(sql.find(" AND") != std::string::npos);
    }

    SECTION("unknown dbcontent returns empty")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT002", read_set, first);

        CHECK(sql.empty());
        CHECK(first);
    }
}

TEST_CASE("TimestampFilter inactive returns empty", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"active", false},
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
    CHECK(first);
}

TEST_CASE("TimestampFilter reset with ptime values", "[filter][timestamp]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    auto new_min = Utils::Time::fromString("2026-02-01 00:00:00.000");
    auto new_max = Utils::Time::fromString("2026-02-01 23:59:59.999");

    filter.reset(new_min, new_max);

    CHECK(filter.minValue() == new_min);
    CHECK(filter.maxValue() == new_max);
}

TEST_CASE("TimestampFilter viewpoint save/load round-trip", "[filter][timestamp][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"active", true},
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });

    TimestampFilter filter(cfg, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("Timestamp"));
    CHECK(vp_filters["Timestamp"]["Timestamp Minimum"] == "2026-01-05 09:59:58.752");
    CHECK(vp_filters["Timestamp"]["Timestamp Maximum"] == "2026-01-05 12:05:19.968");

    // Load into fresh filter with different values
    auto cfg2 = makeFilterConfig("TimestampFilter", "TimestampFilter0", {
        {"active", true},
        {"min_value", "2020-01-01 00:00:00.000"},
        {"max_value", "2020-01-01 23:59:59.999"}
    });

    TimestampFilter filter2(cfg2, nullptr, mock);
    filter2.loadViewPointConditions(vp_filters);

    CHECK(filter2.minValue() == Utils::Time::fromString("2026-01-05 09:59:58.752"));
    CHECK(filter2.maxValue() == Utils::Time::fromString("2026-01-05 12:05:19.968"));

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}
