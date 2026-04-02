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
#include "acidfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("ACIDFilter construction", "[filter][acid]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"values_str", "AEE"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Aircraft Identification");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.valuesString() == "AEE");
}

TEST_CASE("ACIDFilter filters applicability", "[filter][acid]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"values_str", "AEE"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    // CAT001 has no ACID meta var
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK(filter.filters("CAT010"));
    CHECK(filter.filters("CAT020"));
    CHECK(filter.filters("CAT021"));
    CHECK(filter.filters("CAT048"));
    // CAT062 always returns true (special case)
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("ACIDFilter getConditionString", "[filter][acid]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"active", true},
        {"values_str", "AEE"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    SECTION("non-CAT062 uses LIKE")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK_FALSE(first);
        CHECK(sql.find("target_id LIKE '%AEE%'") != std::string::npos);
        // should not contain callsign_fpl for non-CAT062
        CHECK(sql.find("callsign_fpl") == std::string::npos);
    }

    SECTION("CAT062 also checks callsign_fpl")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT062", read_set, first);

        CHECK(sql.find("target_id LIKE '%AEE%'") != std::string::npos);
        CHECK(sql.find("callsign_fpl LIKE '%AEE%'") != std::string::npos);
    }
}

TEST_CASE("ACIDFilter inactive returns empty", "[filter][acid]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"active", false},
        {"values_str", "AEE"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("ACIDFilter viewpoint save/load round-trip", "[filter][acid][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"active", true},
        {"values_str", "AEE,DLH"}
    });

    ACIDFilter filter(cfg, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("Aircraft Identification"));
    CHECK(vp_filters["Aircraft Identification"].contains("Aircraft Identification Values"));
    CHECK(vp_filters["Aircraft Identification"]["Aircraft Identification Values"] == "AEE,DLH");

    // Load into fresh filter with different initial value
    auto cfg2 = makeFilterConfig("ACIDFilter", "Aircraft Identification", {
        {"active", true},
        {"values_str", "XXX"}
    });

    ACIDFilter filter2(cfg2, nullptr, mock);
    CHECK(filter2.valuesString() == "XXX");

    filter2.loadViewPointConditions(vp_filters);
    CHECK(filter2.valuesString() == "AEE,DLH");

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}
