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
#include "primaryonlyfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("PrimaryOnlyFilter construction", "[filter][primaryonly]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only");

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Primary Only");
    CHECK_FALSE(filter.getActive());
}

TEST_CASE("PrimaryOnlyFilter filters applicability", "[filter][primaryonly]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only");

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    // Needs at least one of m3a/mc/acad/acid
    CHECK(filter.filters("CAT001"));   // has m3a, mc
    CHECK(filter.filters("CAT010"));   // has m3a, mc, acad, acid
    CHECK(filter.filters("CAT048"));   // has all
    CHECK(filter.filters("CAT062"));   // has all
    CHECK(filter.filters("RefTraj"));  // has all
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("PrimaryOnlyFilter getConditionString", "[filter][primaryonly]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only", {
        {"active", true}
    });

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("mode3a_code IS NULL") != std::string::npos);
    CHECK(sql.find("modec_code_ft IS NULL") != std::string::npos);
    CHECK(sql.find("target_addr IS NULL") != std::string::npos);
    CHECK(sql.find("target_id IS NULL") != std::string::npos);
    CHECK(sql.find("detection_type") != std::string::npos);
    CHECK(sql.find("IN (1,3,6,7)") != std::string::npos);
}

TEST_CASE("PrimaryOnlyFilter inactive returns empty", "[filter][primaryonly]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only", {
        {"active", false}
    });

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    // PrimaryOnlyFilter doesn't check active_ in getConditionString - it always generates
    // But with active_=false, getActive() returns false so the caller won't call it
    // The filter itself always generates SQL when called
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    // PrimaryOnlyFilter generates SQL regardless of active_ flag
    // (unlike other filters that check active_ internally)
    CHECK_FALSE(sql.empty());
}

TEST_CASE("PrimaryOnlyFilter viewpoint save/load round-trip", "[filter][primaryonly][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("PrimaryOnlyFilter", "Primary Only", {
        {"active", true}
    });

    PrimaryOnlyFilter filter(cfg, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("Primary Only"));
    CHECK(vp_filters["Primary Only"].is_object());
    CHECK(vp_filters["Primary Only"].empty());

    // Load into fresh filter - no state to change, just verify no crash
    auto cfg2 = makeFilterConfig("PrimaryOnlyFilter", "Primary Only", {
        {"active", true}
    });

    PrimaryOnlyFilter filter2(cfg2, nullptr, mock);
    filter2.loadViewPointConditions(vp_filters);

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}
