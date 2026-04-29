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
#include "reftrajaccuracyfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("RefTrajAccuracyFilter construction", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "RefTraj Accuracy");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.minValue() == Approx(30.0f));
}

TEST_CASE("RefTrajAccuracyFilter filters applicability", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0");

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK_FALSE(filter.filters("CAT048"));
    CHECK_FALSE(filter.filters("CAT062"));
    CHECK_FALSE(filter.filters("CAT021"));
}

TEST_CASE("RefTrajAccuracyFilter getConditionString", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("RefTraj", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("sqrt(pow(x_stddev,2)") != std::string::npos);
    CHECK(sql.find("pow(y_stddev,2)") != std::string::npos);
    CHECK(sql.find("<= 30") != std::string::npos);
}

TEST_CASE("RefTrajAccuracyFilter inactive returns empty", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", false},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("RefTraj", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("RefTrajAccuracyFilter non-RefTraj with x/y stddev still emits", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    // The guard in getConditionString is on x/y stddev availability, not on
    // dbcontent name; CAT062 has both in the standard mock so SQL is emitted.
    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT062", read_set, first);

    CHECK_FALSE(sql.empty());
}

TEST_CASE("RefTrajAccuracyFilter without x/y stddev returns empty", "[filter][reftrajaccuracy][partial]")
{
    // dbcontent without X StdDev / Y StdDev meta vars must not assert in
    // metaGetVariableDBColumn — the filter must early-return.
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    // CAT048 has neither x_stddev nor y_stddev in the standard mock.
    CHECK_FALSE(filter.filters("CAT048"));

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
    CHECK(first);
}

TEST_CASE("RefTrajAccuracyFilter setter", "[filter][reftrajaccuracy]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    filter.minValue(50.0f);
    CHECK(filter.minValue() == Approx(50.0f));
}

TEST_CASE("RefTrajAccuracyFilter viewpoint save/load round-trip", "[filter][reftrajaccuracy][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("RefTraj Accuracy"));
    CHECK(vp_filters["RefTraj Accuracy"].contains("Accuracy Minimum"));

    // Load into fresh filter with different value
    auto cfg2 = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 99.0}
    });

    RefTrajAccuracyFilter filter2(cfg2, nullptr, mock);
    CHECK(filter2.minValue() == Approx(99.0f));

    filter2.loadViewPointConditions(vp_filters);
    CHECK(filter2.minValue() == Approx(30.0f));

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}

TEST_CASE("RefTrajAccuracyFilter viewpoint load from string value", "[filter][reftrajaccuracy][viewpoint]")
{
    auto mock = createStandardMock();

    // Simulate hand-written viewpoint file where value is a string
    nlohmann::json vp_filters;
    vp_filters["RefTraj Accuracy"] = {{"Accuracy Minimum", "25.5"}};

    auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "RefTrajAccuracyFilter0", {
        {"active", true},
        {"min_value", 30.0}
    });

    RefTrajAccuracyFilter filter(cfg, nullptr, mock);
    filter.loadViewPointConditions(vp_filters);

    CHECK(filter.minValue() == Approx(25.5f));
}
