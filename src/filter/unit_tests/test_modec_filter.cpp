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
#include "modecfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("ModeCFilter construction", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"min_value", -1000.0},
        {"max_value", 3000.0},
        {"null_wanted", false}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Mode C Codes");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.minValue() == Approx(-1000.0f));
    CHECK(filter.maxValue() == Approx(3000.0f));
    CHECK_FALSE(filter.nullWanted());
}

TEST_CASE("ModeCFilter filters applicability", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"min_value", -1000.0},
        {"max_value", 3000.0}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("ModeCFilter getConditionString basic", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", true},
        {"min_value", -1000.0},
        {"max_value", 3000.0},
        {"null_wanted", false}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("modec_code_ft BETWEEN") != std::string::npos);
    CHECK(sql.find("-1000") != std::string::npos);
    CHECK(sql.find("3000") != std::string::npos);
}

TEST_CASE("ModeCFilter getConditionString CAT062 adds baro_alt and fl_measured", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", true},
        {"min_value", -1000.0},
        {"max_value", 3000.0},
        {"null_wanted", false}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT062", read_set, first);

    CHECK(sql.find("modec_code_ft BETWEEN") != std::string::npos);
    CHECK(sql.find("baro_alt BETWEEN") != std::string::npos);
    CHECK(sql.find("fl_measured BETWEEN") != std::string::npos);
}

TEST_CASE("ModeCFilter null_wanted adds IS NULL", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", true},
        {"min_value", -1000.0},
        {"max_value", 3000.0},
        {"null_wanted", true}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.find("IS NULL") != std::string::npos);
}

TEST_CASE("ModeCFilter inactive returns empty", "[filter][modec]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ModeCFilter", "Mode C Codes", {
        {"active", false},
        {"min_value", -1000.0},
        {"max_value", 3000.0}
    });

    ModeCFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}
