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
#include "acadfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("ACADFilter construction", "[filter][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Aircraft Address");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.valuesString() == "4C8070");
}

TEST_CASE("ACADFilter filters applicability", "[filter][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    // ACAD not in CAT001
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK(filter.filters("CAT010"));
    CHECK(filter.filters("CAT020"));
    CHECK(filter.filters("CAT021"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("ACADFilter getConditionString", "[filter][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"active", true},
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK_FALSE(first);
    // 4C8070 hex = 5013616 decimal
    CHECK(sql.find("target_addr IN (5013616)") != std::string::npos);
}

TEST_CASE("ACADFilter inactive returns empty", "[filter][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"active", false},
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("ACADFilter unknown dbcontent returns empty", "[filter][acad]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ACADFilter", "Aircraft Address", {
        {"active", true},
        {"values_str", "4C8070"}
    });

    ACADFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT001", read_set, first);

    CHECK(sql.empty());
}
