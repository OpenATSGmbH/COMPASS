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
#include "mode3afilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("Mode3AFilter construction", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Mode 3/A Codes");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.valuesString() == "3771");
}

TEST_CASE("Mode3AFilter filters applicability", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT010"));
    CHECK(filter.filters("CAT020"));
    CHECK(filter.filters("CAT021"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("Mode3AFilter getConditionString", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"active", true},
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK_FALSE(first);
    // 3771 octal = 2041 decimal
    CHECK(sql.find("mode3a_code IN (2041)") != std::string::npos);
}

TEST_CASE("Mode3AFilter octal conversion 7700", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"active", true},
        {"values_str", "7700"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    // 7700 octal = 4032 decimal
    CHECK(sql.find("4032") != std::string::npos);
}

TEST_CASE("Mode3AFilter inactive returns empty", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"active", false},
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("Mode3AFilter first=false prepends AND", "[filter][mode3a]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("Mode3AFilter", "Mode 3/A Codes", {
        {"active", true},
        {"values_str", "3771"}
    });

    Mode3AFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = false;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.substr(0, 4) == " AND");
}
