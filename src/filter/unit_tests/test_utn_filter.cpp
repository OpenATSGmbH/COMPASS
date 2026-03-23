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
#include "utnfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("UTNFilter construction", "[filter][utn]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("UTNFilter", "UTNFilter0", {
        {"utns_str", "0"}
    });

    UTNFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "UTNs");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.utns() == "0");
}

TEST_CASE("UTNFilter filters applicability", "[filter][utn]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("UTNFilter", "UTNFilter0", {
        {"utns_str", "0"}
    });

    UTNFilter filter(cfg, nullptr, mock);

    // UTN always returns true for all DBContent types
    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK(filter.filters("CAT002"));
    CHECK(filter.filters("Unknown"));
}

TEST_CASE("UTNFilter getConditionString", "[filter][utn]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("UTNFilter", "UTNFilter0", {
        {"active", true},
        {"utns_str", "0"}
    });

    UTNFilter filter(cfg, nullptr, mock);

    SECTION("known dbcontent with UTN var")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK_FALSE(first);
        CHECK(sql.find("utn IN (0)") != std::string::npos);
    }

    SECTION("non-associated dbcontent without UTN var returns false")
    {
        // Create a mock without UTN for a specific dbcontent
        MockVariableResolver mock2;
        mock2.addMetaVariable(DBContent::meta_var_timestamp_,
            {{"NoUTN", "timestamp"}});

        auto cfg2 = makeFilterConfig("UTNFilter", "UTNFilter0", {
            {"active", true},
            {"utns_str", "0"}
        });

        UTNFilter filter2(cfg2, nullptr, mock2);

        VariableSet read_set;
        bool first = true;
        std::string sql = filter2.getConditionString("NoUTN", read_set, first);

        CHECK_FALSE(first);
        CHECK(sql.find("false") != std::string::npos);
    }
}

TEST_CASE("UTNFilter inactive returns empty", "[filter][utn]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("UTNFilter", "UTNFilter0", {
        {"active", false},
        {"utns_str", "0"}
    });

    UTNFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("UTNFilter multiple UTNs", "[filter][utn]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("UTNFilter", "UTNFilter0", {
        {"active", true},
        {"utns_str", "1,5,10"}
    });

    UTNFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.find("utn IN (") != std::string::npos);
    CHECK(sql.find("1") != std::string::npos);
    CHECK(sql.find("5") != std::string::npos);
    CHECK(sql.find("10") != std::string::npos);
}
