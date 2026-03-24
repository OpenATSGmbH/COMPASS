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
#include "mlatrufilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("MLATRUFilter construction", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0", {
        {"rus_str", "nulL,kor,schas"},
        {"match_all", false}
    });

    MLATRUFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "MLAT RUs");
    CHECK_FALSE(filter.getActive());
    CHECK(filter.rus() == "nulL,kor,schas");
    CHECK_FALSE(filter.matchAll());
}

TEST_CASE("MLATRUFilter filters applicability", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0");

    MLATRUFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT020"));
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK_FALSE(filter.filters("CAT021"));
    CHECK_FALSE(filter.filters("CAT048"));
    CHECK_FALSE(filter.filters("CAT062"));
}

TEST_CASE("MLATRUFilter getConditionString with data sources", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0", {
        {"active", true},
        {"rus_str", "kor"},
        {"match_all", false}
    });

    MLATRUFilter filter(cfg, nullptr, mock);

    // Push MLAT data sources: ds_id 100 has RU name "kor" at indexes {3, 7}
    std::map<unsigned int, std::map<std::string, std::vector<unsigned int>>> lookup;
    lookup[100]["kor"] = {3, 7};
    lookup[100]["schas"] = {1};
    filter.updateMLATDataSources(lookup);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT020", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("ds_id = 100") != std::string::npos);
    CHECK(sql.find("json_contains(contrib_receivers") != std::string::npos);
}

TEST_CASE("MLATRUFilter inactive returns empty", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0", {
        {"active", false},
        {"rus_str", "kor"}
    });

    MLATRUFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT020", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("MLATRUFilter checkRUs", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0");

    MLATRUFilter filter(cfg, nullptr, mock);

    std::set<std::string> known_names = {"kor", "schas", "west"};
    filter.updateMLATKnownRUNames(known_names);

    CHECK(filter.checkRUs("kor,schas"));
    CHECK(filter.checkRUs("null"));
    CHECK(filter.checkRUs("42"));
    CHECK_FALSE(filter.checkRUs("unknown_name"));
}

TEST_CASE("MLATRUFilter pushed data sources", "[filter][mlatru]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("MLATRUFilter", "MLATRUFilter0");

    MLATRUFilter filter(cfg, nullptr, mock);

    std::map<unsigned int, std::map<std::string, std::vector<unsigned int>>> lookup;
    lookup[200]["alpha"] = {0, 1};
    filter.updateMLATDataSources(lookup);

    std::set<std::string> known_names = {"alpha", "beta"};
    filter.updateMLATKnownRUNames(known_names);

    CHECK(filter.checkRUs("alpha"));
    CHECK_FALSE(filter.checkRUs("gamma"));
}
