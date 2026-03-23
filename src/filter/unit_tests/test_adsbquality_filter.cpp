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
#include "adsbqualityfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("ADSBQualityFilter construction", "[filter][adsbquality]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"use_v0", false},
        {"use_v1", false},
        {"use_v2", true},
        {"use_min_nucp", false},
        {"min_nucp", 4},
        {"use_max_nucp", false},
        {"max_nucp", 4},
        {"use_min_nic", false},
        {"min_nic", 5},
        {"use_max_nic", false},
        {"max_nic", 5},
        {"use_min_nacp", false},
        {"min_nacp", 5},
        {"use_max_nacp", false},
        {"max_nacp", 5},
        {"use_min_sil_v1", false},
        {"min_sil_v1", 0},
        {"use_max_sil_v1", false},
        {"max_sil_v1", 0},
        {"use_min_sil_v2", true},
        {"min_sil_v2", 0},
        {"use_max_sil_v2", true},
        {"max_sil_v2", 0}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "ADSB Quality");
    CHECK_FALSE(filter.getActive());
}

TEST_CASE("ADSBQualityFilter filters applicability", "[filter][adsbquality]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter");

    ADSBQualityFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT021"));
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK_FALSE(filter.filters("CAT048"));
    CHECK_FALSE(filter.filters("CAT062"));
    CHECK_FALSE(filter.filters("RefTraj"));
}

TEST_CASE("ADSBQualityFilter getConditionString", "[filter][adsbquality]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", true},
        {"use_v1", true},
        {"use_v2", true}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    SECTION("CAT021 generates mops_version IN clause")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT021", read_set, first);

        CHECK_FALSE(first);
        CHECK(sql.find("mops_version IN (0,1,2)") != std::string::npos);
    }

    SECTION("non-CAT021 returns empty")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK(sql.empty());
    }
}

TEST_CASE("ADSBQualityFilter inactive returns empty", "[filter][adsbquality]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", false},
        {"use_v0", true},
        {"use_v1", true},
        {"use_v2", true}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT021", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("ADSBQualityFilter with quality checks", "[filter][adsbquality]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", true},
        {"use_v1", true},
        {"use_v2", true},
        {"use_min_nucp", true},
        {"min_nucp", 4},
        {"use_min_nic", true},
        {"min_nic", 5}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT021", read_set, first);

    CHECK(sql.find("nucp_nic >= 4") != std::string::npos);
    CHECK(sql.find("nucp_nic >= 5") != std::string::npos);
}
