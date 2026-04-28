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

TEST_CASE("ADSBQualityFilter all version flags off does not emit empty IN()", "[filter][adsbquality][empty]")
{
    // All three version flags off, all min/max checks off, but filter is
    // active. Must NOT emit "mops_version IN ()" (invalid SQL) nor a
    // dangling " AND ".
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", false}, {"use_v1", false}, {"use_v2", false},
        {"use_min_nucp", false}, {"use_max_nucp", false},
        {"use_min_nic", false}, {"use_max_nic", false},
        {"use_min_nacp", false}, {"use_max_nacp", false},
        {"use_min_sil_v1", false}, {"use_max_sil_v1", false},
        {"use_min_sil_v2", false}, {"use_max_sil_v2", false}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT021", read_set, first);

    CHECK(sql.empty());
    CHECK(first);   // outer query must be unaffected
    CHECK(sql.find("IN ()") == std::string::npos);
    CHECK(sql.find("IN()") == std::string::npos);
}

TEST_CASE("ADSBQualityFilter all versions off but min check on", "[filter][adsbquality][empty]")
{
    // No version flags but a min/max check is enabled. Must skip the
    // version IN clause entirely and emit only the min/max clause —
    // without a leading dangling " AND".
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", false}, {"use_v1", false}, {"use_v2", false},
        {"use_min_nucp", true}, {"min_nucp", 4}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT021", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("IN ()") == std::string::npos);
    CHECK(sql.find("IN()") == std::string::npos);
    CHECK(sql.find("nucp_nic >= 4") != std::string::npos);
    // SQL must not start with " AND" (would be left over from missing version IN clause).
    REQUIRE(sql.size() > 4);
    CHECK(sql.substr(0, 5) != " AND ");
}

TEST_CASE("ADSBQualityFilter viewpoint save/load round-trip", "[filter][adsbquality][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", false}, {"use_v1", true}, {"use_v2", true},
        {"use_min_nucp", true}, {"min_nucp", 4},
        {"use_max_nucp", false}, {"max_nucp", 4},
        {"use_min_nic", true}, {"min_nic", 5},
        {"use_max_nic", false}, {"max_nic", 5},
        {"use_min_nacp", true}, {"min_nacp", 5},
        {"use_max_nacp", false}, {"max_nacp", 5},
        {"use_min_sil_v1", false}, {"min_sil_v1", 0},
        {"use_max_sil_v1", false}, {"max_sil_v1", 0},
        {"use_min_sil_v2", true}, {"min_sil_v2", 2},
        {"use_max_sil_v2", true}, {"max_sil_v2", 3}
    });

    ADSBQualityFilter filter(cfg, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("ADSB Quality"));
    const auto& f = vp_filters["ADSB Quality"];
    CHECK(f["use_v0"] == false);
    CHECK(f["use_v1"] == true);
    CHECK(f["use_v2"] == true);
    CHECK(f["use_min_nucp"] == true);
    CHECK(f["min_nucp"] == 4);
    CHECK(f["use_min_sil_v2"] == true);
    CHECK(f["min_sil_v2"] == 2);
    CHECK(f["max_sil_v2"] == 3);

    // Load into fresh filter with different values
    auto cfg2 = makeFilterConfig("ADSBQualityFilter", "ADSBQualityFilter", {
        {"active", true},
        {"use_v0", true}, {"use_v1", false}, {"use_v2", false},
        {"use_min_nucp", false}, {"min_nucp", 0},
        {"use_max_nucp", false}, {"max_nucp", 0},
        {"use_min_nic", false}, {"min_nic", 0},
        {"use_max_nic", false}, {"max_nic", 0},
        {"use_min_nacp", false}, {"min_nacp", 0},
        {"use_max_nacp", false}, {"max_nacp", 0},
        {"use_min_sil_v1", false}, {"min_sil_v1", 0},
        {"use_max_sil_v1", false}, {"max_sil_v1", 0},
        {"use_min_sil_v2", false}, {"min_sil_v2", 0},
        {"use_max_sil_v2", false}, {"max_sil_v2", 0}
    });

    ADSBQualityFilter filter2(cfg2, nullptr, mock);
    filter2.loadViewPointConditions(vp_filters);

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}
