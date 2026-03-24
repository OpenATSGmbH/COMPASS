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
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("DBFilter Position with 4 conditions", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "Position", {
        {"active", true},
        {"is_custom", false},
        {"name", "Position"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Latitude Maximum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", "<="}, {"reset_value", "MAX"},
                {"value", "45.958201"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Latitude Minimum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", ">="}, {"reset_value", "MIN"},
                {"value", "45.524856"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Longitude Maximum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", "<="}, {"reset_value", "MAX"},
                {"value", "16.295223"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Longitude"}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Longitude Minimum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", ">="}, {"reset_value", "MIN"},
                {"value", "15.843169"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Longitude"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    CHECK(filter.getNumConditions() == 4);
    CHECK(filter.getName() == "Position");

    SECTION("generates SQL with lat/lon conditions")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK(sql.find("latitude") != std::string::npos);
        CHECK(sql.find("<=45.958201") != std::string::npos);
        CHECK(sql.find(">=45.524856") != std::string::npos);
        CHECK(sql.find("longitude") != std::string::npos);
        CHECK(sql.find("<=16.295223") != std::string::npos);
        CHECK(sql.find(">=15.843169") != std::string::npos);
    }

    SECTION("filters returns true for dbcontent with lat/lon")
    {
        CHECK(filter.filters("CAT048"));
        CHECK(filter.filters("CAT062"));
        CHECK(filter.filters("RefTraj"));
    }
}

TEST_CASE("DBFilter Ground Bit condition", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "DBFilter1", {
        {"active", true},
        {"is_custom", true},
        {"name", "Ground Bit"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Ground BitCondition0"},
            {"parameters", {
                {"absolute_value", false},
                {"display_instance_id", false},
                {"display_instance_name", false},
                {"op_and", true},
                {"operator", "="},
                {"reset_value", "1"},
                {"value", "1"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Ground Bit"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    CHECK(filter.getNumConditions() == 1);
    CHECK(filter.getName() == "Ground Bit");

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT062", read_set, first);

    CHECK(sql.find("ground_bit") != std::string::npos);
    CHECK(sql.find("=1") != std::string::npos);
}

TEST_CASE("DBFilter ADSBMOPS condition with IN operator", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "ADSBMOPS", {
        {"active", true},
        {"is_custom", false},
        {"name", "ADSBMOPS"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "ADSBMOPSCondition0"},
            {"parameters", {
                {"absolute_value", false},
                {"display_instance_id", false},
                {"display_instance_name", false},
                {"op_and", true},
                {"operator", "IN"},
                {"reset_value", "0"},
                {"value", "0"},
                {"variable_dbcontent_name", "CAT021"},
                {"variable_name", "MOPS Version"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    CHECK(filter.getNumConditions() == 1);

    SECTION("only filters CAT021")
    {
        CHECK(filter.filters("CAT021"));
        CHECK_FALSE(filter.filters("CAT048"));
        CHECK_FALSE(filter.filters("CAT062"));
    }

    SECTION("generates IN clause")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT021", read_set, first);

        CHECK(sql.find("mops_version") != std::string::npos);
        CHECK(sql.find("IN") != std::string::npos);
        CHECK(sql.find("(0)") != std::string::npos);
    }
}

TEST_CASE("DBFilter Detection Type condition", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "Detection Type", {
        {"active", true},
        {"is_custom", false},
        {"name", "Detection Type"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Radar Detection TypeCondition0"},
            {"parameters", {
                {"absolute_value", false},
                {"display_instance_id", false},
                {"display_instance_name", false},
                {"op_and", true},
                {"operator", "IN"},
                {"reset_value", "1"},
                {"value", "1"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Type"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    CHECK(filter.getNumConditions() == 1);

    // Detection Type meta var exists in CAT001, CAT048, CAT062
    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK_FALSE(filter.filters("CAT010"));

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.find("detection_type") != std::string::npos);
    CHECK(sql.find("IN") != std::string::npos);
}

TEST_CASE("DBFilter inactive returns empty condition", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "Position", {
        {"active", false},
        {"name", "Position"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Latitude Maximum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", "<="}, {"reset_value", "MAX"},
                {"value", "45.958201"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
    CHECK(first);
}

TEST_CASE("DBFilter viewpoint save/load round-trip", "[filter][condition]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "Position", {
        {"active", true},
        {"name", "Position"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Latitude Maximum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", "<="}, {"reset_value", "MAX"},
                {"value", "45.958201"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("Position"));
    CHECK(vp_filters["Position"].contains("Latitude Maximum"));

    // Load into fresh filter — must use a separate config copy since
    // the first filter's constructor consumed cfg
    auto cfg2 = makeFilterConfig("DBFilter", "Position", {
        {"active", true},
        {"name", "Position"}
    });

    cfg2["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "Latitude Maximum"},
            {"parameters", {
                {"absolute_value", false}, {"op_and", true},
                {"operator", "<="}, {"reset_value", "MAX"},
                {"value", "45.958201"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"}
            }}
        }
    });

    DBFilter filter2(cfg2, true, nullptr, mock);

    filter2.loadViewPointConditions(vp_filters);

    // Verify same SQL output
    VariableSet read_set1, read_set2;
    bool first1 = true, first2 = true;
    std::string sql1 = filter.getConditionString("CAT048", read_set1, first1);
    std::string sql2 = filter2.getConditionString("CAT048", read_set2, first2);

    CHECK(sql1 == sql2);
}
