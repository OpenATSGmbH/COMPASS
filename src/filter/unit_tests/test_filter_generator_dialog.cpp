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
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

// Tests for the filter edit round-trip:
// Build a DBFilter with conditions from JSON config, then verify that all condition
// properties are accessible via getConditions() — the same path that
// FilterGeneratorDialog::loadConditionsFromFilter() uses.

TEST_CASE("Filter edit round-trip: single condition", "[filter][dialog]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "TestCustom", {
        {"active", true},
        {"is_custom", true},
        {"name", "TestCustom"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "TestCustomCondition0"},
            {"parameters", {
                {"absolute_value", false},
                {"operator", "="},
                {"value", "1"},
                {"reset_value", "1"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Ground Bit"},
                {"include_null", false}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    REQUIRE(filter.getNumConditions() == 1);

    // Verify condition properties — same fields loadConditionsFromFilter() reads
    const auto& cond = filter.getConditions().at(0);
    CHECK(cond->getVariableName() == "Ground Bit");
    CHECK(cond->getVariableDBContentName() == "Meta");
    CHECK(cond->getOperator() == "=");
    CHECK(cond->getValue() == "1");
    CHECK(cond->getResetValue() == "1");
    CHECK(cond->getAbsoluteValue() == false);
    CHECK(cond->getIncludeNull() == false);
}

TEST_CASE("Filter edit round-trip: multiple conditions preserved", "[filter][dialog]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "MultiCond", {
        {"active", true},
        {"is_custom", true},
        {"name", "MultiCond"},
        {"condition_logic", "OR"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "MultiCondCondition0"},
            {"parameters", {
                {"absolute_value", true},
                {"operator", ">"},
                {"value", "5000"},
                {"reset_value", "5000"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Mode C Code"},
                {"include_null", true}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "MultiCondCondition1"},
            {"parameters", {
                {"absolute_value", false},
                {"operator", "IN"},
                {"value", "7000,7777"},
                {"reset_value", "7000,7777"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Mode 3/A Code"},
                {"include_null", false}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "MultiCondCondition2"},
            {"parameters", {
                {"absolute_value", false},
                {"operator", ">="},
                {"value", "45.0"},
                {"reset_value", "45.0"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Latitude"},
                {"include_null", false}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    REQUIRE(filter.getNumConditions() == 3);
    CHECK(filter.conditionLogic() == "OR");

    // Condition 0: ABS(Mode C Code) > 5000 [+NULL]
    {
        const auto& c = filter.getConditions().at(0);
        CHECK(c->getVariableName() == "Mode C Code");
        CHECK(c->getVariableDBContentName() == "Meta");
        CHECK(c->getOperator() == ">");
        CHECK(c->getValue() == "5000");
        CHECK(c->getAbsoluteValue() == true);
        CHECK(c->getIncludeNull() == true);
    }

    // Condition 1: Mode 3/A Code IN 7000,7777
    {
        const auto& c = filter.getConditions().at(1);
        CHECK(c->getVariableName() == "Mode 3/A Code");
        CHECK(c->getOperator() == "IN");
        CHECK(c->getValue() == "7000,7777");
        CHECK(c->getAbsoluteValue() == false);
        CHECK(c->getIncludeNull() == false);
    }

    // Condition 2: Latitude >= 45.0
    {
        const auto& c = filter.getConditions().at(2);
        CHECK(c->getVariableName() == "Latitude");
        CHECK(c->getOperator() == ">=");
        CHECK(c->getValue() == "45.0");
    }
}

TEST_CASE("Filter edit round-trip: clearConditions and rebuild", "[filter][dialog]")
{
    // Simulates what FilterGeneratorDialog::accept() does in edit mode:
    // clearConditions(), removeSubConfigurations(), then rebuild from templates.
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "EditTest", {
        {"active", true},
        {"is_custom", true},
        {"name", "EditTest"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "EditTestCondition0"},
            {"parameters", {
                {"operator", "="},
                {"value", "1"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Ground Bit"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);
    REQUIRE(filter.getNumConditions() == 1);

    // Simulate edit: clear and rebuild with different conditions
    filter.clearConditions();
    CHECK(filter.getNumConditions() == 0);

    // Add two new conditions (simulating what accept() does)
    {
        auto& cond_json = filter.addNewSubConfiguration("DBFilterCondition", "EditTestCondition0");
        cond_json[Configuration::ParameterSection]["operator"] = ">=";
        cond_json[Configuration::ParameterSection]["value"] = "45.0";
        cond_json[Configuration::ParameterSection]["variable_dbcontent_name"] = "Meta";
        cond_json[Configuration::ParameterSection]["variable_name"] = "Latitude";
        cond_json[Configuration::ParameterSection]["absolute_value"] = false;
        cond_json[Configuration::ParameterSection]["include_null"] = false;
        cond_json[Configuration::ParameterSection]["reset_value"] = "45.0";
        filter.generateSubConfigurable(cond_json);
    }
    {
        auto& cond_json = filter.addNewSubConfiguration("DBFilterCondition", "EditTestCondition1");
        cond_json[Configuration::ParameterSection]["operator"] = "<=";
        cond_json[Configuration::ParameterSection]["value"] = "50.0";
        cond_json[Configuration::ParameterSection]["variable_dbcontent_name"] = "Meta";
        cond_json[Configuration::ParameterSection]["variable_name"] = "Latitude";
        cond_json[Configuration::ParameterSection]["absolute_value"] = false;
        cond_json[Configuration::ParameterSection]["include_null"] = false;
        cond_json[Configuration::ParameterSection]["reset_value"] = "50.0";
        filter.generateSubConfigurable(cond_json);
    }

    REQUIRE(filter.getNumConditions() == 2);

    // Verify new conditions
    CHECK(filter.getConditions().at(0)->getOperator() == ">=");
    CHECK(filter.getConditions().at(0)->getValue() == "45.0");
    CHECK(filter.getConditions().at(1)->getOperator() == "<=");
    CHECK(filter.getConditions().at(1)->getValue() == "50.0");

    // Verify SQL works with new conditions
    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);
    CHECK(sql.find("latitude") != std::string::npos);
    CHECK(sql.find(">=45.0") != std::string::npos);
    CHECK(sql.find("<=50.0") != std::string::npos);
}

TEST_CASE("Filter edit round-trip: condition logic change", "[filter][dialog]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "LogicTest", {
        {"active", true},
        {"is_custom", true},
        {"name", "LogicTest"},
        {"condition_logic", "AND"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "LogicTestCondition0"},
            {"parameters", {
                {"operator", ">"}, {"value", "100"},
                {"variable_dbcontent_name", "Meta"}, {"variable_name", "Mode C Code"}
            }}
        },
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "LogicTestCondition1"},
            {"parameters", {
                {"operator", "<"}, {"value", "500"},
                {"variable_dbcontent_name", "Meta"}, {"variable_name", "Mode C Code"}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);
    CHECK(filter.conditionLogic() == "AND");

    // Simulate edit mode changing logic to OR
    filter.conditionLogic("OR");
    CHECK(filter.conditionLogic() == "OR");

    // SQL should use OR instead of AND
    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);
    CHECK(sql.find("OR") != std::string::npos);
}

TEST_CASE("Filter edit round-trip: include_null preserved", "[filter][dialog]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "NullTest", {
        {"active", true},
        {"is_custom", true},
        {"name", "NullTest"}
    });

    cfg["sub_configs"] = nlohmann::json::array({
        {
            {"class_name", "DBFilterCondition"},
            {"instance_name", "NullTestCondition0"},
            {"parameters", {
                {"operator", "="},
                {"value", "1"},
                {"variable_dbcontent_name", "Meta"},
                {"variable_name", "Ground Bit"},
                {"include_null", true}
            }}
        }
    });

    DBFilter filter(cfg, true, nullptr, mock);
    REQUIRE(filter.getNumConditions() == 1);

    const auto& cond = filter.getConditions().at(0);
    CHECK(cond->getIncludeNull() == true);

    // SQL should contain IS NULL clause
    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT062", read_set, first);
    CHECK(sql.find("IS NULL") != std::string::npos);
}
