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

#include <algorithm>

using namespace dbContent;

namespace
{
    // a Report Variable is named by its report, "<report name>: <display name>", and joined onto
    // the host data content (here CAT048) through the Report Table join alias
    const std::string ReportName   = "Radar Test Evaluation";
    const std::string ReportVar    = "Radar Test Evaluation: Range Offset";
    const std::string JoinAlias    = "rt_1";
    const std::string ReportColumn = "range_offset";

    MockVariableResolver createReportMock()
    {
        auto mock = createStandardMock();
        mock.addReportVariable(ReportName, ReportVar, { "CAT048" }, JoinAlias, ReportColumn,
                               PropertyDataType::DOUBLE);
        return mock;
    }

    nlohmann::json reportFilterConfig(const std::string& name, const std::string& report_name,
                                      const std::string& op, const std::string& value, bool absolute)
    {
        auto cfg = makeFilterConfig("DBFilter", name, {
            {"active", true},
            {"is_custom", true},
            {"name", name}
        });

        cfg["sub_configs"] = nlohmann::json::array({
            {
                {"class_name", "DBFilterCondition"},
                {"instance_name", name + "Condition0"},
                {"parameters", {
                    {"absolute_value", absolute},
                    {"operator", op},
                    {"reset_value", value},
                    {"value", value},
                    {"variable_dbcontent_name", report_name},
                    {"variable_name", ReportVar}
                }}
            }
        });

        return cfg;
    }
}

TEST_CASE("DBFilter Report Variable condition", "[filter][condition][report]")
{
    auto mock = createReportMock();
    auto cfg  = reportFilterConfig("RV Filter", ReportName, ">", "1.9025", false);

    DBFilter filter(cfg, true, nullptr, mock);

    REQUIRE(filter.getNumConditions() == 1);
    CHECK_FALSE(filter.getConditions().at(0)->valueInvalid());

    SECTION("filters only the host data content")
    {
        CHECK(filter.filters("CAT048"));
        CHECK_FALSE(filter.filters("CAT062"));
        CHECK_FALSE(filter.filters("CAT021"));
    }

    SECTION("generates SQL on the joined Report Table column")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT048", read_set, first);

        CHECK(sql.find(JoinAlias + "." + ReportColumn + " >1.9025") != std::string::npos);
        CHECK_FALSE(first);
    }

    SECTION("no SQL for another data content")
    {
        VariableSet read_set;
        bool first = true;
        std::string sql = filter.getConditionString("CAT062", read_set, first);

        CHECK(sql.empty());
        CHECK(first);
    }
}

TEST_CASE("DBFilter Report Variable condition with ABS", "[filter][condition][report]")
{
    auto mock = createReportMock();
    auto cfg  = reportFilterConfig("RV ABS Filter", ReportName, ">", "5.0604", true);

    DBFilter filter(cfg, true, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.find("ABS(" + JoinAlias + "." + ReportColumn + ") >5.0604") != std::string::npos);
}

TEST_CASE("DBFilter Report Variable condition value set through a view point", "[filter][condition][report]")
{
    // the value check of a condition used the report name as a data content and asserted,
    // e.g. when a view point set the value of a filter with a Report Variable condition
    auto mock = createReportMock();
    auto cfg  = reportFilterConfig("RV Filter", ReportName, ">", "1.9025", false);

    DBFilter filter(cfg, true, nullptr, mock);

    nlohmann::json vp_filters;
    vp_filters[ "RV Filter" ][ "RV FilterCondition0" ] = "3.5";

    filter.loadViewPointConditions(vp_filters);

    auto* cond = filter.getConditions().at(0).get();

    CHECK_FALSE(cond->valueInvalid());
    CHECK(cond->getValue() == "3.5");

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.find(JoinAlias + "." + ReportColumn + " >3.5") != std::string::npos);
}

TEST_CASE("DBFilterCondition sqlFor requires the Report Variable for the join", "[filter][condition][report]")
{
    auto mock = createReportMock();

    SECTION("host data content")
    {
        FilterClause clause = DBFilterCondition::sqlFor(mock, "CAT048", ReportVar, ReportName, ">", "1.5");

        CHECK(clause.sql == JoinAlias + "." + ReportColumn + " >1.5");

        // the Report Table join is generated from the read set, so the variable is required
        const auto& adds = mock.readSetAdds();
        CHECK(std::find(adds.begin(), adds.end(), "CAT048|" + ReportName + "|" + ReportVar) != adds.end());
    }

    SECTION("other data content")
    {
        FilterClause clause = DBFilterCondition::sqlFor(mock, "CAT062", ReportVar, ReportName, ">", "1.5");

        CHECK(clause.empty());
        CHECK(mock.readSetAdds().empty());
    }
}

TEST_CASE("DBFilter Report Variable condition of a missing report", "[filter][condition][report]")
{
    // e.g. a filter from the configuration whose report is not in the opened database
    auto mock = createReportMock();
    auto cfg  = reportFilterConfig("RV Missing Filter", "Missing Report Evaluation", ">", "1.0", false);

    DBFilter filter(cfg, true, nullptr, mock);

    REQUIRE(filter.getNumConditions() == 1);
    auto* cond = filter.getConditions().at(0).get();

    CHECK_FALSE(filter.filters("CAT048"));

    // setting a value checks it, an unavailable variable makes it invalid instead of asserting
    cond->setValue("2.0");
    CHECK(cond->valueInvalid());

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}
