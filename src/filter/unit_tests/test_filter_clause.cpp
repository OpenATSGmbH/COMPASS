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
#include "filterclause.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "timestampfilter.h"
#include "mode3afilter.h"
#include "utnfilter.h"
#include "acadfilter.h"
#include "acidfilter.h"
#include "modecfilter.h"
#include "reftrajaccuracyfilter.h"
#include "adsbqualityfilter.h"
#include "mlatrufilter.h"
#include "primaryonlyfilter.h"
#include "excludedtimewindowsfilter.h"
#include "trackertracknumberfilter.h"
#include "dbcontent/variable/variableset.h"
#include "util/timeconv.h"

#include <string>
#include <vector>

using namespace dbContent;

namespace
{
// getClause renders a clean fragment; the legacy path prepends a join space when joining -
// strip leading whitespace before comparing.
std::string ltrim(const std::string& s)
{
    size_t i = s.find_first_not_of(' ');
    return i == std::string::npos ? std::string() : s.substr(i);
}
}

// Step-7 migration step 1: DBFilterCondition::getClause() must render the same SQL as the
// legacy getConditionString() path (with first=true, i.e. no leading AND/OR join). The leaf
// primitive is the real dedup point, so this locks its output to the current behavior before
// anything switches onto it. (required_vars parity is exercised by the real resolver in
// integration; the unit mock's addVariableToReadSet is a no-op.)
TEST_CASE("FilterClause: getClause SQL matches legacy getConditionString", "[filter][clause]")
{
    auto mock = createStandardMock();

    struct Case
    {
        std::string    label;
        std::string    dbcontent;
        nlohmann::json params;
    };

    std::vector<Case> cases = {
        {"le",        "CAT048", {{"operator", "<="}, {"value", "45.958201"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Latitude"}}},
        {"ge",        "CAT048", {{"operator", ">="}, {"value", "15.843169"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Longitude"}}},
        {"eq",        "CAT062", {{"operator", "="}, {"value", "1"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Ground Bit"}}},
        {"in",        "CAT048", {{"operator", "IN"}, {"value", "0"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Track Number"}}},
        {"in_null",   "CAT048", {{"operator", "IN"}, {"value", "0,NULL"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Track Number"}}},
        {"only_null", "CAT048", {{"operator", "IN"}, {"value", "NULL"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Track Number"}}},
        {"eq_null",   "CAT048", {{"operator", "="}, {"value", "NULL"},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Track Number"}}},
        {"abs",       "CAT048", {{"operator", "<="}, {"value", "10"}, {"absolute_value", true},
                                 {"variable_dbcontent_name", "Meta"}, {"variable_name", "Latitude"}}},
    };

    for (const auto& c : cases)
    {
        auto cfg = makeFilterConfig("DBFilter", "ClauseTest", {
            {"active", true}, {"is_custom", true}, {"name", "ClauseTest"}
        });
        cfg["sub_configs"] = nlohmann::json::array({
            {
                {"class_name", "DBFilterCondition"},
                {"instance_name", "Cond0"},
                {"parameters", c.params}
            }
        });

        DBFilter filter(cfg, true, nullptr, mock);
        REQUIRE(filter.getNumConditions() == 1);

        auto* cond = filter.getConditions().at(0).get();

        VariableSet read_set;
        bool first = true;
        std::string legacy = cond->getConditionString(c.dbcontent, read_set, first, "AND");

        FilterClause clause = cond->getClause(c.dbcontent);

        INFO("case '" << c.label << "' legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK(clause.sql == legacy);
    }
}

// The static leaf renderer with EXPLICIT (variable, operator, value) params must produce the
// same SQL as the config-driven condition - this is what a consumer building a generic
// condition (e.g. a position bbox) relies on.
TEST_CASE("FilterClause: DBFilterCondition::sqlFor(explicit) matches config-driven getClause", "[filter][clause]")
{
    auto mock = createStandardMock();

    struct Case
    {
        std::string variable;
        std::string variable_dbcontent;
        std::string op;
        std::string value;
        std::string dbcontent;
    };

    std::vector<Case> cases = {
        {"Latitude",     "Meta", "<=", "45.958201", "CAT048"},
        {"Longitude",    "Meta", ">=", "15.843169", "CAT048"},
        {"Ground Bit",   "Meta", "=",  "1",         "CAT062"},
        {"Track Number", "Meta", "IN", "0,NULL",    "CAT048"},
    };

    for (const auto& c : cases)
    {
        // config-driven: a one-condition DBFilter with this variable/op/value
        auto cfg = makeFilterConfig("DBFilter", "SqlForTest", {
            {"active", true}, {"is_custom", true}, {"name", "SqlForTest"}
        });
        cfg["sub_configs"] = nlohmann::json::array({
            { {"class_name", "DBFilterCondition"}, {"instance_name", "C0"},
              {"parameters", {{"operator", c.op}, {"value", c.value},
                              {"variable_dbcontent_name", c.variable_dbcontent},
                              {"variable_name", c.variable}}} }
        });
        DBFilter filter(cfg, true, nullptr, mock);
        REQUIRE(filter.getNumConditions() == 1);

        FilterClause config_clause = filter.getConditions().at(0)->getClause(c.dbcontent);

        // explicit params via the static leaf (what a generic-statement consumer calls)
        FilterClause explicit_clause = DBFilterCondition::sqlFor(
            mock, c.dbcontent, c.variable, c.variable_dbcontent, c.op, c.value);

        INFO("var '" << c.variable << "' op '" << c.op << "' value '" << c.value
             << "' config='" << config_clause.sql << "' explicit='" << explicit_clause.sql << "'");
        CHECK_FALSE(explicit_clause.sql.empty());
        CHECK(explicit_clause.sql == config_clause.sql);
    }
}

// Specialized filters: DBFilter::getClause overrides must render the same SQL as their
// getConditionString overrides (with first=true), modulo the legacy leading join space.
TEST_CASE("FilterClause: specialized filter getClause matches getConditionString", "[filter][clause]")
{
    auto mock = createStandardMock();

    SECTION("TimestampFilter")
    {
        auto cfg = makeFilterConfig("TimestampFilter", "T", {
            {"active", true},
            {"min_value", "2026-01-05 09:59:58.752"},
            {"max_value", "2026-01-05 12:05:19.968"}
        });
        TimestampFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("Mode3AFilter values + null")
    {
        auto cfg = makeFilterConfig("Mode3AFilter", "M", {
            {"active", true},
            {"values_str", "3771,NULL"}
        });
        Mode3AFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("UTNFilter associated")
    {
        auto cfg = makeFilterConfig("UTNFilter", "U", {
            {"active", true},
            {"utns_str", "0"}
        });
        UTNFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("UTNFilter non-associated -> false")
    {
        MockVariableResolver mock2;
        mock2.addMetaVariable(dbcontent_vars::meta_var_timestamp_, {{"NoUTN", "timestamp"}});

        auto cfg = makeFilterConfig("UTNFilter", "U", {
            {"active", true},
            {"utns_str", "0"}
        });
        UTNFilter filter(cfg, nullptr, mock2);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("NoUTN", rs, first);
        FilterClause clause = filter.getClause("NoUTN");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK(clause.sql == "false");
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("ACADFilter values")
    {
        auto cfg = makeFilterConfig("ACADFilter", "A", {
            {"active", true},
            {"values_str", "4C8070"}
        });
        ACADFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("ModeCFilter range (CAT062 extras)")
    {
        auto cfg = makeFilterConfig("ModeCFilter", "MC", {
            {"active", true},
            {"min_value", -1000.0},
            {"max_value", 3000.0},
            {"null_wanted", true}
        });
        ModeCFilter filter(cfg, nullptr, mock);

        for (const char* dbc : {"CAT048", "CAT062"})
        {
            VariableSet rs; bool first = true;
            std::string legacy = filter.getConditionString(dbc, rs, first);
            FilterClause clause = filter.getClause(dbc);

            INFO("dbc " << dbc << " legacy='" << legacy << "' clause='" << clause.sql << "'");
            CHECK_FALSE(clause.sql.empty());
            CHECK(clause.sql == ltrim(legacy));
        }
    }

    SECTION("ACIDFilter LIKE list")
    {
        auto cfg = makeFilterConfig("ACIDFilter", "ACID", {
            {"active", true},
            {"values_str", "AEE"}
        });
        ACIDFilter filter(cfg, nullptr, mock);

        for (const char* dbc : {"CAT048", "CAT062"})
        {
            VariableSet rs; bool first = true;
            std::string legacy = filter.getConditionString(dbc, rs, first);
            FilterClause clause = filter.getClause(dbc);

            INFO("dbc " << dbc << " legacy='" << legacy << "' clause='" << clause.sql << "'");
            CHECK_FALSE(clause.sql.empty());
            CHECK(clause.sql == ltrim(legacy));
        }
    }

    SECTION("RefTrajAccuracyFilter")
    {
        auto cfg = makeFilterConfig("RefTrajAccuracyFilter", "R", {
            {"active", true},
            {"min_value", 30.0}
        });
        RefTrajAccuracyFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("RefTraj", rs, first);
        FilterClause clause = filter.getClause("RefTraj");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("ADSBQualityFilter (delegating)")
    {
        auto cfg = makeFilterConfig("ADSBQualityFilter", "Q", {
            {"active", true},
            {"use_v0", true}, {"use_v1", true}, {"use_v2", true}
        });
        ADSBQualityFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT021", rs, first);
        FilterClause clause = filter.getClause("CAT021");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("MLATRUFilter (delegating)")
    {
        auto cfg = makeFilterConfig("MLATRUFilter", "M", {
            {"active", true},
            {"rus_str", "kor"},
            {"match_all", false}
        });
        MLATRUFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT020", rs, first);
        FilterClause clause = filter.getClause("CAT020");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("PrimaryOnlyFilter (IS NULL chain + detection type)")
    {
        auto cfg = makeFilterConfig("PrimaryOnlyFilter", "P", {{"active", true}});
        PrimaryOnlyFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("ExcludedTimeWindowsFilter (NOT BETWEEN)")
    {
        auto min_ts = Utils::Time::fromString("2026-01-05 10:00:00.000");
        auto max_ts = Utils::Time::fromString("2026-01-05 10:05:00.000");

        nlohmann::json windows = nlohmann::json::array();
        nlohmann::json window  = nlohmann::json::array();
        window.push_back(Utils::Time::toString(min_ts));
        window.push_back(Utils::Time::toString(max_ts));
        windows.push_back(window);

        auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "E", {
            {"active", true},
            {"time_windows_json", windows}
        });
        ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

        VariableSet rs; bool first = true;
        std::string legacy = filter.getConditionString("CAT048", rs, first);
        FilterClause clause = filter.getClause("CAT048");

        INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
        CHECK_FALSE(clause.sql.empty());
        CHECK(clause.sql == ltrim(legacy));
    }

    SECTION("TrackerTrackNumberFilter (structural: empty + non-CAT062)")
    {
        // getActiveTrackerTrackNums() depends on data-source state, so a populated set isn't
        // reachable in the mock; the populated inner-loop is a verbatim copy of
        // getConditionString. Validate the structural paths match here.
        auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TT", {
            {"active", true},
            {"tracker_track_nums", nlohmann::json::object()}
        });
        TrackerTrackNumberFilter filter(cfg, nullptr, mock);

        for (const char* dbc : {"CAT062", "CAT048"}) // CAT062 empty-set, CAT048 early-return
        {
            VariableSet rs; bool first = true;
            std::string legacy = filter.getConditionString(dbc, rs, first);
            FilterClause clause = filter.getClause(dbc);

            INFO("dbc " << dbc << " legacy='" << legacy << "' clause='" << clause.sql << "'");
            CHECK(clause.sql == ltrim(legacy));
        }
    }
}

// FilterManager::viewClause combines active filters' getClause with AND. Validate that
// combining equals the legacy getSQLCondition threading (shared `first` across filters).
// (A full FilterManager isn't constructible in the unit harness; this exercises the same
// per-filter loop + combine logic viewClause runs.)
TEST_CASE("FilterClause: combineAnd over filters matches getSQLCondition threading", "[filter][clause]")
{
    auto mock = createStandardMock();

    auto ts_cfg = makeFilterConfig("TimestampFilter", "T", {
        {"active", true},
        {"min_value", "2026-01-05 09:59:58.752"},
        {"max_value", "2026-01-05 12:05:19.968"}
    });
    TimestampFilter ts(ts_cfg, nullptr, mock);

    auto m3a_cfg = makeFilterConfig("Mode3AFilter", "M", {{"active", true}, {"values_str", "3771"}});
    Mode3AFilter m3a(m3a_cfg, nullptr, mock);

    auto pos_cfg = makeFilterConfig("DBFilter", "Position", {
        {"active", true}, {"is_custom", false}, {"name", "Position"}
    });
    pos_cfg["sub_configs"] = nlohmann::json::array({
        { {"class_name", "DBFilterCondition"}, {"instance_name", "Lat Max"},
          {"parameters", {{"operator", "<="}, {"value", "45.9"},
                          {"variable_dbcontent_name", "Meta"}, {"variable_name", "Latitude"}}} }
    });
    DBFilter pos(pos_cfg, true, nullptr, mock);

    std::vector<DBFilter*> filters { &ts, &m3a, &pos };
    const std::string dbc = "CAT048";

    // legacy: getSQLCondition-style accumulation with a shared first flag
    VariableSet rs; bool first = true;
    std::string legacy;
    for (auto* f : filters)
        if (f->getActive() && f->filters(dbc))
            legacy += f->getConditionString(dbc, rs, first);

    // new: combineAnd over per-filter getClause (what viewClause does)
    std::vector<FilterClause> parts;
    for (auto* f : filters)
        if (f->getActive() && f->filters(dbc))
            parts.push_back(f->getClause(dbc));
    FilterClause combined = combineAnd(parts);

    INFO("legacy='" << legacy << "' combined='" << combined.sql << "'");
    CHECK_FALSE(combined.sql.empty());
    CHECK(combined.sql == ltrim(legacy));
}

// Base DBFilter condition-list path: getClause must match getConditionString for AND-joined
// multi-condition filters (e.g. Position).
TEST_CASE("FilterClause: condition-list DBFilter getClause matches getConditionString", "[filter][clause]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("DBFilter", "Position", {
        {"active", true}, {"is_custom", false}, {"name", "Position"}
    });
    cfg["sub_configs"] = nlohmann::json::array({
        { {"class_name", "DBFilterCondition"}, {"instance_name", "Lat Max"},
          {"parameters", {{"operator", "<="}, {"value", "45.958201"},
                          {"variable_dbcontent_name", "Meta"}, {"variable_name", "Latitude"}}} },
        { {"class_name", "DBFilterCondition"}, {"instance_name", "Lat Min"},
          {"parameters", {{"operator", ">="}, {"value", "45.524856"},
                          {"variable_dbcontent_name", "Meta"}, {"variable_name", "Latitude"}}} },
        { {"class_name", "DBFilterCondition"}, {"instance_name", "Lon Max"},
          {"parameters", {{"operator", "<="}, {"value", "16.295223"},
                          {"variable_dbcontent_name", "Meta"}, {"variable_name", "Longitude"}}} }
    });

    DBFilter filter(cfg, true, nullptr, mock);

    VariableSet rs; bool first = true;
    std::string legacy = filter.getConditionString("CAT048", rs, first);
    FilterClause clause = filter.getClause("CAT048");

    INFO("legacy='" << legacy << "' clause='" << clause.sql << "'");
    CHECK_FALSE(clause.sql.empty());
    CHECK(clause.sql == ltrim(legacy));
}
