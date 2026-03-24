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
#include "excludedtimewindowsfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"
#include "util/timeconv.h"

using namespace dbContent;

TEST_CASE("ExcludedTimeWindowsFilter construction", "[filter][excludedtimewindows]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"time_windows_json", nlohmann::json::array()}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Excluded Time Windows");
    CHECK_FALSE(filter.getActive());
}

TEST_CASE("ExcludedTimeWindowsFilter filters applicability", "[filter][excludedtimewindows]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"time_windows_json", nlohmann::json::array()}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT001"));
    CHECK(filter.filters("CAT048"));
    CHECK(filter.filters("CAT062"));
    CHECK(filter.filters("RefTraj"));
    CHECK_FALSE(filter.filters("CAT002"));
}

TEST_CASE("ExcludedTimeWindowsFilter getConditionString with windows", "[filter][excludedtimewindows]")
{
    // Create time windows JSON
    auto min_ts = Utils::Time::fromString("2026-01-05 10:00:00.000");
    auto max_ts = Utils::Time::fromString("2026-01-05 10:05:00.000");

    nlohmann::json windows = nlohmann::json::array();
    nlohmann::json window = nlohmann::json::array();
    window.push_back(Utils::Time::toString(min_ts));
    window.push_back(Utils::Time::toString(max_ts));
    windows.push_back(window);

    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"active", true},
        {"time_windows_json", windows}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("NOT (") != std::string::npos);
    CHECK(sql.find("timestamp BETWEEN") != std::string::npos);
}

TEST_CASE("ExcludedTimeWindowsFilter inactive returns empty", "[filter][excludedtimewindows]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"active", false},
        {"time_windows_json", nlohmann::json::array()}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("ExcludedTimeWindowsFilter empty windows returns empty", "[filter][excludedtimewindows]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"active", true},
        {"time_windows_json", nlohmann::json::array()}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("ExcludedTimeWindowsFilter updateMinMaxTimestamp", "[filter][excludedtimewindows]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("ExcludedTimeWindowsFilter", "ExcludedTimeWindowsFilter0", {
        {"time_windows_json", nlohmann::json::array()}
    });

    ExcludedTimeWindowsFilter filter(cfg, nullptr, mock);

    CHECK_FALSE(filter.hasMinMaxTimestamp());

    auto min_ts = Utils::Time::fromString("2026-01-05 08:00:00.000");
    auto max_ts = Utils::Time::fromString("2026-01-05 12:00:00.000");
    filter.updateMinMaxTimestamp(min_ts, max_ts);

    CHECK(filter.hasMinMaxTimestamp());
    auto [got_min, got_max] = filter.minMaxTimestamp();
    CHECK(got_min == min_ts);
    CHECK(got_max == max_ts);
}
