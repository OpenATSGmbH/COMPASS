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
#include "trackertracknumberfilter.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variableset.h"

using namespace dbContent;

TEST_CASE("TrackerTrackNumberFilter construction", "[filter][trackertracknum]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0", {
        {"tracker_track_nums", nlohmann::json::object()}
    });

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    CHECK(filter.getName() == "Tracker Track Number");
    CHECK_FALSE(filter.getActive());
}

TEST_CASE("TrackerTrackNumberFilter filters applicability", "[filter][trackertracknum]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0");

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    CHECK(filter.filters("CAT062"));
    CHECK_FALSE(filter.filters("CAT001"));
    CHECK_FALSE(filter.filters("CAT048"));
    CHECK_FALSE(filter.filters("CAT021"));
    CHECK_FALSE(filter.filters("RefTraj"));
}

TEST_CASE("TrackerTrackNumberFilter getConditionString", "[filter][trackertracknum]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0", {
        {"active", true},
        {"tracker_track_nums", nlohmann::json::object()}
    });

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    // Push tracker data sources: ds_id 10, line 1 has 5 reports
    std::map<unsigned int, std::map<unsigned int, unsigned int>> tracker_lines;
    tracker_lines[10][1] = 5;
    std::map<unsigned int, std::string> ds_names;
    ds_names[10] = "ARTAS";
    filter.updateTrackerDataSources(tracker_lines, ds_names);

    // Set a track number for ds_id=10, line=1
    filter.setTrackerTrackNum(10, 1, "4227");

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT062", read_set, first);

    CHECK_FALSE(first);
    CHECK(sql.find("ds_id = 10") != std::string::npos);
    CHECK(sql.find("line_id = 1") != std::string::npos);
    CHECK(sql.find("track_num IN (4227)") != std::string::npos);
}

TEST_CASE("TrackerTrackNumberFilter non-CAT062 returns empty", "[filter][trackertracknum]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0", {
        {"active", true}
    });

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    VariableSet read_set;
    bool first = true;
    std::string sql = filter.getConditionString("CAT048", read_set, first);

    CHECK(sql.empty());
}

TEST_CASE("TrackerTrackNumberFilter pushed data sources", "[filter][trackertracknum]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0");

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    std::map<unsigned int, std::map<unsigned int, unsigned int>> tracker_lines;
    tracker_lines[10][1] = 5;
    tracker_lines[20][2] = 10;
    std::map<unsigned int, std::string> ds_names;
    ds_names[10] = "ARTAS";
    ds_names[20] = "Other";
    filter.updateTrackerDataSources(tracker_lines, ds_names);

    CHECK(filter.hasDataSourceName(10));
    CHECK(filter.dataSourceName(10) == "ARTAS");
    CHECK(filter.hasDataSourceName(20));
    CHECK(filter.dataSourceName(20) == "Other");
    CHECK_FALSE(filter.hasDataSourceName(99));
}

TEST_CASE("TrackerTrackNumberFilter viewpoint save/load round-trip", "[filter][trackertracknum][viewpoint]")
{
    auto mock = createStandardMock();
    auto cfg = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0", {
        {"active", true},
        {"tracker_track_nums", nlohmann::json::object()}
    });

    TrackerTrackNumberFilter filter(cfg, nullptr, mock);

    // Set up tracker data sources (required for save to include values)
    std::map<unsigned int, std::map<unsigned int, unsigned int>> tracker_lines;
    tracker_lines[10][0] = 5;
    tracker_lines[10][1] = 3;
    std::map<unsigned int, std::string> ds_names;
    ds_names[10] = "ARTAS";
    filter.updateTrackerDataSources(tracker_lines, ds_names);

    filter.setTrackerTrackNum(10, 0, "4171,4281");
    filter.setTrackerTrackNum(10, 1, "4197");

    // Save
    nlohmann::json vp_filters;
    filter.saveViewPointConditions(vp_filters);

    CHECK(vp_filters.contains("Tracker Track Number"));
    CHECK(vp_filters["Tracker Track Number"].contains("Values"));
    const auto& vals = vp_filters["Tracker Track Number"]["Values"];
    CHECK(vals.contains("10"));
    CHECK(vals["10"]["0"] == "4171,4281");
    CHECK(vals["10"]["1"] == "4197");

    // Load into fresh filter with same tracker data sources
    auto cfg2 = makeFilterConfig("TrackerTrackNumberFilter", "TrackerTrackNumberFilter0", {
        {"active", true},
        {"tracker_track_nums", nlohmann::json::object()}
    });

    TrackerTrackNumberFilter filter2(cfg2, nullptr, mock);
    filter2.updateTrackerDataSources(tracker_lines, ds_names);
    filter2.loadViewPointConditions(vp_filters);

    // Re-save and compare
    nlohmann::json vp_filters2;
    filter2.saveViewPointConditions(vp_filters2);
    CHECK(vp_filters == vp_filters2);
}
