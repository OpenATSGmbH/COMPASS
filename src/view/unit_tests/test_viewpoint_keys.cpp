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

#include "viewpointkeys.h"
#include "viewpointgenerator.h"

#include "json.hpp"

// The ViewPointGenVP field names are literals (static strings in different files have
// no defined initialization order). They must stay equal to the ViewPointKeys keys.
TEST_CASE("ViewPointGenVP field names equal the ViewPointKeys keys", "[viewpoint]")
{
    REQUIRE(ViewPointGenVP::ViewPointFieldName        == ViewPointKeys::VP_NAME_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldID          == ViewPointKeys::VP_ID_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldType        == ViewPointKeys::VP_TYPE_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldStatus      == ViewPointKeys::VP_STATUS_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldAnnotations == ViewPointKeys::VP_ANNOTATION_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldPosLat      == ViewPointKeys::VP_POS_LAT_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldPosLon      == ViewPointKeys::VP_POS_LON_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldWinLat      == ViewPointKeys::VP_POS_WIN_LAT_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldWinLon      == ViewPointKeys::VP_POS_WIN_LON_KEY);
}

// Scan of a view point with one histogram annotation, as set by set_view_point
TEST_CASE("ViewPointGenVP scanForFeatures finds annotation features", "[viewpoint]")
{
    nlohmann::json vp = {
        { "id", 1 }, { "name", "vp" }, { "type", "Saved" }, { "status", "open" },
        { "annotations", nlohmann::json::array({
            { { "name", "Anno" }, { "hidden", false },
              { "features", nlohmann::json::array({ { { "type", "histogram" }, { "histogram", nlohmann::json::object() } } }) } } }) }
    };

    auto features = ViewPointGenVP::scanForFeatures(vp, { "histogram" });
    REQUIRE(features.size() == 1);

    REQUIRE(ViewPointGenVP::scanForFeatures(vp, { "grid" }).empty());
}

TEST_CASE("view point load_data decides whether a view point loads data", "[viewpoint]")
{
    nlohmann::json vp = { { "id", 1 }, { "type", "Saved" } };

    SECTION("missing key loads no data")
    {
        CHECK(!view_point_load::loadsData(vp));
        CHECK(view_point_load::loadData(vp) == nullptr);
    }

    SECTION("null loads no data")
    {
        vp[ "load_data" ] = nullptr;

        CHECK(!view_point_load::loadsData(vp));
    }

    SECTION("object loads data")
    {
        vp[ "load_data" ] = { { "data_source_types", "all" }, { "data_sources", "all" } };

        REQUIRE(view_point_load::loadsData(vp));
        CHECK(view_point_load::loadData(vp) == &vp.at("load_data"));
        CHECK(view_point_load::checkLoadData(vp.at("load_data")).empty());
    }

    SECTION("explicit lists are valid")
    {
        nlohmann::json load_data = { { "data_source_types", { "Radar" } },
                                     { "data_sources", nlohmann::json::array({ { 12750, { 0, 1 } } }) } };

        CHECK(view_point_load::checkLoadData(load_data).empty());

        load_data[ "data_sources" ] = { { "12750", { 0, 1 } } };
        CHECK(view_point_load::checkLoadData(load_data).empty());
    }

    SECTION("a missing required key is an error")
    {
        CHECK(!view_point_load::checkLoadData({ { "data_sources", "all" } }).empty());
        CHECK(!view_point_load::checkLoadData({ { "data_source_types", "all" } }).empty());
        CHECK(!view_point_load::checkLoadData(nlohmann::json::object()).empty());
    }

    SECTION("a value that is neither a list nor all is an error")
    {
        CHECK(!view_point_load::checkLoadData({ { "data_source_types", "some" }, { "data_sources", "all" } }).empty());
        CHECK(!view_point_load::checkLoadData({ { "data_source_types", "all" }, { "data_sources", 5 } }).empty());
    }
}

TEST_CASE("ViewPointGenVP writes load_data only if it loads data", "[viewpoint]")
{
    ViewPointGenVP vp("Figure", 0, "Histogram");

    SECTION("default loads no data")
    {
        nlohmann::json j;
        vp.toJSON(j);

        CHECK(!j.contains("load_data"));
        CHECK(!view_point_load::loadsData(j));
    }

    SECTION("data loading writes both required keys and the filters")
    {
        vp.enableDataLoading().filters().addFilter(std::make_unique<ViewPointGenFilterUTN>(4));

        nlohmann::json j;
        vp.toJSON(j);

        REQUIRE(view_point_load::loadsData(j));

        const auto& load_data = j.at("load_data");
        CHECK(view_point_load::checkLoadData(load_data).empty());
        CHECK(load_data.at("data_source_types") == "all");
        CHECK(load_data.at("data_sources") == "all");
        CHECK(load_data.at("filters").contains("UTNs"));
        CHECK(!j.contains("filters"));
    }
}
