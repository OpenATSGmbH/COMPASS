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

#include "viewpoint.h"
#include "viewpointgenerator.h"

#include "json.hpp"

// The ViewPointGenVP field names are literals (static strings in different files have
// no defined initialization order). They must stay equal to the ViewPoint keys.
TEST_CASE("ViewPointGenVP field names equal the ViewPoint keys", "[viewpoint]")
{
    REQUIRE(ViewPointGenVP::ViewPointFieldName        == ViewPoint::VP_NAME_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldID          == ViewPoint::VP_ID_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldType        == ViewPoint::VP_TYPE_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldStatus      == ViewPoint::VP_STATUS_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldAnnotations == ViewPoint::VP_ANNOTATION_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldFilters     == ViewPoint::VP_FILTERS_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldPosLat      == ViewPoint::VP_POS_LAT_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldPosLon      == ViewPoint::VP_POS_LON_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldWinLat      == ViewPoint::VP_POS_WIN_LAT_KEY);
    REQUIRE(ViewPointGenVP::ViewPointFieldWinLon      == ViewPoint::VP_POS_WIN_LON_KEY);
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
