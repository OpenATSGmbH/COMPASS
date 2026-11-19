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
#include "viewpointgenerator.h"
#include "rasterreference.h"

#include "json.hpp"

#include <QRectF>

TEST_CASE("GeoImage feature from a file without a reference leaves the reference to the file", "[viewpoint][geoimage]")
{
    ViewPointGenFeatureGeoImage feature("/data/world.tif", RasterReference());

    nlohmann::json j;
    feature.toJSON(j);

    REQUIRE(j.contains("source"));
    REQUIRE(j.at("source").value("fn", "") == "/data/world.tif");
    REQUIRE(!j.contains("reference"));
}

TEST_CASE("GeoImage feature with a reference writes it", "[viewpoint][geoimage]")
{
    RasterReference ref;
    ref.set("wgs84", QRectF(10.0, 40.0, 2.0, 1.0), 200, 100, true);

    ViewPointGenFeatureGeoImage feature("/data/world.tif", ref);

    nlohmann::json j;
    feature.toJSON(j);

    REQUIRE(j.contains("reference"));
    REQUIRE(j.at("reference").at("srs") == "wgs84");
}
