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

#include "gridbackground_defs.h"

#include <QColor>
#include <QImage>

#include <cmath>
#include <set>

using namespace gridbackground;

TEST_CASE("GridBackground map files", "[gridbackground]")
{
    SECTION("eligible content")
    {
        const std::string xyz =
            "<!-- comment -->\n"
            "<map name=\"OpenStreetMap\" type=\"projected\">\n"
            "  <options><profile>spherical-mercator</profile></options>\n"
            "  <XYZImage name=\"osm\"><url>http://x/{z}/{x}/{y}.png</url></XYZImage>\n"
            "</map>\n";

        REQUIRE(isEligibleMapFile(xyz));

        const std::string mbtiles =
            "<map type=\"projected\"><MBTilesImage name=\"local\"><filename>a.mbtiles</filename></MBTilesImage></map>";

        REQUIRE(isEligibleMapFile(mbtiles));
    }

    SECTION("not eligible content")
    {
        //geocentric
        REQUIRE_FALSE(isEligibleMapFile("<map type=\"geocentric\"><XYZImage name=\"a\"/></map>"));
        REQUIRE_FALSE(isEligibleMapFile("<map><XYZImage name=\"a\"/></map>"));

        //shader
        REQUIRE_FALSE(isEligibleMapFile("<map type=\"projected\"><XYZImage name=\"a\"><shader>x</shader></XYZImage></map>"));

        //no image layer
        REQUIRE_FALSE(isEligibleMapFile("<map type=\"projected\"><OGRFeatures name=\"b\"/></map>"));

        //not a map
        REQUIRE_FALSE(isEligibleMapFile("<earth type=\"projected\"><XYZImage name=\"a\"/></earth>"));
        REQUIRE_FALSE(isEligibleMapFile("<map type=\"projected\"><XYZImage"));
    }

    SECTION("map files of the repository")
    {
        auto files = eligibleMapFiles(mapDirectory());

        std::set<std::string> titles;
        for (const auto& f : files)
            titles.insert(f.title);

        REQUIRE(titles.count("openstreetmap_2d") == 1);
        REQUIRE(titles.count("openstreetmap_german_2d") == 1);
        REQUIRE(titles.count("arcgis_2d") == 1);

        REQUIRE(titles.count("openstreetmap_dark_2d") == 0); // shader
        REQUIRE(titles.count("minimal_2d") == 0);            // no image layer
        REQUIRE(titles.count("openstreetmap") == 0);         // geocentric

        REQUIRE(mapTitle("openstreetmap_2d.earth") == "openstreetmap_2d");
    }
}

TEST_CASE("GridBackground mercator", "[gridbackground]")
{
    using namespace mercator;

    SECTION("tile coordinates")
    {
        //Vienna at zoom 8 => tile 139 / 88
        REQUIRE((int)std::floor(lonToTileX(16.37, 8)) == 139);
        REQUIRE((int)std::floor(latToTileY(48.2, 8)) == 88);

        REQUIRE(lonToTileX(-180.0, 3) == Approx(0.0));
        REQUIRE(lonToTileX( 180.0, 3) == Approx(8.0));
        REQUIRE(latToTileY(0.0, 3) == Approx(4.0));

        for (double lat : { -80.0, -45.0, 0.0, 12.5, 48.2, 80.0 })
            REQUIRE(tileYToLat(latToTileY(lat, 10), 10) == Approx(lat).margin(1e-9));

        for (double lon : { -179.0, -10.0, 0.0, 16.37, 179.0 })
            REQUIRE(tileXToLon(lonToTileX(lon, 10), 10) == Approx(lon).margin(1e-9));
    }

    SECTION("zoom for resolution")
    {
        //zoom 0: 360 / 256 degrees per pixel
        REQUIRE(zoomForResolution(360.0 / 256.0) == 0);
        REQUIRE(zoomForResolution(360.0 / 256.0 / 2.0) == 1);
        REQUIRE(zoomForResolution(360.0 / 256.0 / 3.0) == 2); // finer tiles than needed
        REQUIRE(zoomForResolution(1e-9) == MaxTileZoom);
        REQUIRE(zoomForResolution(0.0) == MaxTileZoom);
        REQUIRE(zoomForResolution(1000.0) == 0);
    }

    SECTION("tile range")
    {
        auto range = tileRange(QRectF(16.0, 48.0, 1.0, 0.5), 8);

        REQUIRE(range.valid());
        REQUIRE(range.zoom == 8);
        REQUIRE(range.x0 == (int)std::floor(lonToTileX(16.0, 8)));
        REQUIRE(range.x1 == (int)std::floor(lonToTileX(17.0, 8)));
        REQUIRE(range.y0 == (int)std::floor(latToTileY(48.5, 8)));
        REQUIRE(range.y1 == (int)std::floor(latToTileY(48.0, 8)));

        //too many tiles => lower zoom
        auto range_world = tileRange(QRectF(-180.0, -80.0, 360.0, 160.0), 10, 64);

        REQUIRE(range_world.valid());
        REQUIRE(range_world.numTiles() <= 64);
        REQUIRE(range_world.zoom < 10);

        //outside of the mercator area
        REQUIRE_FALSE(tileRange(QRectF(200.0, 10.0, 10.0, 10.0), 5).valid());
        REQUIRE_FALSE(tileRange(QRectF(10.0, 86.0, 10.0, 3.0), 5).valid());
    }

    SECTION("remap to lon/lat")
    {
        //zoom 1, whole world, each mosaic row encodes its row number
        TileRange range;
        range.zoom = 1;
        range.x0   = 0;
        range.x1   = 1;
        range.y0   = 0;
        range.y1   = 1;

        const int ts = TileSize;

        QImage mosaic(2 * ts, 2 * ts, QImage::Format_ARGB32);
        for (int y = 0; y < mosaic.height(); ++y)
            for (int x = 0; x < mosaic.width(); ++x)
                mosaic.setPixel(x, y, qRgba(y % 256, y / 256, x / 256, 255));

        const QRectF lonlat(-180.0, -80.0, 360.0, 160.0);
        const QSize  size(360, 160);

        QImage out = remapToLonLat(mosaic, range, ts, lonlat, size);

        REQUIRE(out.size() == size);

        for (int j : { 0, 30, 80, 120, 159 })
        {
            const double lat      = lonlat.bottom() - (j + 0.5) / size.height() * lonlat.height();
            const int    expected = (int)std::floor(latToTileY(lat, 1) * ts);

            QRgb p = out.pixel(100, j);

            REQUIRE(qAlpha(p) == 255);
            REQUIRE(qRed(p) + 256 * qGreen(p) == expected);
        }

        //longitude: west half from the first tile column, east half from the second
        REQUIRE(qBlue(out.pixel(10, 80)) == 0);
        REQUIRE(qBlue(out.pixel(350, 80)) == 1);

        //outside of the mercator latitude range => transparent
        QImage polar = remapToLonLat(mosaic, range, ts, QRectF(0.0, 80.0, 10.0, 10.0), QSize(10, 100));
        REQUIRE(qAlpha(polar.pixel(5, 0)) == 0);   // 89.95 deg
        REQUIRE(qAlpha(polar.pixel(5, 99)) == 255); // 80.05 deg
    }
}

TEST_CASE("GridBackground viewport", "[gridbackground]")
{
    using namespace viewport;

    SECTION("aspect ratio")
    {
        //at 60 deg one degree of longitude is half a degree of latitude
        QRectF r = adjustAspect(QRectF(10.0, 59.0, 2.0, 2.0), QSizeF(100.0, 100.0));

        REQUIRE(r.center().x() == Approx(11.0));
        REQUIRE(r.center().y() == Approx(60.0));
        REQUIRE(r.width() * std::cos(60.0 * M_PI / 180.0) / r.height() == Approx(1.0));
        REQUIRE(r.width() == Approx(4.0));
        REQUIRE(r.height() == Approx(2.0));

        //wide plot at the equator
        QRectF e = adjustAspect(QRectF(0.0, -1.0, 2.0, 2.0), QSizeF(200.0, 100.0));
        REQUIRE(e.width() / e.height() == Approx(2.0));
    }

    SECTION("clamp")
    {
        const QRectF max_extent(0.0, 0.0, 10.0, 10.0);

        //too large => scaled down and moved inside
        QRectF r = clamp(QRectF(-20.0, -20.0, 40.0, 40.0), max_extent, 0.1, 0.1);
        REQUIRE(r.width() == Approx(10.0));
        REQUIRE(r.height() == Approx(10.0));
        REQUIRE(r.left() == Approx(0.0));
        REQUIRE(r.top() == Approx(0.0));

        //outside => moved inside
        QRectF m = clamp(QRectF(8.0, 8.0, 4.0, 4.0), max_extent, 0.1, 0.1);
        REQUIRE(m.width() == Approx(4.0));
        REQUIRE(m.right() == Approx(10.0));
        REQUIRE(m.bottom() == Approx(10.0));

        //too small => scaled up around the center, aspect kept
        QRectF s = clamp(QRectF(5.0, 5.0, 0.01, 0.02), max_extent, 0.1, 0.1);
        REQUIRE(s.width() == Approx(0.1));
        REQUIRE(s.height() == Approx(0.2));
        REQUIRE(s.center().x() == Approx(5.005));
        REQUIRE(s.center().y() == Approx(5.01));
    }

    SECTION("limits")
    {
        //margin and aspect
        QRectF e = maxExtent(QRectF(0.0, -1.0, 10.0, 2.0), QSizeF(100.0, 100.0), 0.1);
        REQUIRE(e.width() == Approx(12.0));
        REQUIRE(e.height() == Approx(12.0).epsilon(1e-3)); // aspect at the equator
        REQUIRE(e.center().x() == Approx(5.0));

        //quarter cell above the tile limit
        REQUIRE(minWidth(1.0, 1000.0) == Approx(0.25));

        //tile limit for tiny cells
        const double tile_limit = 1000.0 * 360.0 / (TileSize * std::ldexp(1.0, MaxTileZoom));
        REQUIRE(minWidth(1e-9, 1000.0) == Approx(tile_limit));
    }
}

TEST_CASE("GridBackground national borders", "[gridbackground]")
{
    auto lines = readVectorFileLines(mapDirectory() + "shapefiles/TM_WORLD_BORDERS-0.3.shp");

    REQUIRE(lines.size() > 200);

    bool vienna_inside = false;

    for (const auto& l : lines)
    {
        REQUIRE(l.points.size() >= 2);
        //right() / bottom() are left + width / top + height => rounding
        const double eps = 1e-9;

        REQUIRE(l.bounds.left() >= -180.0 - eps);
        REQUIRE(l.bounds.right() <= 180.0 + eps);
        REQUIRE(l.bounds.top() >= -90.0 - eps);
        REQUIRE(l.bounds.bottom() <= 90.0 + eps);

        if (l.points.containsPoint(QPointF(16.37, 48.2), Qt::OddEvenFill))
            vienna_inside = true;
    }

    REQUIRE(vienna_inside);
}
