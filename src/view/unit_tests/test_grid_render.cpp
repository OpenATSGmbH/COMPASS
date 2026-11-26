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

#include "grid2d.h"
#include "grid2dlayer.h"
#include "grid2drendersettings.h"
#include "grid2dlayerrenderer.h"

#include <QImage>
#include <QColor>

namespace
{

/// 2 rows x 2 columns: row 0 = (0.0, 1.0), row 1 = (null, 1.0)
Grid2DLayer makeLayer(bool north_up)
{
    Grid2DLayer layer;
    layer.name = "test";
    layer.data.resize(2, 2);
    layer.data << 0.0,                  1.0,
                  Grid2D::InvalidValue, 1.0;

    layer.ref.srs              = "wgs84";
    layer.ref.img_origin_x     = 10.0;
    layer.ref.img_origin_y     = north_up ? 50.0 : 49.8;
    layer.ref.img_pixel_size_x = 0.1;
    layer.ref.img_pixel_size_y = 0.1;
    layer.ref.is_north_up      = north_up;

    return layer;
}

Grid2DRenderSettings binarySettings(const QColor& c0, const QColor& c1)
{
    Grid2DRenderSettings rs;
    rs.color_map.create(c0, c1, 2, ColorMap::Type::Binary);
    rs.min_value = 0.0;
    rs.max_value = 1.0;
    return rs;
}

} // namespace

TEST_CASE("Grid2DLayerRenderer colors", "[grid2d]")
{
    auto rs = binarySettings(QColor(Qt::red), QColor(Qt::green));

    const QColor c0 = rs.color_map.getColor(0);
    const QColor c1 = rs.color_map.getColor(1);

    SECTION("pixel color at each cell, null cell transparent (not north up)")
    {
        auto img = Grid2DLayerRenderer::render(makeLayer(false), rs).first;

        REQUIRE(img.width()  == 2);
        REQUIRE(img.height() == 2);

        // not north up: image row = data row
        REQUIRE(img.pixelColor(0, 0) == c0);
        REQUIRE(img.pixelColor(1, 0) == c1);
        REQUIRE(img.pixelColor(0, 1).alpha() == 0);
        REQUIRE(img.pixelColor(1, 1) == c1);
    }

    SECTION("north up flips the rows")
    {
        auto img = Grid2DLayerRenderer::render(makeLayer(true), rs).first;

        REQUIRE(img.pixelColor(0, 1) == c0);
        REQUIRE(img.pixelColor(1, 1) == c1);
        REQUIRE(img.pixelColor(0, 0).alpha() == 0);
        REQUIRE(img.pixelColor(1, 0) == c1);
    }

    SECTION("other color map gives other colors")
    {
        auto rs2 = binarySettings(QColor(Qt::blue), QColor(Qt::yellow));

        auto img  = Grid2DLayerRenderer::render(makeLayer(false), rs).first;
        auto img2 = Grid2DLayerRenderer::render(makeLayer(false), rs2).first;

        REQUIRE(img2.pixelColor(0, 0) == rs2.color_map.getColor(0));
        REQUIRE(img2.pixelColor(1, 0) == rs2.color_map.getColor(1));
        REQUIRE(img2.pixelColor(0, 0) != img.pixelColor(0, 0));
        REQUIRE(img2.pixelColor(1, 0) != img.pixelColor(1, 0));
        REQUIRE(img2.pixelColor(0, 1).alpha() == 0);
    }

    SECTION("same colors after a JSON round trip of the render settings")
    {
        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(rs.toJSON()));

        auto img  = Grid2DLayerRenderer::render(makeLayer(false), rs).first;
        auto img2 = Grid2DLayerRenderer::render(makeLayer(false), rs2).first;

        REQUIRE(img2 == img);
    }

    SECTION("pixels per cell")
    {
        rs.pixels_per_cell = 3;

        auto rendered = Grid2DLayerRenderer::render(makeLayer(false), rs);

        REQUIRE(rendered.first.width()  == 6);
        REQUIRE(rendered.first.height() == 6);
        REQUIRE(rendered.first.pixelColor(5, 0) == c1);
        REQUIRE(rendered.second.img_pixel_size_x == Approx(0.1 / 3.0));
    }
}
