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
#include "grid2dlayerrenderer.h"
#include "grid2drendersettings.h"

#include <QImage>

#include <algorithm>

namespace
{

/// 4 rows x 5 columns, cell size 0.1 deg, region lon 10.0 - 10.5, lat 49.6 - 50.0, all cells null.
Grid2DLayer makeLayer(bool north_up)
{
    Grid2DLayer layer;
    layer.name = "test";
    layer.data = Eigen::MatrixXd::Constant(4, 5, Grid2D::InvalidValue);

    layer.ref.srs              = "wgs84";
    layer.ref.img_origin_x     = 10.0;
    layer.ref.img_origin_y     = north_up ? 50.0 : 49.6;
    layer.ref.img_pixel_size_x = 0.1;
    layer.ref.img_pixel_size_y = 0.1;
    layer.ref.is_north_up      = north_up;

    return layer;
}

void checkROI(const QRectF& roi, double lat_min, double lat_max, double lon_min, double lon_max)
{
    REQUIRE(!roi.isEmpty());
    REQUIRE(roi.left()   == Approx(lat_min));
    REQUIRE(roi.right()  == Approx(lat_max));
    REQUIRE(roi.top()    == Approx(lon_min));
    REQUIRE(roi.bottom() == Approx(lon_max));
}

/// Region of the opaque pixels of the rendered image (x = latitude, y = longitude), as the producers
/// computed it before the grid feature.
QRectF renderedROI(const Grid2DLayer& layer, int pixels_per_cell)
{
    Grid2DRenderSettings rs;
    rs.min_value       = 0.0;
    rs.max_value       = 1.0;
    rs.pixels_per_cell = pixels_per_cell;

    auto rendered = Grid2DLayerRenderer::render(layer, rs);

    const QImage&          img = rendered.first;
    const RasterReference& ref = rendered.second;

    int x_min = img.width(),  x_max = -1;
    int y_min = img.height(), y_max = -1;

    for (int y = 0; y < img.height(); ++y)
    {
        for (int x = 0; x < img.width(); ++x)
        {
            if (qAlpha(img.pixel(x, y)) == 0)
                continue;

            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }
    }

    if (x_max < x_min || y_max < y_min)
        return QRectF();

    const double lon_min = ref.img_origin_x +  x_min      * ref.img_pixel_size_x;
    const double lon_max = ref.img_origin_x + (x_max + 1) * ref.img_pixel_size_x;

    double lat_min, lat_max;
    if (ref.is_north_up)
    {
        lat_max = ref.img_origin_y -  y_min      * ref.img_pixel_size_y;
        lat_min = ref.img_origin_y - (y_max + 1) * ref.img_pixel_size_y;
    }
    else
    {
        lat_min = ref.img_origin_y +  y_min      * ref.img_pixel_size_y;
        lat_max = ref.img_origin_y + (y_max + 1) * ref.img_pixel_size_y;
    }

    return QRectF(lat_min, lon_min, lat_max - lat_min, lon_max - lon_min);
}

} // namespace

TEST_CASE("Grid2DLayer region of the cells with values", "[grid2d]")
{
    for (bool north_up : { true, false })
    {
        DYNAMIC_SECTION("north up " << north_up)
        {
            auto layer = makeLayer(north_up);

            SECTION("all cells set")
            {
                layer.data.setConstant(0.5);

                auto roi = layer.geoROIOfValidCells();
                checkROI(roi, 49.6, 50.0, 10.0, 10.5);

                // same as the full region of the raster reference (x = lon, y = lat there)
                auto full = layer.ref.getROI((int)layer.data.cols(), (int)layer.data.rows());
                REQUIRE(roi.left()   == Approx(full.top()));
                REQUIRE(roi.right()  == Approx(full.bottom()));
                REQUIRE(roi.top()    == Approx(full.left()));
                REQUIRE(roi.bottom() == Approx(full.right()));
            }

            SECTION("only some cells set, null cells excluded")
            {
                // data rows 0 and 1, columns 1 and 3
                layer.data(0, 1) = 0.2;
                layer.data(1, 3) = 0.8;

                auto roi = layer.geoROIOfValidCells();

                // north up: data row 0 is the southern image row, so rows 0 - 1 are lat 49.6 - 49.8
                // not north up: data row 0 is at the origin (lat 49.6), so also lat 49.6 - 49.8
                checkROI(roi, 49.6, 49.8, 10.1, 10.4);
            }

            SECTION("single cell")
            {
                layer.data(3, 4) = 0.0;
                checkROI(layer.geoROIOfValidCells(), 49.9, 50.0, 10.4, 10.5);
            }

            SECTION("same region as the rendered image")
            {
                layer.data(0, 1) = 0.2;
                layer.data(2, 2) = 0.4;
                layer.data(1, 3) = 0.8;

                auto roi = layer.geoROIOfValidCells();

                for (int ppc : { 1, 3 })
                {
                    auto roi_img = renderedROI(layer, ppc);
                    REQUIRE(roi.left()   == Approx(roi_img.left()));
                    REQUIRE(roi.right()  == Approx(roi_img.right()));
                    REQUIRE(roi.top()    == Approx(roi_img.top()));
                    REQUIRE(roi.bottom() == Approx(roi_img.bottom()));
                }
            }

            SECTION("all cells null gives an empty region")
            {
                REQUIRE(layer.geoROIOfValidCells().isEmpty());
            }
        }
    }

    SECTION("empty grid gives an empty region")
    {
        Grid2DLayer layer;
        REQUIRE(layer.geoROIOfValidCells().isEmpty());
    }
}
