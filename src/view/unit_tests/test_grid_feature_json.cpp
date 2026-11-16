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
#include "viewpointgenerator.h"
#include "plotmetadata.h"

#include "json.hpp"

#include <QByteArray>

namespace
{

Grid2DLayer makeLayer()
{
    Grid2DLayer layer;
    layer.name = "pd";
    layer.data.resize(2, 3);
    layer.data << 0.25, Grid2D::InvalidValue, 0.5,
                  1.0,  0.75,                 Grid2D::InvalidValue;

    layer.ref.srs              = "wgs84";
    layer.ref.img_origin_x     = 16.0;
    layer.ref.img_origin_y     = 48.5;
    layer.ref.img_pixel_size_x = 0.01;
    layer.ref.img_pixel_size_y = 0.02;
    layer.ref.is_north_up      = true;

    return layer;
}

} // namespace

TEST_CASE("ViewPointGenFeatureGrid JSON", "[grid2d]")
{
    auto layer = makeLayer();

    Grid2DRenderSettings rs;
    rs.color_map.create(QColor(Qt::red), QColor(Qt::green), 2, ColorMap::Type::Binary);
    rs.min_value        = 0.0;
    rs.max_value        = 1.0;
    rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
    rs.legend_precision = 2;
    rs.legend_unit      = "%";

    PlotMetadata meta("Sensor Coverage", "PD - Horizontal", "Longitude (deg)", "Latitude (deg)");

    ViewPointGenFeatureGrid feature(layer, rs, meta);
    feature.writeBinaryIfPossible(true); // ignored for grids: always json text

    nlohmann::json j;
    feature.toJSON(j);

    REQUIRE(j.at(ViewPointGenFeature::FeatureTypeFieldType) == ViewPointGenFeatureGrid::FeatureName);
    REQUIRE(j.contains(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid));
    REQUIRE(j.contains(ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings));

    const auto& jgrid = j.at(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid);

    SECTION("grid values as json text, no binary data")
    {
        REQUIRE(jgrid.contains(Grid2DLayer::TagData));
        REQUIRE(!jgrid.contains(Grid2DLayer::TagDataRaw));
    }

    SECTION("grid round trip: values incl. null cells, reference")
    {
        Grid2DLayer layer2;
        REQUIRE(layer2.fromJSON(jgrid));

        REQUIRE(layer2.name == layer.name);
        REQUIRE(layer2.data.rows() == layer.data.rows());
        REQUIRE(layer2.data.cols() == layer.data.cols());
        REQUIRE(layer2.data == layer.data);

        REQUIRE(layer2.ref.srs              == layer.ref.srs);
        REQUIRE(layer2.ref.img_origin_x     == Approx(layer.ref.img_origin_x));
        REQUIRE(layer2.ref.img_origin_y     == Approx(layer.ref.img_origin_y));
        REQUIRE(layer2.ref.img_pixel_size_x == Approx(layer.ref.img_pixel_size_x));
        REQUIRE(layer2.ref.img_pixel_size_y == Approx(layer.ref.img_pixel_size_y));
        REQUIRE(layer2.ref.is_north_up      == layer.ref.is_north_up);

        REQUIRE(layer2.geoROIOfValidCells() == layer.geoROIOfValidCells());
    }

    SECTION("render settings round trip")
    {
        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(j.at(ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings)));

        REQUIRE(rs2.color_map.type() == ColorMap::Type::Binary);
        REQUIRE(rs2.color_map.getColors() == rs.color_map.getColors());
        REQUIRE(*rs2.min_value == Approx(0.0));
        REQUIRE(*rs2.max_value == Approx(1.0));
        REQUIRE(rs2.legend_order == Grid2DRenderSettings::LegendOrder::Reversed);
        REQUIRE(*rs2.legend_precision == 2);
        REQUIRE(rs2.legend_unit == "%");
    }

    SECTION("axis labels")
    {
        REQUIRE(j.contains(ViewPointGenFeature::FeatureFieldNamePlotMetadata));

        PlotMetadata meta2;
        REQUIRE(meta2.fromJSON(j.at(ViewPointGenFeature::FeatureFieldNamePlotMetadata)));
        REQUIRE(meta2.xAxisLabel() == "Longitude (deg)");
        REQUIRE(meta2.yAxisLabel() == "Latitude (deg)");
    }

    SECTION("binary grid data of stored results is still read")
    {
        nlohmann::json jold = jgrid;
        jold.erase(Grid2DLayer::TagData);

        QByteArray ba((const char*)layer.data.data(), (int)(layer.data.size() * sizeof(double)));
        jold[ Grid2DLayer::TagDataRaw ] = QString(ba.toBase64()).toStdString();

        Grid2DLayer layer2;
        REQUIRE(layer2.fromJSON(jold));
        REQUIRE(layer2.data == layer.data);
    }
}
