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

#include "gridviewmodel.h"
#include "grid2d.h"
#include "viewpointgenerator.h"
#include "plotmetadata.h"

#include "json.hpp"

#include <algorithm>

namespace
{
    /// 2 rows, 3 columns, one cell without value. srs "": a grid over other axes (e.g. altitude)
    Grid2DLayer makeLayer(const std::string& srs)
    {
        Grid2DLayer layer;
        layer.name = "pd";
        layer.data.resize(2, 3);
        layer.data << 0.25, Grid2D::InvalidValue, 0.5,
                      1.0,  0.75,                 0.0;

        layer.ref.srs              = srs;
        layer.ref.img_origin_x     = 16.0;
        layer.ref.img_origin_y     = 3000.0;
        layer.ref.img_pixel_size_x = 0.1;
        layer.ref.img_pixel_size_y = 1000.0;
        layer.ref.is_north_up      = true;

        return layer;
    }

    Grid2DRenderSettings makeRenderSettings()
    {
        Grid2DRenderSettings rs;
        rs.color_map.create(colorscale::ColorScale::Red2Green, 3, ColorMap::Type::LinearSamples, std::make_pair(0.0, 1.0));
        rs.min_value        = 0.0;
        rs.max_value        = 1.0;
        rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
        rs.legend_precision = 2;

        return rs;
    }

    nlohmann::json feature(const Grid2DLayer& layer, const boost::optional<Grid2DRenderSettings>& rs)
    {
        ViewPointGenFeatureGrid f(layer, rs);

        nlohmann::json j;
        f.toJSON(j);

        return j;
    }

    const PlotMetadata Metadata("PD - Altitude/Longitude", "", "Longitude (deg)", "Altitude (ft)");
}

TEST_CASE("grid model display data from an annotation", "[view][gridmodel]")
{
    SECTION("a grid over other axes with render settings")
    {
        const auto layer = makeLayer("");

        //no model object and no manager
        auto data = GridViewModel::displayDataFromAnnotation(feature(layer, makeRenderSettings()), Metadata);
        REQUIRE(data);

        CHECK(data->valid);
        CHECK(!data->is_geographic);

        CHECK(data->title        == "PD - Altitude/Longitude");
        CHECK(data->x_axis_label == "Longitude (deg)");
        CHECK(data->y_axis_label == "Altitude (ft)");

        CHECK(data->legend_data_type == PropertyDataType::DOUBLE);
        CHECK(data->legend_repr      == dbContent::Representation::STANDARD);
        CHECK(data->axis_data_type[ 0 ] == PropertyDataType::DOUBLE);
        CHECK(data->axis_data_type[ 1 ] == PropertyDataType::DOUBLE);

        REQUIRE(data->layer.has_value());
        CHECK(data->layer->data == layer.data);
        CHECK(data->layer->ref.srs.empty());
        CHECK(data->layer->ref.img_origin_y == Approx(3000.0));

        REQUIRE(data->render_settings.has_value());
        CHECK(data->render_settings->color_map.colorScale() == colorscale::ColorScale::Red2Green);
        CHECK(data->render_settings->color_map.numColors() == 3);
        CHECK(*data->render_settings->min_value == Approx(0.0));
        CHECK(*data->render_settings->max_value == Approx(1.0));
        CHECK(data->render_settings->legend_order == Grid2DRenderSettings::LegendOrder::Reversed);
        CHECK(*data->render_settings->legend_precision == 2);

        //the database extent is for the empty chart of the variables
        CHECK(data->database_extent.isEmpty());
        CHECK(data->sectors.empty());
        CHECK(data->labels.empty());
        CHECK(!data->background.has_value());
        CHECK(!data->domain_id.empty());
    }

    SECTION("a longitude / latitude grid is geographic and gets the sector lines")
    {
        GridSectorLine line;
        line.layer_name  = "Layer A";
        line.sector_name = "Sector 1";
        line.sector_id   = 1;

        auto data = GridViewModel::displayDataFromAnnotation(feature(makeLayer("wgs84"), {}), Metadata, { line });
        REQUIRE(data);

        CHECK(data->valid);
        CHECK(data->is_geographic);

        REQUIRE(data->sectors.size() == 1);
        CHECK(data->sectors[ 0 ].sector_name == "Sector 1");
    }

    SECTION("without render settings the render options apply")
    {
        auto data = GridViewModel::displayDataFromAnnotation(feature(makeLayer(""), {}), Metadata);
        REQUIRE(data);

        CHECK(!data->render_settings.has_value());
    }

    SECTION("malformed annotations give no display data")
    {
        std::string error;

        CHECK(!GridViewModel::displayDataFromAnnotation(nlohmann::json::array(), Metadata, {}, {}, boost::none, &error));
        CHECK(error == "missing 'grid' field");

        auto no_grid = feature(makeLayer(""), {});
        no_grid.erase(ViewPointGenFeatureGrid::FeatureGridFieldNameGrid);

        CHECK(!GridViewModel::displayDataFromAnnotation(no_grid, Metadata, {}, {}, boost::none, &error));
        CHECK(error == "missing 'grid' field");

        auto bad_grid = feature(makeLayer(""), {});
        bad_grid[ ViewPointGenFeatureGrid::FeatureGridFieldNameGrid ] = nlohmann::json::object();

        CHECK(!GridViewModel::displayDataFromAnnotation(bad_grid, Metadata, {}, {}, boost::none, &error));
        CHECK(error == "could not read grid layer");

        Grid2DLayer empty_layer = makeLayer("");
        empty_layer.data.resize(0, 0);

        CHECK(!GridViewModel::displayDataFromAnnotation(feature(empty_layer, {}), Metadata, {}, {}, boost::none, &error));
        CHECK(error == "grid layer empty");
    }

    SECTION("unreadable render settings are ignored")
    {
        auto j = feature(makeLayer(""), {});
        j[ ViewPointGenFeatureGrid::FeatureGridFieldNameRenderSettings ] = "not an object";

        auto data = GridViewModel::displayDataFromAnnotation(j, Metadata);
        REQUIRE(data);

        CHECK(!data->render_settings.has_value());
    }
}

namespace
{
    /// view point with a geographic grid annotation and a text annotation with two labels
    nlohmann::json labelViewPoint()
    {
        ViewPointGenVP vp("labels", 0, "Grid");

        vp.annotations().getOrCreateAnnotation("grid")->addFeature(new ViewPointGenFeatureGrid(makeLayer("wgs84")));

        auto text_anno = vp.annotations().getOrCreateAnnotation("RU position");

        auto ru = new ViewPointGenFeatureText("RU 1", 48.5, 16.25, 12.0f);
        ru->setColor(QColor("#00FFFF"));
        text_anno->addFeature(ru);

        //a text in a child annotation is a label as well
        auto child = text_anno->addAnnotation("child");
        child->addFeature(new ViewPointGenFeatureText("RU 2", 48.0, 16.5, 8.0f,
                                                      ViewPointGenFeatureText::TextDirection::LeftDown));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }
}

TEST_CASE("grid model labels and background of the view point", "[view][gridmodel]")
{
    SECTION("text features of the view point give the labels")
    {
        auto labels = GridViewModel::textLabels(labelViewPoint());
        REQUIRE(labels.size() == 2);

        std::sort(labels.begin(), labels.end(),
                  [] (const GridTextLabel& a, const GridTextLabel& b) { return a.text < b.text; });

        //longitude is x, latitude is y
        CHECK(labels[ 0 ].text == "RU 1");
        CHECK(labels[ 0 ].position.x() == Approx(16.25));
        CHECK(labels[ 0 ].position.y() == Approx(48.5));
        CHECK(labels[ 0 ].color.rgb() == QColor("#00FFFF").rgb());
        CHECK(labels[ 0 ].font_size == Approx(12.0));
        CHECK(labels[ 0 ].direction == GridTextLabel::Direction::RightUp);

        CHECK(labels[ 1 ].text == "RU 2");
        CHECK(labels[ 1 ].font_size == Approx(8.0));
        CHECK(labels[ 1 ].direction == GridTextLabel::Direction::LeftDown);
    }

    SECTION("a malformed text feature is skipped")
    {
        auto vp = labelViewPoint();
        auto features = ViewPointGenVP::scanForFeatures(vp, { ViewPointGenFeatureText::FeatureName });
        REQUIRE(features.size() == 2);

        //first annotation is the grid, the second holds the first text
        auto& text_feature = vp[ ViewPointGenVP::ViewPointFieldAnnotations ][ 1 ][ ViewPointGenAnnotation::AnnotationFieldFeatures ][ 0 ];
        text_feature[ ViewPointGenFeatureText::FeatureTextFieldNamePos ] = "nowhere";

        const auto labels = GridViewModel::textLabels(vp);
        REQUIRE(labels.size() == 1);
        CHECK(labels[ 0 ].text == "RU 2");
    }

    SECTION("the labels and the background apply to a geographic grid only")
    {
        const auto labels = GridViewModel::textLabels(labelViewPoint());

        GridBackgroundSettings background = GridBackgroundSettings::reportDefaults();
        background.borders_show = false;

        auto geo = GridViewModel::displayDataFromAnnotation(feature(makeLayer("wgs84"), {}), Metadata, {}, labels, background);
        REQUIRE(geo);
        CHECK(geo->labels.size() == 2);
        REQUIRE(geo->background.has_value());
        CHECK(!geo->background->borders_show);

        auto other = GridViewModel::displayDataFromAnnotation(feature(makeLayer(""), {}), Metadata, {}, labels, background);
        REQUIRE(other);
        CHECK(other->labels.empty());
        CHECK(!other->background.has_value());
    }

    SECTION("the background set of the view point")
    {
        auto vp = labelViewPoint();
        CHECK(!GridViewModel::backgroundFromViewPoint(vp).has_value());

        GridBackgroundSettings background;
        background.color                = "#000000";
        background.hidden_sector_layers = { "TMA" };

        vp[ ViewPointGenFeatureGrid::ViewPointFieldNameBackground ] = background.toJSON();

        auto read = GridViewModel::backgroundFromViewPoint(vp);
        REQUIRE(read.has_value());
        CHECK(*read == background);

        vp[ ViewPointGenFeatureGrid::ViewPointFieldNameBackground ] = "black";
        CHECK(!GridViewModel::backgroundFromViewPoint(vp).has_value());
    }
}
