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

#include "gridviewrenderwidget.h"
#include "chartview.h"
#include "grid2d.h"
#include "apppalette.h"

#include "json.hpp"

#include <QDir>
#include <QImage>

#include <QtCharts/QAbstractAxis>
#include <QtCharts/QChart>

#include <algorithm>
#include <limits>

namespace
{
    const QColor Low (Qt::red);
    const QColor High(Qt::green);

    /// one row of cells, x 0 .. number of values, y 1000 .. 2000 (e.g. altitude), no spatial reference
    Grid2DLayer rowLayer(const std::vector<double>& values)
    {
        Grid2DLayer layer;
        layer.name = "test";
        layer.data.resize(1, (int)values.size());

        for (size_t i = 0; i < values.size(); ++i)
            layer.data(0, (int)i) = values[ i ];

        layer.ref.srs              = "";
        layer.ref.img_origin_x     = 0.0;
        layer.ref.img_origin_y     = 2000.0;
        layer.ref.img_pixel_size_x = 1.0;
        layer.ref.img_pixel_size_y = 1000.0;
        layer.ref.is_north_up      = true;

        return layer;
    }

    Grid2DRenderSettings binarySettings()
    {
        Grid2DRenderSettings rs;
        rs.color_map.create(Low, High, 2, ColorMap::Type::Binary);
        rs.min_value = 0.0;
        rs.max_value = 1.0;

        return rs;
    }

    std::shared_ptr<GridDisplayData> makeData(const std::vector<double>& values,
                                              const boost::optional<Grid2DRenderSettings>& rs = binarySettings(),
                                              const std::string& domain_id = "annotation")
    {
        auto data = std::make_shared<GridDisplayData>();
        data->layer           = rowLayer(values);
        data->render_settings = rs;
        data->title           = "PD - Altitude/Longitude";
        data->x_axis_label    = "Longitude (deg)";
        data->y_axis_label    = "Altitude (ft)";
        data->domain_id       = domain_id;
        data->valid           = true;

        return data;
    }

    /// x positions of the pixels of a color, in the left part of the image (the chart, not the legend)
    struct ColorColumns
    {
        int min = std::numeric_limits<int>::max();
        int max = -1;

        bool found() const { return max >= 0; }
    };

    ColorColumns columnsOf(const QImage& image, const QColor& color, int x_begin, int x_end)
    {
        ColorColumns columns;

        for (int y = 0; y < image.height(); ++y)
        {
            for (int x = x_begin; x < x_end; ++x)
            {
                if (image.pixelColor(x, y).rgb() != color.rgb())
                    continue;

                columns.min = std::min(columns.min, x);
                columns.max = std::max(columns.max, x);
            }
        }

        return columns;
    }

    QImage renderImage(GridViewRenderWidget& widget, const QSize& size = QSize(600, 400), double scale = 1.0)
    {
        QImage image;
        REQUIRE(widget.render(image, size, scale));

        return image.convertToFormat(QImage::Format_ARGB32);
    }

    std::vector<double> legendNumbers(const GridViewRenderWidget& widget)
    {
        std::vector<double> numbers;
        for (const auto& entry : widget.currentLegend().entries())
            numbers.push_back(QString::fromStdString(entry.second).trimmed().toDouble());

        return numbers;
    }
}

TEST_CASE("grid render widget draws the grid", "[view][gridrender]")
{
    GridViewRenderWidget widget;
    widget.setFixedPalette(app_palette::light());

    GridRenderOptions options;

    SECTION("pixel color per cell, the cells in the order of the values")
    {
        widget.setDisplayData(makeData({ 0.0, 1.0 }), options);

        CHECK(widget.isDrawn());
        CHECK(widget.hasContent());

        const QImage image = renderImage(widget);

        const auto low  = columnsOf(image, Low , 0, image.width() * 3 / 4);
        const auto high = columnsOf(image, High, 0, image.width() * 3 / 4);

        REQUIRE(low.found());
        REQUIRE(high.found());

        CHECK(low.max < high.min);
    }

    SECTION("a cell without value has no color")
    {
        widget.setDisplayData(makeData({ 0.0, Grid2D::InvalidValue, 1.0 }), options);

        const QImage image = renderImage(widget);

        const auto low  = columnsOf(image, Low , 0, image.width() * 3 / 4);
        const auto high = columnsOf(image, High, 0, image.width() * 3 / 4);

        REQUIRE(low.found());
        REQUIRE(high.found());

        //the middle cell is about as wide as the others
        const int cell_width = low.max - low.min;

        CHECK(high.min - low.max > cell_width / 2);
    }

    SECTION("the axis labels and the title of the display data")
    {
        widget.setDisplayData(makeData({ 0.0, 1.0 }), options);

        REQUIRE(widget.chartView());
        auto chart = widget.chartView()->chart();

        CHECK(chart->title() == "PD - Altitude/Longitude");
        CHECK(chart->axes(Qt::Horizontal).first()->titleText() == "Longitude (deg)");
        CHECK(chart->axes(Qt::Vertical).first()->titleText() == "Altitude (ft)");

        //the axes show the grid
        const auto ranges = widget.axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left()   == Approx(0.0));
        CHECK(ranges->right()  == Approx(2.0));
        CHECK(ranges->top()    == Approx(1000.0));
        CHECK(ranges->bottom() == Approx(2000.0));
    }

    SECTION("invalid data gives the empty chart")
    {
        auto data = makeData({ 0.0, 1.0 });
        data->valid = false;

        widget.setDisplayData(data, options);

        CHECK(widget.isDrawn());
        CHECK(!widget.hasContent());

        widget.setDisplayData(nullptr, options);

        CHECK(widget.isDrawn());
        CHECK(!widget.hasContent());
        CHECK(widget.currentLegend().entries().empty());
    }

    SECTION("grid opacity")
    {
        options.grid_opacity = 0;
        widget.setDisplayData(makeData({ 0.0, 1.0 }), options);

        //the colors are in the legend only
        const QImage image = renderImage(widget);

        CHECK(!columnsOf(image, Low, 0, image.width() * 3 / 4).found());
        CHECK(columnsOf(image, Low, image.width() * 3 / 4, image.width()).found());
    }
}

TEST_CASE("grid render widget colors from the options and the display data", "[view][gridrender]")
{
    GridViewRenderWidget widget;

    GridRenderOptions options;

    SECTION("without render settings of the display data the options give the color map")
    {
        options.render_settings.min_value = 0.5;
        options.render_settings.max_value = 1.0;

        widget.setDisplayData(makeData({ 0.0, 1.0 }, boost::none), options);

        CHECK(!widget.customRangeInvalid());

        const auto numbers = legendNumbers(widget);
        REQUIRE(numbers.size() > 2);

        CHECK(*std::min_element(numbers.begin(), numbers.end()) == Approx(0.5));
        CHECK(*std::max_element(numbers.begin(), numbers.end()) == Approx(1.0));
    }

    SECTION("a minimum above the maximum is no valid range")
    {
        options.render_settings.min_value = 2.0;
        options.render_settings.max_value = 1.0;

        widget.setDisplayData(makeData({ 0.0, 1.0 }, boost::none), options);

        CHECK(widget.customRangeInvalid());
    }

    SECTION("the render settings of the display data win over the options")
    {
        options.render_settings.min_value = 5.0;
        options.render_settings.max_value = 10.0;

        widget.setDisplayData(makeData({ 0.0, 1.0 }), options);

        const auto& entries = widget.currentLegend().entries();
        REQUIRE(entries.size() == 2);

        CHECK(entries[ 0 ].first.rgb() == Low.rgb());
        CHECK(entries[ 1 ].first.rgb() == High.rgb());

        const auto numbers = legendNumbers(widget);
        CHECK(*std::max_element(numbers.begin(), numbers.end()) <= 1.0);

        //the shown grid with the colors of the display data
        const auto grid = widget.currentGrid();
        REQUIRE(grid.has_value());
        CHECK(grid->second.color_map.getColor(0).rgb() == Low.rgb());
    }

    SECTION("the view info lists the legend")
    {
        widget.setDisplayData(makeData({ 0.0, 1.0 }), options);

        nlohmann::json info;
        widget.viewInfoJSON(info);

        REQUIRE(info.contains("legend_entries"));
        CHECK(info.at("legend_entries").size() == 2);
        CHECK(info.at("background_available") == false);
        CHECK(info.at("axis_bounds_valid") == true);
        CHECK(info.at("chart").at("y_axis_label") == "Altitude (ft)");
    }
}

TEST_CASE("grid render widget zoom of the user", "[view][gridrender]")
{
    GridViewRenderWidget widget;

    GridRenderOptions options;

    const QRectF user_view(0.5, 1200.0, 1.0, 500.0);

    auto data = makeData({ 0.0, 1.0 }, binarySettings(), "domain A");
    widget.setDisplayData(data, options);

    widget.setUserView(user_view);

    SECTION("the same domain keeps the user view over a new chart")
    {
        widget.setDisplayData(makeData({ 1.0, 0.0 }, binarySettings(), "domain A"), options);

        const auto ranges = widget.axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left()  == Approx(user_view.left()));
        CHECK(ranges->right() == Approx(user_view.right()));
        CHECK(widget.hasUserView());
    }

    SECTION("a chart without content keeps the user view, e.g. during a load")
    {
        auto empty = makeData({ 0.0, 1.0 }, binarySettings(), "domain A");
        empty->valid = false;

        widget.setDisplayData(empty, options);
        widget.setDisplayData(makeData({ 1.0, 0.0 }, binarySettings(), "domain A"), options);

        const auto ranges = widget.axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left() == Approx(user_view.left()));
    }

    SECTION("another domain shows the grid and drops the user view")
    {
        widget.setDisplayData(makeData({ 1.0, 0.0 }, binarySettings(), "domain B"), options);

        const auto ranges = widget.axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left()  == Approx(0.0));
        CHECK(ranges->right() == Approx(2.0));
        CHECK(!widget.hasUserView());
    }

    SECTION("zoom to home shows the grid and drops the user view")
    {
        widget.setDisplayData(makeData({ 1.0, 0.0 }, binarySettings(), "domain A"), options);
        widget.zoomToHome();

        const auto ranges = widget.axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left() == Approx(0.0));
        CHECK(!widget.hasUserView());
    }

    SECTION("a new grid opacity repaints the chart without a new chart")
    {
        const ChartView* chart_view = widget.chartView();

        options.grid_opacity = 50;
        widget.setDisplayData(data, options);

        CHECK(widget.chartView() == chart_view);
        CHECK(widget.gridOpacity() == 50);
    }
}

TEST_CASE("grid render widget image", "[view][gridrender]")
{
    GridViewRenderWidget widget;
    widget.setFixedPalette(app_palette::light());
    widget.setDisplayData(makeData({ 0.0, 0.5, 1.0 }), GridRenderOptions());

    SECTION("requested size, not blank, the same image twice")
    {
        QImage image;
        REQUIRE(widget.render(image, QSize(1200, 800), 2.0));

        QImage image_again;
        REQUIRE(widget.render(image_again, QSize(1200, 800), 2.0));

        CHECK(image.size() == QSize(2400, 1600));
        CHECK(image == image_again);

        CHECK(image.pixelColor(1, 1).rgb() == app_palette::light().color(QPalette::Window).rgb());
    }

    SECTION("the color legend is right of the chart")
    {
        const QImage image = renderImage(widget, QSize(1200, 800), 1.0);

        const auto chart_low  = columnsOf(image, Low, 0, image.width());
        REQUIRE(chart_low.found());

        //the legend squares are the rightmost pixels of the color
        CHECK(chart_low.max > image.width() * 3 / 4);
        CHECK(columnsOf(image, High, image.width() * 3 / 4, image.width()).found());
    }

    SECTION("an image only widget builds no display")
    {
        GridViewRenderWidget image_widget;
        image_widget.setImageOnly(true);
        image_widget.setDisplayData(makeData({ 0.0, 1.0 }), GridRenderOptions());

        CHECK(!image_widget.isDrawn());
        CHECK(!image_widget.chartView());

        QImage image;
        CHECK(image_widget.render(image, QSize(600, 400), 1.0));
    }
}

// Not run by default. Writes a grid image with its color legend into the directory
// COMPASS_TEST_IMAGE_DIR. scripts/check_image_display_scaling.py runs it with the display scaling 1 and 2
// and compares the images.
TEST_CASE("grid image for the display scaling check", "[.][displayscaling]")
{
    const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
    REQUIRE(!dir.isEmpty());

    Grid2DRenderSettings rs;
    rs.color_map.create(colorscale::ColorScale::Red2Green, 3, ColorMap::Type::LinearSamples, std::make_pair(0.0, 1.0));
    rs.min_value        = 0.0;
    rs.max_value        = 1.0;
    rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
    rs.legend_precision = 2;

    auto data = makeData({ 0.0, 0.25, Grid2D::InvalidValue, 0.75, 1.0 }, rs);

    //own tick labels on the x axis
    data->axis_data_type[ 0 ] = PropertyDataType::TIMESTAMP;

    GridViewRenderWidget widget;
    widget.setFixedPalette(app_palette::light());
    widget.setImageOnly(true);
    widget.setDisplayData(data, GridRenderOptions());

    QImage image;
    REQUIRE(widget.render(image, QSize(1200, 800), 2.0));
    REQUIRE(image.save(QDir(dir).filePath("grid.png")));
}
