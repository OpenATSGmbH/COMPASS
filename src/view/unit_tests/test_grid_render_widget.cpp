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

namespace
{
    const QColor GeoLow     ("#0000C0");
    const QColor GeoHigh    ("#000080");
    const QColor SectorColor("#FF00FF");
    const QColor LabelColor ("#00FFFF");
    const QColor BorderColor(255, 255, 0);

    /// 3 x 3 cells of 1 degree over Austria (longitude 14 to 17, latitude 46 to 49), the middle cell without value
    Grid2DLayer geoLayer()
    {
        Grid2DLayer layer;
        layer.name = "geo";
        layer.data.resize(3, 3);
        layer.data << 0.0, 1.0, 0.0,
                      1.0, Grid2D::InvalidValue, 1.0,
                      0.0, 1.0, 0.0;

        layer.ref.srs              = "wgs84";
        layer.ref.img_origin_x     = 14.0;
        layer.ref.img_origin_y     = 49.0;
        layer.ref.img_pixel_size_x = 1.0;
        layer.ref.img_pixel_size_y = 1.0;
        layer.ref.is_north_up      = true;

        return layer;
    }

    /// a diamond inside the middle cell
    GridSectorLine sectorLine()
    {
        QPolygonF poly;
        poly << QPointF(15.5, 47.2) << QPointF(15.8, 47.5) << QPointF(15.5, 47.8) << QPointF(15.2, 47.5) << QPointF(15.5, 47.2);

        GridSectorLine line;
        line.line        = gridbackground::makeLine(poly);
        line.color       = SectorColor;
        line.layer_name  = "TMA";
        line.sector_name = "TMA 1";
        line.sector_id   = 7;

        return line;
    }

    /// borders off, a background color no other part has, the sector shown
    GridBackgroundSettings geoBackground()
    {
        GridBackgroundSettings background;
        background.color        = "#102030";
        background.borders_show = false;

        return background;
    }

    std::shared_ptr<GridDisplayData> makeGeoData(const boost::optional<GridBackgroundSettings>& background = geoBackground())
    {
        Grid2DRenderSettings rs;
        rs.color_map.create(GeoLow, GeoHigh, 2, ColorMap::Type::Binary);
        rs.min_value = 0.0;
        rs.max_value = 1.0;

        auto data = std::make_shared<GridDisplayData>();
        data->layer           = geoLayer();
        data->render_settings = rs;
        data->is_geographic   = true;
        data->title           = "PD - Horizontal";
        data->x_axis_label    = "Longitude (deg)";
        data->y_axis_label    = "Latitude (deg)";
        data->sectors         = { sectorLine() };
        data->background      = background;
        data->domain_id       = "annotation";
        data->valid           = true;

        return data;
    }

    GridTextLabel label(double lon, double lat)
    {
        GridTextLabel l;
        l.position  = QPointF(lon, lat);
        l.text      = "RU";
        l.color     = LabelColor;
        l.font_size = 24.0;

        return l;
    }

    QImage renderGeo(const std::shared_ptr<GridDisplayData>& data,
                     const GridRenderOptions& options = GridRenderOptions(),
                     double scale = 1.0)
    {
        GridViewRenderWidget widget;
        widget.setFixedPalette(app_palette::light());
        widget.setImageOnly(true);
        widget.setDisplayData(data, options);

        QImage image;
        REQUIRE(widget.render(image, QSize(600, 400), scale));

        return image.convertToFormat(QImage::Format_ARGB32);
    }

    template <typename Pred>
    size_t countPixels(const QImage& image, Pred pred)
    {
        size_t n = 0;
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                if (pred(image.pixelColor(x, y)))
                    ++n;

        return n;
    }

    size_t countColor(const QImage& image, const QColor& color)
    {
        return countPixels(image, [ & ] (const QColor& c) { return c.rgb() == color.rgb(); });
    }

    /// the pixel is mainly the sector color, also at an antialiased edge
    bool isSectorPixel(const QColor& c)
    {
        return c.red() > 150 && c.blue() > 150 && c.green() < 100;
    }

    /// mean x of the pixels of a color
    double meanX(const QImage& image, const QColor& color)
    {
        double sum = 0.0;
        size_t n   = 0;

        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                if (image.pixelColor(x, y).rgb() == color.rgb())
                {
                    sum += x;
                    ++n;
                }

        return n > 0 ? sum / n : -1.0;
    }
}

TEST_CASE("grid render widget background of a geographic grid", "[view][gridrender]")
{
    SECTION("the background of the display data wins over the options")
    {
        GridRenderOptions options;
        options.background.color = "#203040";

        const QImage image = renderGeo(makeGeoData(), options);

        CHECK(countColor(image, QColor("#102030")) > 100);
        CHECK(countColor(image, QColor("#203040")) == 0);
    }

    SECTION("without a background in the display data the options apply")
    {
        GridRenderOptions options;
        options.background.color        = "#203040";
        options.background.borders_show = false;

        const QImage image = renderGeo(makeGeoData(boost::none), options);

        CHECK(countColor(image, QColor("#203040")) > 100);
        CHECK(countColor(image, QColor("#102030")) == 0);
    }

    SECTION("national borders")
    {
        auto background = geoBackground();

        //Austria borders on Czechia, Slovakia, Hungary and Slovenia inside the grid
        background.borders_show = true;
        const QImage with = renderGeo(makeGeoData(background));

        background.borders_show = false;
        const QImage without = renderGeo(makeGeoData(background));

        CHECK(countColor(with, BorderColor) > 50);
        CHECK(countColor(without, BorderColor) == 0);
    }

    SECTION("a shown sector draws its outline, a hidden one does not")
    {
        auto background = geoBackground();

        CHECK(countPixels(renderGeo(makeGeoData(background)), isSectorPixel) > 50);

        background.hidden_sector_layers = { "TMA" };
        CHECK(countPixels(renderGeo(makeGeoData(background)), isSectorPixel) == 0);

        background = geoBackground();
        background.hidden_sectors = { 7 };
        CHECK(countPixels(renderGeo(makeGeoData(background)), isSectorPixel) == 0);

        background = geoBackground();
        background.sectors_show = false;
        CHECK(countPixels(renderGeo(makeGeoData(background)), isSectorPixel) == 0);
    }

    SECTION("a text label changes the pixels at its position")
    {
        auto data = makeGeoData();
        CHECK(countColor(renderGeo(data), LabelColor) == 0);

        auto west = makeGeoData();
        west->labels = { label(14.5, 47.5) };

        auto east = makeGeoData();
        east->labels = { label(16.5, 47.5) };

        const QImage image_west = renderGeo(west);
        const QImage image_east = renderGeo(east);

        CHECK(countColor(image_west, LabelColor) > 10);
        CHECK(countColor(image_east, LabelColor) > 10);

        //a degree of longitude is more than 50 pixels here
        CHECK(meanX(image_east, LabelColor) - meanX(image_west, LabelColor) > 100.0);
    }

    SECTION("a text label is drawn above the sectors")
    {
        //a large label from the lower left edge of the diamond covers parts of its outline
        auto data = makeGeoData();
        data->labels = { label(15.3, 47.3) };
        data->labels[ 0 ].font_size = 40.0;
        data->labels[ 0 ].text      = "MMM";

        CHECK(countPixels(renderGeo(data), isSectorPixel) < countPixels(renderGeo(makeGeoData()), isSectorPixel));
    }

    SECTION("labels of a grid over other axes are not drawn")
    {
        auto data = makeData({ 0.0, 1.0 });
        data->labels = { label(0.5, 1500.0) };

        GridViewRenderWidget widget;
        widget.setFixedPalette(app_palette::light());
        widget.setDisplayData(data, GridRenderOptions());

        CHECK(countColor(renderImage(widget), LabelColor) == 0);
    }
}

TEST_CASE("grid render widget draws borders and sectors at the pixels of the image", "[view][gridrender]")
{
    const QImage image_1 = renderGeo(makeGeoData(), GridRenderOptions(), 1.0);
    const QImage image_2 = renderGeo(makeGeoData(), GridRenderOptions(), 2.0);

    REQUIRE(image_2.size() == QSize(1200, 800));

    SECTION("the lines have the same look: twice as long and twice as wide")
    {
        const double ratio = countPixels(image_2, isSectorPixel) / (double)countPixels(image_1, isSectorPixel);

        CHECK(ratio > 3.0);
        CHECK(ratio < 5.0);
    }

    SECTION("the lines are drawn at the resolution of the image, not scaled up")
    {
        //an image drawn at layout size and scaled up by 2 has columns in identical pairs
        size_t pairs     = 0;
        size_t identical = 0;

        auto column = [ & ] (int x)
        {
            std::vector<int> rows;
            for (int y = 0; y < image_2.height(); ++y)
                if (isSectorPixel(image_2.pixelColor(x, y)))
                    rows.push_back(y);
            return rows;
        };

        for (int x = 0; x + 1 < image_2.width(); ++x)
        {
            const auto c0 = column(x);
            const auto c1 = column(x + 1);

            if (c0.empty() || c1.empty())
                continue;

            ++pairs;
            if (c0 == c1)
                ++identical;
        }

        REQUIRE(pairs > 20);
        CHECK(identical / (double)pairs < 0.3);
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

// Not run by default. Writes a geographic grid image with national borders, a sector and a text label into
// the directory COMPASS_TEST_IMAGE_DIR, see the test above.
TEST_CASE("geographic grid image for the display scaling check", "[.][displayscaling]")
{
    const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
    REQUIRE(!dir.isEmpty());

    auto background = geoBackground();
    background.borders_show = true;

    auto data = makeGeoData(background);
    data->labels = { label(16.2, 47.6) };
    data->labels[ 0 ].font_size = 13.0;

    GridViewRenderWidget widget;
    widget.setFixedPalette(app_palette::light());
    widget.setImageOnly(true);
    widget.setDisplayData(data, GridRenderOptions());

    QImage image;
    REQUIRE(widget.render(image, QSize(1200, 800), 2.0));
    REQUIRE(image.save(QDir(dir).filePath("grid_geo.png")));
}
