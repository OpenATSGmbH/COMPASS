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
#include "reportfigure.h"
#include "histogramviewrenderwidget.h"
#include "scatterplotviewrenderwidget.h"
#include "scatterplotviewmodel.h"
#include "gridviewrenderwidget.h"
#include "grid2d.h"
#include "scatterplotpointsitem.h"
#include "viewtoolswitcher.h"
#include "chartview.h"
#include "viewpointgenerator.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QImage>

#include <QtCharts/QAbstractBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QChart>
#include <QtCharts/QLegend>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QXYSeries>

#include <algorithm>
#include <fstream>
#include <functional>
#include <limits>
#include <random>

namespace
{
    /// view point of a report figure with one histogram annotation of two series, loads no data
    nlohmann::json viewPoint()
    {
        RawHistogram h0;
        h0.addBin(RawHistogramBin(3, "0-10"));
        h0.addBin(RawHistogramBin(12, "10-20"));

        RawHistogram h1;
        h1.addBin(RawHistogramBin(1, "0-10"));
        h1.addBin(RawHistogramBin(7, "10-20"));

        RawHistogramCollection collection;
        collection.addDataSeries(h0, "Radar A", QColor("#ff8000"));
        collection.addDataSeries(h1, "Radar B", QColor("#0080ff"));

        ViewPointGenVP vp("Range Error Histogram", 0, "Histogram");
        vp.annotations().getOrCreateAnnotation("Range Error")->addFeature(
            new ViewPointGenFeatureHistogram(collection, PlotMetadata("Range Error", "", "Error [m]", "Count")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    QList<QtCharts::QBarSet*> barSets(QtCharts::QChart* chart)
    {
        QList<QtCharts::QBarSet*> sets;

        for (auto series : chart->series())
            if (auto bar_series = dynamic_cast<QtCharts::QAbstractBarSeries*>(series))
                sets << bar_series->barSets();

        return sets;
    }
}

TEST_CASE("report figure factory builds the histogram render widget", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(viewPoint());
    REQUIRE(entries.size() == 1);

    auto widget = report_figure::createRenderWidget(entries[ 0 ]);
    REQUIRE(widget);

    auto histogram_widget = dynamic_cast<HistogramViewRenderWidget*>(widget.get());
    REQUIRE(histogram_widget);

    CHECK(histogram_widget->isDrawn());
    CHECK(histogram_widget->hasContent());

    REQUIRE(histogram_widget->chartView());
    auto chart = histogram_widget->chartView()->chart();

    SECTION("the chart shows the series and the title of the annotation")
    {
        CHECK(chart->title() == "Range Error");

        const auto sets = barSets(chart);
        REQUIRE(sets.size() == 2);

        CHECK(sets[ 0 ]->label() == "Radar A");
        CHECK(sets[ 0 ]->color() == QColor("#ff8000"));
        CHECK(sets[ 0 ]->count() == 2);
        CHECK(sets[ 0 ]->at(0) == Approx(3));
        CHECK(sets[ 0 ]->at(1) == Approx(12));

        CHECK(sets[ 1 ]->label() == "Radar B");
        CHECK(sets[ 1 ]->color() == QColor("#0080ff"));
        CHECK(sets[ 1 ]->at(0) == Approx(1));
        CHECK(sets[ 1 ]->at(1) == Approx(7));
    }

    SECTION("the widget has the display of a report figure")
    {
        CHECK(chart->legend()->isVisible());
        CHECK(chart->legend()->alignment() == Qt::AlignRight);
        CHECK(widget->wheelIgnored());
    }
}

TEST_CASE("report figure factory shows the empty chart for a malformed annotation", "[view][reportfigure]")
{
    report_figure::RenderEntry entry;
    entry.image_name = "histogram_0";
    entry.feature    = std::make_shared<const nlohmann::json>(nlohmann::json::object({ { "type", "histogram" } }));

    auto widget = report_figure::createRenderWidget(entry);
    REQUIRE(widget);

    CHECK(widget->isDrawn());
    CHECK(!widget->hasContent());
}

TEST_CASE("report figure export image", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(viewPoint());
    REQUIRE(entries.size() == 1);

    const QPalette palette_before = QApplication::palette();

    QPalette dark;
    dark.setColor(QPalette::Window, QColor(40, 40, 40));
    dark.setColor(QPalette::Base  , QColor(20, 20, 20));

    QApplication::setPalette(dark);

    QImage image;
    const bool ok = report_figure::renderExportImage(entries[ 0 ], image);

    QApplication::setPalette(palette_before);

    REQUIRE(ok);

    CHECK(image.size() == QSize(2400, 1600));
    CHECK(image.pixel(1, 1) == app_palette::light().color(QPalette::Window).rgb());

    //the bars of both series
    bool has_color_a = false;
    bool has_color_b = false;

    for (int y = 0; y < image.height(); ++y)
    {
        for (int x = 0; x < image.width(); ++x)
        {
            has_color_a = has_color_a || image.pixel(x, y) == QColor("#ff8000").rgb();
            has_color_b = has_color_b || image.pixel(x, y) == QColor("#0080ff").rgb();
        }
    }

    CHECK(has_color_a);
    CHECK(has_color_b);
}

namespace
{
    /// view point of a report figure with one scatter series annotation of two series, loads no data
    nlohmann::json scatterViewPoint()
    {
        ScatterSeries points_a;
        points_a.points = { { 0.0, 0.0 }, { 50.0, 20.0 }, { 100.0, 100.0 } };

        ScatterSeries points_b;
        points_b.points = { { 10.0, 80.0 }, { 90.0, 30.0 } };

        ScatterSeriesCollection collection;
        collection.addDataSeries(points_a, "Radar A", QColor("#ff8000"), 10.0);
        collection.addDataSeries(points_b, "Radar B", QColor("#0080ff"), 10.0);

        ViewPointGenVP vp("Range Bias", 0, "Scatterplot");
        vp.annotations().getOrCreateAnnotation("Range Bias")->addFeature(
            new ViewPointGenFeatureScatterSeries(collection, PlotMetadata("Range Bias", "", "Range [m]", "Bias [m]")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    bool hasColor(const QImage& image, const QColor& color)
    {
        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                if (image.pixel(x, y) == color.rgb())
                    return true;

        return false;
    }
}

TEST_CASE("report figure factory builds the scatterplot render widget", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(scatterViewPoint());
    REQUIRE(entries.size() == 1);
    REQUIRE(entries[ 0 ].kind == report_figure::Kind::Scatterplot);

    auto widget = report_figure::createRenderWidget(entries[ 0 ]);
    REQUIRE(widget);

    auto scatter_widget = dynamic_cast<ScatterPlotViewRenderWidget*>(widget.get());
    REQUIRE(scatter_widget);

    CHECK(scatter_widget->isDrawn());
    CHECK(scatter_widget->hasContent());

    REQUIRE(scatter_widget->chartView());
    auto chart = scatter_widget->chartView()->chart();

    SECTION("the chart shows the title, the axis labels and the data bounds of the annotation")
    {
        CHECK(chart->title() == "Range Bias");
        CHECK(chart->axes(Qt::Horizontal).first()->titleText() == "Range [m]");
        CHECK(chart->axes(Qt::Vertical).first()->titleText() == "Bias [m]");

        const auto ranges = scatter_widget->axisRanges();
        REQUIRE(ranges.has_value());

        CHECK(ranges->left()  == Approx(-3.0));
        CHECK(ranges->right() == Approx(103.0));
    }

    SECTION("the legend at the right side lists the series")
    {
        scatter_widget->chartView()->layoutForSize(report_figure::ExportLayoutSize);

        CHECK(chart->legend()->isVisible());
        CHECK(chart->legend()->alignment() == Qt::AlignRight);

        QStringList entries_shown;
        for (auto marker : chart->legend()->markers())
            if (marker->isVisible())
                entries_shown << marker->label();

        CHECK(entries_shown == QStringList({ "Radar A", "Radar B" }));
    }

    SECTION("the chart has no OpenGL series")
    {
        for (auto series : chart->series())
            if (auto xy_series = dynamic_cast<QtCharts::QXYSeries*>(series))
                CHECK(!xy_series->useOpenGL());

        CHECK(scatter_widget->chartView()->selectionSeries() == nullptr);
    }

    SECTION("the figure has no interaction")
    {
        CHECK(widget->wheelIgnored());
        CHECK(!scatter_widget->dragEnabled());

        //only the navigate tool of the widget, which has no shortcut and no toolbar entry
        const auto& tools = scatter_widget->toolSwitcher().registeredTools();

        REQUIRE(tools.size() == 1);
        CHECK(tools.front().key.isEmpty());
        CHECK(tools.front().icon.isNull());
    }

    SECTION("the height follows the width")
    {
        CHECK(widget->heightFollowsWidth());
    }
}

TEST_CASE("report figure factory shows the empty chart for a malformed scatter annotation", "[view][reportfigure]")
{
    report_figure::RenderEntry entry;
    entry.kind       = report_figure::Kind::Scatterplot;
    entry.image_name = "scatterplot_0";
    entry.feature    = std::make_shared<const nlohmann::json>(nlohmann::json::object({ { "type", "scatterseries" } }));

    auto widget = report_figure::createRenderWidget(entry);
    REQUIRE(widget);

    CHECK(widget->isDrawn());
    CHECK(!widget->hasContent());

    QImage image;
    CHECK(report_figure::renderExportImage(entry, image));
    CHECK(image.size() == QSize(2400, 1600));
}

TEST_CASE("report figure export image of a scatterplot", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(scatterViewPoint());
    REQUIRE(entries.size() == 1);

    const QPalette palette_before = QApplication::palette();

    QPalette dark;
    dark.setColor(QPalette::Window, QColor(40, 40, 40));
    dark.setColor(QPalette::Base  , QColor(20, 20, 20));

    QApplication::setPalette(dark);

    QImage image;
    const bool ok = report_figure::renderExportImage(entries[ 0 ], image);

    QImage image_again;
    const bool ok_again = report_figure::renderExportImage(entries[ 0 ], image_again);

    QApplication::setPalette(palette_before);

    REQUIRE(ok);
    REQUIRE(ok_again);

    CHECK(image.size() == QSize(2400, 1600));
    CHECK(image.pixel(1, 1) == app_palette::light().color(QPalette::Window).rgb());

    //the points of both series
    CHECK(hasColor(image, QColor("#ff8000")));
    CHECK(hasColor(image, QColor("#0080ff")));

    CHECK(image == image_again);
}

namespace
{
    /// view point of a report figure with one grid annotation over longitude and altitude, loads no data
    nlohmann::json gridViewPoint()
    {
        Grid2DLayer layer;
        layer.name = "pd";
        layer.data.resize(2, 3);
        layer.data << 0.0, 0.5,                  1.0,
                      1.0, Grid2D::InvalidValue, 0.0;

        layer.ref.srs              = "";
        layer.ref.img_origin_x     = 16.0;
        layer.ref.img_origin_y     = 4000.0;
        layer.ref.img_pixel_size_x = 0.1;
        layer.ref.img_pixel_size_y = 1000.0;
        layer.ref.is_north_up      = true;

        Grid2DRenderSettings rs;
        rs.color_map.create(QColor("#ff0000"), QColor("#00ff00"), 2, ColorMap::Type::Binary);
        rs.min_value = 0.0;
        rs.max_value = 1.0;

        ViewPointGenVP vp("PD - Altitude/Longitude", 0, "Grid");
        vp.annotations().getOrCreateAnnotation("PD")->addFeature(
            new ViewPointGenFeatureGrid(layer, rs, PlotMetadata("PD - Altitude/Longitude", "", "Longitude (deg)", "Altitude (ft)")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }
}

TEST_CASE("report figure factory builds the grid render widget", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(gridViewPoint());
    REQUIRE(entries.size() == 1);
    REQUIRE(entries[ 0 ].kind == report_figure::Kind::Grid);
    CHECK(entries[ 0 ].image_name == "grid_0");

    auto widget = report_figure::createRenderWidget(entries[ 0 ]);
    REQUIRE(widget);

    auto grid_widget = dynamic_cast<GridViewRenderWidget*>(widget.get());
    REQUIRE(grid_widget);

    CHECK(grid_widget->isDrawn());
    CHECK(grid_widget->hasContent());

    REQUIRE(grid_widget->chartView());
    auto chart = grid_widget->chartView()->chart();

    SECTION("the chart shows the title and the axis labels of the annotation, no background")
    {
        CHECK(chart->title() == "PD - Altitude/Longitude");
        CHECK(chart->axes(Qt::Horizontal).first()->titleText() == "Longitude (deg)");
        CHECK(chart->axes(Qt::Vertical).first()->titleText() == "Altitude (ft)");

        CHECK(!grid_widget->background().available());
    }

    SECTION("the color legend of the annotation")
    {
        CHECK(grid_widget->currentLegend().entries().size() == 2);
    }

    SECTION("the figure has no interaction")
    {
        CHECK(widget->wheelIgnored());
        CHECK(!grid_widget->dragEnabled());

        //only the navigate tool of the widget, which has no shortcut and no toolbar entry
        const auto& tools = grid_widget->toolSwitcher().registeredTools();

        REQUIRE(tools.size() == 1);
        CHECK(tools.front().key.isEmpty());
        CHECK(tools.front().icon.isNull());
    }

    SECTION("the height follows the width")
    {
        CHECK(widget->heightFollowsWidth());
    }
}

TEST_CASE("report figure export image of a grid", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(gridViewPoint());
    REQUIRE(entries.size() == 1);

    const QPalette palette_before = QApplication::palette();

    QPalette dark;
    dark.setColor(QPalette::Window, QColor(40, 40, 40));
    dark.setColor(QPalette::Base  , QColor(20, 20, 20));

    QApplication::setPalette(dark);

    QImage image;
    const bool ok = report_figure::renderExportImage(entries[ 0 ], image);

    QImage image_again;
    const bool ok_again = report_figure::renderExportImage(entries[ 0 ], image_again);

    QApplication::setPalette(palette_before);

    REQUIRE(ok);
    REQUIRE(ok_again);

    CHECK(image.size() == QSize(2400, 1600));
    CHECK(image.pixel(1, 1) == app_palette::light().color(QPalette::Window).rgb());

    //the cells of both colors
    CHECK(hasColor(image, QColor("#ff0000")));
    CHECK(hasColor(image, QColor("#00ff00")));

    CHECK(image == image_again);
}

namespace
{
    /// view point of a report figure with a longitude / latitude grid over Austria and a text label
    nlohmann::json geoGridViewPoint(const boost::optional<GridBackgroundSettings>& background)
    {
        Grid2DLayer layer;
        layer.name = "pd";
        layer.data.resize(2, 2);
        layer.data << 0.0,                  1.0,
                      Grid2D::InvalidValue, 0.0;

        layer.ref.srs              = "wgs84";
        layer.ref.img_origin_x     = 14.0;
        layer.ref.img_origin_y     = 49.0;
        layer.ref.img_pixel_size_x = 1.5;
        layer.ref.img_pixel_size_y = 1.5;
        layer.ref.is_north_up      = true;

        Grid2DRenderSettings rs;
        rs.color_map.create(QColor("#0000c0"), QColor("#000080"), 2, ColorMap::Type::Binary);
        rs.min_value = 0.0;
        rs.max_value = 1.0;

        ViewPointGenVP vp("PD - Horizontal", 0, "Grid");
        vp.annotations().getOrCreateAnnotation("PD")->addFeature(
            new ViewPointGenFeatureGrid(layer, rs, PlotMetadata("PD - Horizontal", "", "Longitude (deg)", "Latitude (deg)")));

        auto text = new ViewPointGenFeatureText("RU 1", 47.5, 15.5, 30.0f);
        text->setColor(QColor("#00ffff"));
        vp.annotations().getOrCreateAnnotation("RU position")->addFeature(text);

        nlohmann::json j;
        vp.toJSON(j);

        if (background.has_value())
            j[ ViewPointGenFeatureGrid::ViewPointFieldNameBackground ] = background->toJSON();

        return j;
    }

    size_t numPixels(const QImage& image, const QColor& color)
    {
        const QImage img = image.convertToFormat(QImage::Format_ARGB32);

        size_t n = 0;
        for (int y = 0; y < img.height(); ++y)
            for (int x = 0; x < img.width(); ++x)
                if (img.pixel(x, y) == color.rgb())
                    ++n;

        return n;
    }
}

TEST_CASE("report figure export image of a geographic grid", "[view][reportfigure]")
{
    SECTION("without a background set: the report defaults, national borders on black")
    {
        const auto defaults = GridBackgroundSettings::reportDefaults();

        CHECK(defaults.color == "#000000");
        CHECK(!defaults.map_show);
        CHECK(defaults.map_opacity == 50);
        CHECK(defaults.borders_show);
        CHECK(defaults.sectors_show);
        CHECK(defaults.hidden_sector_layers.empty());
        CHECK(!defaults.map_file.empty());

        const auto entries = report_figure::selectRenderEntries(geoGridViewPoint(boost::none));
        REQUIRE(entries.size() == 1);

        QImage image;
        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));

        CHECK(image.size() == QSize(2400, 1600));
        CHECK(numPixels(image, Qt::black) > 10000);
        CHECK(numPixels(image, QColor(255, 255, 0)) > 100);

        //the label of the text feature
        CHECK(numPixels(image, QColor("#00ffff")) > 50);
    }

    SECTION("the points and lines of the view point are drawn, the circles are not")
    {
        auto with_overlays = geoGridViewPoint(boost::none);

        //yellow as the RU position circle of the MLAT RU coverage, the color of the national borders
        ViewPointGenAnnotation marker("RU marker");
        marker.addFeature(new ViewPointGenFeatureErrEllipses(3.0f, 32, { Eigen::Vector2d(47.5, 15.5) },
                                                             { QColor("#FFFF00") },
                                                             { Eigen::Vector3d(20000.0, 20000.0, 0.0) }));
        marker.addFeature(new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, 12.0f,
                                                        { Eigen::Vector2d(47.0, 16.0) }, { QColor("#FF00FF") }));
        marker.addFeature(new ViewPointGenFeatureLines(3.0f, ViewPointGenFeatureStyledLine::LineStyle::Solid,
                                                       { Eigen::Vector2d(47.0, 14.5), Eigen::Vector2d(48.5, 16.5) },
                                                       { QColor("#FF00FF") }));

        nlohmann::json marker_json;
        marker.toJSON(marker_json);
        with_overlays[ ViewPointGenVP::ViewPointFieldAnnotations ].push_back(marker_json);

        const auto entries          = report_figure::selectRenderEntries(geoGridViewPoint(boost::none));
        const auto overlays_entries = report_figure::selectRenderEntries(with_overlays);

        REQUIRE(entries.size() == 1);
        REQUIRE(overlays_entries.size() == 1);

        QImage image, image_overlays;
        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));
        REQUIRE(report_figure::renderExportImage(overlays_entries[ 0 ], image_overlays));

        CHECK(numPixels(image, QColor("#FF00FF")) == 0);
        CHECK(numPixels(image_overlays, QColor("#FF00FF")) > 50);
    }

    SECTION("the background set of the view point")
    {
        GridBackgroundSettings background = GridBackgroundSettings::reportDefaults();
        background.color        = "#102030";
        background.borders_show = false;

        const auto entries = report_figure::selectRenderEntries(geoGridViewPoint(background));
        REQUIRE(entries.size() == 1);

        QImage image;
        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));

        CHECK(numPixels(image, QColor("#102030")) > 10000);
        CHECK(numPixels(image, QColor(255, 255, 0)) == 0);
    }
}

TEST_CASE("report figure export image of a map", "[view][reportfigure]")
{
    //lines below the points, a label on top, as the MLAT RU effect maps
    ViewPointGenVP vp("RU01 - Map", 0, "Geographic");

    vp.annotations().getOrCreateAnnotation("Reference Lines")->addFeature(
        new ViewPointGenFeatureLines(4.0f, ViewPointGenFeatureStyledLine::LineStyle::Solid,
                                     { Eigen::Vector2d(48.0, 16.0), Eigen::Vector2d(48.2, 16.4) },
                                     { QColor("#00FF00"), QColor("#00FF00") }, true));

    auto* points = new ViewPointGenFeaturePoints(ViewPointGenFeaturePoints::Symbol::Circle, 12.0f,
                                                 { Eigen::Vector2d(48.0, 16.0), Eigen::Vector2d(48.2, 16.4) },
                                                 {}, false);
    points->setColor(QColor("#FF00FF"));
    vp.annotations().getOrCreateAnnotation("Reports")->addFeature(points);

    auto* text = new ViewPointGenFeatureText("RU01", 48.1, 16.2, 30.0f);
    text->setColor(QColor("#00FFFF"));
    vp.annotations().getOrCreateAnnotation("Remote Unit")->addFeature(text);

    nlohmann::json j;
    vp.toJSON(j);

    const auto entries = report_figure::selectRenderEntries(j);
    REQUIRE(entries.size() == 1);
    REQUIRE(entries[ 0 ].kind == report_figure::Kind::Map);

    QImage image;
    REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));

    CHECK(image.size() == QSize(2400, 1600));

    //the report defaults: black background
    CHECK(numPixels(image, Qt::black) > 10000);

    CHECK(numPixels(image, QColor("#00FF00")) > 50);
    CHECK(numPixels(image, QColor("#FF00FF")) > 50);
    CHECK(numPixels(image, QColor("#00FFFF")) > 50);

    SECTION("the GUI widget of the map has content")
    {
        auto widget = report_figure::createRenderWidget(entries[ 0 ]);
        REQUIRE(widget);

        nlohmann::json info;
        widget->viewInfoJSON(info);

        CHECK(info.value("num_overlays", 0) == 2);
        CHECK(info.value("num_labels", 0) == 1);
        CHECK(info.value("background_available", false));
    }
}

TEST_CASE("report figure factory shows the empty chart for a malformed grid annotation", "[view][reportfigure]")
{
    report_figure::RenderEntry entry;
    entry.kind       = report_figure::Kind::Grid;
    entry.image_name = "grid_0";
    entry.feature    = std::make_shared<const nlohmann::json>(nlohmann::json::object({ { "type", "grid" } }));

    auto widget = report_figure::createRenderWidget(entry);
    REQUIRE(widget);

    CHECK(widget->isDrawn());
    CHECK(!widget->hasContent());

    QImage image;
    CHECK(report_figure::renderExportImage(entry, image));
    CHECK(image.size() == QSize(2400, 1600));
}

namespace
{
    /// view point of a report figure with one scatter series annotation of 10 series and num_points points
    nlohmann::json largeScatterViewPoint(size_t num_points)
    {
        std::mt19937                           rng(42);
        std::uniform_real_distribution<double> value(0.0, 1000.0);

        const size_t num_series = 10;

        ScatterSeriesCollection collection;

        for (size_t s = 0; s < num_series; ++s)
        {
            ScatterSeries series;
            for (size_t i = 0; i < num_points / num_series; ++i)
                series.points.emplace_back(value(rng), value(rng));

            collection.addDataSeries(series, "Series " + std::to_string(s), QColor::fromHsv((int)(s * 36), 255, 220), 10.0);
        }

        ViewPointGenVP vp("Draw Time", 0, "Scatterplot");
        vp.annotations().getOrCreateAnnotation("Draw Time")->addFeature(
            new ViewPointGenFeatureScatterSeries(collection, PlotMetadata("Draw Time", "", "X", "Y")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    /// the fastest of some runs, in milliseconds
    double minTimeMS(const std::function<void()>& func, int runs = 3)
    {
        double best = std::numeric_limits<double>::max();

        for (int i = 0; i < runs; ++i)
        {
            QElapsedTimer timer;
            timer.start();

            func();

            best = std::min(best, timer.nsecsElapsed() / 1e6);
        }

        return best;
    }

    /**
     * Draw time of the export image of each scatterplot of the view point, split into its parts:
     * annotation to display data, points image alone (ScatterPlotPointsItem::drawPoints for the full
     * image size, an upper bound of the plot area), and the complete export image.
     * renderExportImage() builds a new widget and a new chart in each run, nothing is cached.
     */
    void measureScatterDrawTime(const nlohmann::json& view_point, const std::string& label)
    {
        const auto entries = report_figure::selectRenderEntries(view_point);

        for (const auto& entry : entries)
        {
            if (entry.kind != report_figure::Kind::Scatterplot)
                continue;

            std::shared_ptr<const ScatterPlotDisplayData> data;

            const double t_data = minTimeMS([ & ] ()
            {
                data = ScatterPlotViewModel::displayDataFromAnnotation(*entry.feature, entry.metadata);
            });

            REQUIRE(data);
            REQUIRE(data->bounds.has_value());

            const QSize size_px(report_figure::ExportLayoutSize * report_figure::ExportScale);

            const double t_points = minTimeMS([ & ] ()
            {
                ScatterPlotPointsItem::drawPoints(*data, false, size_px, report_figure::ExportScale, *data->bounds);
            });

            QImage image;

            const double t_image = minTimeMS([ & ] ()
            {
                REQUIRE(report_figure::renderExportImage(entry, image));
            });

            //the points are in the image
            const auto drawn = data->seriesInDrawOrder();
            REQUIRE(!drawn.empty());
            CHECK(hasColor(image, drawn.back()->color));

            WARN(label << " " << entry.image_name << " '" << entry.metadata.title_ << "'"
                 << ": series " << data->series.numDataSeries()
                 << " points " << data->series.numPoints(true)
                 << " | display data " << t_data << " ms"
                 << " | points image " << t_points << " ms"
                 << " | export image " << t_image << " ms");
        }
    }
}

// Not run by default. Measures the draw time of report scatterplot images without OpenGL.
// COMPASS_TEST_VIEW_POINTS_FILE (optional): a JSON array of report figure view points, e.g. the
// reconstruction debug figures. Then synthetic figures with 10 series and up to 1000000 points.
TEST_CASE("report scatterplot draw time", "[.][scatterdrawtime]")
{
    const QString file = qEnvironmentVariable("COMPASS_TEST_VIEW_POINTS_FILE");

    if (!file.isEmpty())
    {
        std::ifstream stream(file.toStdString());
        REQUIRE(stream.is_open());

        const auto view_points = nlohmann::json::parse(stream);
        REQUIRE(view_points.is_array());

        for (size_t i = 0; i < view_points.size(); ++i)
            measureScatterDrawTime(view_points[ i ], "figure " + std::to_string(i));
    }

    for (size_t num_points : { 1000, 10000, 100000, 1000000 })
        measureScatterDrawTime(largeScatterViewPoint(num_points), "synthetic");
}

// Not run by default. Export time of a geographic grid figure with the map on, for each map file of
// COMPASS_TEST_MAP_FILES (comma separated, default openstreetmap_2d.earth). The map tiles come from the tile
// cache of the map file or from the network, the export waits for them up to
// gridbackground::DefaultExportWaitTime. The first export loads the tiles, the second finds them in the cache.
TEST_CASE("report geographic grid export time with the map", "[.][geomapexporttime]")
{
    QString files = qEnvironmentVariable("COMPASS_TEST_MAP_FILES");
    if (files.isEmpty())
        files = "openstreetmap_2d.earth";

    auto exportTime = [] (const boost::optional<GridBackgroundSettings>& background, QImage& image)
    {
        const auto entries = report_figure::selectRenderEntries(geoGridViewPoint(background));
        REQUIRE(entries.size() == 1);

        QElapsedTimer timer;
        timer.start();

        REQUIRE(report_figure::renderExportImage(entries[ 0 ], image));

        return timer.elapsed();
    };

    QImage image;
    for (int run = 0; run < 2; ++run)
        WARN("map off run " << run << ": " << exportTime(boost::none, image) << " ms");

    for (const auto& file : files.split(','))
    {
        GridBackgroundSettings background = GridBackgroundSettings::reportDefaults();
        background.map_show    = true;
        background.map_file    = file.trimmed().toStdString();
        background.map_opacity = 100;

        for (int run = 0; run < 2; ++run)
        {
            const qint64 ms = exportTime(background, image);

            WARN(file.toStdString() << " run " << run << ": " << ms << " ms, black pixels " << numPixels(image, Qt::black));

            const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
            if (!dir.isEmpty())
                image.save(QDir(dir).filePath(file.trimmed() + ".png"));
        }
    }
}
