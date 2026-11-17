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
#include "scatterplotpointsitem.h"
#include "viewtoolswitcher.h"
#include "chartview.h"
#include "viewpointgenerator.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
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
        vp.noDataLoaded(true);
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
        vp.noDataLoaded(true);
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
        vp.noDataLoaded(true);
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
