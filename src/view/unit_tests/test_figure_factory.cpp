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
#include "chartview.h"
#include "viewpointgenerator.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
#include <QImage>

#include <QtCharts/QAbstractBarSeries>
#include <QtCharts/QBarSet>
#include <QtCharts/QChart>
#include <QtCharts/QLegend>

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
