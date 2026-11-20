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
#include "histograminitializer.h"
#include "chartview.h"
#include "viewpointgenerator.h"

#include "json.hpp"

#include <QDir>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QImage>
#include <QPainter>

#include <QtCharts/QBarCategoryAxis>
#include <QtCharts/QChart>
#include <QtCharts/QLegend>
#include <QtCharts/QLogValueAxis>
#include <QtCharts/QValueAxis>

namespace
{
    /// size of a report figure in the Reports tool
    const QSize GuiSize(664, 443);

    /// error values like the position errors of an evaluation: a huge first bin, a thin tail up to 3850 m
    std::vector<double> errorValues()
    {
        std::vector<double> values;

        for (int i = 0; i < 126000; ++i)
            values.push_back(190.0 * i / 126000.0);

        const std::vector<int> tail_counts = { 60, 25, 12, 0, 5, 3, 0, 1, 2 };

        for (size_t k = 0; k < tail_counts.size(); ++k)
            for (int i = 0; i < tail_counts[ k ]; ++i)
                values.push_back(385.0 * (k + 1) + 100.0);

        values.push_back(3850.0);

        return values;
    }

    RawHistogram errorHistogram(unsigned int num_bins)
    {
        RawHistogram h;
        HistogramInitializerT<double> init;
        REQUIRE(init.createRAW(h, errorValues(), false, num_bins));

        return h;
    }

    RawHistogram errorCountHistogram()
    {
        RawHistogram h;
        HistogramInitializerT<std::string> init;
        REQUIRE(init.createRAW(h, { "#CF", "#CP" }, { 217, 126608 }, false));

        return h;
    }

    /// view point like the Sector Overview of Horizontal Position Accuracy: two histograms of one unnamed series
    nlohmann::json evaluationViewPoint()
    {
        RawHistogramCollection counts;
        counts.addDataSeries(errorCountHistogram(), "", QColor(0, 0, 255));

        RawHistogramCollection distribution;
        distribution.addDataSeries(errorHistogram(10), "", QColor(0, 0, 255));

        ViewPointGenVP vp("Sector Overview", 0, "Histogram");
        vp.annotations().getOrCreateAnnotation("Error Count")->addFeature(
            new ViewPointGenFeatureHistogram(counts, PlotMetadata("Position Error", "Error Count", "Error Count", "")));
        vp.annotations().getOrCreateAnnotation("Full distribution")->addFeature(
            new ViewPointGenFeatureHistogram(distribution, PlotMetadata("Position Error", "Full distribution",
                                                                        "Full distribution [m]", "")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    nlohmann::json viewPoint(const RawHistogramCollection& collection)
    {
        ViewPointGenVP vp("Histogram", 0, "Histogram");
        vp.annotations().getOrCreateAnnotation("Histogram")->addFeature(
            new ViewPointGenFeatureHistogram(collection, PlotMetadata("Histogram", "", "Value", "Count")));

        nlohmann::json j;
        vp.toJSON(j);

        return j;
    }

    /// the report widget of the only histogram of the view point, laid out for the size
    std::unique_ptr<ViewRenderWidget> layoutWidget(const report_figure::RenderEntry& entry, const QSize& size)
    {
        auto widget = report_figure::createRenderWidget(entry);
        REQUIRE(widget);

        auto histogram_widget = dynamic_cast<HistogramViewRenderWidget*>(widget.get());
        REQUIRE(histogram_widget);
        REQUIRE(histogram_widget->chartView());

        histogram_widget->chartView()->layoutForSize(size);

        return widget;
    }

    QtCharts::QChart* chartOf(ViewRenderWidget& widget)
    {
        return dynamic_cast<HistogramViewRenderWidget&>(widget).chartView()->chart();
    }

    bool hasLogAxis(QtCharts::QChart* chart)
    {
        const auto axes = chart->axes(Qt::Vertical);
        REQUIRE(axes.size() == 1);

        return dynamic_cast<QtCharts::QLogValueAxis*>(axes.first()) != nullptr;
    }

    /// the texts of the axis labels the chart shows, as Qt Charts draws them (elided with "...")
    QStringList shownTexts(QtCharts::QChart* chart)
    {
        QStringList texts;

        for (auto item : chart->scene()->items())
            if (auto text_item = dynamic_cast<QGraphicsTextItem*>(item))
                if (text_item->isVisible())
                    texts << text_item->toPlainText();

        return texts;
    }

    QStringList categories(QtCharts::QChart* chart)
    {
        auto axis = dynamic_cast<QtCharts::QBarCategoryAxis*>(chart->axes(Qt::Horizontal).first());
        REQUIRE(axis);

        return axis->categories();
    }

    /// saves the image of the chart as the report GUI shows it, if COMPASS_TEST_IMAGE_DIR is set
    void saveGuiImage(ViewRenderWidget& widget, const QString& name)
    {
        const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
        if (dir.isEmpty())
            return;

        auto chart_view = dynamic_cast<HistogramViewRenderWidget&>(widget).chartView();

        QImage image(GuiSize, QImage::Format_ARGB32_Premultiplied);
        image.fill(Qt::white);

        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);

        const QRectF rect(QPointF(0, 0), QSizeF(GuiSize));
        chart_view->scene()->render(&painter, rect, rect);
        chart_view->paintImageItems(painter);
        painter.end();

        REQUIRE(image.save(QDir(dir).filePath(name)));
    }

    void saveExportImage(const report_figure::RenderEntry& entry, const QString& name)
    {
        QImage image;
        REQUIRE(report_figure::renderExportImage(entry, image));

        const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
        if (!dir.isEmpty())
            REQUIRE(image.save(QDir(dir).filePath(name)));
    }
}

TEST_CASE("report histogram of an evaluation: log axis, no legend, readable bin labels", "[view][reportfigure]")
{
    const auto entries = report_figure::selectRenderEntries(evaluationViewPoint());
    REQUIRE(entries.size() == 2);

    for (const auto& entry : entries)
    {
        for (const QSize& size : { GuiSize, report_figure::ExportLayoutSize })
        {
            auto widget = layoutWidget(entry, size);
            auto chart  = chartOf(*widget);

            INFO(chart->title().toStdString() << " at " << size.width() << " x " << size.height());

            CHECK(!chart->legend()->isVisible());
            CHECK(hasLogAxis(chart));

            const QStringList texts = shownTexts(chart);

            for (const auto& label : categories(chart))
                CHECK(texts.contains(label));

            for (const auto& text : texts)
                CHECK(!text.endsWith("..."));

            if (size == GuiSize)
                saveGuiImage(*widget, QString::fromStdString(entry.image_name) + "_gui.png");
        }

        saveExportImage(entry, QString::fromStdString(entry.image_name) + "_export.png");
    }
}

TEST_CASE("report histogram of one named series shows the legend", "[view][reportfigure]")
{
    RawHistogramCollection collection;
    collection.addDataSeries(errorHistogram(10), "Radar A", QColor(0, 0, 255));

    const auto entries = report_figure::selectRenderEntries(viewPoint(collection));
    REQUIRE(entries.size() == 1);

    auto widget = layoutWidget(entries[ 0 ], GuiSize);
    auto chart  = chartOf(*widget);

    CHECK(chart->legend()->isVisible());
    CHECK(hasLogAxis(chart));
}

TEST_CASE("report histogram of stacked series has a linear axis", "[view][reportfigure]")
{
    RawHistogramCollection collection;
    collection.addDataSeries(errorHistogram(10), "Contributed", QColor(192, 57, 43));
    collection.addDataSeries(errorHistogram(10), "Did not contribute", QColor(41, 128, 185));

    const auto entries = report_figure::selectRenderEntries(viewPoint(collection));
    REQUIRE(entries.size() == 1);

    auto widget = layoutWidget(entries[ 0 ], GuiSize);
    auto chart  = chartOf(*widget);

    CHECK(!hasLogAxis(chart));
    CHECK(chart->legend()->isVisible());

    SECTION("a scale set by the annotation wins")
    {
        collection.setUseLogScale(true);

        const auto entries_log = report_figure::selectRenderEntries(viewPoint(collection));
        REQUIRE(entries_log.size() == 1);

        auto widget_log = layoutWidget(entries_log[ 0 ], GuiSize);

        CHECK(hasLogAxis(chartOf(*widget_log)));
    }
}
