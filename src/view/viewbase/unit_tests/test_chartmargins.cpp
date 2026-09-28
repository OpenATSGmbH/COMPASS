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
#include "chartview.h"

#include <QtCharts/QChart>
#include <QtCharts/QCategoryAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>
#include <QFontMetricsF>

#include <memory>

namespace
{
    /**
     * A chart with one horizontal category axis carrying the given labels.
     */
    std::unique_ptr<QtCharts::QChart> chartWithLabels(const QStringList& labels,
                                                      int labels_angle = 0)
    {
        auto chart = std::make_unique<QtCharts::QChart>();

        auto series = new QtCharts::QLineSeries;
        series->append(0.0, 0.0);
        series->append((qreal)labels.size(), 1.0);
        chart->addSeries(series);

        auto axis = new QtCharts::QCategoryAxis;
        axis->setLabelsPosition(QtCharts::QCategoryAxis::AxisLabelsPositionOnValue);
        axis->setLabelsAngle(labels_angle);

        chart->addAxis(axis, Qt::AlignBottom);
        series->attachAxis(axis);

        axis->setRange(0.0, (qreal)labels.size());
        axis->setStartValue(-0.001);

        for (int i = 0; i < labels.size(); ++i)
            axis->append(labels[ i ], (qreal)(i + 1));

        return chart;
    }

    qreal halfLabelWidth(const QtCharts::QChart* chart, const QString& label)
    {
        auto axis = chart->axes(Qt::Horizontal).first();
        return QFontMetricsF(axis->labelsFont()).horizontalAdvance(label) / 2.0;
    }
}

TEST_CASE("chart margins hold a wide outermost label", "[view][chartmargins]")
{
    //a time of day label is wider than the default margin of 20, so half of it
    //would reach past the plot area and be cut off
    const QStringList labels = { "10:00:00", "10:05:00", "10:10:00", "10:15:00" };

    auto chart = chartWithLabels(labels);

    ChartView::reserveHorizontalLabelMargins(chart.get());

    const qreal half_first = halfLabelWidth(chart.get(), labels.first());
    const qreal half_last  = halfLabelWidth(chart.get(), labels.last() );

    CHECK((qreal)chart->margins().left()  >= half_first);
    CHECK((qreal)chart->margins().right() >= half_last );
}

TEST_CASE("chart margins never fall below the default", "[view][chartmargins]")
{
    //a short label needs less room than the default, which must still be kept
    auto chart = chartWithLabels({ "1", "2", "3" });

    ChartView::reserveHorizontalLabelMargins(chart.get());

    CHECK(chart->margins().left()  >= 20);
    CHECK(chart->margins().right() >= 20);
}

TEST_CASE("chart margins do not creep across relabelings", "[view][chartmargins]")
{
    auto chart_wide = chartWithLabels({ "2026-09-23 10:00:00", "2026-09-23 11:00:00" });
    ChartView::reserveHorizontalLabelMargins(chart_wide.get());

    const int wide_right = chart_wide->margins().right();
    CHECK(wide_right > 20);

    //the same chart relabeled with short labels must give the room back, a zoom
    //must not leave a wide margin behind
    auto axis = dynamic_cast<QtCharts::QCategoryAxis*>(chart_wide->axes(Qt::Horizontal).first());
    REQUIRE(axis);

    for (const auto& l : axis->categoriesLabels())
        axis->remove(l);

    axis->setStartValue(-0.001);
    axis->append("1", 1.0);
    axis->append("2", 2.0);

    ChartView::reserveHorizontalLabelMargins(chart_wide.get());

    CHECK(chart_wide->margins().right() < wide_right);
    CHECK(chart_wide->margins().right() >= 20);
}

TEST_CASE("chart margins ignore a rotated label", "[view][chartmargins]")
{
    //a label rotated upright does not reach out sideways
    auto chart = chartWithLabels({ "2026-09-23 10:00:00", "2026-09-23 11:00:00" }, 85);

    ChartView::reserveHorizontalLabelMargins(chart.get());

    CHECK(chart->margins().right() == 20);
}

TEST_CASE("chart margins leave a plain value axis alone", "[view][chartmargins]")
{
    //Qt handles the labels of a value axis itself
    auto chart = std::make_unique<QtCharts::QChart>();

    auto series = new QtCharts::QLineSeries;
    series->append(0.0, 0.0);
    series->append(1.0, 1.0);
    chart->addSeries(series);
    chart->createDefaultAxes();

    const auto margins_before = chart->margins();

    ChartView::reserveHorizontalLabelMargins(chart.get());

    CHECK(chart->margins() == margins_before);
}
