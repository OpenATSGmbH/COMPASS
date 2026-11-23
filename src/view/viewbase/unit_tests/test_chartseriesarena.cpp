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
#include "chartseriesarena.h"

#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

#include <memory>
#include <vector>

TEST_CASE("a selection series is above data series allocated after it", "[view][chartseriesarena]")
{
    // Qt Charts draws accelerated series in ascending pointer order, the order of
    // the allocation must not matter
    std::unique_ptr<QtCharts::QLineSeries> selection(chart_series_arena::createSelectionLineSeries());

    // heap churn of the same size, as a chart rebuild leaves it
    std::vector<std::unique_ptr<QtCharts::QLineSeries>> heap;
    for (int i = 0; i < 1000; ++i)
        heap.emplace_back(new QtCharts::QLineSeries);

    std::vector<std::unique_ptr<QtCharts::QAbstractSeries>> data;
    for (int i = 0; i < 100; ++i)
    {
        data.emplace_back(chart_series_arena::createDataScatterSeries());
        data.emplace_back(chart_series_arena::createDataLineSeries());
    }

    CHECK(chart_series_arena::inSelectionRegion(selection.get()));

    for (const auto& s : data)
    {
        CHECK(chart_series_arena::inDataRegion(s.get()));
        CHECK((const void*)s.get() < (const void*)selection.get());
    }
}

TEST_CASE("arena series are normal series of their type", "[view][chartseriesarena]")
{
    std::unique_ptr<QtCharts::QAbstractSeries> scatter(chart_series_arena::createDataScatterSeries());
    std::unique_ptr<QtCharts::QAbstractSeries> line   (chart_series_arena::createDataLineSeries());
    std::unique_ptr<QtCharts::QAbstractSeries> sel    (chart_series_arena::createSelectionLineSeries());

    CHECK(qobject_cast<QtCharts::QScatterSeries*>(scatter.get()));
    CHECK(qobject_cast<QtCharts::QLineSeries*>(line.get()));
    CHECK(qobject_cast<QtCharts::QLineSeries*>(sel.get()));

    CHECK(scatter->type() == QtCharts::QAbstractSeries::SeriesTypeScatter);
    CHECK(line->type()    == QtCharts::QAbstractSeries::SeriesTypeLine);
}

TEST_CASE("a chart returns its arena series on delete", "[view][chartseriesarena]")
{
    const size_t num_data      = chart_series_arena::numDataSeries();
    const size_t num_selection = chart_series_arena::numSelectionSeries();

    {
        QtCharts::QChart chart;

        for (int i = 0; i < 10; ++i)
        {
            auto s = chart_series_arena::createDataScatterSeries();
            s->append(0.0, 0.0);
            chart.addSeries(s);
        }

        chart.addSeries(chart_series_arena::createSelectionLineSeries());

        CHECK(chart_series_arena::numDataSeries()      == num_data + 10);
        CHECK(chart_series_arena::numSelectionSeries() == num_selection + 1);
    }

    // the chart deleted its series, the slots are free again
    CHECK(chart_series_arena::numDataSeries()      == num_data);
    CHECK(chart_series_arena::numSelectionSeries() == num_selection);
}
