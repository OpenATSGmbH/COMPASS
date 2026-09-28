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
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QRubberBand>

#include <memory>

namespace
{
    /**
     * ChartView is abstract, this is the smallest concrete one.
     */
    class TestChartView : public ChartView
    {
    public:
        TestChartView(QtCharts::QChart* chart, ChartView::SelectionStyle style)
        :   ChartView(chart, style, nullptr) {}

    protected:
        bool handleMousePress  (Qt::MouseButtons, const QPointF&      ) override { return false; }
        bool handleMouseRelease(Qt::MouseButtons, const QPointF&, bool) override { return false; }
        bool handleMouseMove   (Qt::MouseButtons, const QPointF&      ) override { return false; }
    };

    /**
     * A chart carrying one accelerated data series, as the scatter plot builds it.
     */
    QtCharts::QChart* chartWithGLData(int num_series = 1)
    {
        auto chart = new QtCharts::QChart;

        for (int i = 0; i < num_series; ++i)
        {
            auto data = new QtCharts::QScatterSeries;
            data->append(0.0, 0.0);
            data->append(1.0, 1.0);
            data->setUseOpenGL(true);
            chart->addSeries(data);
        }

        chart->createDefaultAxes();

        return chart;
    }

    std::unique_ptr<TestChartView> makeView(ChartView::SelectionStyle style,
                                            int num_series = 1)
    {
        auto view = std::make_unique<TestChartView>(chartWithGLData(num_series), style);
        view->resize(400, 300);

        return view;
    }

    void checkSelectionOnTop(const TestChartView& view)
    {
        REQUIRE(view.selectionSeries());

        const void* selection = (const void*)view.selectionSeries();

        for (auto s : view.chart()->series())
        {
            if (s == view.selectionSeries())
                continue;

            INFO("series at " << (const void*)s << " vs selection at " << selection);
            CHECK((const void*)s < selection);
        }
    }
}

TEST_CASE("the line selection is a series of the chart", "[view][chartselection]")
{
    auto view = makeView(ChartView::SelectionStyle::SeriesLines);

    // the series used to be created but never added, which left the whole
    // style unusable
    REQUIRE(view->selectionSeries());
    CHECK(view->chart()->series().contains(
              const_cast<QtCharts::QAbstractSeries*>(view->selectionSeries())));
}

TEST_CASE("the line selection outranks the data in pointer order", "[view][chartselection]")
{
    // Qt Charts renders accelerated series in ascending pointer order, so the
    // selection is only on top if its pointer is the highest of the chart. This
    // is what made the rectangle disappear behind the pooled selection overlay.
    auto view = makeView(ChartView::SelectionStyle::SeriesLines);

    checkSelectionOnTop(*view);
}

TEST_CASE("the line selection is accelerated like the data", "[view][chartselection]")
{
    auto view = makeView(ChartView::SelectionStyle::SeriesLines);

    auto line_series = dynamic_cast<const QtCharts::QLineSeries*>(view->selectionSeries());

    REQUIRE(line_series);

    // a series without acceleration is rendered into the scene, which the
    // OpenGL widget of the data covers
    CHECK(line_series->useOpenGL());
}

TEST_CASE("the line selection is attached to both axes", "[view][chartselection]")
{
    auto view = makeView(ChartView::SelectionStyle::SeriesLines);

    auto series = const_cast<QtCharts::QAbstractSeries*>(view->selectionSeries());
    REQUIRE(series);

    // without axes the selection would not follow the data coordinates
    CHECK(series->attachedAxes().size() == 2);
}

TEST_CASE("the line selection starts hidden", "[view][chartselection]")
{
    auto view = makeView(ChartView::SelectionStyle::SeriesLines);

    REQUIRE(view->selectionSeries());
    CHECK(!view->selectionSeries()->isVisible());
}

TEST_CASE("the rubber band style keeps no selection series", "[view][chartselection]")
{
    auto view = makeView(ChartView::SelectionStyle::RubberBand);

    // the grid view still uses this style, it has no accelerated series
    CHECK(view->selectionSeries() == nullptr);

    auto rubber_band = view->findChild<QRubberBand*>();
    REQUIRE(rubber_band);
    CHECK(!rubber_band->isVisible());
}
