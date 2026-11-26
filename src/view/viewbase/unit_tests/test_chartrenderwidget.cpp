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
#include "chartrenderwidget.h"
#include "chartview.h"
#include "viewtoolswitcher.h"

#include <QCoreApplication>
#include <QImage>

#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>

#include <memory>
#include <vector>

namespace
{
    class TestChartView : public ChartView
    {
    public:
        explicit TestChartView(QtCharts::QChart* chart)
        :   ChartView(chart, ChartView::SelectionStyle::RubberBand, nullptr) {}

    protected:
        bool handleMousePress  (Qt::MouseButtons, const QPointF&      ) override { return false; }
        bool handleMouseRelease(Qt::MouseButtons, const QPointF&, bool) override { return false; }
        bool handleMouseMove   (Qt::MouseButtons, const QPointF&      ) override { return false; }
    };

    /**
     * ChartRenderWidget is abstract, this one shows a line chart.
     */
    class TestChartRenderWidget : public ChartRenderWidget
    {
    public:
        bool with_content = true;

    protected:
        std::unique_ptr<ChartView> createChartView(bool& has_content) override
        {
            auto chart = new QtCharts::QChart;

            auto series = new QtCharts::QLineSeries;
            series->append(0.0, 0.0);
            series->append(1.0, 1.0);
            chart->addSeries(series);
            chart->createDefaultAxes();

            has_content = with_content;

            return std::make_unique<TestChartView>(chart);
        }
    };

    enum ToolId
    {
        Navigate = 0,
        Select,
        Zoom
    };

    ViewTool modeTool(int id)
    {
        ViewTool tool;
        tool.id   = id;
        tool.kind = ViewTool::Kind::Mode;

        return tool;
    }

    /// a select tool as a view registers it: the next rectangle goes to the given list
    ViewTool rectangleTool(int id, ChartRenderWidget& render_widget, std::vector<QRectF>& rectangles)
    {
        ViewTool tool = modeTool(id);
        tool.callback = [ &render_widget, &rectangles ] (bool on)
        {
            if (on)
                render_widget.requestRectangle([ &rectangles ] (const QRectF& r) { rectangles.push_back(r); });
        };

        return tool;
    }
}

TEST_CASE("a rectangle without a request calls nothing", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    CHECK(!render_widget.rectangleRequested());

    //must not crash and must leave no request behind
    render_widget.rectangleSelected(QRectF(1.0, 2.0, 3.0, 4.0));
    QCoreApplication::processEvents();

    CHECK(!render_widget.rectangleRequested());

    //a request made later does not get the old rectangle
    std::vector<QRectF> rectangles;
    render_widget.requestRectangle([ &rectangles ] (const QRectF& r) { rectangles.push_back(r); });
    QCoreApplication::processEvents();

    CHECK(rectangles.empty());
}

TEST_CASE("a requested rectangle reaches the callback once, queued", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    std::vector<QRectF> rectangles;
    render_widget.requestRectangle([ &rectangles ] (const QRectF& r) { rectangles.push_back(r); });

    REQUIRE(render_widget.rectangleRequested());

    const QRectF rect(1.5, -2.0, 3.25, 4.0);
    render_widget.rectangleSelected(rect);

    //the callback can build the chart again, so it does not run inside the mouse event
    CHECK(rectangles.empty());

    QCoreApplication::processEvents();

    //the chart coordinates arrive unchanged
    REQUIRE(rectangles.size() == 1);
    CHECK(rectangles[ 0 ] == rect);
    CHECK(!render_widget.rectangleRequested());

    SECTION("the next rectangle has no receiver")
    {
        render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));
        QCoreApplication::processEvents();

        CHECK(rectangles.size() == 1);
    }
}

TEST_CASE("two rectangles before the delivery give one callback", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    std::vector<QRectF> rectangles;
    render_widget.requestRectangle([ &rectangles ] (const QRectF& r) { rectangles.push_back(r); });

    render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));
    render_widget.rectangleSelected(QRectF(5.0, 5.0, 1.0, 1.0));
    QCoreApplication::processEvents();

    REQUIRE(rectangles.size() == 1);
    CHECK(rectangles[ 0 ] == QRectF(0.0, 0.0, 1.0, 1.0));
}

TEST_CASE("the default mode tool is active after the rectangle", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    std::vector<QRectF> rectangles;

    render_widget.registerTool(modeTool(Navigate));
    render_widget.registerTool(rectangleTool(Select, render_widget, rectangles));

    auto& switcher = render_widget.toolSwitcher();

    switcher.setCurrentTool(Select);
    REQUIRE(switcher.currentTool() == Select);
    REQUIRE(render_widget.rectangleRequested());

    render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));

    //the tool stays until the rectangle is delivered
    CHECK(switcher.currentTool() == Select);

    QCoreApplication::processEvents();

    CHECK(rectangles.size() == 1);
    CHECK(switcher.currentTool() == Navigate);
    CHECK(!render_widget.rectangleRequested());
}

TEST_CASE("a change of the mode tool drops the request", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    std::vector<QRectF> select_rectangles, zoom_rectangles;

    render_widget.registerTool(modeTool(Navigate));
    render_widget.registerTool(rectangleTool(Select, render_widget, select_rectangles));
    render_widget.registerTool(rectangleTool(Zoom, render_widget, zoom_rectangles));

    auto& switcher = render_widget.toolSwitcher();

    SECTION("back to the default tool")
    {
        switcher.setCurrentTool(Select);
        switcher.endCurrentTool();

        CHECK(!render_widget.rectangleRequested());

        render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));
        QCoreApplication::processEvents();

        CHECK(select_rectangles.empty());
    }

    SECTION("to another rectangle tool")
    {
        switcher.setCurrentTool(Select);
        switcher.setCurrentTool(Zoom);

        REQUIRE(render_widget.rectangleRequested());

        render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));
        QCoreApplication::processEvents();

        CHECK(select_rectangles.empty());
        CHECK(zoom_rectangles.size() == 1);
    }

    SECTION("a tool change between rectangle and delivery")
    {
        switcher.setCurrentTool(Select);

        render_widget.rectangleSelected(QRectF(0.0, 0.0, 1.0, 1.0));
        switcher.endCurrentTool();
        QCoreApplication::processEvents();

        CHECK(select_rectangles.empty());
    }
}

TEST_CASE("redraw and clear set the draw state and the chart view", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    CHECK(!render_widget.isDrawn());
    CHECK(!render_widget.hasContent());
    CHECK(render_widget.chartView() == nullptr);

    render_widget.redraw();

    CHECK(render_widget.isDrawn());
    CHECK(render_widget.hasContent());
    REQUIRE(render_widget.chartView());

    //the UI tests address the chart view by this name
    CHECK(render_widget.chartView()->objectName() == "chart_view");
    CHECK(render_widget.findChild<ChartView*>("chart_view") == render_widget.chartView());

    SECTION("a chart without content is drawn, but has no content")
    {
        render_widget.with_content = false;
        render_widget.redraw();

        CHECK(render_widget.isDrawn());
        CHECK(!render_widget.hasContent());
    }

    SECTION("a redraw replaces the chart view")
    {
        render_widget.redraw();

        CHECK(render_widget.findChildren<ChartView*>("chart_view").size() == 1);
    }

    SECTION("clear removes the chart")
    {
        render_widget.clear();

        CHECK(!render_widget.isDrawn());
        CHECK(!render_widget.hasContent());
        CHECK(render_widget.chartView() == nullptr);
    }

    SECTION("the display can be shown without counting as drawn")
    {
        render_widget.resetDrawState();

        CHECK(!render_widget.isDrawn());
        CHECK(render_widget.chartView() != nullptr);
    }
}

TEST_CASE("render gives an image of the requested size", "[view][chartrenderwidget]")
{
    TestChartRenderWidget render_widget;

    QImage image;

    SECTION("scale 1")
    {
        REQUIRE(render_widget.render(image, QSize(600, 400)));

        CHECK(image.size() == QSize(600, 400));
    }

    SECTION("scale 2 keeps the layout size and doubles the pixels")
    {
        REQUIRE(render_widget.render(image, QSize(600, 400), 2.0));

        CHECK(image.size() == QSize(1200, 800));
        CHECK(image.devicePixelRatio() == Approx(2.0));
    }

    SECTION("an empty size gives no image")
    {
        CHECK(!render_widget.render(image, QSize(0, 400)));
        CHECK(image.isNull());
    }

    SECTION("the widget on the screen is not touched")
    {
        render_widget.redraw();
        auto chart_view = render_widget.chartView();

        REQUIRE(render_widget.render(image, QSize(600, 400)));

        CHECK(render_widget.chartView() == chart_view);
    }
}
