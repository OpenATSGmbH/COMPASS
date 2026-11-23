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
#include "scatterplotviewrenderwidget.h"
#include "scatterplotpointsitem.h"
#include "scatterplotdisplaydata.h"
#include "chartview.h"
#include "viewtoolswitcher.h"
#include "apppalette.h"

#include "json.hpp"

#include <QApplication>
#include <QDir>
#include <QImage>
#include <QLabel>
#include <QMouseEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QTest>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <QtCharts/QChart>
#include <QtCharts/QCategoryAxis>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QLegend>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace
{
    const QColor Red   ("#ff0000");
    const QColor Green ("#00ff00");
    const QColor Blue  ("#0000ff");
    const QColor Yellow("#ffff00");

    const std::vector<QColor> SeriesColors = { Red, Green, Blue };

    const QSize LayoutSize(1200, 800);

    typedef std::vector<std::pair<double, double>> Points;

    void addSeries(ScatterPlotDisplayData& data,
                   const std::string& name,
                   const QColor& color,
                   const Points& points,
                   double marker_size = 8.0)
    {
        ScatterSeries series;
        for (const auto& p : points)
            series.points.emplace_back(p.first, p.second);

        data.series.addDataSeries(series, name, color, marker_size);
    }

    /// the state a model gives to data with series
    void finish(ScatterPlotDisplayData& data)
    {
        data.bounds = data.series.getDataBounds();
        data.valid  = data.series.numPoints(true) > 0;
    }

    /**
     * Display data with the given number of series. Series s has the name "series<s>", a color of
     * SeriesColors and the points (0, s), (50, 50 + s), (100, 100 - s). So the bounds are 0 to 100 on
     * both axes.
     */
    std::shared_ptr<ScatterPlotDisplayData> makeData(size_t num_series)
    {
        auto data = std::make_shared<ScatterPlotDisplayData>();

        for (size_t s = 0; s < num_series; ++s)
        {
            QString name = QString("series%1").arg(s, 3, 10, QChar('0'));

            addSeries(*data, name.toStdString(), SeriesColors[ s % SeriesColors.size() ],
                      { { 0.0, (double)s }, { 50.0, 50.0 + s }, { 100.0, 100.0 - s } });
        }

        data->title        = "Test Title";
        data->x_axis_label = "Test X";
        data->y_axis_label = "Test Y";
        data->domain_id    = "domain a";

        finish(*data);

        return data;
    }

    /// two series with the points that the pixel tests look at, inside bounds of 0 to 100 on both axes
    std::shared_ptr<ScatterPlotDisplayData> makePixelData()
    {
        auto data = std::make_shared<ScatterPlotDisplayData>();

        //the corners keep the bounds
        addSeries(*data, "a red" , Red , { { 0.0, 0.0 }, { 25.0, 50.0 }, { 100.0, 100.0 } }, 12.0);
        addSeries(*data, "b blue", Blue, { { 75.0, 50.0 } }, 12.0);

        data->domain_id = "pixels";

        finish(*data);

        return data;
    }

    ScatterPlotRenderOptions viewOptions(bool connection_lines = false)
    {
        ScatterPlotRenderOptions o;
        o.use_connection_lines = connection_lines;

        return o;
    }

    /// the options of a report figure
    ScatterPlotRenderOptions imageOptions(bool connection_lines = false,
                                          ScatterPlotLegendMode legend_mode = ScatterPlotLegendMode::RightSide)
    {
        ScatterPlotRenderOptions o;
        o.use_connection_lines = connection_lines;
        o.use_opengl           = false;
        o.legend_mode          = legend_mode;

        return o;
    }

    QtCharts::QChart* chartOf(ScatterPlotViewRenderWidget& render_widget)
    {
        REQUIRE(render_widget.chartView());
        REQUIRE(render_widget.chartView()->chart());

        return render_widget.chartView()->chart();
    }

    /// the series of the data, without the rectangle of the selection and the series of the legend
    template <typename T>
    QList<T*> dataSeriesOf(ScatterPlotViewRenderWidget& render_widget)
    {
        QList<T*> list;

        for (auto s : chartOf(render_widget)->series())
        {
            if (s == render_widget.chartView()->selectionSeries() || s->name().startsWith("..."))
                continue;

            if (auto series = dynamic_cast<T*>(s))
                list << series;
        }

        return list;
    }

    QtCharts::QScatterSeries* scatterSeriesNamed(ScatterPlotViewRenderWidget& render_widget, const QString& name)
    {
        for (auto s : dataSeriesOf<QtCharts::QScatterSeries>(render_widget))
            if (s->name() == name)
                return s;

        return nullptr;
    }

    QtCharts::QAbstractAxis* axisOf(ScatterPlotViewRenderWidget& render_widget, Qt::Orientation orientation)
    {
        auto axes = chartOf(render_widget)->axes(orientation);
        REQUIRE(axes.size() == 1);

        return axes.first();
    }

    QRectF axisRangesOf(ScatterPlotViewRenderWidget& render_widget)
    {
        auto ranges = render_widget.axisRanges();
        REQUIRE(ranges.has_value());

        return ranges.value();
    }

    /// the axis ranges of zoom to home for bounds of 0 to 100 on both axes
    void checkHomeRanges(const QRectF& ranges)
    {
        CHECK(ranges.left()   == Approx(-3.0));
        CHECK(ranges.right()  == Approx(103.0));
        CHECK(ranges.top()    == Approx(-3.0));
        CHECK(ranges.bottom() == Approx(103.0));
    }

    void checkRanges(const QRectF& ranges, const QRectF& expected)
    {
        CHECK(ranges.left()   == Approx(expected.left()));
        CHECK(ranges.right()  == Approx(expected.right()));
        CHECK(ranges.top()    == Approx(expected.top()));
        CHECK(ranges.bottom() == Approx(expected.bottom()));
    }

    /// the labels of the legend entries that are shown, in the order of the legend
    QStringList shownLegendEntries(ScatterPlotViewRenderWidget& render_widget)
    {
        QStringList labels;

        for (auto marker : chartOf(render_widget)->legend()->markers())
            if (marker->isVisible())
                labels << marker->label();

        return labels;
    }

    size_t numPixels(const QImage& image, const QColor& color, const QRect& rect = QRect())
    {
        const QRect r = rect.isNull() ? image.rect() : rect.intersected(image.rect());

        size_t n = 0;

        for (int y = r.top(); y <= r.bottom(); ++y)
            for (int x = r.left(); x <= r.right(); ++x)
                if (image.pixel(x, y) == color.rgb())
                    ++n;

        return n;
    }

    size_t numColors(const QImage& image)
    {
        std::set<QRgb> colors;

        for (int y = 0; y < image.height(); ++y)
            for (int x = 0; x < image.width(); ++x)
                colors.insert(image.pixel(x, y));

        return colors.size();
    }

    /// the pixels around a position
    QRect around(const QPoint& pos, int distance)
    {
        return QRect(pos.x() - distance, pos.y() - distance, 2 * distance + 1, 2 * distance + 1);
    }

    /**
     * Position of a value in an image of render() with LayoutSize and scale 1. The chart in the widget
     * gets the same layout. Its fonts have another hinting than the fonts of an image, so the position
     * can be off by a pixel.
     */
    QPoint imagePosition(ScatterPlotViewRenderWidget& render_widget, double x, double y)
    {
        render_widget.chartView()->layoutForSize(LayoutSize);

        return chartOf(render_widget)->mapToPosition(QPointF(x, y)).toPoint();
    }

    QImage renderImage(ScatterPlotViewRenderWidget& render_widget, double scale = 1.0)
    {
        QImage image;
        REQUIRE(render_widget.render(image, LayoutSize, scale));

        return image;
    }

    /// sets the application palette, the destructor sets the palette from before again
    struct ApplicationPaletteGuard
    {
        explicit ApplicationPaletteGuard(const QPalette& palette)
        :   palette_before(QApplication::palette())
        {
            QApplication::setPalette(palette);
        }

        ~ApplicationPaletteGuard()
        {
            QApplication::setPalette(palette_before);
        }

        QPalette palette_before;
    };

    void showWidget(QWidget& widget, const QSize& size)
    {
        widget.resize(size);
        widget.show();

        REQUIRE(QTest::qWaitForWindowExposed(&widget));

        for (int i = 0; i < 10; ++i)
            QApplication::processEvents();
    }

    /// a wheel event of the user over the plot area, returns if the chart view used it
    bool wheelOverChart(ScatterPlotViewRenderWidget& render_widget, int angle_delta)
    {
        auto viewport = render_widget.chartView()->viewport();

        const QPointF pos = chartOf(render_widget)->plotArea().center();

        QWheelEvent event(pos, viewport->mapToGlobal(pos.toPoint()), QPoint(), QPoint(0, angle_delta),
                          Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
        event.ignore();

        //as a wheel event of the user: Qt gives only these on to the parent widgets
        QSpontaneKeyEvent::setSpontaneous(&event);
        qApp->notify(viewport, &event);

        return event.isAccepted();
    }

    /// drags the left mouse button over the plot area, positions as fractions of the plot area
    void dragOverChart(ScatterPlotViewRenderWidget& render_widget, const QPointF& from, const QPointF& to)
    {
        auto chart_view = render_widget.chartView();

        const QRectF plot_area = chart_view->chart()->plotArea();

        auto position = [ & ] (const QPointF& f)
        {
            return QPoint((int)(plot_area.left() + f.x() * plot_area.width()),
                          (int)(plot_area.top() + f.y() * plot_area.height()));
        };

        const QPoint p0 = position(from);
        const QPoint p1 = position(to);

        QTest::mousePress(chart_view->viewport(), Qt::LeftButton, Qt::NoModifier, p0);

        QMouseEvent move(QEvent::MouseMove, p1, Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(chart_view->viewport(), &move);

        QTest::mouseRelease(chart_view->viewport(), Qt::LeftButton, Qt::NoModifier, p1);
    }
}

// ---- chart content ------------------------------------------------------------------------------

TEST_CASE("scatter chart shows the series of the display data", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = makeData(3);
    data->series.dataSeries().at("series001").marker_size = 4.0;

    render_widget.setDisplayData(data, viewOptions());

    CHECK(render_widget.isDrawn());
    CHECK(render_widget.hasContent());

    const auto series = dataSeriesOf<QtCharts::QScatterSeries>(render_widget);
    REQUIRE(series.size() == 3);

    for (int s = 0; s < 3; ++s)
    {
        INFO("series " << s);

        auto chart_series = scatterSeriesNamed(render_widget, QString("series00%1").arg(s));
        REQUIRE(chart_series);

        CHECK(chart_series->color() == SeriesColors[ s ]);
        CHECK(chart_series->count() == 3);
        CHECK(chart_series->markerSize() == Approx(s == 1 ? 4.0 : 8.0));
        CHECK(chart_series->useOpenGL());
        CHECK(chart_series->at(1) == QPointF(50.0, 50.0 + s));
    }

    //no connection lines
    CHECK(dataSeriesOf<QtCharts::QLineSeries>(render_widget).isEmpty());
}

TEST_CASE("scatter chart shows title and axis labels", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2), viewOptions());

    CHECK(chartOf(render_widget)->title() == "Test Title");
    CHECK(axisOf(render_widget, Qt::Horizontal)->titleText() == "Test X");
    CHECK(axisOf(render_widget, Qt::Vertical)->titleText() == "Test Y");

    SECTION("the view has no legend, its layer panel lists the series")
    {
        CHECK(chartOf(render_widget)->legend()->maximumSize() == QSizeF(0, 0));
    }
}

TEST_CASE("scatter chart does not draw a hidden series", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = makeData(3);
    data->series.dataSeries().at("series001").visible = false;

    for (bool use_opengl : { true, false })
    {
        INFO("OpenGL " << use_opengl);

        render_widget.setDisplayData(data, use_opengl ? viewOptions() : imageOptions());

        CHECK(dataSeriesOf<QtCharts::QScatterSeries>(render_widget).size() == 2);
        CHECK(scatterSeriesNamed(render_widget, "series000"));
        CHECK(!scatterSeriesNamed(render_widget, "series001"));
        CHECK(scatterSeriesNamed(render_widget, "series002"));
    }
}

TEST_CASE("scatter chart axis follows the data type of the display data", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = makeData(1);

    SECTION("a floating point value gives a value axis")
    {
        render_widget.setDisplayData(data, viewOptions());

        for (auto orientation : { Qt::Horizontal, Qt::Vertical })
        {
            auto axis = axisOf(render_widget, orientation);

            CHECK(dynamic_cast<QtCharts::QValueAxis*>(axis));
            CHECK(!dynamic_cast<QtCharts::QCategoryAxis*>(axis));
        }
    }

    SECTION("the date time flag gives a date time axis")
    {
        //2023-04-22 05:00:00 UTC and 30 minutes later
        const double t0 = 1682139600000.0;
        const double t1 = t0 + 1800000.0;

        auto data_time = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data_time, "series", Red, { { t0, 1.0 }, { t1, 2.0 } });

        data_time->x_axis_is_datetime  = true;
        data_time->axis_data_type[ 0 ] = PropertyDataType::TIMESTAMP;
        finish(*data_time);

        render_widget.setDisplayData(data_time, viewOptions());

        auto axis_x = dynamic_cast<QtCharts::QDateTimeAxis*>(axisOf(render_widget, Qt::Horizontal));
        REQUIRE(axis_x);

        CHECK(dynamic_cast<QtCharts::QValueAxis*>(axisOf(render_widget, Qt::Vertical)));

        //the axis ranges count in milliseconds since epoch, with the border of zoom to home
        const QRectF ranges = axisRangesOf(render_widget);

        CHECK(ranges.left()  == Approx(t0 - 0.03 * 1800000.0));
        CHECK(ranges.right() == Approx(t1 + 0.03 * 1800000.0));

        CHECK(axis_x->format() == "hh:mm:ss");

        SECTION("a zoom below two seconds shows the milliseconds")
        {
            render_widget.zoomToRectangle(QRectF(QPointF(t0, 0.0), QPointF(t0 + 1000.0, 3.0)));

            CHECK(axis_x->format() == "hh:mm:ss.zzz");
        }
    }

    SECTION("a value that Qt shows in another way than COMPASS gives own tick labels")
    {
        data->axis_data_type[ 0 ] = PropertyDataType::UINT;
        data->axis_repr[ 0 ]      = dbContent::Representation::DEC_TO_OCTAL;

        render_widget.setDisplayData(data, viewOptions());

        auto axis_x = dynamic_cast<QtCharts::QCategoryAxis*>(axisOf(render_widget, Qt::Horizontal));
        REQUIRE(axis_x);

        const QStringList labels = axis_x->categoriesLabels();
        CHECK(!labels.isEmpty());

        SECTION("a zoom gives ticks of its own")
        {
            render_widget.zoomToRectangle(QRectF(QPointF(10.0, 0.0), QPointF(20.0, 100.0)));

            CHECK(!axis_x->categoriesLabels().isEmpty());
            CHECK(axis_x->categoriesLabels() != labels);
        }
    }
}

TEST_CASE("scatter connection lines follow the option", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    SECTION("without the option the chart has no line series")
    {
        render_widget.setDisplayData(makeData(2), viewOptions(false));

        CHECK(dataSeriesOf<QtCharts::QLineSeries>(render_widget).isEmpty());
        CHECK(!render_widget.usesConnectionLines());
    }

    SECTION("with the option each series has a line series with its points and color")
    {
        render_widget.setDisplayData(makeData(2), viewOptions(true));

        const auto lines = dataSeriesOf<QtCharts::QLineSeries>(render_widget);
        REQUIRE(lines.size() == 2);

        std::set<QRgb> colors;

        for (auto line : lines)
        {
            CHECK(line->count() == 3);
            CHECK(line->useOpenGL());

            colors.insert(line->color().rgb());
        }

        CHECK(colors == std::set<QRgb>({ Red.rgb(), Green.rgb() }));
        CHECK(render_widget.usesConnectionLines());
    }
}

TEST_CASE("the connection lines of the display data win over the option", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    SECTION("data without lines, option with lines")
    {
        auto data = makeData(2);
        data->series.setUseConnectionLines(false);

        render_widget.setDisplayData(data, viewOptions(true));

        CHECK(!render_widget.usesConnectionLines());
        CHECK(dataSeriesOf<QtCharts::QLineSeries>(render_widget).isEmpty());
    }

    SECTION("data with lines, option without lines")
    {
        auto data = makeData(2);
        data->series.setUseConnectionLines(true);

        render_widget.setDisplayData(data, viewOptions(false));

        CHECK(render_widget.usesConnectionLines());
        CHECK(dataSeriesOf<QtCharts::QLineSeries>(render_widget).size() == 2);
    }
}

TEST_CASE("scatter selection series is on top with OpenGL", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    //in the order of the collection the selected points are not the last series
    auto data = std::make_shared<ScatterPlotDisplayData>();
    addSeries(*data, "Alpha", Red, { { 0.0, 0.0 }, { 100.0, 100.0 } });
    addSeries(*data, ScatterPlotDisplayData::SelectedSeriesName, Yellow, { { 50.0, 50.0 } });
    addSeries(*data, "Zulu", Blue, { { 50.0, 50.0 } });
    finish(*data);

    for (bool connection_lines : { false, true })
    {
        INFO("connection lines " << connection_lines);

        render_widget.setDisplayData(data, viewOptions(connection_lines));

        auto selected = scatterSeriesNamed(render_widget, "Selected");
        REQUIRE(selected);

        //Qt Charts draws accelerated series in ascending pointer order
        for (auto series : dataSeriesOf<QtCharts::QScatterSeries>(render_widget))
        {
            if (series == selected)
                continue;

            INFO("series " << series->name().toStdString());
            CHECK((const void*)series < (const void*)selected);
        }

        //the rectangle of a selection is above all data
        REQUIRE(render_widget.chartView()->selectionSeries());

        //only accelerated series are drawn in pointer order
        for (auto series : chartOf(render_widget)->series())
            if (series->useOpenGL() && series != render_widget.chartView()->selectionSeries())
                CHECK((const void*)series < (const void*)render_widget.chartView()->selectionSeries());
    }
}

TEST_CASE("scatter chart without valid data is the empty chart", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto checkEmptyChart = [ & ] ()
    {
        CHECK(render_widget.isDrawn());
        CHECK(!render_widget.hasContent());

        CHECK(dataSeriesOf<QtCharts::QScatterSeries>(render_widget).size() == 1);
        CHECK(dataSeriesOf<QtCharts::QScatterSeries>(render_widget).first()->count() == 0);

        for (auto orientation : { Qt::Horizontal, Qt::Vertical })
        {
            auto axis = axisOf(render_widget, orientation);

            CHECK(!axis->labelsVisible());
            CHECK(!axis->isGridLineVisible());
        }
    };

    for (bool use_opengl : { true, false })
    {
        INFO("OpenGL " << use_opengl);

        const auto options = use_opengl ? viewOptions() : imageOptions();

        SECTION(std::string("no display data, OpenGL ") + (use_opengl ? "on" : "off"))
        {
            render_widget.setDisplayData({}, options);

            checkEmptyChart();
            CHECK(chartOf(render_widget)->title().isEmpty());

            //the default range
            checkRanges(axisRangesOf(render_widget), QRectF(0.0, 0.0, 1.0, 1.0));
        }

        SECTION(std::string("display data flagged invalid, OpenGL ") + (use_opengl ? "on" : "off"))
        {
            auto data = makeData(2);
            data->valid = false;

            render_widget.setDisplayData(data, options);

            checkEmptyChart();

            //title and axis labels stay
            CHECK(chartOf(render_widget)->title() == "Test Title");
            CHECK(axisOf(render_widget, Qt::Horizontal)->titleText() == "Test X");
        }

        SECTION(std::string("all series hidden, OpenGL ") + (use_opengl ? "on" : "off"))
        {
            auto data = makeData(2);
            for (auto& s : data->series.dataSeries())
                s.second.visible = false;

            render_widget.setDisplayData(data, options);

            checkEmptyChart();
        }

        SECTION(std::string("data after the empty chart, OpenGL ") + (use_opengl ? "on" : "off"))
        {
            render_widget.setDisplayData({}, options);
            render_widget.setDisplayData(makeData(2), options);

            CHECK(render_widget.hasContent());
            CHECK(dataSeriesOf<QtCharts::QScatterSeries>(render_widget).size() == 2);
            CHECK(axisOf(render_widget, Qt::Horizontal)->labelsVisible());
        }
    }
}

TEST_CASE("scatter render widget has no chart before the first display data", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    CHECK(!render_widget.isDrawn());
    CHECK(render_widget.chartView() == nullptr);
    CHECK(!render_widget.axisRanges().has_value());
}

// ---- zoom state ---------------------------------------------------------------------------------

TEST_CASE("scatter axis ranges follow the domain of the display data", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    const QRectF zoom(QPointF(20.0, 30.0), QPointF(60.0, 80.0));

    for (bool use_opengl : { true, false })
    {
        const auto options = use_opengl ? viewOptions() : imageOptions();
        const std::string mode = use_opengl ? ", OpenGL on" : ", OpenGL off";

        SECTION("the first draw with content gives the data bounds" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);

            checkHomeRanges(axisRangesOf(render_widget));
        }

        SECTION("the same domain keeps the axis ranges over a redraw" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);
            render_widget.zoomToRectangle(zoom);

            checkRanges(axisRangesOf(render_widget), zoom);

            //new display data of the same domain, e.g. after a selection
            render_widget.setDisplayData(makeData(3), options);

            checkRanges(axisRangesOf(render_widget), zoom);

            //the same data with another option
            auto options_lines = options;
            options_lines.use_connection_lines = true;

            render_widget.setDisplayData(makeData(3), options_lines);

            checkRanges(axisRangesOf(render_widget), zoom);

            render_widget.redraw();

            checkRanges(axisRangesOf(render_widget), zoom);
        }

        SECTION("another domain gives the data bounds" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);
            render_widget.zoomToRectangle(zoom);

            auto data = makeData(2);
            data->domain_id = "domain b";

            render_widget.setDisplayData(data, options);

            checkHomeRanges(axisRangesOf(render_widget));

            //and back
            render_widget.zoomToRectangle(zoom);
            render_widget.setDisplayData(makeData(2), options);

            checkHomeRanges(axisRangesOf(render_widget));
        }

        SECTION("a draw without content gives the data bounds at the next draw" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);
            render_widget.zoomToRectangle(zoom);

            render_widget.setDisplayData({}, options);
            render_widget.setDisplayData(makeData(2), options);

            checkHomeRanges(axisRangesOf(render_widget));
        }

        SECTION("a clear gives the data bounds at the next draw" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);
            render_widget.zoomToRectangle(zoom);

            render_widget.clear();
            render_widget.setDisplayData(makeData(2), options);

            checkHomeRanges(axisRangesOf(render_widget));
        }

        SECTION("zoom to home gives the data bounds" + mode)
        {
            render_widget.setDisplayData(makeData(2), options);
            render_widget.zoomToRectangle(zoom);
            render_widget.zoomToHome();

            checkHomeRanges(axisRangesOf(render_widget));
        }
    }
}

TEST_CASE("scatter zoom to home without bounds gives the default range", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = makeData(2);
    data->bounds.reset();

    render_widget.setDisplayData(data, viewOptions());

    checkRanges(axisRangesOf(render_widget), QRectF(0.0, 0.0, 1.0, 1.0));
}

TEST_CASE("scatter zoom to home of a single value has a border", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = std::make_shared<ScatterPlotDisplayData>();
    addSeries(*data, "series", Red, { { 5.0, 7.0 } });

    data->bounds = QRectF(QPointF(5.0, 7.0), QPointF(5.0, 7.0));
    data->valid  = true;

    render_widget.setDisplayData(data, viewOptions());

    checkRanges(axisRangesOf(render_widget), QRectF(QPointF(4.9, 6.9), QPointF(5.1, 7.1)));
}

// ---- drawing without OpenGL ---------------------------------------------------------------------

TEST_CASE("scatter points image has the points at their value positions", "[view][scatterplot][scatterrender]")
{
    auto data = makePixelData();

    //2 pixels per value on the x axis, 1 pixel per value on the y axis
    const QSize  size(200, 100);
    const QRectF range(0.0, 0.0, 100.0, 100.0);

    const QImage image = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0, range);

    REQUIRE(image.size() == size);

    SECTION("the pixel at a point has the color of its series")
    {
        //the y axis goes up
        CHECK(image.pixel(50, 50)  == Red.rgb());
        CHECK(image.pixel(150, 50) == Blue.rgb());
    }

    SECTION("a position without a point is transparent")
    {
        CHECK(qAlpha(image.pixel(100, 50)) == 0);
        CHECK(qAlpha(image.pixel(50, 20))  == 0);
        CHECK(qAlpha(image.pixel(150, 80)) == 0);
    }

    SECTION("a marker has the size of its series")
    {
        //diameter 12: the center row is filled over about 12 pixels
        CHECK(image.pixel(50 - 4, 50) == Red.rgb());
        CHECK(image.pixel(50 + 4, 50) == Red.rgb());
        CHECK(qAlpha(image.pixel(50 - 8, 50)) == 0);
        CHECK(qAlpha(image.pixel(50 + 8, 50)) == 0);

        auto data_small = makePixelData();
        data_small->series.dataSeries().at("a red").marker_size = 4.0;

        const QImage image_small = ScatterPlotPointsItem::drawPoints(*data_small, false, size, 1.0, range);

        CHECK(image_small.pixel(50, 50) == Red.rgb());
        CHECK(qAlpha(image_small.pixel(50 - 4, 50)) == 0);
        CHECK(qAlpha(image_small.pixel(50 + 4, 50)) == 0);
    }

    SECTION("the scale gives more pixels per marker")
    {
        const QImage image_2 = ScatterPlotPointsItem::drawPoints(*data, false, size * 2, 2.0, range);

        REQUIRE(image_2.size() == QSize(400, 200));

        CHECK(image_2.pixel(100, 100) == Red.rgb());
        CHECK(image_2.pixel(100 - 9, 100) == Red.rgb());
        CHECK(image_2.pixel(100 + 9, 100) == Red.rgb());
        CHECK(qAlpha(image_2.pixel(100 - 15, 100)) == 0);
    }

    SECTION("a point outside of the value range is not drawn")
    {
        //the range shows the blue point only
        const QImage image_zoom = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0,
                                                                    QRectF(QPointF(60.0, 40.0), QPointF(90.0, 60.0)));

        CHECK(numPixels(image_zoom, Red) == 0);
        CHECK(image_zoom.pixel(100, 50) == Blue.rgb());
    }

    SECTION("a value range without extent gives the empty image")
    {
        const QImage image_empty = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0,
                                                                     QRectF(QPointF(50.0, 0.0), QPointF(50.0, 100.0)));

        CHECK(image_empty.size() == size);
        CHECK(numPixels(image_empty, Red) == 0);
        CHECK(numPixels(image_empty, Blue) == 0);
    }

    SECTION("a value that is no number is not drawn")
    {
        auto data_nan = makePixelData();
        data_nan->series.dataSeries().at("b blue").scatter_series.points.emplace_back(std::nan(""), 50.0);

        const QImage image_nan = ScatterPlotPointsItem::drawPoints(*data_nan, true, size, 1.0, range);

        CHECK(image_nan.pixel(150, 50) == Blue.rgb());
    }
}

TEST_CASE("scatter points image draws the series in the order of the display data", "[view][scatterplot][scatterrender]")
{
    const QSize  size(200, 100);
    const QRectF range(0.0, 0.0, 100.0, 100.0);

    SECTION("a later series covers an earlier one")
    {
        //the collection orders the series by name, not by the order they were added in
        auto data = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data, "b", Blue, { { 50.0, 50.0 } });
        addSeries(*data, "a", Red , { { 50.0, 50.0 }, { 20.0, 50.0 } });
        finish(*data);

        const QImage image = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0, range);

        CHECK(image.pixel(100, 50) == Blue.rgb());
        CHECK(image.pixel(40, 50)  == Red.rgb());
    }

    SECTION("the selection series is on top")
    {
        auto data = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data, "Alpha", Red, { { 50.0, 50.0 } });
        addSeries(*data, ScatterPlotDisplayData::SelectedSeriesName, Yellow, { { 50.0, 50.0 } });
        addSeries(*data, "Zulu", Blue, { { 50.0, 50.0 }, { 80.0, 50.0 } });
        finish(*data);

        const QImage image = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0, range);

        CHECK(image.pixel(100, 50) == Yellow.rgb());
        CHECK(image.pixel(160, 50) == Blue.rgb());
    }

    SECTION("a hidden series is not drawn")
    {
        auto data = makePixelData();
        data->series.dataSeries().at("a red").visible = false;

        const QImage image = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0, range);

        CHECK(numPixels(image, Red) == 0);
        CHECK(image.pixel(150, 50) == Blue.rgb());
    }
}

TEST_CASE("scatter points image draws the connection lines", "[view][scatterplot][scatterrender]")
{
    const QSize  size(200, 100);
    const QRectF range(0.0, 0.0, 100.0, 100.0);

    auto data = std::make_shared<ScatterPlotDisplayData>();
    addSeries(*data, "a", Red , { { 20.0, 30.0 }, { 80.0, 30.0 } });
    addSeries(*data, "b", Blue, { { 20.0, 70.0 }, { 80.0, 70.0 } });
    finish(*data);

    //the middle between the two points of a series
    const QPoint middle_a(100, 70);
    const QPoint middle_b(100, 30);

    SECTION("without lines the middle between two points is empty")
    {
        const QImage image = ScatterPlotPointsItem::drawPoints(*data, false, size, 1.0, range);

        CHECK(qAlpha(image.pixel(middle_a)) == 0);
        CHECK(qAlpha(image.pixel(middle_b)) == 0);
    }

    SECTION("with lines the middle has the color of the series")
    {
        const QImage image = ScatterPlotPointsItem::drawPoints(*data, true, size, 1.0, range);

        CHECK(image.pixel(middle_a) == Red.rgb());
        CHECK(image.pixel(middle_b) == Blue.rgb());

        //the line is 2 pixels wide
        CHECK(qAlpha(image.pixel(middle_a + QPoint(0, 3))) == 0);
        CHECK(qAlpha(image.pixel(middle_a - QPoint(0, 3))) == 0);
    }

    SECTION("a series with one point has no line")
    {
        auto data_single = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data_single, "a", Red, { { 50.0, 50.0 } });
        finish(*data_single);

        const QImage image = ScatterPlotPointsItem::drawPoints(*data_single, true, size, 1.0, range);

        CHECK(image.pixel(100, 50) == Red.rgb());
        CHECK(qAlpha(image.pixel(60, 50)) == 0);
    }
}

TEST_CASE("scatter chart without OpenGL draws the points into the plot area", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setFixedPalette(app_palette::light());

    auto data = makePixelData();

    render_widget.setDisplayData(data, imageOptions());

    SECTION("the series of the chart hold no points and are not accelerated")
    {
        const auto series = dataSeriesOf<QtCharts::QScatterSeries>(render_widget);
        REQUIRE(series.size() == 2);

        for (auto s : series)
        {
            CHECK(s->count() == 0);
            CHECK(!s->useOpenGL());
        }

        CHECK(scatterSeriesNamed(render_widget, "a red")->color() == Red);

        //no accelerated series for the rectangle of a selection
        CHECK(render_widget.chartView()->selectionSeries() == nullptr);
    }

    SECTION("the image has the color of a series at the position of its point")
    {
        const QImage image = renderImage(render_widget);

        const QPoint pos_red  = imagePosition(render_widget, 25.0, 50.0);
        const QPoint pos_blue = imagePosition(render_widget, 75.0, 50.0);

        CHECK(numPixels(image, Red , around(pos_red , 1)) > 0);
        CHECK(numPixels(image, Blue, around(pos_blue, 1)) > 0);

        //no point of the other series there
        CHECK(numPixels(image, Blue, around(pos_red , 12)) == 0);
        CHECK(numPixels(image, Red , around(pos_blue, 12)) == 0);
    }

    SECTION("a position without a point has the background of the plot area")
    {
        const QImage image = renderImage(render_widget);

        //the same image without the series
        for (auto& s : data->series.dataSeries())
            s.second.visible = false;

        data->valid = true;

        auto data_one = makePixelData();
        data_one->series.dataSeries().at("a red").visible = false;

        render_widget.setDisplayData(data_one, imageOptions());

        const QImage image_blue = renderImage(render_widget);

        const QPoint pos_red   = imagePosition(render_widget, 25.0, 50.0);
        const QPoint pos_blue  = imagePosition(render_widget, 75.0, 50.0);
        const QPoint pos_empty = imagePosition(render_widget, 50.0, 25.0);

        //the hidden series is not drawn
        CHECK(numPixels(image_blue, Red) == 0);
        CHECK(numPixels(image_blue, Blue, around(pos_blue, 1)) > 0);

        //without the red point the background shows, as at a position that never had a point
        const QRgb background = image_blue.pixel(pos_red);

        CHECK(background != Red.rgb());
        CHECK(background != Blue.rgb());
        CHECK(image.pixel(pos_empty) == image_blue.pixel(pos_empty));
        CHECK(numPixels(image, Red , around(pos_empty, 12)) == 0);
        CHECK(numPixels(image, Blue, around(pos_empty, 12)) == 0);
    }

    SECTION("the points are inside the plot area")
    {
        const QImage image = renderImage(render_widget);

        render_widget.chartView()->layoutForSize(LayoutSize);
        const QRect plot_area = chartOf(render_widget)->plotArea().toAlignedRect().adjusted(-2, -2, 2, 2);

        //the legend has the series colors too, it is right of the plot area
        const QRect left_part(0, 0, plot_area.right() + 1, image.height());

        CHECK(numPixels(image, Red , left_part) == numPixels(image, Red , plot_area));
        CHECK(numPixels(image, Blue, left_part) == numPixels(image, Blue, plot_area));
        CHECK(numPixels(image, Red , plot_area) > 0);
    }

    SECTION("connection lines are in the image")
    {
        auto data_lines = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data_lines, "a", Red, { { 0.0, 0.0 }, { 20.0, 50.0 }, { 80.0, 50.0 }, { 100.0, 100.0 } });
        finish(*data_lines);

        render_widget.setDisplayData(data_lines, imageOptions(true));

        const QImage image_lines = renderImage(render_widget);

        render_widget.setDisplayData(data_lines, imageOptions(false));

        const QImage image_points = renderImage(render_widget);

        const QPoint middle = imagePosition(render_widget, 50.0, 50.0);

        CHECK(numPixels(image_lines , Red, around(middle, 2)) > 0);
        CHECK(numPixels(image_points, Red, around(middle, 2)) == 0);
    }
}

TEST_CASE("scatter image shows the axis ranges of the display", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto data = makePixelData();

    SECTION("a zoom of the display is in the image")
    {
        render_widget.setDisplayData(data, imageOptions(false, ScatterPlotLegendMode::Hidden));

        CHECK(numPixels(renderImage(render_widget), Red) > 0);
        CHECK(numPixels(renderImage(render_widget), Blue) > 0);

        //the blue point only
        render_widget.zoomToRectangle(QRectF(QPointF(60.0, 40.0), QPointF(90.0, 60.0)));

        const QImage image = renderImage(render_widget);

        CHECK(numPixels(image, Red) == 0);
        CHECK(numPixels(image, Blue) > 0);
    }

    SECTION("a widget for images only shows the data bounds")
    {
        render_widget.setImageOnly(true);
        render_widget.setDisplayData(data, imageOptions(false, ScatterPlotLegendMode::Hidden));

        CHECK(!render_widget.isDrawn());
        CHECK(render_widget.chartView() == nullptr);

        const QImage image = renderImage(render_widget);

        CHECK(numPixels(image, Red) > 0);
        CHECK(numPixels(image, Blue) > 0);
    }
}

// ---- render() -----------------------------------------------------------------------------------

TEST_CASE("scatter render without OpenGL gives the requested image", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(3), imageOptions());

    SECTION("requested size")
    {
        QImage image;
        REQUIRE(render_widget.render(image, QSize(640, 480)));

        CHECK(image.size() == QSize(640, 480));
    }

    SECTION("size 1200 x 800 and scale 2 give 2400 x 1600 pixels")
    {
        const QImage image = renderImage(render_widget, 2.0);

        CHECK(image.size() == QSize(2400, 1600));
        CHECK(image.devicePixelRatio() == Approx(2.0));
    }

    SECTION("not blank and with the colors of the series")
    {
        const QImage image = renderImage(render_widget);

        CHECK(numColors(image) > 3);

        for (const auto& c : SeriesColors)
            CHECK(numPixels(image, c) > 0);
    }

    SECTION("the same input gives the same image twice")
    {
        const QImage image_0 = renderImage(render_widget, 2.0);
        const QImage image_1 = renderImage(render_widget, 2.0);

        CHECK(image_0 == image_1);

        ScatterPlotViewRenderWidget other_widget;
        other_widget.setDisplayData(makeData(3), imageOptions());

        CHECK(renderImage(other_widget, 2.0) == image_0);
    }

    SECTION("another input gives another image")
    {
        const QImage image_0 = renderImage(render_widget);

        render_widget.setDisplayData(makeData(2), imageOptions());

        CHECK(renderImage(render_widget) != image_0);
    }

    SECTION("the scale keeps the layout, a marker has twice the pixels")
    {
        render_widget.setDisplayData(makePixelData(), imageOptions());

        const QImage image_1 = renderImage(render_widget, 1.0);
        const QImage image_2 = renderImage(render_widget, 2.0);

        const QPoint pos = imagePosition(render_widget, 25.0, 50.0);

        CHECK(numPixels(image_1, Red, around(pos, 1)) > 0);
        CHECK(numPixels(image_2, Red, around(pos * 2, 2)) > 0);

        //4 times the pixels of the 3 markers, the border pixels of a marker are blended
        const double ratio = numPixels(image_2, Red) / (double)numPixels(image_1, Red);

        CHECK(ratio > 3.0);
        CHECK(ratio < 5.0);
    }

    SECTION("the empty chart renders too")
    {
        render_widget.setDisplayData({}, imageOptions());

        QImage image;
        CHECK(render_widget.render(image, LayoutSize));
        CHECK(image.size() == LayoutSize);
    }

    SECTION("the image uses the fixed palette")
    {
        QPalette dark;
        dark.setColor(QPalette::Window, QColor(40, 40, 40));
        dark.setColor(QPalette::Base  , QColor(20, 20, 20));

        ApplicationPaletteGuard guard(dark);

        render_widget.setFixedPalette(app_palette::light());

        const QImage image = renderImage(render_widget);

        CHECK(image.pixel(1, 1) == app_palette::light().color(QPalette::Window).rgb());
    }
}

// ---- legend -------------------------------------------------------------------------------------

TEST_CASE("scatter legend at the right side lists the series", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    for (bool use_opengl : { false, true })
    {
        auto options = imageOptions();
        options.use_opengl = use_opengl;

        const std::string mode = use_opengl ? ", OpenGL on" : ", OpenGL off";

        SECTION("few series, all entries are shown" + mode)
        {
            render_widget.setDisplayData(makeData(4), options);
            render_widget.chartView()->layoutForSize(LayoutSize);

            auto legend = chartOf(render_widget)->legend();

            CHECK(legend->isVisible());
            CHECK(legend->alignment() == Qt::AlignRight);
            CHECK(shownLegendEntries(render_widget) == QStringList({ "series000", "series001", "series002", "series003" }));
        }

        SECTION("many series, the last entry tells the count of the series that do not fit" + mode)
        {
            const int num_series = 100;

            render_widget.setDisplayData(makeData(num_series), options);
            render_widget.chartView()->layoutForSize(LayoutSize);

            const QStringList entries = shownLegendEntries(render_widget);

            REQUIRE(entries.size() > 2);
            REQUIRE(entries.size() < num_series);

            const int num_shown = entries.size() - 1;

            for (int i = 0; i < num_shown; ++i)
                CHECK(entries[ i ] == QString("series%1").arg(i, 3, 10, QChar('0')));

            CHECK(entries.last() == QString("... (%1 more)").arg(num_series - num_shown));
        }

        SECTION("the selection series has no entry" + mode)
        {
            auto data = makeData(2);
            addSeries(*data, ScatterPlotDisplayData::SelectedSeriesName, Yellow, { { 50.0, 50.0 } });
            finish(*data);

            render_widget.setDisplayData(data, options);
            render_widget.chartView()->layoutForSize(LayoutSize);

            CHECK(shownLegendEntries(render_widget) == QStringList({ "series000", "series001" }));
        }

        SECTION("connection lines have no entry" + mode)
        {
            auto options_lines = options;
            options_lines.use_connection_lines = true;

            render_widget.setDisplayData(makeData(2), options_lines);
            render_widget.chartView()->layoutForSize(LayoutSize);

            CHECK(shownLegendEntries(render_widget) == QStringList({ "series000", "series001" }));
        }

        SECTION("a hidden series has no entry" + mode)
        {
            auto data = makeData(3);
            data->series.dataSeries().at("series001").visible = false;

            render_widget.setDisplayData(data, options);
            render_widget.chartView()->layoutForSize(LayoutSize);

            CHECK(shownLegendEntries(render_widget) == QStringList({ "series000", "series002" }));
        }

        SECTION("the chart without data has no legend" + mode)
        {
            render_widget.setDisplayData({}, options);

            CHECK(chartOf(render_widget)->legend()->maximumSize() == QSizeF(0, 0));
        }
    }
}

TEST_CASE("scatter render draws the legend at the right side", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    /// number of pixels with a series color in the columns right of the plot area
    auto numLegendPixels = [ & ] (const ScatterPlotRenderOptions& render_options, size_t num_series)
    {
        render_widget.setDisplayData(makeData(num_series), render_options);

        const QImage image = renderImage(render_widget);

        render_widget.chartView()->layoutForSize(LayoutSize);
        const int plot_right = (int)std::ceil(chartOf(render_widget)->plotArea().right()) + 2;

        REQUIRE(plot_right < image.width() - 1);

        const QRect right_part(plot_right + 1, 0, image.width() - plot_right - 1, image.height());

        size_t num_pixels = 0;

        for (const auto& c : SeriesColors)
            num_pixels += numPixels(image, c, right_part);

        return num_pixels;
    };

    SECTION("few series")
    {
        CHECK(numLegendPixels(imageOptions(), 3) > 0);
    }

    SECTION("many series")
    {
        CHECK(numLegendPixels(imageOptions(), 100) > 0);
    }

    SECTION("without the legend the plot area reaches to the right border")
    {
        CHECK(numLegendPixels(imageOptions(false, ScatterPlotLegendMode::Hidden), 3) == 0);
    }
}

// ---- tools and mouse ----------------------------------------------------------------------------

TEST_CASE("scatter render widget registers the navigate tool", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    auto& switcher = render_widget.toolSwitcher();

    //a host without a toolbar gets no other tool, and so no shortcuts
    REQUIRE(switcher.registeredTools().size() == 1);
    CHECK(switcher.registeredTools().front().kind == ViewTool::Kind::Mode);
    CHECK(switcher.registeredTools().front().key.isEmpty());
    CHECK(switcher.currentTool() == SP_NAVIGATE_TOOL);

    CHECK(render_widget.dragEnabled());

    SECTION("the host registers the zoom tools")
    {
        render_widget.registerZoomRectangleTool();
        render_widget.registerZoomHomeTool();

        REQUIRE(switcher.registeredTools().size() == 3);

        auto zoom = switcher.registeredTool(SP_ZOOM_RECT_TOOL);
        REQUIRE(zoom);
        CHECK(zoom->kind == ViewTool::Kind::Mode);
        CHECK(zoom->name == "Zoom to Rectangle");

        auto zoom_home = switcher.registeredTool(SP_ZOOM_HOME_TOOL);
        REQUIRE(zoom_home);
        CHECK(zoom_home->kind == ViewTool::Kind::Trigger);
        CHECK(zoom_home->name == "Zoom to Home");

        //navigate stays the default tool
        CHECK(switcher.currentTool() == SP_NAVIGATE_TOOL);
    }

    SECTION("the zoom to rectangle tool asks for a rectangle and takes it as axis ranges")
    {
        render_widget.registerZoomRectangleTool();
        render_widget.setDisplayData(makeData(2), viewOptions());

        CHECK(!render_widget.rectangleRequested());

        switcher.setCurrentTool(SP_ZOOM_RECT_TOOL);

        CHECK(render_widget.rectangleRequested());
        CHECK(!render_widget.dragEnabled());

        const QRectF zoom(QPointF(20.0, 30.0), QPointF(60.0, 80.0));

        render_widget.rectangleSelected(zoom);
        QCoreApplication::processEvents();

        checkRanges(axisRangesOf(render_widget), zoom);

        //back to the default tool
        CHECK(switcher.currentTool() == SP_NAVIGATE_TOOL);
        CHECK(render_widget.dragEnabled());
        CHECK(!render_widget.rectangleRequested());
    }

    SECTION("the zoom to home tool shows the data bounds")
    {
        render_widget.registerZoomHomeTool();
        render_widget.setDisplayData(makeData(2), viewOptions());
        render_widget.zoomToRectangle(QRectF(QPointF(20.0, 30.0), QPointF(60.0, 80.0)));

        auto zoom_home = switcher.registeredTool(SP_ZOOM_HOME_TOOL);
        REQUIRE(zoom_home);
        REQUIRE(zoom_home->callback);

        zoom_home->callback(true);

        checkHomeRanges(axisRangesOf(render_widget));
    }
}

TEST_CASE("scatter rectangle of any corner order becomes the axis ranges", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2), viewOptions());

    //from the bottom right to the top left
    render_widget.zoomToRectangle(QRectF(QPointF(60.0, 80.0), QPointF(20.0, 30.0)));

    checkRanges(axisRangesOf(render_widget), QRectF(QPointF(20.0, 30.0), QPointF(60.0, 80.0)));
}

TEST_CASE("scatter chart mouse drags and rectangles", "[view][scatterplot][scatterrender]")
{
    //without OpenGL: the offscreen platform of the tests shows no accelerated series
    ScatterPlotViewRenderWidget render_widget;
    render_widget.registerZoomRectangleTool();
    render_widget.setDisplayData(makeData(2), imageOptions(false, ScatterPlotLegendMode::Hidden));

    showWidget(render_widget, QSize(800, 600));

    const QRectF home = axisRangesOf(render_widget);

    SECTION("with the navigate tool a drag moves the axis ranges")
    {
        dragOverChart(render_widget, QPointF(0.5, 0.5), QPointF(0.25, 0.5));
        QCoreApplication::processEvents();

        const QRectF ranges = axisRangesOf(render_widget);

        //the content moves to the left, so the axis shows higher values
        CHECK(ranges.left() > home.left() + 10.0);
        CHECK(ranges.width() == Approx(home.width()).epsilon(0.02));
        CHECK(ranges.top() == Approx(home.top()).margin(1.0));
    }

    SECTION("without drag a drag changes nothing")
    {
        render_widget.setDragEnabled(false);

        dragOverChart(render_widget, QPointF(0.5, 0.5), QPointF(0.25, 0.5));
        QCoreApplication::processEvents();

        checkRanges(axisRangesOf(render_widget), home);
    }

    SECTION("with the zoom tool the rectangle of the mouse becomes the axis ranges")
    {
        render_widget.toolSwitcher().setCurrentTool(SP_ZOOM_RECT_TOOL);

        dragOverChart(render_widget, QPointF(0.25, 0.25), QPointF(0.75, 0.75));

        //queued
        checkRanges(axisRangesOf(render_widget), home);

        QCoreApplication::processEvents();

        const QRectF ranges = axisRangesOf(render_widget);

        //the middle half of the home ranges
        CHECK(ranges.left()   == Approx(home.left() + 0.25 * home.width()).margin(1.0));
        CHECK(ranges.right()  == Approx(home.left() + 0.75 * home.width()).margin(1.0));
        CHECK(ranges.top()    == Approx(home.top() + 0.25 * home.height()).margin(1.0));
        CHECK(ranges.bottom() == Approx(home.top() + 0.75 * home.height()).margin(1.0));

        CHECK(render_widget.toolSwitcher().currentTool() == SP_NAVIGATE_TOOL);
    }
}

TEST_CASE("scatter chart mouse wheel", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2), imageOptions(false, ScatterPlotLegendMode::Hidden));

    SECTION("the wheel zooms the chart of a view")
    {
        showWidget(render_widget, QSize(800, 600));

        const QRectF home = axisRangesOf(render_widget);

        wheelOverChart(render_widget, 120);

        const QRectF ranges_in = axisRangesOf(render_widget);

        CHECK(ranges_in.width() < home.width());
        CHECK(ranges_in.height() < home.height());

        wheelOverChart(render_widget, -120);
        wheelOverChart(render_widget, -120);

        CHECK(axisRangesOf(render_widget).width() > home.width());
    }

    SECTION("a widget that ignores the wheel does not zoom")
    {
        render_widget.setWheelIgnored(true);

        showWidget(render_widget, QSize(800, 600));

        const QRectF home = axisRangesOf(render_widget);

        CHECK(!wheelOverChart(render_widget, 120));
        CHECK(!wheelOverChart(render_widget, -120));

        checkRanges(axisRangesOf(render_widget), home);
    }

    SECTION("in a scroll area the wheel scrolls the area and the chart keeps its axis ranges")
    {
        auto figure = new ScatterPlotViewRenderWidget;
        figure->setWheelIgnored(true);
        figure->setHeightForWidth(2.0 / 3.0, 400);
        figure->setDisplayData(makeData(2), imageOptions());

        auto section = new QWidget;

        auto layout = new QVBoxLayout;
        layout->addWidget(new QLabel("Figure: Test"));
        layout->addWidget(figure);
        layout->addStretch();

        section->setLayout(layout);

        QScrollArea scroll_area;
        scroll_area.setWidgetResizable(true);
        scroll_area.setWidget(section);

        showWidget(scroll_area, QSize(900, 300));

        auto scroll_bar = scroll_area.verticalScrollBar();
        REQUIRE(scroll_bar->maximum() > 0);
        REQUIRE(scroll_bar->value() == 0);

        const QRectF home = axisRangesOf(*figure);

        wheelOverChart(*figure, -120);

        CHECK(scroll_bar->value() > 0);
        checkRanges(axisRangesOf(*figure), home);
    }
}

// ---- view info ----------------------------------------------------------------------------------

TEST_CASE("scatter view info lists the chart content", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;
    render_widget.setDisplayData(makeData(2), viewOptions(true));

    nlohmann::json info;
    render_widget.viewInfoJSON(info);

    REQUIRE(info.contains("chart"));

    const auto& chart = info[ "chart" ];

    CHECK(chart[ "x_axis_label" ] == "Test X");
    CHECK(chart[ "y_axis_label" ] == "Test Y");
    CHECK(chart[ "x_axis_type"  ] == "value");
    CHECK(chart[ "y_axis_type"  ] == "value");
    CHECK(chart[ "has_connection_lines" ] == true);

    //per series the points and the lines, without the rectangle of the selection
    CHECK(chart[ "num_series" ] == 4);
    REQUIRE(chart[ "series" ].size() == 4);

    CHECK(chart[ "series" ][ 0 ][ "type"  ] == "points");
    CHECK(chart[ "series" ][ 0 ][ "name"  ] == "series000");
    CHECK(chart[ "series" ][ 0 ][ "count" ] == 3);
    CHECK(chart[ "series" ][ 0 ][ "color" ] == "#ff0000");
    CHECK(chart[ "series" ][ 1 ][ "type"  ] == "lines");
    CHECK(chart[ "series" ][ 2 ][ "name"  ] == "series001");

    CHECK(info[ "axis_bounds_valid" ] == true);
    CHECK(info[ "axis_bounds_xmin"  ].get<double>() == Approx(-3.0));
    CHECK(info[ "axis_bounds_xmax"  ].get<double>() == Approx(103.0));
    CHECK(info[ "axis_zoom_active"  ] == false);

    SECTION("a zoom is active if the axis ranges do not show all data")
    {
        render_widget.zoomToRectangle(QRectF(QPointF(20.0, 30.0), QPointF(60.0, 80.0)));

        nlohmann::json info_zoom;
        render_widget.viewInfoJSON(info_zoom);

        CHECK(info_zoom[ "axis_zoom_active" ] == true);
        CHECK(info_zoom[ "axis_bounds_ymin" ].get<double>() == Approx(30.0));
        CHECK(info_zoom[ "axis_bounds_ymax" ].get<double>() == Approx(80.0));
    }

    SECTION("the selected points are listed in the order of the collection")
    {
        auto data = makeData(1);
        addSeries(*data, "Alpha", Blue, { { 10.0, 10.0 } });
        addSeries(*data, ScatterPlotDisplayData::SelectedSeriesName, Yellow, { { 50.0, 50.0 } });
        finish(*data);

        render_widget.setDisplayData(data, viewOptions(false));

        nlohmann::json info_selected;
        render_widget.viewInfoJSON(info_selected);

        const auto& series = info_selected[ "chart" ][ "series" ];
        REQUIRE(series.size() == 3);

        CHECK(series[ 0 ][ "name" ] == "Alpha");
        CHECK(series[ 1 ][ "name" ] == "Selected");
        CHECK(series[ 2 ][ "name" ] == "series000");
    }
}

TEST_CASE("scatter view info without a chart adds nothing", "[view][scatterplot][scatterrender]")
{
    ScatterPlotViewRenderWidget render_widget;

    nlohmann::json info;
    render_widget.viewInfoJSON(info);

    CHECK(info.is_null());
}

// Not run by default. Writes images of scatterplot charts into the directory in COMPASS_TEST_IMAGE_DIR.
// scripts/check_image_display_scaling.py runs it with the display scaling 1 and 2 and compares the images.
// The check needs a platform with real font settings (wayland, xcb): on the offscreen platform of the
// unit tests the images are equal also with fonts that depend on the display scaling.
TEST_CASE("scatterplot images for the display scaling check", "[.][displayscaling]")
{
    const QString dir = qEnvironmentVariable("COMPASS_TEST_IMAGE_DIR");
    REQUIRE(!dir.isEmpty());

    auto save = [ & ] (std::shared_ptr<ScatterPlotDisplayData> data, bool connection_lines, const QString& name)
    {
        ScatterPlotViewRenderWidget render_widget;
        render_widget.setFixedPalette(app_palette::light());
        render_widget.setImageOnly(true);
        render_widget.setDisplayData(data, imageOptions(connection_lines));

        QImage image;
        REQUIRE(render_widget.render(image, LayoutSize, 2.0));
        REQUIRE(image.save(QDir(dir).filePath(name)));
    };

    SECTION("value axes with a rotated y axis title")
    {
        save(makeData(4), true, "scatterplot.png");
    }

    SECTION("own tick labels on the x axis, many series in the legend")
    {
        auto data = makeData(60);
        data->axis_data_type[ 0 ] = PropertyDataType::DOUBLE;
        data->axis_repr[ 0 ]      = dbContent::Representation::SECONDS_TO_TIME;

        save(data, false, "scatterplot_time_of_day.png");
    }

    SECTION("date time axis with rotated tick labels")
    {
        //2023-04-22 05:00:00 UTC and 30 hours later, the axis shows the day
        const double t0 = 1682139600000.0;
        const double t1 = t0 + 30.0 * 3600000.0;

        auto data = std::make_shared<ScatterPlotDisplayData>();
        addSeries(*data, "Radar A", Red , { { t0, 1.0 }, { (t0 + t1) / 2.0, 3.0 }, { t1, 2.0 } });
        addSeries(*data, "Radar B", Blue, { { t0, 2.0 }, { t1, 1.5 } });

        data->title               = "Bias over Time";
        data->x_axis_label        = "Time";
        data->y_axis_label        = "Bias [m]";
        data->x_axis_is_datetime  = true;
        data->axis_data_type[ 0 ] = PropertyDataType::TIMESTAMP;

        finish(*data);

        save(data, false, "scatterplot_datetime.png");
    }
}
