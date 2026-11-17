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

#include "scatterplotviewrenderwidget.h"
#include "scatterplotviewchartview.h"
#include "scatterplotpointsitem.h"
#include "axisticks.h"
#include "files.h"
#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

#include <QtCharts/QChart>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QValueAxis>
#include <QtCharts/QCategoryAxis>
#include <QtCharts/QDateTimeAxis>

#include <QAction>
#include <QDateTime>
#include <QGraphicsLayout>

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

QT_CHARTS_USE_NAMESPACE

namespace
{
    /**
     * Fills the axis with tick positions and labels for its current range. Called once on creation
     * and on every range change, since a zoom needs ticks of its own.
     */
    void updateAxisTicks(QCategoryAxis* axis, PropertyDataType dtype, dbContent::Representation repr)
    {
        traced_assert(axis);

        //drop the ticks of the previous range, categories are keyed by their label
        const auto old_labels = axis->categoriesLabels();
        for (const auto& l : old_labels)
            axis->remove(l);

        const double vmin = axis->min();
        const double vmax = axis->max();

        auto ticks       = axis_ticks::generate(vmin, vmax, dtype, repr);
        auto tick_labels = axis_ticks::labels(ticks, dtype, repr);

        traced_assert(ticks.values.size() == tick_labels.size());

        //a category must end above the start value, so keep the start just below
        const double span  = std::max(std::fabs(vmax - vmin), 1e-9);
        const double start = vmin - span * 1e-9;

        axis->setStartValue(start);

        std::set<QString> used;

        for (size_t i = 0; i < ticks.values.size(); ++i)
        {
            if (ticks.values[ i ] <= start)
                continue;

            QString label = QString::fromStdString(tick_labels[ i ]);

            //Qt keys categories by label and drops a duplicate without a word
            if (used.count(label))
                continue;

            used.insert(label);

            axis->append(label, ticks.values[ i ]);
        }
    }

    /**
     * Picks the date time format from the span the axis shows.
     */
    void updateDateTimeFormat(QDateTimeAxis* axis)
    {
        traced_assert(axis);

        const auto dt0 = axis->min();
        const auto dt1 = axis->max();

        QString format = "hh:mm:ss";
        int     angle  = 0;

        if (dt0.isValid() && dt1.isValid())
        {
            const qint64 span_ms = dt0.msecsTo(dt1);

            if (dt0.date().year() != dt1.date().year())
            {
                format = "yyyy-MM-dd hh:mm:ss";
                angle  = -90;
            }
            else if (dt0.date().month() != dt1.date().month())
            {
                format = "MMM dd hh:mm:ss";
                angle  = -90;
            }
            else if (dt0.date().day() != dt1.date().day())
            {
                format = "ddd hh:mm:ss";
                angle  = -90;
            }
            else if (span_ms > 0 && span_ms < 2000)
            {
                //a zoom below two seconds needs the milliseconds
                format = "hh:mm:ss.zzz";
            }
        }

        if (axis->format() != format)
            axis->setFormat(format);

        if (axis->labelsAngle() != angle)
            axis->setLabelsAngle(angle);
    }

    /**
     * The x and the y axis for the display data, attached to all series of the chart.
     * !take care: this may assert if it is called when no series has yet been added to the chart!
     */
    void createAxes(QChart* chart, const ScatterPlotDisplayData* data)
    {
        auto genDateTimeAxis = [ & ] (const std::string& title)
        {
            auto axis = new QDateTimeAxis;
            axis->setTitleText(QString::fromStdString(title));

            //the format follows the span the axis shows, so a zoom into one
            //second does not keep a day wide format
            QObject::connect(axis, &QDateTimeAxis::rangeChanged, axis,
                             [ axis ] (QDateTime, QDateTime) { updateDateTimeFormat(axis); });

            updateDateTimeFormat(axis);

            return dynamic_cast<QAbstractAxis*>(axis);
        };

        auto genValueAxis = [ & ] (const std::string& title, int axis_id)
        {
            const auto dtype = data ? data->axis_data_type[ axis_id ] : PropertyDataType::DOUBLE;
            const auto repr  = data ? data->axis_repr[ axis_id ]      : dbContent::Representation::STANDARD;

            //a plain floating point value is already presented correctly by Qt
            if (!axis_ticks::needsCustomLabels(dtype, repr))
            {
                auto axis = new QValueAxis;
                axis->setTitleText(QString::fromStdString(title));

                return dynamic_cast<QAbstractAxis*>(axis);
            }

            //everything else needs tick labels of our own. QCategoryAxis derives
            //from QValueAxis, so the value coordinate system stays intact and
            //zoom, selection and range handling keep working unchanged
            auto axis = new QCategoryAxis;
            axis->setTitleText(QString::fromStdString(title));
            axis->setLabelsPosition(QCategoryAxis::AxisLabelsPositionOnValue);

            QObject::connect(axis, &QCategoryAxis::rangeChanged, axis,
                             [ chart, axis, dtype, repr ] (qreal, qreal)
                             {
                                 updateAxisTicks(axis, dtype, repr);

                                 //the labels changed, so the room they need changed too
                                 ::ChartView::reserveHorizontalLabelMargins(chart);
                             });

            updateAxisTicks(axis, dtype, repr);

            return dynamic_cast<QAbstractAxis*>(axis);
        };

        auto createAxis = [ & ] (int axis_id,
                                 const std::string& title,
                                 bool is_date_time,
                                 Qt::Alignment alignment)
        {
            QAbstractAxis* axis = is_date_time ? genDateTimeAxis(title) : genValueAxis(title, axis_id);
            traced_assert(axis);

            chart->addAxis(axis, alignment);

            for (auto series : chart->series())
                series->attachAxis(axis);
        };

        createAxis(0, data ? data->x_axis_label : std::string(), data && data->x_axis_is_datetime, Qt::AlignBottom);
        createAxis(1, data ? data->y_axis_label : std::string(), data && data->y_axis_is_datetime, Qt::AlignLeft);

        traced_assert(chart->axes(Qt::Horizontal).size() == 1);
        traced_assert(chart->axes(Qt::Vertical).size() == 1);

        ::ChartView::reserveHorizontalLabelMargins(chart);
    }

    /**
     * vmin and vmax are expected to be in the chart's coordinate system.
     */
    void setAxisRange(QAbstractAxis* axis, double vmin, double vmax)
    {
        traced_assert(axis);

        //handle datetime axis
        auto axis_dt = dynamic_cast<QDateTimeAxis*>(axis);
        if (axis_dt)
        {
            // convert to datetime
            QDateTime dt_min = QDateTime::fromMSecsSinceEpoch(vmin);
            QDateTime dt_max = QDateTime::fromMSecsSinceEpoch(vmax);

            logdbg << "ts min "
                   << dt_min.toString().toStdString()
                   << " max " << dt_max.toString().toStdString();

            axis_dt->setMin(dt_min);
            axis_dt->setMax(dt_max);

            return;
        }

        //default range
        axis->setRange(vmin, vmax);
    }

    /**
     * ranges: rectangle in chart coordinates.
     */
    void setAxisRanges(QChart* chart, const QRectF& ranges)
    {
        auto axes_x = chart->axes(Qt::Horizontal);
        auto axes_y = chart->axes(Qt::Vertical);

        if (axes_x.empty() || axes_y.empty())
            return;

        setAxisRange(axes_x.first(), ranges.left(), ranges.right());
        setAxisRange(axes_y.first(), ranges.top(), ranges.bottom());
    }

    /**
     * The axis ranges of zoom to home: the data bounds with a border, the default range without bounds.
     */
    QRectF homeAxisRanges(const boost::optional<QRectF>& bounds)
    {
        if (!bounds.has_value())
            return QRectF(0.0, 0.0, 1.0, 1.0);

        double x0 = bounds->left();
        double x1 = bounds->right();
        double y0 = bounds->top();
        double y1 = bounds->bottom();
        double w  = bounds->width();
        double h  = bounds->height();

        double bx = w < 1e-12 ? 0.1 : w * 0.03;
        double by = h < 1e-12 ? 0.1 : h * 0.03;

        return QRectF(QPointF(x0 - bx, y0 - by), QPointF(x1 + bx, y1 + by));
    }

    /**
     * The series of the chart for the drawing with OpenGL, as the Scatterplot View needs it for millions
     * of points. Per visible series a scatter series, and a line series with connection lines.
     * no_legend: gets the series that are no legend entry (lines, selected points).
     */
    void addOpenGLSeries(QChart* chart,
                         const ScatterPlotDisplayData& data,
                         bool use_connection_lines,
                         std::vector<const QAbstractSeries*>& no_legend)
    {
        struct SymbolLineSeries { QScatterSeries* scatter_series;
                                  QLineSeries* line_series;};

        //generate needed series and sort pointers
        //qtcharts when rendering opengl will sort the series into a map using the pointer of the series as a key
        //we exploit this behavior to obtain a fixed render order for our data

        const auto& data_series = data.series.dataSeries();

        //generate pointers
        std::vector<SymbolLineSeries> series;

        //add a SymbolLineSeries per visible data series (might contain selection)
        for (auto& series_it : data_series)
        {
            if (!series_it.second.visible)
                continue;

            series.push_back(SymbolLineSeries());

            auto& s = series.back();

            s.scatter_series = new QScatterSeries;
            s.scatter_series->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            s.scatter_series->setUseOpenGL(true);

            if (use_connection_lines)
            {
                s.line_series = new QLineSeries;
                QPen pen = s.line_series->pen();
                pen.setWidthF(ScatterPlotPointsItem::ConnectionLineWidth);
                s.line_series->setPen(pen);

                s.line_series->setUseOpenGL(true);
            }
            else
            {
                s.line_series = nullptr;
            }
        }

        //sort pointers to obtain fixed render order (as it happens inside qtcharts)
        //!the highest pointer is always on top, so the selection series is always assumed to be the last one!
        std::sort(series.begin(), series.end(),
            [&](const SymbolLineSeries& a, const SymbolLineSeries& b) {return a.scatter_series < b.scatter_series; });

        //now add data to series
        size_t cnt = 0;
        for (auto& series_it : data_series)
        {
            if (!series_it.second.visible)
                continue;

            const bool selected = series_it.second.name == ScatterPlotDisplayData::SelectedSeriesName;

            //selection always on top (=highest index)
            auto& s = selected ? series.back() : series[cnt++];

            const auto& ds = series_it.second;

            QScatterSeries* chart_symbol_series = s.scatter_series;
            chart_symbol_series->setColor(ds.color);
            chart_symbol_series->setMarkerSize(ds.marker_size);

            QLineSeries* chart_line_series = nullptr;

            if (use_connection_lines)
            {
                chart_line_series = s.line_series;
                chart_line_series->setColor(ds.color);
            }

            //hand the points to the chart in one replace instead of one append per
            //point: per point appends realloc the series' internal list and signal
            //per call, which showed up as 0.5 GB of allocation churn on large loads
            {
                QList<QPointF> chart_points;
                chart_points.reserve((int)ds.scatter_series.points.size());

                for (const auto& pos : ds.scatter_series.points)
                    chart_points.append(QPointF(pos.x(), pos.y()));

                if (use_connection_lines)
                {
                    traced_assert(chart_line_series);
                    chart_line_series->replace(chart_points);
                }

                chart_symbol_series->replace(std::move(chart_points));
            }

            chart_symbol_series->setName(ds.name.c_str());
            chart->addSeries(chart_symbol_series);

            if (selected)
                no_legend.push_back(chart_symbol_series);

            if (use_connection_lines)
            {
                traced_assert(chart_line_series);
                chart->addSeries(chart_line_series);

                no_legend.push_back(chart_line_series);

                logdbg << "connection lines " << chart_line_series->count();
            }
        }
    }

    /**
     * The series of the chart for the drawing without OpenGL. They hold no points, a ScatterPlotPointsItem
     * draws them. A series gives the axes something to attach to and the legend its entry.
     */
    void addLegendSeries(QChart* chart, const ScatterPlotDisplayData& data)
    {
        for (const auto* ds : data.seriesInDrawOrder())
        {
            //the selected points get no legend entry
            if (ds->name == ScatterPlotDisplayData::SelectedSeriesName)
                continue;

            auto series = new QScatterSeries;
            series->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            series->setColor(ds->color);
            series->setMarkerSize(ds->marker_size);
            series->setName(ds->name.c_str());

            chart->addSeries(series);
        }
    }
}

/**
 */
ScatterPlotViewRenderWidget::ScatterPlotViewRenderWidget(QWidget* parent)
:   ChartRenderWidget(parent)
{
    ViewTool navigate;
    navigate.id       = SP_NAVIGATE_TOOL;
    navigate.name     = "Navigate";
    navigate.cursor   = Qt::ArrowCursor;
    navigate.kind     = ViewTool::Kind::Mode;
    navigate.callback = [ this ] (bool on) { setDragEnabled(on); };

    registerTool(navigate);
}

/**
 */
ScatterPlotViewRenderWidget::~ScatterPlotViewRenderWidget() = default;

/**
 */
void ScatterPlotViewRenderWidget::registerZoomRectangleTool()
{
    ViewTool zoom;
    zoom.id       = SP_ZOOM_RECT_TOOL;
    zoom.name     = "Zoom to Rectangle";
    zoom.key      = Qt::Key_R;
    zoom.icon     = Utils::Files::IconProvider::getIcon("zoom_select_action.png");
    zoom.cursor   = Qt::CrossCursor;
    zoom.kind     = ViewTool::Kind::Mode;
    zoom.update   = [ this ] (QAction* a) { a->setEnabled(isDrawn()); };
    zoom.callback = [ this ] (bool on)
    {
        if (on)
            requestRectangle([ this ] (const QRectF& rect) { zoomToRectangle(rect); });
    };

    registerTool(zoom);
}

/**
 */
void ScatterPlotViewRenderWidget::registerZoomHomeTool()
{
    ViewTool zoom_home;
    zoom_home.id       = SP_ZOOM_HOME_TOOL;
    zoom_home.name     = "Zoom to Home";
    zoom_home.key      = Qt::Key_Space;
    zoom_home.icon     = Utils::Files::IconProvider::getIcon("zoom_home.png");
    zoom_home.kind     = ViewTool::Kind::Trigger;
    zoom_home.callback = [ this ] (bool) { zoomToHome(); };

    registerTool(zoom_home);
}

/**
 */
void ScatterPlotViewRenderWidget::setDisplayData(std::shared_ptr<const ScatterPlotDisplayData> data,
                                                 const ScatterPlotRenderOptions& options)
{
    data_    = std::move(data);
    options_ = options;

    redraw();
}

/**
 */
bool ScatterPlotViewRenderWidget::usesConnectionLines() const
{
    if (data_ && data_->series.useConnectionLines().has_value())
        return data_->series.useConnectionLines().value();

    return options_.use_connection_lines;
}

/**
 * A redraw caused by e.g. a selection change must not throw away the zoom of the user. Another
 * annotation or another axis variable is another value domain, there the ranges of before would hide
 * parts of the new data. The ranges of a chart without content are the meaningless default.
 */
void ScatterPlotViewRenderWidget::captureAxisRanges()
{
    const bool keep = hasContent() && data_ && data_->domain_id == drawn_domain_id_;

    kept_axis_ranges_ = keep ? axisRanges() : boost::optional<QRectF>();
}

/**
 */
void ScatterPlotViewRenderWidget::redraw()
{
    //before the chart in the widget is released
    captureAxisRanges();

    ChartRenderWidget::redraw();

    drawn_domain_id_ = data_ ? data_->domain_id : std::string();
}

/**
 */
bool ScatterPlotViewRenderWidget::render(QImage& image, const QSize& size, double scale)
{
    captureAxisRanges();

    return ChartRenderWidget::render(image, size, scale);
}

/**
 */
boost::optional<QRectF> ScatterPlotViewRenderWidget::axisRanges() const
{
    return chartView() ? ScatterPlotPointsItem::axisRanges(chartView()->chart()) : boost::optional<QRectF>();
}

/**
 */
void ScatterPlotViewRenderWidget::zoomToRectangle(const QRectF& chart_rect)
{
    loginf;

    //TODO: prevent from going nuts when zero rect is passed!

    if (chartView() && chartView()->chart())
        setAxisRanges(chartView()->chart(), chart_rect.normalized());
}

/**
 */
void ScatterPlotViewRenderWidget::zoomToHome()
{
    loginf;

    if (chartView() && chartView()->chart())
        setAxisRanges(chartView()->chart(), homeAxisRanges(data_ ? data_->bounds : boost::optional<QRectF>()));
}

/**
 */
std::unique_ptr<ChartView> ScatterPlotViewRenderWidget::createChartView(bool& has_content)
{
    const bool has_data    = data_ && data_->valid && data_->series.numPoints(true) > 0;
    const bool show_legend = has_data && options_.legend_mode == ScatterPlotLegendMode::RightSide;

    QChart* chart = new QChart();
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->setDropShadowEnabled(false);
    chart->setTitle(data_ ? QString::fromStdString(data_->title) : QString());

    std::vector<const QAbstractSeries*> no_legend;

    if (has_data)
    {
        if (options_.use_opengl)
            addOpenGLSeries(chart, *data_, usesConnectionLines(), no_legend);
        else
            addLegendSeries(chart, *data_);
    }

    //!add at least one series first! (otherwise createAxes() will crash)
    if (chart->series().isEmpty())
    {
        QScatterSeries* series = new QScatterSeries;
        chart->addSeries(series);

        no_legend.push_back(series);
    }

    createAxes(chart, data_.get());

    if (!has_data)
    {
        //no data, bad data range or vars not in buffer -> generate default empty layout
        logdbg << "content empty";

        chart->axes(Qt::Horizontal).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setMinorGridLineVisible(false);

        chart->axes(Qt::Vertical).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Vertical).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Vertical).at(0)->setMinorGridLineVisible(false);
    }

    chart->update();

    //the layer panel of a view is its legend
    if (!show_legend)
        chart->legend()->setMaximumSize(0,0);

    auto chart_view = std::make_unique<ScatterPlotViewChartView>(this, chart, options_.use_opengl);

    for (auto series_it : chart->series())
    {
        QScatterSeries* scat_series = dynamic_cast<QScatterSeries*>(series_it);
        if (!scat_series)
            continue;

        connect (scat_series, &QScatterSeries::pressed,
                 chart_view.get(), &ScatterPlotViewChartView::seriesPressedSlot);
        connect (scat_series, &QScatterSeries::released,
                 chart_view.get(), &ScatterPlotViewChartView::seriesReleasedSlot);
    }

    //owned by the chart
    if (has_data && !options_.use_opengl)
        new ScatterPlotPointsItem(chart, data_, usesConnectionLines());

    if (show_legend)
    {
        for (auto series : no_legend)
            chart_view->excludeFromLegend(series);

        chart_view->showLegendAtRightSide();
    }

    setAxisRanges(chart, kept_axis_ranges_.has_value() ? kept_axis_ranges_.value()
                                                       : homeAxisRanges(data_ ? data_->bounds : boost::optional<QRectF>()));

    has_content = has_data;

    return chart_view;
}

/**
 * The margins for the outermost tick labels follow the label widths, so they are computed again
 * with the final fonts.
 */
void ScatterPlotViewRenderWidget::finishChartView(ChartView& chart_view)
{
    ChartView::reserveHorizontalLabelMargins(chart_view.chart());
}

/**
 */
void ScatterPlotViewRenderWidget::viewInfoJSON(nlohmann::json& info) const
{
    auto chart_view = chartView();
    if (!chart_view)
        return;

    const boost::optional<QRectF> bounds      = data_ ? data_->bounds : boost::optional<QRectF>();
    const boost::optional<QRectF> axis_ranges = axisRanges();

    auto zoomActive = [ & ] (const QRectF& bounds_data, const QRectF& bounds_axis)
    {
        if (!bounds_data.isValid() || !bounds_axis.isValid())
            return false;

        return (bounds_axis.left()   > bounds_data.left()  ||
                bounds_axis.right()  < bounds_data.right() ||
                bounds_axis.top()    > bounds_data.top()   ||
                bounds_axis.bottom() < bounds_data.bottom());
    };

    nlohmann::json chart_info;

    auto series = chart_view->chart()->series();

    auto axis_x = chart_view->chart()->axes(Qt::Horizontal).first();
    auto axis_y = chart_view->chart()->axes(Qt::Vertical  ).first();
    traced_assert(axis_x);
    traced_assert(axis_y);

    chart_info[ "x_axis_label" ] = axis_x->titleText().toStdString();
    chart_info[ "y_axis_label" ] = axis_y->titleText().toStdString();

    //Qt keeps the tick text of a value or date time axis to itself, only a
    //category axis can report what it actually shows
    auto axisType = [ ] (QAbstractAxis* axis)
    {
        if (dynamic_cast<QDateTimeAxis*>(axis))
            return std::string("datetime");
        if (dynamic_cast<QCategoryAxis*>(axis))
            return std::string("category");

        return std::string("value");
    };

    auto axisFormat = [ ] (QAbstractAxis* axis)
    {
        if (auto axis_dt = dynamic_cast<QDateTimeAxis*>(axis))
            return axis_dt->format().toStdString();

        return std::string();
    };

    auto axisTickLabels = [ ] (QAbstractAxis* axis)
    {
        nlohmann::json labels = nlohmann::json::array();

        if (auto axis_cat = dynamic_cast<QCategoryAxis*>(axis))
            for (const auto& l : axis_cat->categoriesLabels())
                labels.push_back(l.toStdString());

        return labels;
    };

    chart_info[ "x_axis_type"        ] = axisType(axis_x);
    chart_info[ "y_axis_type"        ] = axisType(axis_y);
    chart_info[ "x_axis_format"      ] = axisFormat(axis_x);
    chart_info[ "y_axis_format"      ] = axisFormat(axis_y);
    chart_info[ "x_axis_tick_labels" ] = axisTickLabels(axis_x);
    chart_info[ "y_axis_tick_labels" ] = axisTickLabels(axis_y);

    const bool   has_axis_bounds = axis_ranges.has_value();
    const QRectF axis_bounds     = has_axis_bounds ? axis_ranges.value() : QRectF();

    info[ "axis_bounds_valid" ] = has_axis_bounds;
    info[ "axis_bounds_xmin"  ] = has_axis_bounds ? axis_bounds.left()   : 0.0;
    info[ "axis_bounds_xmax"  ] = has_axis_bounds ? axis_bounds.right()  : 0.0;
    info[ "axis_bounds_ymin"  ] = has_axis_bounds ? axis_bounds.top()    : 0.0;
    info[ "axis_bounds_ymax"  ] = has_axis_bounds ? axis_bounds.bottom() : 0.0;

    info[ "axis_zoom_active"  ] = bounds.has_value() ? zoomActive(bounds.value(), axis_bounds) : false;

    //the selection series is not part of the data
    chart_info[ "num_series"] = series.count() - (chart_view->selectionSeries() ? 1 : 0);

    nlohmann::json series_infos = nlohmann::json::array();

    bool has_connection_lines = false;

    for (auto s : series)
    {
        auto xy_series = dynamic_cast<QXYSeries*>(s);
        if (!xy_series)
            continue;

        //the selection is drawn as a series of its own, it is not data
        if (s == chart_view->selectionSeries())
            continue;

        bool line_type = xy_series->type() == QAbstractSeries::SeriesType::SeriesTypeLine;

        nlohmann::json series_info;
        series_info[ "type"  ] = line_type ? "lines" : "points";
        series_info[ "count" ] = xy_series->count();
        series_info[ "name"  ] = xy_series->name().toStdString();
        series_info[ "color" ] = xy_series->color().name().toStdString();

        series_infos.push_back(series_info);

        if (line_type)
            has_connection_lines = true;
    }

    chart_info[ "series"               ] = series_infos;
    chart_info[ "has_connection_lines" ] = has_connection_lines;

    info[ "chart" ] = chart_info;
}
