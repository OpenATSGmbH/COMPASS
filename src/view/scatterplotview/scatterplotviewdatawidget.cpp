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

#include "scatterplotviewdatawidget.h"
#include "scatterplotviewwidget.h"
#include "scatterplotview.h"
#include "viewvariable.h"
#include "viewpointgenerator.h"

#include "buffer.h"

#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "scatterplotviewdatasource.h"
#include "scatterplotviewchartview.h"
#include "logger.h"
#include "property_templates.h"
#include "timeconv.h"

#include "tbbhack.h"

#include <QHBoxLayout>
#include <QMessageBox>
#include <QTabWidget>
#include <QStackedLayout>

#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QLegend>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QValueAxis>
#include <QtCharts/QDateTimeAxis>
#include <QGraphicsLayout>
#include <QShortcut>
#include <QApplication>

#include <algorithm>

QT_CHARTS_USE_NAMESPACE

using namespace std;
using namespace dbContent;
using namespace Utils;

/**
*/
ScatterPlotViewDataWidget::ScatterPlotViewDataWidget(ScatterPlotViewWidget* view_widget,
                                                     QWidget* parent,
                                                     Qt::WindowFlags f)
:   VariableViewStashDataWidget(view_widget, view_widget->getView(), true, parent, f)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    data_source_ = view_->getDataSource();
    traced_assert(data_source_);

    main_layout_ = new QHBoxLayout();
    main_layout_->setMargin(0);

    setLayout(main_layout_);

    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();

    updateDateTimeInfoFromVariables();
    updateChart();

    connect (&data_model_, &ScatterSeriesModel::visibilityChangedSignal, this, &ScatterPlotViewDataWidget::updateChartSlot);
}

/**
*/
ScatterPlotViewDataWidget::~ScatterPlotViewDataWidget()
{
    logdbg << "start";
}

/**
*/
void ScatterPlotViewDataWidget::resetSeries()
{
    scatter_series_.clear();

    x_axis_name_ = "";
    y_axis_name_ = "";
    title_       = "";

    x_axis_is_datetime_ = false;
    y_axis_is_datetime_ = false;

    bounds_ = {};

    data_model_.updateFrom(scatter_series_);
}

/**
*/
bool ScatterPlotViewDataWidget::postLoadTrigger()
{
    // disable connection lines?
    if (view_->useConnectionLines() && loadedDataCount() > ConnectLinesDataCountMax) 
    {
        loginf << "loaded data items >" << ConnectLinesDataCountMax << ", disabling connection lines";

        //no redraw, happens later anyway
        view_->useConnectionLines(false, false);
    }

    return false;
}

/**
*/
void ScatterPlotViewDataWidget::resetVariableDisplay()
{
    chart_view_.reset(nullptr);
}

/**
*/
ViewDataWidget::DrawState ScatterPlotViewDataWidget::updateVariableDisplay() 
{
    loginf;

    auto draw_state = updateChart();

    //reset zoom after update
    resetZoomSlot();

    return draw_state;
}

/**
*/
void ScatterPlotViewDataWidget::resetStashDependentData()
{
    resetSeries();
}

/**
*/
void ScatterPlotViewDataWidget::updateDateTimeInfoFromVariables()
{
    x_axis_is_datetime_ = variableIsDateTime(0);
    y_axis_is_datetime_ = variableIsDateTime(1);
}

/**
*/
void ScatterPlotViewDataWidget::correctSeriesDateTime(ScatterSeriesCollection& collection)
{
    if (!x_axis_is_datetime_ && !y_axis_is_datetime_)
        return;

    //loginf << "correcting datetime...";

    for (auto& s : collection.dataSeries())
    {
        for (auto& pos : s.second.scatter_series.points)
        {
            if (x_axis_is_datetime_) pos[ 0 ] = Utils::Time::correctLongQtUTC((long)pos[ 0 ]);
            if (y_axis_is_datetime_) pos[ 1 ] = Utils::Time::correctLongQtUTC((long)pos[ 1 ]);
        }
    }

    //loginf << "corrected datetime!";
}

/**
*/
void ScatterPlotViewDataWidget::processStash(const VariableViewStash<double>& stash)
{
    loginf << "start"
           << " valid: " << stash.valid_count_ << " "
           << " selected: " << stash.selected_count_ << " "
           << " nan: " << stash.nan_value_count_;

    bounds_ = {};

    //generate dataseries from stash
    ScatterSeries selected_series;
    selected_series.points.reserve(stash.selected_count_);

    size_t num_null_values = 0;

    for (const auto& dbc_stash : stash.groupedStashes())
    {
        //if (!dbc_stash.second.valid_count)
        //    continue;

        ScatterSeries dbc_series;
        dbc_series.points.reserve(dbc_stash.second.unsel_count);

        const auto& x_values = dbc_stash.second.variable_stashes[ 0 ].values;
        const auto& y_values = dbc_stash.second.variable_stashes[ 1 ].values;

        size_t n = x_values.size();

        for (size_t i = 0; i < n; ++i)
        {
            if (dbc_stash.second.nan_values[ i ])
            {
                //nan = null, each value of the triplet must not be null
                ++num_null_values;
                continue;
            }
            else if (dbc_stash.second.selected_values[ i ])
            {
                selected_series.points.emplace_back(x_values[ i ], y_values[ i ]);
            }
            else
            {
                dbc_series.points.emplace_back(x_values[ i ], y_values[ i ]);
            }
        }

        if (!dbc_series.points.empty())
        {
            std::string name = dbc_stash.first;

            scatter_series_.addDataSeries(dbc_series, name, colorForGroupName(dbc_stash.first), MarkerSizePx);
        }
    }

    //@TODO
    addNullCount(num_null_values);

    //add selected dataset as the last one (important for render order)
    if (!selected_series.points.empty())
    {
        std::string name = "Selected";

        scatter_series_.addDataSeries(selected_series, name, ColorSelected, MarkerSizeSelectedPx);
    }

    x_axis_name_ = view_->variable(0).description();
    y_axis_name_ = view_->variable(1).description();
    title_       = "";

    updateDateTimeInfoFromVariables();

    correctSeriesDateTime(scatter_series_);

    data_model_.updateFrom(scatter_series_);

    bounds_ = scatter_series_.getDataBounds();

    loginf << "done, generated " << scatter_series_.numDataSeries() << " series";
}

/**
*/
bool ScatterPlotViewDataWidget::updateFromAnnotations()
{
    loginf;

    bounds_ = {};

    if (!view_->hasCurrentAnnotation())
        return false;

    const auto& anno = view_->currentAnnotation();

    title_       = anno.metadata.title_;
    x_axis_name_ = anno.metadata.xAxisLabel();
    y_axis_name_ = anno.metadata.yAxisLabel();

    const auto& feature = anno.feature_json;

    if (!feature.is_object() || !feature.contains(ViewPointGenFeatureScatterSeries::FeatureHistogramFieldNameScatterSeries))
        return false;
    
    if (!scatter_series_.fromJSON(feature[ ViewPointGenFeatureScatterSeries::FeatureHistogramFieldNameScatterSeries ]))
    {
        scatter_series_.clear();
        return false;
    }

    x_axis_is_datetime_ = scatter_series_.commonDataTypeX() == ScatterSeries::DataTypeTimestamp;
    y_axis_is_datetime_ = scatter_series_.commonDataTypeY() == ScatterSeries::DataTypeTimestamp;

    correctSeriesDateTime(scatter_series_);

    data_model_.updateFrom(scatter_series_);

    bounds_ = scatter_series_.getDataBounds();

    if (scatter_series_.useConnectionLines().has_value())
    {
        view_->useConnectionLines(scatter_series_.useConnectionLines().value(), false);
        view_->updateComponents();
    }

    loginf << "done, generated " << scatter_series_.numDataSeries() << " series";

    return true;
}

/**
*/
void ScatterPlotViewDataWidget::toolChanged_impl(int mode)
{
    selected_tool_ = (ScatterPlotViewDataTool)mode;

    if (chart_view_)
        chart_view_->onToolChanged();
}

/**
*/
ScatterPlotViewDataTool ScatterPlotViewDataWidget::selectedTool() const
{
    return selected_tool_;
}

/**
*/
QPixmap ScatterPlotViewDataWidget::renderPixmap()
{
    traced_assert(chart_view_);
    return chart_view_->grab();
    //QPixmap p (chart_view_->size());
    //chart_view_->render(&p);
}

/**
*/
boost::optional<QRectF> ScatterPlotViewDataWidget::getViewBounds() const
{
    bool bounds_valid = bounds_.has_value();

    loginf << "data range bounds"
           << " x min " << (bounds_valid ? bounds_->left() : 0)
           << " max " << (bounds_valid ? bounds_->right() : 0)
           << " y min " << (bounds_valid ? bounds_->top() : 0)
           << " max " << (bounds_valid ? bounds_->bottom() : 0)
           << " bounds valid " << bounds_valid
           << " bounds empty " << (bounds_valid? bounds_->isEmpty() : true);

    return bounds_;
}

/**
 * p1 and p2 are in the chart's coordinate system.
*/
void ScatterPlotViewDataWidget::rectangleSelectedSlot (QPointF p1, QPointF p2) // TODO
{
    loginf;

    if (chart_view_ && chart_view_->chart())
    {
        if (selected_tool_ == SP_ZOOM_RECT_TOOL)
        {
            loginf << "zoom";

            //TODO: prevent from going nuts when zero rect is passed!

            auto axes_x = chart_view_->chart()->axes(Qt::Horizontal);
            auto axes_y = chart_view_->chart()->axes(Qt::Vertical);
            
            if (!axes_x.empty() && !axes_y.empty())
            {
                setAxisRange(axes_x.first(), min(p1.x(), p2.x()), max(p1.x(), p2.x()));
                setAxisRange(axes_y.first(), min(p1.y(), p2.y()), max(p1.y(), p2.y()));
            }
        }
        else if (selected_tool_ == SP_SELECT_TOOL)
        {
            loginf << "select";

            //!datetime in the rect needs correction back to utc when selecting data!
            bool correct_datetime_utc = true;
            selectData(min(p1.x(), p2.x()), max(p1.x(), p2.x()), min(p1.y(), p2.y()), max(p1.y(), p2.y()), 0, 1, correct_datetime_utc);
        }
        else
        {
            throw std::runtime_error("ScatterPlotViewDataWidget: rectangleSelectedSlot: unknown tool "
                                     +to_string((unsigned int)selected_tool_));
        }
    }

    endTool();
}

/**
*/
void ScatterPlotViewDataWidget::invertSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
        {
            if (selected_vec.isNull(cnt))
                selected_vec.set(cnt, true);
            else
                selected_vec.set(cnt, !selected_vec.get(cnt));
        }
    }

    emit view_->selectionChangedSignal();
}

/**
*/
void ScatterPlotViewDataWidget::clearSelectionSlot()
{
    loginf;

    for (auto& buf_it : viewData())
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
            selected_vec.set(cnt, false);
    }

    emit view_->selectionChangedSignal();
}

/**
 * vmin and vmax are expected to be in the chart's coordinate system.
*/
void ScatterPlotViewDataWidget::setAxisRange(QAbstractAxis* axis, double vmin, double vmax)
{
    traced_assert(axis);

    //handle datetime axis
    auto axis_dt = dynamic_cast<QDateTimeAxis*>(axis);
    if (axis_dt)
    {
        // convert to datetime
        QDateTime dt_min = QDateTime::fromMSecsSinceEpoch(vmin);
        QDateTime dt_max = QDateTime::fromMSecsSinceEpoch(vmax);

        loginf << "ts min "
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
*/
boost::optional<std::pair<double, double>> ScatterPlotViewDataWidget::getAxisRange(QtCharts::QAbstractAxis* axis) const
{
    traced_assert(axis);

    //handle datetime axis
    if (auto axis_dt = dynamic_cast<QDateTimeAxis*>(axis))
    {
        QDateTime dt_min = axis_dt->min();
        QDateTime dt_max = axis_dt->max();

        return std::pair<double, double>(dt_min.toMSecsSinceEpoch(), dt_max.toMSecsSinceEpoch());
    }
    else if (auto axis_value = dynamic_cast<QValueAxis*>(axis))
    {
        //return value axis range
        return std::pair<double, double>(axis_value->min(), axis_value->max());
    }

    return {};
}

/**
*/
ScatterSeriesModel& ScatterPlotViewDataWidget::dataModel()
{
    return data_model_;
}

/**
*/
void ScatterPlotViewDataWidget::resetZoomSlot()
{
    loginf;

    if (chart_view_ && chart_view_->chart())
    {
        auto axes_x = chart_view_->chart()->axes(Qt::Horizontal);
        auto axes_y = chart_view_->chart()->axes(Qt::Vertical);
        
        if (!axes_x.empty() && !axes_y.empty())
        {
            auto bounds       = getViewBounds();
            bool bounds_valid = bounds.has_value();

            loginf << "start"
                   << " x min " << (bounds_valid ? bounds->left() : 0)
                   << " max " << (bounds_valid ? bounds->right() : 0)
                   << " y min " << (bounds_valid ? bounds->top() : 0)
                   << " max " << (bounds_valid ? bounds->bottom() : 0)
                   << " bounds valid " << bounds_valid
                   << " bounds empty " << (bounds_valid? bounds->isEmpty() : true);

            if (bounds_valid)
            {
                double x0 = bounds->left();
                double x1 = bounds->right();
                double y0 = bounds->top();
                double y1 = bounds->bottom();
                double w  = bounds->width();
                double h  = bounds->height();

                double bx = w < 1e-12 ? 0.1 : w * 0.03;
                double by = h < 1e-12 ? 0.1 : h * 0.03;

                x0 -= bx;
                y0 -= by;
                x1 += bx;
                y1 += by;

                setAxisRange(axes_x.first(), x0, x1);
                setAxisRange(axes_y.first(), y0, y1);
            }
            else
            {
                // reset to default range
                setAxisRange(axes_x.first(), 0, 1);
                setAxisRange(axes_y.first(), 0, 1);
            }
        }
    }
}

/**
*/
void ScatterPlotViewDataWidget::updateChartSlot()
{
    //remember current axis ranges if available
    bool has_axes0 = chart_view_ && 
                     chart_view_->chart() &&
                    !chart_view_->chart()->axes(Qt::Horizontal).empty() &&
                    !chart_view_->chart()->axes(Qt::Vertical).empty();

    boost::optional<std::pair<double, double>> x_range, y_range;
    if (has_axes0)
    {
        x_range = getAxisRange(chart_view_->chart()->axes(Qt::Horizontal).first());
        y_range = getAxisRange(chart_view_->chart()->axes(Qt::Vertical).first());
    }

    //redraw chart
    auto draw_state = updateChart();
    setDrawState(draw_state);

    //recover original axis ranges if available
    //(we assume that the type of axes has not changed)
    bool has_axes1 = chart_view_ && 
                     chart_view_->chart() &&
                    !chart_view_->chart()->axes(Qt::Horizontal).empty() &&
                    !chart_view_->chart()->axes(Qt::Vertical).empty();

    if (has_axes0 && 
        has_axes1 && 
        x_range.has_value() &&
        y_range.has_value())
    {
        //set to axes original range
        setAxisRange(chart_view_->chart()->axes(Qt::Horizontal).first(), x_range.value().first, x_range.value().second);
        setAxisRange(chart_view_->chart()->axes(Qt::Vertical).first(), y_range.value().first, y_range.value().second);
    }
    else
    {
        //no valid bounds => just reset zoom
        resetZoomSlot();
    }
}

/**
*/
ViewDataWidget::DrawState ScatterPlotViewDataWidget::updateChart()
{
    logdbg << "start";

    traced_assert(main_layout_);

    chart_view_.reset(nullptr);

    QChart* chart = new QChart();
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->setDropShadowEnabled(false);
    chart->setTitle(QString::fromStdString(title_));

    // chart->legend()->setAlignment(Qt::AlignRight);

    auto draw_state = updateDataSeries(chart);

    chart->update();

    chart->legend()->setMaximumSize(0,0);

    chart_view_.reset(new ScatterPlotViewChartView(this, chart));
    chart_view_->setObjectName("chart_view");

    connect (chart_view_.get(), &ScatterPlotViewChartView::rectangleSelectedSignal,
             this, &ScatterPlotViewDataWidget::rectangleSelectedSlot, Qt::ConnectionType::QueuedConnection);

    // queued needed, otherwise crash when signal is emitted in ScatterPlotViewChartView::seriesReleasedSlot

    for (auto series_it : chart->series())
    {
        QScatterSeries* scat_series = dynamic_cast<QScatterSeries*>(series_it);
        if (!scat_series)
            continue;

        traced_assert(scat_series);

        connect (scat_series, &QScatterSeries::pressed,
                 chart_view_.get(), &ScatterPlotViewChartView::seriesPressedSlot);
        connect (scat_series, &QScatterSeries::released,
                 chart_view_.get(), &ScatterPlotViewChartView::seriesReleasedSlot);
    }

    main_layout_->addWidget(chart_view_.get());

    return draw_state;
}

/**
*/
ViewDataWidget::DrawState ScatterPlotViewDataWidget::updateDataSeries(QtCharts::QChart* chart)
{
    bool has_data = scatter_series_.numPoints(true) > 0 && (variablesOk() || view_->showsAnnotation());

    DrawState draw_state = DrawState::NotDrawn;

    //!take care: this functional may assert if it is called when no series has yet been added to the chart!
    auto createAxes = [ & ] ()
    {
        bool dynamic_labels = false;

        const std::pair<QString, bool> DateTimeFormatDefault = { "hh:mm:ss", false };

        auto dateTimeFormatFromSeries = [ & ] (int axis_id)
        {
            if (dynamic_labels)
            {
                boost::optional<qreal> tmin, tmax;
                for (auto s : chart->series())
                {
                    auto scatter_series = dynamic_cast<QScatterSeries*>(s);
                    if (!scatter_series)
                        continue;

                    for (const auto& pos : scatter_series->points())
                    {
                        qreal v = axis_id == 0 ? pos.x() : pos.y();
                        if (!tmin.has_value() || v < tmin.value()) tmin = v;
                        if (!tmax.has_value() || v > tmax.value()) tmax = v;
                    }
                }

                if (!tmin.has_value() || !tmax.has_value())
                    return std::pair<QString, bool>("yyyy-MM-dd hh:mm:ss", true);

                auto date_time0 = QDateTime::fromMSecsSinceEpoch(tmin.value());
                auto date_time1 = QDateTime::fromMSecsSinceEpoch(tmax.value());

                bool needs_year  = date_time0.date().year()  != date_time1.date().year();
                bool needs_month = date_time0.date().month() != date_time1.date().month();
                bool needs_day   = date_time0.date().day()   != date_time1.date().day();

                if (needs_year)
                    return std::pair<QString, bool>("yyyy-MM-dd hh:mm:ss", true);
                if (needs_month)
                    return std::pair<QString, bool>("MMM dd hh:mm:ss", true);
                if (needs_day)
                    return std::pair<QString, bool>("ddd hh:mm:ss", false);

                return std::pair<QString, bool>("hh:mm:ss", false);
            }
            else
            {
                return DateTimeFormatDefault;
            }
        };

        auto genDateTimeAxis = [ & ] (const std::string& title, int axis_id)
        {
            auto format = dateTimeFormatFromSeries(axis_id);

            auto axis = new QDateTimeAxis;
            axis->setFormat(format.first);
            axis->setTitleText(QString::fromStdString(title));
            axis->setLabelsAngle(format.second ? -90 : 0);

            return dynamic_cast<QAbstractAxis*>(axis);
        };

        auto genValueAxis = [ & ] (const std::string& title)
        {
            auto axis = new QValueAxis;
            axis->setTitleText(QString::fromStdString(title));

            return dynamic_cast<QAbstractAxis*>(axis);
        };

        auto createAxis = [ & ] (int axis_id,
                                 const std::string& title,
                                 bool is_date_time,
                                 Qt::Alignment alignment)
        {
            QAbstractAxis* axis = is_date_time ? genDateTimeAxis(title, axis_id) : genValueAxis(title);
            traced_assert(axis);

            chart->addAxis(axis, alignment);

            for (auto series : chart->series())
                series->attachAxis(axis);
        };

        //config x axis
        loginf << "title x '" << view_->variable(0).description() << "' is_datetime " << x_axis_is_datetime_;

        createAxis(0, x_axis_name_, x_axis_is_datetime_, Qt::AlignBottom);

        //config y axis
        loginf << "title y '" << view_->variable(1).description() << "' is_datetime " << y_axis_is_datetime_;

        createAxis(1, y_axis_name_, y_axis_is_datetime_, Qt::AlignLeft);

        traced_assert(chart->axes(Qt::Horizontal).size() == 1);
        traced_assert(chart->axes(Qt::Vertical).size() == 1);
    };

    if (has_data)
    {
        //data available

        //chart->legend()->setVisible(true);

        bool use_connection_lines = view_->useConnectionLines();

        struct SymbolLineSeries { QScatterSeries* scatter_series;
                                  QLineSeries* line_series;};

        //generate needed series and sort pointers
        //qtcharts when rendering opengl will sort the series into a map using the pointer of the series as a key
        //we exploit this behavior to obtain a fixed render order for our data

        //size_t n_series = scatter_series_.numDataSeries();

        const auto& data_series = scatter_series_.dataSeries();

        //generate pointers
        std::vector<SymbolLineSeries> series;

        for (auto& series_it : data_series)
        {
            if (!series_it.second.visible)
                continue;

            series.push_back(SymbolLineSeries());

            auto& s = series.back();

            s.scatter_series = new QScatterSeries;
            s.scatter_series->setMarkerShape(QScatterSeries::MarkerShapeCircle);
            s.scatter_series->setMarkerSize(MarkerSizePx);
            s.scatter_series->setUseOpenGL(true);

            if (use_connection_lines)
            {
                s.line_series = new QLineSeries;
                QPen pen = s.line_series->pen();
                pen.setWidth(2);
                //pen.setBrush(QBrush("red")); // or just pen.setColor("red");
                s.line_series->setPen(pen);

                s.line_series->setUseOpenGL(true);
            }
            else
                s.line_series = nullptr;
        }

        //sort pointers to obtain fixed render order (as it happens inside qtcharts)
        //!the highest pointer is always on top, so the selection series is always assumed to be the last one!
        sort(series.begin(), series.end(),
            [&](const SymbolLineSeries& a, const SymbolLineSeries& b) {return a.scatter_series < b.scatter_series; });

        //now add data to series
        size_t cnt = 0;
        for (auto& series_it : data_series)
        {
            if (!series_it.second.visible)
                continue;

            auto& s = series[cnt++];
            
            string name = series_it.first;
            //const auto& ds = data_series[ i ];
            const auto& ds = series_it.second;

            //PWa: empty series should not be a problem
            //assert (!ds.scatter_series.points.empty());

            QScatterSeries* chart_symbol_series = s.scatter_series;
            chart_symbol_series->setColor(ds.color);
            chart_symbol_series->setMarkerSize(ds.marker_size);

            QLineSeries* chart_line_series = nullptr;

            if (use_connection_lines)
            {
                chart_line_series = s.line_series;
                chart_line_series->setColor(ds.color);
            }

            for (const auto& pos : ds.scatter_series.points)
            {
                double x = pos.x();
                double y = pos.y();

                chart_symbol_series->append(x, y);

                if (use_connection_lines)
                {
                    traced_assert(chart_line_series);
                    chart_line_series->append(x, y);
                }
            }

            chart_symbol_series->setName(ds.name.c_str());
            chart->addSeries(chart_symbol_series);

            if (use_connection_lines)
            {
                traced_assert(chart_line_series);
                chart->addSeries(chart_line_series);

                //chart->legend()->markers(chart_line_series)[0]->setVisible(false); // remove line marker in legend

                logdbg << "connection lines " << chart_line_series->count();
            }
        }

        //safe to create axes
        createAxes();

        //content drawn
        draw_state = DrawState::DrawnContent;
    }
    else
    {
        //bad data range or vars not in buffer
        logdbg << "content empty";

        //no data -> generate default empty layout
        //chart->legend()->setVisible(false);

        //!add at least one series first! (otherwise createAxes() will crash)
        if (chart->series().isEmpty())
        {
            QScatterSeries* series = new QScatterSeries;
            chart->addSeries(series);
        }
        
        createAxes();

        chart->axes(Qt::Horizontal).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Horizontal).at(0)->setMinorGridLineVisible(false);

        chart->axes(Qt::Vertical).at(0)->setLabelsVisible(false);
        chart->axes(Qt::Vertical).at(0)->setGridLineVisible(false);
        chart->axes(Qt::Vertical).at(0)->setMinorGridLineVisible(false);

        //at least we have drawn something
        draw_state = DrawState::Drawn;
    }

    return draw_state;
}

/**
*/
void ScatterPlotViewDataWidget::mouseMoveEvent(QMouseEvent* event)
{
    //setCursor(current_cursor_);
    //osgEarth::QtGui::ViewerWidget::mouseMoveEvent(event);

    QWidget::mouseMoveEvent(event);
}

/**
 */
void ScatterPlotViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableViewStashDataWidget::viewInfoJSON_impl(info);

    auto bounds       = getViewBounds();
    bool bounds_valid = bounds.has_value();

    info[ "data_bounds_valid" ] = bounds_valid;
    info[ "data_bounds_xmin"  ] = bounds_valid ? bounds->left()   : 0.0;
    info[ "data_bounds_ymin"  ] = bounds_valid ? bounds->top()    : 0.0;
    info[ "data_bounds_xmax"  ] = bounds_valid ? bounds->right()  : 0.0;
    info[ "data_bounds_ymax"  ] = bounds_valid ? bounds->bottom() : 0.0;

    auto zoomActive = [ & ] (const QRectF& bounds_data, const QRectF& bounds_axis)
    {
        if (!bounds_data.isValid() || !bounds_axis.isValid())
            return false;

        return (bounds_axis.left()   > bounds_data.left()  ||
                bounds_axis.right()  < bounds_data.right() ||
                bounds_axis.top()    > bounds_data.top()   ||
                bounds_axis.bottom() < bounds_data.bottom());
    };

    if (chart_view_)
    {
        nlohmann::json chart_info;

        auto series = chart_view_->chart()->series();

        auto axis_x = dynamic_cast<QValueAxis*>(chart_view_->chart()->axes(Qt::Horizontal).first());
        auto axis_y = dynamic_cast<QValueAxis*>(chart_view_->chart()->axes(Qt::Vertical).first());

        chart_info[ "x_axis_label" ] = axis_x->titleText().toStdString();
        chart_info[ "y_axis_label" ] = axis_y->titleText().toStdString();

        bool   has_axis_bounds = (axis_x != nullptr && axis_y != nullptr);
        QRectF axis_bounds     = has_axis_bounds ? QRectF(axis_x->min(), 
                                                          axis_y->min(),
                                                          axis_x->max() - axis_x->min(),
                                                          axis_y->max() - axis_y->min()) : QRectF();

        info[ "axis_bounds_valid" ] = has_axis_bounds;
        info[ "axis_bounds_xmin"  ] = has_axis_bounds ? axis_bounds.left()   : 0.0;
        info[ "axis_bounds_xmax"  ] = has_axis_bounds ? axis_bounds.right()  : 0.0;
        info[ "axis_bounds_ymin"  ] = has_axis_bounds ? axis_bounds.top()    : 0.0;
        info[ "axis_bounds_ymax"  ] = has_axis_bounds ? axis_bounds.bottom() : 0.0;

        info[ "axis_zoom_active"  ] = bounds_valid ? zoomActive(bounds.value(), axis_bounds) : false;

        chart_info[ "num_series"] = series.count();

        nlohmann::json series_infos = nlohmann::json::array();

        bool has_connection_lines = false;

        for (auto s : series)
        {
            auto xy_series = dynamic_cast<QXYSeries*>(s);
            if (!xy_series)
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
}
