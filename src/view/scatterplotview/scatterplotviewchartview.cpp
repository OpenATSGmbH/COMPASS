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

#include "scatterplotviewchartview.h"
#include "scatterplotviewrenderwidget.h"
#include "traced_assert.h"
#include "logger.h"

#include <QApplication>
#include <QRubberBand>
#include <QtCharts/QAreaSeries>
#include <QtCharts/QLineSeries>

//#define USE_CHART_SERIES_BASED_SELECTION_BOX

namespace QtCharts
{

/**
 */
ScatterPlotViewChartView::ScatterPlotViewChartView (ScatterPlotViewRenderWidget* render_widget,
                                                    QChart* chart,
                                                    bool use_opengl,
                                                    QWidget* parent)
:   //data drawn with OpenGL acceleration composites above any
    //widget laid over the chart, so the selection has to be an accelerated
    //series too
    ChartView     (chart,
                   use_opengl ? ChartView::SelectionStyle::SeriesLines : ChartView::SelectionStyle::RubberBand,
                   parent)
,   render_widget_(render_widget)
{
    traced_assert(render_widget_);
}

/**
 */
ScatterPlotViewChartView::~ScatterPlotViewChartView() = default;

/**
 */
void ScatterPlotViewChartView::onToolChanged()
{
    //disable drag
    drag_data_init_ = false;

    //don't forget to invoke base behavior
    ChartView::onToolChanged();
}

/**
 */
bool ScatterPlotViewChartView::handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->dragEnabled())
        {
            logdbg << "navigate x " << widget_pos.x() << " y " << widget_pos.y();

            drag_data_      = widget_pos;
            drag_data_init_ = true;

            return true;
        }
        else if (render_widget_->rectangleRequested())
        {
            loginf << "rect x " << widget_pos.x() << " y " << widget_pos.y();

            // view widget coordinates to chart coordinates
            QPointF p = widgetToChart(widget_pos);

            loginf << "rect xc " << p.x() << " yc " << p.y();

            p1_      = widget_pos.toPoint();
            p1_data_ = p;

            p2_      = widget_pos.toPoint();
            p2_data_ = p;

            beginSelection(p1_, p2_, p1_data_, p2_data_);

            return true;
        }
    }

    return false;
}

/**
 */
bool ScatterPlotViewChartView::handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->dragEnabled())
        {
            logdbg << "navigate x " << widget_pos.x() << " y " << widget_pos.y() << " drag_data_init " << drag_data_init_;

            if (drag_data_init_)
            {
                auto dPos = widget_pos - drag_data_;
                chart()->scroll(-dPos.x(), dPos.y());
            }

            drag_data_      = widget_pos;
            drag_data_init_ = true;

            return true;
        }
        else if (render_widget_->rectangleRequested() && isSelectionEnabled())
        {
            logdbg << "rect x " << widget_pos.x() << " y " << widget_pos.y();

            // view widget coordinates to chart coordinates
            QPointF p = widgetToChart(widget_pos);

            p2_      = widget_pos.toPoint();
            p2_data_ = p;

            updateSelection(p1_, p2_, p1_data_, p2_data_);

            return true;
        }
    }

    return false;
}

/**
 */
bool ScatterPlotViewChartView::handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->dragEnabled())
        {
            logdbg << "navigate x " << widget_pos.x() << " y " << widget_pos.y();

            drag_data_init_ = false;

            return true;
        }
        else if (render_widget_->rectangleRequested() && isSelectionEnabled())
        {
            loginf << "rect x " << widget_pos.x() << " y " << widget_pos.y();

            // view widget coordinates to chart coordinates
            QPointF p = widgetToChart(widget_pos);

            loginf << "rect xc " << p.x() << " yc " << p.y();

            if (update_pos)
            {
                p2_      = widget_pos.toPoint();
                p2_data_ = p;
            }
            
            loginf << "region p1 " << p1_data_.x() << "," << p1_data_.y() << " p2 " << p2_data_.x() << "," << p2_data_.y();

            updateSelection(p1_, p2_, p1_data_, p2_data_);
            sendSelectedRegion();
            endSelection();

            return true;
        }
    }

    return false;
}

/**
 * Reports the selected region (chart coordinates) to the render widget.
 */
void ScatterPlotViewChartView::sendSelectedRegion()
{
    render_widget_->rectangleSelected(selectedChartRegion());
}

/**
 */
void ScatterPlotViewChartView::wheelEvent(QWheelEvent* event)
{
    const double ZoomFactorIn  = 1.1;
    const double ZoomFactorOut = 0.9;

    qreal factor = event->angleDelta().y() > 0 ? ZoomFactorIn : ZoomFactorOut;
    chart()->zoom(factor);

    event->accept();

    QChartView::wheelEvent(event);
}

}

