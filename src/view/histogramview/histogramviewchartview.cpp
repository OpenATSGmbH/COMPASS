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

#include "histogramviewchartview.h"
#include "histogramviewchartview.h"
#include "histogramviewrenderwidget.h"
#include "logger.h"
#include "traced_assert.h"

#include <QApplication>
#include <QRubberBand>
#include <QBarSet>

#include <cmath>

namespace QtCharts
{

/**
 */
HistogramViewChartView::HistogramViewChartView(HistogramViewRenderWidget* render_widget, 
                                               QChart* chart, 
                                               QWidget* parent)
:   ChartView     (chart, ChartView::SelectionStyle::RubberBand, parent)
,   render_widget_(render_widget)
{
    setSelectionAxes(SelectionAxes::X);

    traced_assert(render_widget_);
}

/**
 */
HistogramViewChartView::~HistogramViewChartView() = default;

/**
 */
bool HistogramViewChartView::handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->rectangleRequested())
        {
            logdbg << "rect x " << widget_pos.x() << " y " << widget_pos.y();

            // view widget coordinates to chart coordinates
            QPointF p = widgetToChart(widget_pos);

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
bool HistogramViewChartView::handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->rectangleRequested())
        {
            if (isSelectionEnabled())
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
    }
    return false;
}

/**
 */
bool HistogramViewChartView::handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos)
{
    if (buttons & Qt::LeftButton)
    {
        if (render_widget_->rectangleRequested())
        {
            if (isSelectionEnabled())
            {
                logdbg << "rect x " << widget_pos.x() << " y " << widget_pos.y();

                // view widget coordinates to chart coordinates
                QPointF p = widgetToChart(widget_pos);

                if (update_pos)
                {
                    p2_      = widget_pos.toPoint();
                    p2_data_ = p;
                }
                
                logdbg << "region p1 " << p1_data_.x() << "," << p1_data_.y() << " p2 " << p2_data_.x() << "," << p2_data_.y();

                updateSelection(p1_, p2_, p1_data_, p2_data_);
                sendSelectedRegion();
                endSelection();

                return true;
            }  
        }
    }
    return false;
}

/**
 * Reports the selected region to the render widget. A region that covers no bin center is not
 * reported, the user can draw the rectangle again.
 */
void HistogramViewChartView::sendSelectedRegion()
{
    auto region = selectedChartRegion();

    //only send range when valid
    if (render_widget_->binRange(region).has_value())
        render_widget_->rectangleSelected(region);
}

/**
 */
void HistogramViewChartView::wheelEvent(QWheelEvent* event)
{
    //zooming using the wheel only makes sense if some kind of navigation in the chart plane is supported
    //qreal factor = event->angleDelta().y() > 0 ? 1.1 : 0.9;
    //chart()->zoom(factor);
    //event->accept();
    
    QChartView::wheelEvent(event);
}

}

