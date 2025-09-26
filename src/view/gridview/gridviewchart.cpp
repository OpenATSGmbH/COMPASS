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

#include "gridviewchart.h"
#include "gridviewdatawidget.h"
#include "grid2d.h"
#include "gridview.h"

/**
*/
SimpleGridViewChart::SimpleGridViewChart(QWidget* parent)
:   QLabel(parent)
{
    setAlignment(Qt::AlignCenter);

    updateChart();
}

/**
*/
SimpleGridViewChart::~SimpleGridViewChart() = default;

/**
*/
void SimpleGridViewChart::setGrid(const QImage& grid_rendering)
{
    grid_rendering_ = QPixmap::fromImage(grid_rendering);

    updateChart();
}

/**
*/
void SimpleGridViewChart::resizeEvent(QResizeEvent* evt)
{
    updateChart();
}

/**
*/
void SimpleGridViewChart::updateChart()
{
    if (grid_rendering_.isNull())
    {
        setPixmap(QPixmap());
        setText("No Data");
        return;
    }

    setText("");
    setPixmap(grid_rendering_.scaled(size(), Qt::IgnoreAspectRatio, Qt::TransformationMode::FastTransformation));
}

/**************************************************************************************************
 * GridViewChart
 **************************************************************************************************/

namespace QtCharts
{

/**
 */
GridViewChart::GridViewChart(GridViewDataWidget* data_widget, 
                             QChart* chart, 
                             QWidget* parent)
:   ChartView   (chart, ChartView::SelectionStyle::RubberBand, parent)
,   data_widget_(data_widget)
{
    traced_assert(data_widget_);
}

/**
 */
GridViewChart::~GridViewChart() = default;

/**
 */
void GridViewChart::onToolChanged()
{
    //disable drag
    drag_data_init_ = false;

    //don't forget to invoke base behaviour
    ChartView::onToolChanged();
}

/**
 */
bool GridViewChart::handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        GridViewDataTool tool = data_widget_->selectedTool();

        if (tool == GV_NAVIGATE_TOOL)
        {
            logdbg << "navigate x " << widget_pos.x() << " y " << widget_pos.y();

            drag_data_      = widget_pos;
            drag_data_init_ = true;

            return true;
        }
        else if (tool == GV_ZOOM_RECT_TOOL || tool == GV_SELECT_TOOL)
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
bool GridViewChart::handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos)
{
    if (buttons & Qt::LeftButton)
    {
        GridViewDataTool tool = data_widget_->selectedTool();

        if (tool == GV_NAVIGATE_TOOL)
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
        else if ((tool == GV_ZOOM_RECT_TOOL || tool == GV_SELECT_TOOL) && isSelectionEnabled())
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
bool GridViewChart::handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos)
{
    if (buttons & Qt::LeftButton)
    {
        GridViewDataTool tool = data_widget_->selectedTool();

        if (tool == GV_NAVIGATE_TOOL)
        {
            logdbg << "navigate x " << widget_pos.x() << " y " << widget_pos.y();

            drag_data_init_ = false;

            return true;
        }
        else if ((tool == GV_ZOOM_RECT_TOOL || tool == GV_SELECT_TOOL) && isSelectionEnabled())
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
 */
void GridViewChart::sendSelectedRegion()
{
    auto region = selectedChartRegion();

    emit rectangleSelectedSignal(region.topLeft(), region.bottomRight());
}

/**
 */
void GridViewChart::wheelEvent(QWheelEvent* event)
{
    const double ZoomFactorIn  = 1.1;
    const double ZoomFactorOut = 0.9;

    qreal factor = event->angleDelta().y() > 0 ? ZoomFactorIn : ZoomFactorOut;
    chart()->zoom(factor);

    event->accept();

    QChartView::wheelEvent(event);
}

namespace
{
    QRectF mapBounds(QtCharts::QChart* chart, const QRectF& roi, bool is_value)
    {
        auto tl_mapped = is_value ? chart->mapToPosition(roi.topLeft()    ) : chart->mapToValue(roi.topLeft()    );
        auto br_mapped = is_value ? chart->mapToPosition(roi.bottomRight()) : chart->mapToValue(roi.bottomRight());

        double xmin = std::min(tl_mapped.x(), br_mapped.x());
        double xmax = std::max(tl_mapped.x(), br_mapped.x());
        double ymin = std::min(tl_mapped.y(), br_mapped.y());
        double ymax = std::max(tl_mapped.y(), br_mapped.y());

        return QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
    };

    QRectF mapBoundsToValue(QtCharts::QChart* chart, const QRectF& roi_pos)
    {
        return mapBounds(chart, roi_pos, false);
    }

    QRectF mapBoundsToPosition(QtCharts::QChart* chart, const QRectF& roi_value)
    {
        return mapBounds(chart, roi_value, true);
    }

    // void printRect(const std::string& name, const QRectF& r)
    // {
    //     loginf << name << ": (" << r.x() << "," << r.y() << ") " << r.width() << "x" << r.height();
    // }

    // void printRect(const std::string& name, const QRect& r)
    // {
    //     loginf << name << ": (" << r.x() << "," << r.y() << ") " << r.width() << "x" << r.height();
    // }
}

/**
 */
void GridViewChart::paintCustomItems(QPaintEvent* event, QPainter& painter)
{
    const auto& grid_bounds    = data_widget_->gridBounds();
    const auto& grid_rendering = data_widget_->gridRendering();
    bool        grid_north_up  = data_widget_->gridIsNorthUp();

    if (grid_bounds.isEmpty() || grid_rendering.isNull())
        return;

    auto plot_area = chart()->plotArea();

    QRectF grid_bounds_cropped;
    QRect  grid_region_cropped;
    {
        QRectF plot_area_mapped = mapBoundsToValue(chart(), plot_area);

        //loginf << "image area: (full): " << grid_rendering.width() << "x" << grid_rendering.height();

        //printRect("plot area (mapped): ", plot_area_mapped);
        //printRect("grid area (full): ", grid_bounds);

        Grid2D::cropGrid(grid_bounds_cropped, 
                         grid_region_cropped, 
                         plot_area_mapped, 
                         grid_bounds,
                         grid_rendering.width(),
                         grid_rendering.height(),
                         grid_north_up,
                         true, 
                         1);

        //printRect("grid area (cropped): ", grid_bounds_cropped);
        //printRect("image area (cropped): ", grid_region_cropped);
    }

    //in this case the image region should be outside the plot region => do not draw
    if (grid_region_cropped.isEmpty())
        return;

    QRectF r_paint = mapBoundsToPosition(chart(), grid_bounds_cropped);

    int w = r_paint.width();
    int h = r_paint.height();

    auto img = grid_rendering.copy(grid_region_cropped).scaled(w, h, Qt::IgnoreAspectRatio, Qt::FastTransformation);

    painter.setClipRect(plot_area);
    painter.drawImage(QPointF(r_paint.x(), r_paint.y()), img);
}

/**
 */
void GridViewChart::resetZoom()
{
    if (chart() && !chart()->axes(Qt::Horizontal).isEmpty() && !chart()->axes(Qt::Vertical).isEmpty())
    {
        const auto& bounds = data_widget_->gridBounds();

        if (!bounds.isEmpty())
        {
            chart()->axes(Qt::Horizontal).first()->setRange(bounds.x(), bounds.x() + bounds.width());
            chart()->axes(Qt::Vertical).first()->setRange(bounds.y(), bounds.y() + bounds.height());
        }
    }
}

/**
 */
void GridViewChart::zoom(const QPointF& p1, const QPointF& p2)
{
    if (chart() && !chart()->axes(Qt::Horizontal).isEmpty() && !chart()->axes(Qt::Vertical).isEmpty())
    {
        chart()->axes(Qt::Horizontal).first()->setRange(std::min(p1.x(), p2.x()), std::max(p1.x(), p2.x()));
        chart()->axes(Qt::Vertical).first()->setRange(std::min(p1.y(), p2.y()), std::max(p1.y(), p2.y()));
    }
}

}
