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
#include "gridviewrenderwidget.h"
#include "gridbackgroundrenderer.h"
#include "gridbackground_defs.h"
#include "grid2d.h"

#include "logger.h"

#include <QtCharts/QValueAxis>

#include <QTimer>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <cmath>

namespace QtCharts
{

/**
 */
GridViewChart::GridViewChart(GridViewRenderWidget* render_widget,
                             QChart* chart,
                             QWidget* parent)
:   ChartView     (chart, ChartView::SelectionStyle::RubberBand, parent)
,   render_widget_(render_widget)
{
    traced_assert(render_widget_);

    //e.g. a resize => aspect ratio of the background
    //Notice: never inside the chart layout (a range change re-runs the layout => recursion), so deferred and coalesced
    connect(chart, &QChart::plotAreaChanged, this, [ this ] (const QRectF&)
    {
        if (limits_pending_)
            return;

        limits_pending_ = true;

        QTimer::singleShot(0, this, [ this ] ()
        {
            limits_pending_ = false;

            if (!render_widget_)
                return;

            //the user view stays as the user set it: the aspect correction for a changed plot size (e.g. the
            //legend hidden during a load) is applied again on every rebuild, so it must not accumulate
            applyViewLimits();
        });
    });
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

    //don't forget to invoke base behavior
    ChartView::onToolChanged();
}

/**
 */
bool GridViewChart::handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos)
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
bool GridViewChart::handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos)
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
                applyViewLimits();
                userViewChanged();
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
bool GridViewChart::handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos)
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
 */
void GridViewChart::sendSelectedRegion()
{
    render_widget_->rectangleSelected(selectedChartRegion());
}

/**
 */
void GridViewChart::wheelEvent(QWheelEvent* event)
{
    const double ZoomFactorIn  = 1.1;
    const double ZoomFactorOut = 0.9;

    qreal factor = event->angleDelta().y() > 0 ? ZoomFactorIn : ZoomFactorOut;
    chart()->zoom(factor);
    applyViewLimits();
    userViewChanged();

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
    const auto plot_area  = chart()->plotArea();
    const bool background = render_widget_->backgroundActive();
    const auto lonlat     = viewLonLat();

    auto t0 = boost::posix_time::microsec_clock::local_time();

    //background color and map below the grid
    if (background)
        render_widget_->background().paintBelow(painter, plot_area, lonlat);

    auto t1 = boost::posix_time::microsec_clock::local_time();

    painter.save();
    painter.setOpacity(render_widget_->gridOpacity() / 100.0);
    paintGrid(painter, plot_area);
    painter.restore();

    auto t2 = boost::posix_time::microsec_clock::local_time();

    //borders and sectors above the grid (sectors also without the Background Map), the labels on top
    if (render_widget_->background().available())
    {
        render_widget_->background().paintAbove(painter, plot_area, lonlat);
        render_widget_->paintLabels(painter, plot_area, lonlat);
    }

    auto t3 = boost::posix_time::microsec_clock::local_time();

    if (background)
        logdbg << "background " << (t1 - t0).total_milliseconds()
               << " ms grid " << (t2 - t1).total_milliseconds()
               << " ms borders and sectors " << (t3 - t2).total_milliseconds() << " ms";
}

/**
 */
void GridViewChart::paintGrid(QPainter& painter, const QRectF& plot_area)
{
    const auto& grid_bounds    = render_widget_->gridBounds();
    const auto& grid_rendering = render_widget_->gridRendering();
    bool        grid_north_up  = render_widget_->gridIsNorthUp();

    if (grid_bounds.isEmpty() || grid_rendering.isNull())
        return;

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
        //without a grid, the background shows the position extent of the database
        const QRectF bounds = render_widget_->backgroundActive() ? render_widget_->backgroundExtent() : render_widget_->gridBounds();

        if (!bounds.isEmpty())
        {
            chart()->axes(Qt::Horizontal).first()->setRange(bounds.x(), bounds.x() + bounds.width());
            chart()->axes(Qt::Vertical).first()->setRange(bounds.y(), bounds.y() + bounds.height());

            applyViewLimits();

            //home => following the data again
            render_widget_->clearUserView();
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

        applyViewLimits();
        userViewChanged();
    }
}

/**
 */
QRectF GridViewChart::viewLonLat() const
{
    if (!chart() || chart()->axes(Qt::Horizontal).isEmpty() || chart()->axes(Qt::Vertical).isEmpty())
        return {};

    auto x_axis = dynamic_cast<QValueAxis*>(chart()->axes(Qt::Horizontal).first());
    auto y_axis = dynamic_cast<QValueAxis*>(chart()->axes(Qt::Vertical).first());

    if (!x_axis || !y_axis)
        return {};

    return QRectF(x_axis->min(), y_axis->min(), x_axis->max() - x_axis->min(), y_axis->max() - y_axis->min());
}

/**
 */
void GridViewChart::setAxisRanges(const QRectF& r)
{
    chart()->axes(Qt::Horizontal).first()->setRange(r.left(), r.right());
    chart()->axes(Qt::Vertical).first()->setRange(r.top(), r.bottom()); // top() = minimum latitude
}

/**
 * Notice: a range change may change the plot area (axis labels), which calls this again.
 */
void GridViewChart::applyViewLimits()
{
    if (applying_limits_ || !render_widget_->backgroundActive())
        return;

    const QRectF view = viewLonLat();
    const QSizeF plot = chart()->plotArea().size();

    auto finite = [] (const QRectF& rect)
    {
        return std::isfinite(rect.left()) && std::isfinite(rect.top()) &&
               std::isfinite(rect.width()) && std::isfinite(rect.height());
    };

    if (!finite(view) || view.width() <= 0.0 || view.height() <= 0.0 || plot.width() <= 0.0 || plot.height() <= 0.0)
        return;

    namespace vp = gridbackground::viewport;

    //aspect only corrected above a tolerance: the plot size depends on the axis labels, which depend on the
    //ranges => small corrections would never settle
    const double aspect_tolerance = 0.01;

    const double lon_scale_view = std::max(0.01, std::cos(view.center().y() * M_PI / 180.0));
    const double aspect_error   = (view.width() * lon_scale_view / view.height()) / (plot.width() / plot.height()) - 1.0;

    QRectF r = std::fabs(aspect_error) > aspect_tolerance ? vp::adjustAspect(view, plot) : view;

    const double lon_scale  = std::max(0.01, std::cos(r.center().y() * M_PI / 180.0));
    const double min_width  = vp::minWidth(render_widget_->gridCellWidth(), plot.width());
    const double min_height = min_width * lon_scale * plot.height() / plot.width();

    r = vp::clamp(r, vp::maxExtent(render_widget_->backgroundExtent(), plot), min_width, min_height);

    if (!finite(r))
        return;

    //only real changes, so that a label width change converges
    const double eps = 1e-9 * std::max(view.width(), view.height());

    if (std::fabs(r.left()   - view.left())   < eps &&
        std::fabs(r.right()  - view.right())  < eps &&
        std::fabs(r.top()    - view.top())    < eps &&
        std::fabs(r.bottom() - view.bottom()) < eps)
        return;

    applying_limits_ = true;
    setAxisRanges(r);
    applying_limits_ = false;
}

/**
 */
void GridViewChart::setView(const QRectF& r)
{
    if (!chart() || chart()->axes(Qt::Horizontal).isEmpty() || chart()->axes(Qt::Vertical).isEmpty())
        return;

    if (r.width() <= 0.0 || r.height() <= 0.0)
        return;

    setAxisRanges(r);
    applyViewLimits();
}

/**
 */
void GridViewChart::userViewChanged()
{
    const QRectF view = viewLonLat();

    if (view.width() > 0.0 && view.height() > 0.0)
        render_widget_->setUserView(view);
}

}

