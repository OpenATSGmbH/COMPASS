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

#include "scatterplotpointsitem.h"
#include "traced_assert.h"

#include <QDateTime>
#include <QPainter>
#include <QPolygonF>

#include <QtCharts/QChart>
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>

#include <cmath>

QT_CHARTS_USE_NAMESPACE

const double ScatterPlotPointsItem::ConnectionLineWidth = 2.0;

namespace
{
    // Qt Charts stacks the parts of a chart by z value (ChartPresenter::ZValues, not public):
    // background -1, plot area 0, shades 1, grid 2, axes 3, series from 4 on. The points are above
    // the grid and the axis lines and below the legend, as the points of a series.
    const qreal PointsZValue = 4.0;

    /**
     * One marker, a filled circle. Drawn once per series and copied to each point: an antialiased
     * ellipse per point is too slow for the point counts of a scatterplot.
     */
    QImage markerImage(const QColor& color, double diameter_px)
    {
        //one pixel around the circle for its antialiased border
        const int size = (int)std::ceil(diameter_px) + 2;

        QImage image(size, size, QImage::Format_ARGB32_Premultiplied);
        image.fill(Qt::transparent);

        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(QPointF(size / 2.0, size / 2.0), diameter_px / 2.0, diameter_px / 2.0);

        return image;
    }
}

/**
 */
ScatterPlotPointsItem::ScatterPlotPointsItem(QChart* chart,
                                             std::shared_ptr<const ScatterPlotDisplayData> data,
                                             bool connection_lines)
:   QGraphicsObject  (chart)
,   chart_           (chart)
,   data_            (std::move(data))
,   connection_lines_(connection_lines)
{
    traced_assert(chart_);
    traced_assert(data_);

    setZValue(PointsZValue);

    auto geometryChanged = [ this ] ()
    {
        prepareGeometryChange();
        update();
    };

    connect(chart_, &QChart::plotAreaChanged, this, geometryChanged);
    connect(chart_, &QChart::geometryChanged, this, geometryChanged);

    //a zoom needs a new image
    for (auto axis : chart_->axes())
    {
        if (auto axis_dt = dynamic_cast<QDateTimeAxis*>(axis))
            connect(axis_dt, &QDateTimeAxis::rangeChanged, this, [ this ] () { update(); });
        else if (auto axis_value = dynamic_cast<QValueAxis*>(axis))
            connect(axis_value, &QValueAxis::rangeChanged, this, [ this ] () { update(); });
    }
}

/**
 */
ScatterPlotPointsItem::~ScatterPlotPointsItem() = default;

/**
 */
QRectF ScatterPlotPointsItem::boundingRect() const
{
    return chart_->plotArea();
}

/**
 */
void ScatterPlotPointsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    const QRectF plot_area   = chart_->plotArea();
    const auto   value_range = axisRanges(chart_);

    if (plot_area.isEmpty() || !value_range.has_value())
        return;

    //pixels of the paint device per layout pixel: the display scaling of a screen, the scale of an
    //export image. The image has the pixels of the device, so the markers are sharp at each scale.
    const double scale = painter->deviceTransform().m11();

    const QSize size_px((int)std::lround(plot_area.width() * scale), (int)std::lround(plot_area.height() * scale));

    if (scale <= 0.0 || size_px.isEmpty())
        return;

    if (image_.size() != size_px || image_scale_ != scale || image_value_range_ != value_range.value())
    {
        image_ = drawPoints(*data_, connection_lines_, size_px, scale, value_range.value());
        image_.setDevicePixelRatio(scale);

        image_scale_       = scale;
        image_value_range_ = value_range.value();
    }

    painter->drawImage(plot_area.topLeft(), image_);
}

/**
 */
QImage ScatterPlotPointsItem::drawPoints(const ScatterPlotDisplayData& data,
                                         bool connection_lines,
                                         const QSize& size_px,
                                         double scale,
                                         const QRectF& value_range)
{
    QImage image(size_px, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);

    const double x_min  = value_range.left();
    const double y_min  = value_range.top();
    const double x_span = value_range.width();
    const double y_span = value_range.height();

    if (size_px.isEmpty() || !(x_span > 0.0) || !(y_span > 0.0))
        return image;

    const double w  = size_px.width();
    const double h  = size_px.height();
    const double fx = w / x_span;
    const double fy = h / y_span;

    QPainter painter(&image);

    for (const auto* series : data.seriesInDrawOrder())
    {
        const auto& points = series->scatter_series.points;

        if (connection_lines && points.size() > 1)
        {
            QPolygonF line;
            line.reserve((int)points.size());

            for (const auto& p : points)
            {
                const QPointF pos((p.x() - x_min) * fx, h - (p.y() - y_min) * fy);

                if (std::isfinite(pos.x()) && std::isfinite(pos.y()))
                    line << pos;
            }

            QPen pen(series->color);
            pen.setWidthF(ConnectionLineWidth * scale);

            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.setPen(pen);
            painter.drawPolyline(line);
        }

        const QImage marker = markerImage(series->color, series->marker_size * scale);
        const double r      = marker.width() / 2.0;

        for (const auto& p : points)
        {
            const double px = (p.x() - x_min) * fx;
            const double py = h - (p.y() - y_min) * fy;

            //also false for a value that is no number
            const bool inside = px > -r && px < w + r && py > -r && py < h + r;
            if (!inside)
                continue;

            painter.drawImage(QPoint((int)std::lround(px - r), (int)std::lround(py - r)), marker);
        }
    }

    return image;
}

/**
 */
boost::optional<std::pair<double, double>> ScatterPlotPointsItem::axisRange(const QAbstractAxis* axis)
{
    traced_assert(axis);

    if (auto axis_dt = dynamic_cast<const QDateTimeAxis*>(axis))
        return std::pair<double, double>(axis_dt->min().toMSecsSinceEpoch(), axis_dt->max().toMSecsSinceEpoch());

    if (auto axis_value = dynamic_cast<const QValueAxis*>(axis))
        return std::pair<double, double>(axis_value->min(), axis_value->max());

    return {};
}

/**
 */
boost::optional<QRectF> ScatterPlotPointsItem::axisRanges(const QChart* chart)
{
    if (!chart)
        return {};

    const auto axes_x = chart->axes(Qt::Horizontal);
    const auto axes_y = chart->axes(Qt::Vertical);

    if (axes_x.empty() || axes_y.empty())
        return {};

    const auto range_x = axisRange(axes_x.first());
    const auto range_y = axisRange(axes_y.first());

    if (!range_x.has_value() || !range_y.has_value())
        return {};

    return QRectF(range_x->first, range_y->first, range_x->second - range_x->first, range_y->second - range_y->first);
}
