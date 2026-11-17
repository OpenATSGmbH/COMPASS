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

#pragma once

#include "scatterplotdisplaydata.h"

#include <QGraphicsObject>
#include <QImage>
#include <QRectF>
#include <QSize>

#include <boost/optional.hpp>

#include <memory>
#include <utility>

namespace QtCharts
{
    class QChart;
    class QAbstractAxis;
}

/**
 * Draws the points and the connection lines of a ScatterPlotDisplayData into the plot area of a chart,
 * without OpenGL. The chart draws the axes, the title and the legend.
 *
 * QScatterSeries without OpenGL is too slow above about 10000 points. So the points are drawn with
 * QPainter into an image, which the item shows. The image is drawn again when the plot area or an axis
 * range changes.
 */
class ScatterPlotPointsItem : public QGraphicsObject
{
public:
    /// chart: gets the item as child, its first horizontal and vertical axis give the value range.
    ScatterPlotPointsItem(QtCharts::QChart* chart,
                          std::shared_ptr<const ScatterPlotDisplayData> data,
                          bool connection_lines);
    virtual ~ScatterPlotPointsItem();

    QRectF boundingRect() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    /**
     * The image of the visible series, in their draw order. Per series the connection line, then the points.
     * size_px: size of the image in pixels. scale: pixels per layout pixel, for the marker sizes and line widths.
     * value_range: the values at the borders of the image, in chart coordinates.
     */
    static QImage drawPoints(const ScatterPlotDisplayData& data,
                             bool connection_lines,
                             const QSize& size_px,
                             double scale,
                             const QRectF& value_range);

    /// Value range of an axis in chart coordinates, a date time axis counts in milliseconds since epoch.
    static boost::optional<std::pair<double, double>> axisRange(const QtCharts::QAbstractAxis* axis);
    /// Value ranges of the first horizontal and vertical axis, as a rectangle in chart coordinates.
    static boost::optional<QRectF> axisRanges(const QtCharts::QChart* chart);

    static const double ConnectionLineWidth;

private:
    QtCharts::QChart*                             chart_ = nullptr;
    std::shared_ptr<const ScatterPlotDisplayData> data_;
    bool                                          connection_lines_ = false;

    // the image and what it was drawn for
    QImage image_;
    double image_scale_ = 0.0;
    QRectF image_value_range_;
};
