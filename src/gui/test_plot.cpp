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

#include "test_plot.h"

#include <QPaintDevice>
#include <QPainter>
#include <QImage>
#include <QDialog>
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>

/**
*/
TestPlot::TestPlot(QPaintDevice* pd, const QRectF& data_bounds, int border)
:   pd_         (pd         )
,   data_bounds_(data_bounds)
,   border_     (border     )
{
}

/**
*/
TestPlot::~TestPlot() = default;

/**
*/
QPointF TestPlot::mapPos(double x, double y) const
{
    double fx = (x - data_bounds_.x()) / data_bounds_.width();
    double fy = (y - data_bounds_.y()) / data_bounds_.height();

    double x_pd = border_ + (pd_->width()  - 2 * border_) * fx;
    double y_pd = border_ + (pd_->height() - 2 * border_) * fy;

    return QPointF(x_pd, pd_->height() - y_pd);
}

/**
*/
void TestPlot::drawPoints(const std::vector<QPointF>& points, const QColor& color, double point_size)
{
    if (points.size() < 1)
        return;

    QPainter painter(pd_);
    QPen pen(color, point_size);
    QBrush brush(color);

    painter.setPen(pen);
    painter.setBrush(brush);
    
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[ i ];
        painter.drawPoint(mapPos(p.x(), p.y()));
    }
}

/**
*/
void TestPlot::drawLines(const std::vector<QPointF>& points, const QColor& color, double line_width)
{
    if (points.size() < 2)
        return;

    QPainter painter(pd_);
    QPen pen(color, line_width);
    QBrush brush(color);

    painter.setPen(pen);
    painter.setBrush(brush);
    
    for (size_t i = 1; i < points.size(); ++i)
    {
        const auto& p0 = points[ i - 1 ];
        const auto& p1 = points[ i     ];

        painter.drawLine(mapPos(p0.x(), p0.y()), mapPos(p1.x(), p1.y()));
    }
}

/**
*/
void TestPlot::drawPoints(const std::vector<Eigen::Vector2d>& points, const QColor& color, double point_size)
{
    std::vector<QPointF> points_qt(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        points_qt[ i ] = QPointF(points[ i ].x(), points[ i ].y());

    drawPoints(points_qt, color, point_size);
}

/**
*/
void TestPlot::drawLines(const std::vector<Eigen::Vector2d>& points, const QColor& color, double line_width)
{
    std::vector<QPointF> points_qt(points.size());
    for (size_t i = 0; i < points.size(); ++i)
        points_qt[ i ] = QPointF(points[ i ].x(), points[ i ].y());

    drawLines(points_qt, color, line_width);
}

/**
*/
void TestPlot::drawCircle(double x, double y, double r, const QColor& color, double line_width)
{
    QPainter painter(pd_);
    QPen pen(color, line_width);
    QBrush brush(color);

    painter.setPen(pen);
    painter.setBrush(brush);

    painter.drawEllipse(mapPos(x, y), r, r);
}

/**
*/
QSize TestPlot::sizeHint(int width, const QRectF& data_bounds)
{
    double aspect = data_bounds.height() / data_bounds.width();
    return QSize(width, width * aspect);
}

/**
*/
TestImage::TestImage(int w, int h, const QRectF& data_bounds, int border)
:   image_(new QImage(w, h, QImage::Format_RGB32))
,   plot_ (image_.get(), data_bounds, border     )
{
    image_->fill(Qt::white);
}

/**
*/
TestImage::~TestImage() = default;

/**
*/
TestPlot& TestImage::plot()
{
    return plot_;
}

/**
*/
void TestImage::showBlocking(const QString& title, 
                             const QSize& size, 
                             QWidget* parent)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(title);

    QHBoxLayout* layout = new QHBoxLayout;
    dialog.setLayout(layout);

    QLabel* label = new QLabel("");
    label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    label->setPixmap(QPixmap::fromImage(*image_));

    layout->addWidget(label);

    if (!size.isEmpty())
        dialog.resize(size);

    dialog.exec();
}
