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

#include <vector>
#include <memory>

#include <QColor>
#include <QRectF>
#include <QPointF>

#include <QDialog>

#include <Eigen/Core>

class QPaintDevice;
class QPainter;
class QImage;
class QWidget;

/**
*/
class TestPlot
{
public:
    TestPlot(QPaintDevice* pd, const QRectF& data_bounds = QRectF(0.0, 0.0, 1.0, 1.0), int border = 10);
    virtual ~TestPlot();

    void drawPoints(const std::vector<QPointF>& points, const QColor& color, double point_size);
    void drawLines(const std::vector<QPointF>& points, const QColor& color, double line_width);
    void drawPoints(const std::vector<Eigen::Vector2d>& points, const QColor& color, double point_size);
    void drawLines(const std::vector<Eigen::Vector2d>& points, const QColor& color, double line_width);
    void drawCircle(double x, double y, double r, const QColor& color, double line_width);

    static QSize sizeHint(int width, const QRectF& data_bounds);

private:
    void init();
    QPointF mapPos(double x, double y) const;

    QPaintDevice* pd_;
    QRectF        data_bounds_;
    int           border_;
};

/**
*/
class TestImage
{
public:
    TestImage(int w, int h, const QRectF& data_bounds = QRectF(0.0, 0.0, 1.0, 1.0), int border = 10);
    virtual ~TestImage();

    TestPlot& plot();

    void showBlocking(const QString& title = "", 
                      const QSize& size = QSize(), 
                      QWidget* parent = nullptr);

private:
    std::unique_ptr<QImage> image_;
    TestPlot                plot_;
};
