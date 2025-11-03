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

#include "labelplacement_defs.h"

#include <vector>
#include <limits>

//general helpers for automated label placement
namespace label_placement
{
    /**
     * Collect the (scaled) bounding boxes for all given labels.
     */
    std::vector<QRectF> collectBoundingBoxes(const std::vector<Label>& labels, 
                                             double tx, 
                                             double ty)
    {
        if (labels.empty())
            return {};

        size_t n = labels.size();

        std::vector<QRectF> bboxes(n);
        for (size_t i = 0; i < n; ++i)
        {
            const auto& l = labels[ i ];
            bboxes[ i ] = l.boundingBox(tx, ty);
        }

        return bboxes;
    }

    /**
     * Compute a minimum frame covering all given label data.
     */
    QRectF dataFrame(const std::vector<Label>& labels)
    {
        size_t n = labels.size();

        if (n < 2)
            return QRectF();

        double xmin = std::numeric_limits<double>::max();
        double ymin = std::numeric_limits<double>::max();
        double xmax = std::numeric_limits<double>::min();
        double ymax = std::numeric_limits<double>::min();

        auto checkPos = [ & ] (double x, double y) 
        {
            if (x < xmin)
                xmin = x;
            if (y < ymin)
                ymin = y;
            if (x > xmax)
                xmax = x;
            if (y > ymax)
                ymax = y;
        };

        for (size_t i = 0; i < n; ++i)
        {
            const auto& l = labels[ i ];

            checkPos(l.x, l.y);
            checkPos(l.x_anchor, l.y_anchor);
        }

        if (xmin > xmax || ymin > ymax)
            return QRectF();

        return QRectF(xmin, ymin, xmax - xmin, ymax - ymin);
    }

    /**
     * Normalize the position using the given data frame.
     */
    void normalizeData(QPointF& pos, const QRectF& frame)
    {
        if (frame.isEmpty())
            return;

        const double s = 1.0 / std::max(frame.width(), frame.height());
        
        pos = QPointF((pos.x() - frame.left()) * s, (pos.y() - frame.top()) * s);
    }

    /**
     * Denormalize the position using the given data frame.
     */
    void denormalizeData(QPointF& pos, const QRectF& frame)
    {
        if (frame.isEmpty())
            return;

        const double s = std::max(frame.width(), frame.height());
        
        pos = QPointF(pos.x() * s + frame.left(), pos.y() * s + frame.top());
    }

    /**
     * Normalize the value using the given data frame.
     */
    void normalizeData(double& v, const QRectF& frame)
    {
        if (frame.isEmpty())
            return;

        const double s = 1.0 / std::max(frame.width(), frame.height());
        
        v *= s;
    }

    /**
     * Normalize the rectangle using the given data frame.
     */
    void normalizeData(QRectF& r, const QRectF& frame)
    {
        if (frame.isEmpty())
            return;

        QPointF p = r.topLeft();
        double  w = r.width();
        double  h = r.height();

        normalizeData(p, frame);
        normalizeData(w, frame);
        normalizeData(h, frame);

        r = QRectF(p.x(), p.y(), w, h);
    }
}
