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

#include "compass_view_export.h"

#include <chartview.h>

#include <QObject>
#include <QWidget>

class HistogramViewRenderWidget;

namespace QtCharts
{
class QBarSet;

/**
 */
class COMPASS_VIEW_API HistogramViewChartView : public ChartView
{
    Q_OBJECT
public:
    HistogramViewChartView(HistogramViewRenderWidget* render_widget, 
                           QChart* chart, 
                           QWidget* parent = nullptr);
    virtual ~HistogramViewChartView();

protected:
    void sendSelectedRegion();

    void wheelEvent(QWheelEvent* event) override;

    virtual bool handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos) override;
    virtual bool handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos) override;
    virtual bool handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos) override;

    HistogramViewRenderWidget* render_widget_ {nullptr};

    QPoint  p1_;
    QPointF p1_data_;

    QPoint  p2_;
    QPointF p2_data_;
};

}
