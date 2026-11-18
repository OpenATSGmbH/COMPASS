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

#include <algorithm>

class ScatterPlotViewRenderWidget;

namespace QtCharts
{

/**
 */
class COMPASS_VIEW_API ScatterPlotViewChartView : public ChartView
{
    Q_OBJECT
public:
    /// use_opengl: the series of the chart are drawn with OpenGL, then the selection rectangle is too
    ScatterPlotViewChartView (ScatterPlotViewRenderWidget* render_widget,
                              QChart* chart,
                              bool use_opengl,
                              QWidget* parent = nullptr);
    virtual ~ScatterPlotViewChartView();

    virtual void onToolChanged() override;

protected:
    void sendSelectedRegion();

    virtual void wheelEvent(QWheelEvent* event) override;

    virtual bool handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos) override;
    virtual bool handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos) override;
    virtual bool handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos) override;

    ScatterPlotViewRenderWidget* render_widget_ {nullptr};

    QPoint  p1_;
    QPointF p1_data_;

    QPoint  p2_;
    QPointF p2_data_;

    bool    drag_data_init_ {false};
    QPointF drag_data_;
};

}
