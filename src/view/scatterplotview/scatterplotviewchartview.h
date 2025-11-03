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

#include <chartview.h>

#include <QObject>
#include <QWidget>

#include <algorithm>

class ScatterPlotViewDataWidget;

namespace QtCharts
{

/**
 */
class ScatterPlotViewChartView : public ChartView
{
    Q_OBJECT
public:
    ScatterPlotViewChartView (ScatterPlotViewDataWidget* data_widget, QChart* chart, QWidget* parent = nullptr);
    virtual ~ScatterPlotViewChartView();

    virtual void onToolChanged() override;

signals:
    void rectangleSelectedSignal (QPointF p1, QPointF p2);

protected:
    void sendSelectedRegion();

    virtual void wheelEvent(QWheelEvent* event) override;

    virtual bool handleMousePress(Qt::MouseButtons buttons, const QPointF& widget_pos) override;
    virtual bool handleMouseRelease(Qt::MouseButtons buttons, const QPointF& widget_pos, bool update_pos) override;
    virtual bool handleMouseMove(Qt::MouseButtons buttons, const QPointF& widget_pos) override;

    ScatterPlotViewDataWidget* data_widget_ {nullptr};

    QPoint  p1_;
    QPointF p1_data_;

    QPoint  p2_;
    QPointF p2_data_;

    bool    drag_data_init_ {false};
    QPointF drag_data_;
};

}
