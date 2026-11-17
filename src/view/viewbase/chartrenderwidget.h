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

#include "viewrenderwidget.h"

#include <QRectF>

#include <functional>
#include <memory>

class ChartView;

/**
 * Base class of the render widgets that show a chart. Holds the chart view and offers the mouse services
 * of the tools.
 *
 * The derived class builds a complete chart view from its display data (createChartView()). redraw() shows
 * it in the widget, render() draws it into an image. So the drawing code exists once.
 */
class ChartRenderWidget : public ViewRenderWidget
{
    Q_OBJECT
public:
    explicit ChartRenderWidget(QWidget* parent = nullptr);
    virtual ~ChartRenderWidget();

    void redraw() override;
    void clear() override;
    bool render(QImage& image, const QSize& size, double scale = 1.0) override;

    ChartView* chartView() { return chart_view_.get(); }
    const ChartView* chartView() const { return chart_view_.get(); }

    // mouse services

    void setDragEnabled(bool enabled) { drag_enabled_ = enabled; }
    bool dragEnabled() const { return drag_enabled_; }

    /// The next rectangle of the user (chart coordinates) goes to the callback. Then the default mode tool
    /// becomes active again. A change of the mode tool drops the request.
    void requestRectangle(std::function<void(const QRectF&)> callback);
    bool rectangleRequested() const;

    /// The user finished a rectangle (chart coordinates), called by the chart view.
    void rectangleSelected(const QRectF& chart_rect);

signals:
    void rectangleFinishedSignal(const QRectF& chart_rect); // internal, delivers the rectangle queued

protected:
    /// Builds a new chart view from the current display data and render options.
    /// has_content: the chart shows data, not the empty display.
    virtual std::unique_ptr<ChartView> createChartView(bool& has_content) = 0;

    void toolChanged(int id) override;

private:
    /// The chart view of createChartView() with the settings that are the same for all charts.
    /// for_image: the chart is drawn into an image that must not depend on the screen.
    std::unique_ptr<ChartView> buildChartView(bool& has_content, bool for_image);

    void deliverRectangle(const QRectF& chart_rect);
    void releaseChartView();

    std::unique_ptr<ChartView> chart_view_;

    bool drag_enabled_ = false;

    std::function<void(const QRectF&)> rectangle_callback_;
    int                                rectangle_tool_ = -1; // the mode tool that requested the rectangle
};
