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

#include "chartrenderwidget.h"
#include "chartview.h"
#include "viewtoolswitcher.h"
#include "traced_assert.h"

#include <QBoxLayout>
#include <QImage>
#include <QPainter>

#include <QtCharts/QChart>

#include <cmath>

namespace
{
    const int DotsPerMeter96Dpi = 3780; // 96 / 0.0254
}

/**
 */
ChartRenderWidget::ChartRenderWidget(QWidget* parent)
:   ViewRenderWidget(parent)
{
    // queued: the callback can build the chart again, which must not happen during the mouse event
    // of the chart view that reports the rectangle
    connect(this, &ChartRenderWidget::rectangleFinishedSignal,
            this, &ChartRenderWidget::deliverRectangle, Qt::QueuedConnection);
}

/**
 */
ChartRenderWidget::~ChartRenderWidget() = default;

/**
 */
void ChartRenderWidget::redraw()
{
    if (imageOnly())
        return;

    releaseChartView();

    bool has_content = false;

    chart_view_ = buildChartView(has_content, false);
    traced_assert(chart_view_);

    chart_view_->setObjectName("chart_view");

    //first, the derived class can add widgets right of the chart
    contentLayout()->insertWidget(0, chart_view_.get());

    setDrawState(has_content ? RenderState::DrawnContent : RenderState::Drawn);
}

/**
 */
std::unique_ptr<ChartView> ChartRenderWidget::buildChartView(bool& has_content, bool for_image)
{
    auto view = createChartView(has_content);
    if (!view)
        return view;

    //no hinting, as Qt draws on a scaled display: the image has more pixels than its layout
    view->usePixelSizeFonts(for_image ? QFont::PreferNoHinting : QFont::PreferDefaultHinting);
    view->setWheelIgnored(wheelIgnored());

    if (narrowMargins())
        view->useNarrowMargins();

    if (fixedPalette())
        view->setFixedPalette(*fixedPalette());

    finishChartView(*view);

    return view;
}

/**
 */
void ChartRenderWidget::clear()
{
    releaseChartView();

    setDrawState(RenderState::NotDrawn);
}

/**
 * Takes the chart view out of the layout and destroys it via deleteLater, not in place:
 * charts own a QGraphicsScene with their axes, and this can run from an asynchronous commit
 * while Qt still has pending events for it (destroying in place crashed inside QtCharts:
 * QGraphicsScene::clear -> ~QChart -> deleteAllAxes). Hidden right away.
 */
void ChartRenderWidget::releaseChartView()
{
    QWidget* widget = chart_view_.release();
    if (!widget)
        return;

    contentLayout()->removeWidget(widget);

    widget->setParent(nullptr);
    widget->hide();
    widget->deleteLater();
}

/**
 * Draws a new chart view, which is never shown, so the chart on the screen keeps its size.
 *
 * The image is created here with an explicit device pixel ratio and resolution, and the chart is drawn
 * from its scene. QWidget::grab() takes both from the screen, so its image depends on the display scaling.
 */
bool ChartRenderWidget::render(QImage& image, const QSize& size, double scale)
{
    if (size.isEmpty() || scale <= 0.0)
        return false;

    image = createImage(size, scale);

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);

    return drawChartImage(painter, size);
}

/**
 */
QImage ChartRenderWidget::createImage(const QSize& size, double scale)
{
    const QSize size_px((int)std::lround(size.width() * scale), (int)std::lround(size.height() * scale));

    QImage image(size_px, QImage::Format_ARGB32_Premultiplied);
    image.setDevicePixelRatio(scale);
    image.setDotsPerMeterX(DotsPerMeter96Dpi);
    image.setDotsPerMeterY(DotsPerMeter96Dpi);
    image.fill(Qt::white);

    return image;
}

/**
 * The items the chart view paints itself (e.g. the grid) are drawn above the scene, as on the screen.
 */
bool ChartRenderWidget::drawChartImage(QPainter& painter, const QSize& size)
{
    bool has_content = false;

    auto view = buildChartView(has_content, true);
    if (!view || !view->chart() || !view->scene())
        return false;

    view->layoutForSize(size);

    chartImageLaidOut(*view);

    const QRectF rect(QPointF(0, 0), QSizeF(size));

    view->scene()->render(&painter, rect, rect);
    view->paintImageItems(painter);

    return true;
}

/**
 */
void ChartRenderWidget::requestRectangle(std::function<void(const QRectF&)> callback)
{
    rectangle_callback_ = std::move(callback);
    rectangle_tool_     = toolSwitcher().currentTool();
}

/**
 */
bool ChartRenderWidget::rectangleRequested() const
{
    return static_cast<bool>(rectangle_callback_);
}

/**
 */
void ChartRenderWidget::rectangleSelected(const QRectF& chart_rect)
{
    if (!rectangleRequested())
        return;

    emit rectangleFinishedSignal(chart_rect);
}

/**
 */
void ChartRenderWidget::deliverRectangle(const QRectF& chart_rect)
{
    //the tool was changed in between
    if (!rectangleRequested())
        return;

    auto callback = std::move(rectangle_callback_);

    rectangle_callback_ = nullptr;
    rectangle_tool_     = -1;

    callback(chart_rect);

    toolSwitcher().endCurrentTool();
}

/**
 */
void ChartRenderWidget::toolChanged(int id)
{
    //end the selection of the chart view if the tool has been changed
    if (chart_view_)
        chart_view_->onToolChanged();

    //the request belongs to the mode tool that made it
    if (id != rectangle_tool_)
    {
        rectangle_callback_ = nullptr;
        rectangle_tool_     = -1;
    }
}
