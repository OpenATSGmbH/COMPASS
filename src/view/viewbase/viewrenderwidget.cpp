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

#include "viewrenderwidget.h"
#include "viewtoolswitcher.h"

#include <QAbstractScrollArea>
#include <QEvent>
#include <QHBoxLayout>

#include <algorithm>
#include <cmath>

namespace
{
    void setCursorRecursive(QWidget* w, const QCursor& cursor)
    {
        w->setCursor(cursor);

        for (auto c : w->findChildren<QWidget*>())
            c->setCursor(cursor);
    }
}

/**
 * Layout of a render widget. Qt asks the layout of a widget for the height for a width, not the widget,
 * so the layout reports the height rule of the render widget.
 */
class ViewRenderWidget::ContentLayout : public QHBoxLayout
{
public:
    explicit ContentLayout(const ViewRenderWidget& render_widget) : render_widget_(render_widget) {}

    bool hasHeightForWidth() const override
    {
        return render_widget_.heightFollowsWidth() || QHBoxLayout::hasHeightForWidth();
    }

    int heightForWidth(int width) const override
    {
        return render_widget_.heightFollowsWidth() ? render_widget_.heightForWidthRule(width)
                                                   : QHBoxLayout::heightForWidth(width);
    }

    int minimumHeightForWidth(int width) const override
    {
        return render_widget_.heightFollowsWidth() ? render_widget_.heightForWidthRule(width)
                                                   : QHBoxLayout::minimumHeightForWidth(width);
    }

    /// with the height rule the widget takes no free height, the display in it would ask for it
    Qt::Orientations expandingDirections() const override
    {
        return render_widget_.heightFollowsWidth() ? Qt::Horizontal : QHBoxLayout::expandingDirections();
    }

private:
    const ViewRenderWidget& render_widget_;
};

/**
 */
ViewRenderWidget::ViewRenderWidget(QWidget* parent)
:   QWidget       (parent)
,   tool_switcher_(new ViewToolSwitcher)
{
    content_layout_ = new ContentLayout(*this);
    content_layout_->setMargin(0);

    setLayout(content_layout_);

    //the shortcuts of the tools are active in the window of the render widget
    tool_switcher_->setDataWidget(this);

    connect(tool_switcher_.get(), &ViewToolSwitcher::toolChanged, this, [ this ] (int id, const QCursor& cursor)
    {
        //update the cursor to the tool's cursor
        setCursorRecursive(this, cursor);

        toolChanged(id);
    });
}

/**
 */
ViewRenderWidget::~ViewRenderWidget() = default;

/**
 */
void ViewRenderWidget::registerTool(const ViewTool& tool)
{
    tool_switcher_->registerTool(tool);
}

/**
 * Checks if the display has been drawn.
 */
bool ViewRenderWidget::isDrawn() const
{
    return draw_state_ == RenderState::Drawn ||
           draw_state_ == RenderState::DrawnContent;
}

/**
 * Checks if the display has been drawn with content.
 */
bool ViewRenderWidget::hasContent() const
{
    return draw_state_ == RenderState::DrawnContent;
}

/**
 */
void ViewRenderWidget::setFixedPalette(const QPalette& palette)
{
    fixed_palette_ = palette;

    if (isDrawn())
        redraw();
}

/**
 */
void ViewRenderWidget::setHeightForWidth(double ratio, int min_height)
{
    height_ratio_ = ratio;
    height_min_   = min_height;

    QSizePolicy policy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);

    setSizePolicy(policy);
    updateGeometry();
}

/**
 */
int ViewRenderWidget::limitedHeightForWidth(int width, double ratio, int min_height, int max_height)
{
    const int height = (int)std::lround(width * ratio);

    return std::max(min_height, std::min(height, max_height));
}

/**
 */
int ViewRenderWidget::heightForWidthRule(int width) const
{
    auto viewport = scrollViewport();

    return limitedHeightForWidth(width, height_ratio_.value(), height_min_,
                                 viewport ? viewport->height() : QWIDGETSIZE_MAX);
}

/**
 * The viewport of the scroll area around the widget, its height is the visible height.
 */
QWidget* ViewRenderWidget::scrollViewport() const
{
    for (auto w = parentWidget(); w; w = w->parentWidget())
        if (auto scroll_area = qobject_cast<QAbstractScrollArea*>(w))
            return scroll_area->viewport();

    return nullptr;
}

/**
 */
void ViewRenderWidget::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    //the maximum height follows the scroll area, see eventFilter()
    if (heightFollowsWidth())
        if (auto viewport = scrollViewport())
            viewport->installEventFilter(this);
}

/**
 * A new height of the scroll area changes the height limit of the widget. The layouts keep the height
 * of a width until they are invalidated, and a change of the height alone does not invalidate them.
 */
bool ViewRenderWidget::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::Resize && heightFollowsWidth() && watched == scrollViewport())
        updateGeometry();

    return QWidget::eventFilter(watched, event);
}

/**
 */
void ViewRenderWidget::setWheelIgnored(bool ignored)
{
    wheel_ignored_ = ignored;

    if (isDrawn())
        redraw();
}

/**
 */
void ViewRenderWidget::setNarrowMargins(bool narrow)
{
    narrow_margins_ = narrow;

    if (isDrawn())
        redraw();
}

/**
 */
void ViewRenderWidget::resetDrawState()
{
    draw_state_ = RenderState::NotDrawn;
}
