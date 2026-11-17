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
 */
ViewRenderWidget::ViewRenderWidget(QWidget* parent)
:   QWidget       (parent)
,   tool_switcher_(new ViewToolSwitcher)
{
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
    return draw_state_ == DrawState::Drawn ||
           draw_state_ == DrawState::DrawnContent;
}

/**
 * Checks if the display has been drawn with content.
 */
bool ViewRenderWidget::hasContent() const
{
    return draw_state_ == DrawState::DrawnContent;
}

/**
 */
void ViewRenderWidget::resetDrawState()
{
    draw_state_ = DrawState::NotDrawn;
}
