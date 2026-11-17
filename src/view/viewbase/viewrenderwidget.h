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

#include "viewtool.h"
#include "json_fwd.hpp"

#include <QWidget>

#include <memory>

class ViewToolSwitcher;

class QImage;
class QSize;

/**
 * Base class of the render widgets. A render widget draws the display data of a view model.
 * It does not know the model: its owner gives it the display data and the render options in one call,
 * setDisplayData(data, options) of the view specific class. So a report can use it without a view.
 *
 * The render widget owns the tools (registerTool()). The host decides if it shows a toolbar for them.
 */
class ViewRenderWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ViewRenderWidget(QWidget* parent = nullptr);
    virtual ~ViewRenderWidget();

    void registerTool(const ViewTool& tool);
    ViewToolSwitcher& toolSwitcher() { return *tool_switcher_; }

    /// builds the display from the display data and the render options set last
    virtual void redraw() = 0;
    /// removes the display, the widget is not drawn afterwards
    virtual void clear() = 0;
    /// renders the display data into an image of size * scale pixels, with the layout of the given size
    virtual bool render(QImage& image, const QSize& size, double scale = 1.0) = 0;

    bool isDrawn() const;
    bool hasContent() const;

    /// The display stays, but does not count as drawn. For the display a view shows before its first load.
    void resetDrawState();

    virtual void viewInfoJSON(nlohmann::json& info) const {}

protected:
    enum class DrawState
    {
        NotDrawn,    // not drawn yet
        Drawn,       // drawn, but no content
        DrawnContent // drawn with content
    };

    void setDrawState(DrawState state) { draw_state_ = state; }

    /// another mode tool became the current tool
    virtual void toolChanged(int id) {}

private:
    std::unique_ptr<ViewToolSwitcher> tool_switcher_;

    DrawState draw_state_ = DrawState::NotDrawn;
};
