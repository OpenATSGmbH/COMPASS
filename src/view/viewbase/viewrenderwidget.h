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

#include <QPalette>
#include <QWidget>

#include <boost/optional.hpp>

#include <memory>

class ViewToolSwitcher;

class QBoxLayout;
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

    /// The display and the rendered images use this palette in place of the application palette.
    /// A report export sets the light palette, so its images do not depend on the mode of the application.
    void setFixedPalette(const QPalette& palette);
    const boost::optional<QPalette>& fixedPalette() const { return fixed_palette_; }

    /// The widget then reports its height for a width (heightForWidth()), for a figure in a report section:
    /// width * ratio, not below min_height and not above the visible height of the scroll area around it.
    void setHeightForWidth(double ratio, int min_height);
    bool heightFollowsWidth() const { return height_ratio_.has_value(); }
    /// The height rule. The minimum wins over the maximum.
    static int limitedHeightForWidth(int width, double ratio, int min_height, int max_height);

    /// The widget does not use the mouse wheel, so a scroll area around it scrolls.
    void setWheelIgnored(bool ignored);
    bool wheelIgnored() const { return wheel_ignored_; }

    /// The widget is used for render() only: redraw() builds no display, so the widget is never drawn.
    /// A report export sets this, it would build each display twice otherwise.
    void setImageOnly(bool image_only) { image_only_ = image_only; }
    bool imageOnly() const { return image_only_; }

    /// The display stays, but does not count as drawn. For the display a view shows before its first load.
    void resetDrawState();

    virtual void viewInfoJSON(nlohmann::json& info) const {}

protected:
    enum class RenderState
    {
        NotDrawn,    // not drawn yet
        Drawn,       // drawn, but no content
        DrawnContent // drawn with content
    };

    void setDrawState(RenderState state) { draw_state_ = state; }

    /// the layout of the widget, the derived class adds its display to it
    QBoxLayout* contentLayout() { return content_layout_; }

    bool eventFilter(QObject* watched, QEvent* event) override;
    void showEvent(QShowEvent* event) override;

    /// another mode tool became the current tool
    virtual void toolChanged(int id) {}

private:
    class ContentLayout;

    int heightForWidthRule(int width) const;
    QWidget* scrollViewport() const;

    std::unique_ptr<ViewToolSwitcher> tool_switcher_;
    QBoxLayout*                       content_layout_ = nullptr;

    RenderState draw_state_ = RenderState::NotDrawn;

    boost::optional<QPalette> fixed_palette_;

    boost::optional<double> height_ratio_; // set: the height follows the width
    int                     height_min_   = 0;
    bool                    wheel_ignored_ = false;
    bool                    image_only_    = false;
};
