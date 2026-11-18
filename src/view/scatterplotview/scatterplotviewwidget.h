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

#include "variableviewwidget.h"

class ScatterPlotView;
class ScatterPlotViewRenderWidget;
class ScatterPlotViewConfigWidget;

class QSplitter;
class QTabWidget;

/**
*/
class ScatterPlotViewWidget : public VariableViewWidget
{
public:
    enum Mode
    {
        SelectionMode = 0,
        ZoomRectMode
    };

    ScatterPlotViewWidget(ScatterPlotView* view, QWidget* parent = nullptr);
    /// @brief Destructor
    virtual ~ScatterPlotViewWidget();

    ScatterPlotViewRenderWidget* renderWidget() { return render_widget_; }
    ScatterPlotViewConfigWidget* getViewConfigWidget();
    const ScatterPlotViewConfigWidget* getViewConfigWidget() const;

    /// @brief Returns the basis view
    ScatterPlotView* getView();

private:
    void registerTools(ScatterPlotView* view);

    ScatterPlotViewRenderWidget* render_widget_ = nullptr; // owned by the view widget (Qt parent)
};
