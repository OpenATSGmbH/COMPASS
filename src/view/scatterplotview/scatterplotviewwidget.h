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
class ScatterPlotViewDataToolWidget;
class ScatterPlotViewConfigWidget;
class ScatterPlotViewDataWidget;

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

    /// @brief Constructor
    ScatterPlotViewWidget(const std::string& class_id, 
                          const std::string& instance_id,
                          Configurable* config_parent, 
                          ScatterPlotView* view, 
                          QWidget* parent = NULL);
    /// @brief Destructor
    virtual ~ScatterPlotViewWidget();

    ScatterPlotViewDataWidget* getViewDataWidget();
    const ScatterPlotViewDataWidget* getViewDataWidget() const;
    ScatterPlotViewConfigWidget* getViewConfigWidget();
    const ScatterPlotViewConfigWidget* getViewConfigWidget() const;

    /// @brief Returns the basis view
    ScatterPlotView* getView();
};
