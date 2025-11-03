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

#include "viewwidget.h"

class TableView;
class TableViewDataWidget;
class TableViewConfigWidget;

class QSplitter;
class QTabWidget;

/**
 * @brief Used for textual data display in a TableView.
 *
 * Consists of a TableViewDataWidget for data view and a TableViewConfigWidget for configuration
 * and starting the loading process.
 */
class TableViewWidget : public ViewWidget
{
  public:
    /// @brief Constructor
    TableViewWidget(const std::string& class_id, 
                      const std::string& instance_id,
                      Configurable* config_parent, 
                      TableView* view, 
                      QWidget* parent = NULL);
    /// @brief Destructor
    virtual ~TableViewWidget();

    TableViewDataWidget* getViewDataWidget();
    const TableViewDataWidget* getViewDataWidget() const;
    TableViewConfigWidget* getViewConfigWidget();
    const TableViewConfigWidget* getViewConfigWidget() const;

    /// @brief Returns the basis view
    TableView* getView();
};
