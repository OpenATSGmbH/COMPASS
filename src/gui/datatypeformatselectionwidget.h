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

#include "format.h"
//#include "property.h"

#include <QMenu>
#include <QPushButton>

#include <memory>

/**
 * @brief Sets a Unit using a context menu
 */
class DataTypeFormatSelectionWidget : public QPushButton
{
    Q_OBJECT

  protected slots:
    /// @brief Called when menu action is executed
    void triggerSlot(QAction* action);
    /// @brief Shows the context menu
    void showMenuSlot();

  public:
    /// @brief Constructor, references directly used
    DataTypeFormatSelectionWidget(std::string& data_type_str, Format& format);
    DataTypeFormatSelectionWidget();
    /// @brief Destructor
    virtual ~DataTypeFormatSelectionWidget();

    void update(std::string& data_type_str, Format& format);
    void clear();

  protected:
    bool pointers_set_ {false};

    std::string* data_type_str_ {nullptr};
    Format* format_ {nullptr};

    /// Context menu
    std::unique_ptr<QMenu> menu_ {nullptr};

    void showValues();
    void createMenu();
};
