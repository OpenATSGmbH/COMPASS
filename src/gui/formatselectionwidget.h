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

#include <QMenu>
#include <QPushButton>

#include "format.h"
#include "property.h"

/**
 * @brief Sets a Unit using a context menu
 */
class FormatSelectionWidget : public QPushButton
{
    Q_OBJECT

  protected slots:
    /// @brief Called when menu action is executed
    void triggerSlot(QAction* action);
    /// @brief Shows the context menu
    void showMenuSlot();

  public:
    /// @brief Constructor, references directly used
    FormatSelectionWidget(PropertyDataType data_type, Format& format);
    /// @brief Destructor
    virtual ~FormatSelectionWidget();

    void update(PropertyDataType data_type, Format& format);

  protected:
    PropertyDataType data_type_;
    Format& format_;

    /// Context menu
    QMenu menu_;

    void createMenu();
};
