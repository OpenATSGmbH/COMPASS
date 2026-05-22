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

class UnitManager;

/**
 * @brief Sets a Unit using a context menu
 */
class UnitSelectionWidget : public QPushButton
{
    Q_OBJECT

  protected slots:
    void triggerSlot(QAction* action);
    void showMenuSlot();

  public:
    UnitSelectionWidget(UnitManager& unit_man, std::string& dimension, std::string& unit);
    UnitSelectionWidget(UnitManager& unit_man);
    virtual ~UnitSelectionWidget();

    void update(std::string& dimension, std::string& unit);
    void clear();

  protected:
    UnitManager& unit_man_;
    bool pointers_set_ {false};

    std::string* dimension_ {nullptr};
    std::string* unit_ {nullptr};

    QMenu menu_;

    void createMenu();
};
