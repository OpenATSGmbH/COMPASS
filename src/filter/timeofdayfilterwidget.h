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

#include "dbfilterwidget.h"

#include <utility>
#include <vector>

class DBFilter;
class QPushButton;

/**
 */
class TimeOfDayFilterWidget : public DBFilterWidget
{
    Q_OBJECT

public:
    TimeOfDayFilterWidget(DBFilter& filter);
    virtual ~TimeOfDayFilterWidget();

    virtual void update() override;

private:
    void shiftWindow(int minutes);
    void updateButtons();
    std::pair<int, int> effectiveBoundsSecs() const;

    std::vector<std::pair<int, QPushButton*>> step_buttons_;
};
