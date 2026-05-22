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
#include "timestampfilter.h"

#include <vector>
#include <utility>

class QDateTimeEdit;
class QPushButton;

/**
 */
class TimestampFilterWidget : public DBFilterWidget
{
    Q_OBJECT

public slots:
    void minDateTimeChanged(const QDateTime& datetime);
    void maxDateTimeChanged(const QDateTime& datetime);

public:
    TimestampFilterWidget(TimestampFilter& filter);
    virtual ~TimestampFilterWidget();

    virtual void update();

private:
    void updateStepButtons();

protected:
    TimestampFilter& filter_;

    QDateTimeEdit* min_edit_ {nullptr};
    QDateTimeEdit* max_edit_ {nullptr};

    std::vector<std::pair<int, QPushButton*>> step_buttons_;

    bool update_active_ {false};
};
