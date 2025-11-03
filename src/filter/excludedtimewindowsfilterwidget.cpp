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

#include "excludedtimewindowsfilterwidget.h"

#include <QLabel>

ExcludedTimeWindowsFilterWidget::ExcludedTimeWindowsFilterWidget(ExcludedTimeWindowsFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    tw_widget_ = new TimeWindowCollectionWidget(filter_.timeWindows());

    int insert_row = child_layout_->rowCount();

    auto lwidget = new QLabel("Time Windows");
    lwidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

    tw_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    child_layout_->addWidget(lwidget, insert_row, 0);
    child_layout_->addWidget(tw_widget_ , insert_row, 1);
}

ExcludedTimeWindowsFilterWidget::~ExcludedTimeWindowsFilterWidget()
{
    tw_widget_ = nullptr;
}

void ExcludedTimeWindowsFilterWidget::update()
{
    DBFilterWidget::update();

    tw_widget_->refreshList();
}
