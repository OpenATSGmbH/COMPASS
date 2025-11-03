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
#include "reftrajaccuracyfilter.h"

class QCheckBox;
class QLineEdit;

class RangeEditFloat;

/**
 */
class RefTrajAccuracyFilterWidget : public DBFilterWidget
{
    Q_OBJECT

protected slots:
    void minValueEditedSlot(const QString& value);

public:
    RefTrajAccuracyFilterWidget(RefTrajAccuracyFilter& filter);
    virtual ~RefTrajAccuracyFilterWidget();

    virtual void update();

protected:
    static const int Precision = 2;

    RefTrajAccuracyFilter& filter_;

    QLineEdit* min_value_edit_ {nullptr};
};
