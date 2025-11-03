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
#include "modecfilter.h"

class QCheckBox;
class QLineEdit;

class RangeEditFloat;

/**
 */
class ModeCFilterWidget : public DBFilterWidget
{
    Q_OBJECT

protected slots:
    void minValueEditedSlot(const QString& value);
    void maxValueEditedSlot(const QString& value);
    void nullWantedChangedSlot();

public:
    ModeCFilterWidget(ModeCFilter& filter);
    virtual ~ModeCFilterWidget();

    virtual void update();

protected:
    static const int Precision = 2;

    ModeCFilter& filter_;

    QLineEdit* min_value_edit_ {nullptr};
    QLineEdit* max_value_edit_ {nullptr};
    QCheckBox* null_check_ {nullptr};
    RangeEditFloat* range_edit_ {nullptr};
};
