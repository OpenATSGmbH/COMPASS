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
#include "acidfilter.h"

class QLineEdit;

/**
 */
class ACIDFilterWidget : public DBFilterWidget
{
    Q_OBJECT

protected slots:
    void valueEditedSlot(const QString& value);

public:
    ACIDFilterWidget(ACIDFilter& filter);
    virtual ~ACIDFilterWidget();

    virtual void update();

protected:
    ACIDFilter& filter_;

    QLineEdit* value_edit_ {nullptr};
};
