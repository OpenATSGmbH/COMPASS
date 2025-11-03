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
#include "mode3afilter.h"

class QLineEdit;

/**
 */
class Mode3AFilterWidget : public DBFilterWidget
{

    Q_OBJECT

protected slots:
    void valueEditedSlot(const QString& value);

public:
    Mode3AFilterWidget(Mode3AFilter& filter);
    virtual ~Mode3AFilterWidget();

    virtual void update();

protected:
    Mode3AFilter& filter_;

    QLineEdit* value_edit_ {nullptr};
};
