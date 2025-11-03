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

#include <QComboBox>
#include <QList>

class FilterConditionOperatorComboBox : public QComboBox
{
  public:
    FilterConditionOperatorComboBox(bool numeric_only = false, bool string_only = false)
    {
        QList<QString> stringlist;

        if (stringlist.size() == 0)
        {
            stringlist.append("=");
            stringlist.append("!=");

            if (!string_only)
            {
                stringlist.append(">");
                stringlist.append(">=");
                stringlist.append("<");
                stringlist.append("<=");
            }

            stringlist.append("IN");
            stringlist.append("NOT IN");

            if (!numeric_only)
            {
                stringlist.append("LIKE");
            }

            stringlist.append("IS");
            stringlist.append("IS NOT");
        }

        /* Populate the comboBox */
        addItems(stringlist);
    }
    virtual ~FilterConditionOperatorComboBox() {}
};
