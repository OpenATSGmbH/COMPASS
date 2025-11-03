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

#include "primaryonlyfilterwidget.h"
#include "dbcontent.h"

#include <QFormLayout>
#include <QLabel>

PrimaryOnlyFilterWidget::PrimaryOnlyFilterWidget(PrimaryOnlyFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    addNameValuePair(DBContent::meta_var_m3a_.name(), "IS NULL");
    addNameValuePair(DBContent::meta_var_mc_.name(), "IS NULL");
    addNameValuePair(DBContent::meta_var_acad_.name(), "IS NULL");
    addNameValuePair(DBContent::meta_var_acid_.name(), "IS NULL");
    addNameValuePair(DBContent::meta_var_detection_type_.name(), "IN (1,3,6,7) or NULL");
}

PrimaryOnlyFilterWidget::~PrimaryOnlyFilterWidget() = default;
