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

#include "modecfilterwidget.h"
#include "textfielddoublevalidator.h"
#include "logger.h"
#include "rangeedit.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

using namespace std;
//using namespace Utils;

ModeCFilterWidget::ModeCFilterWidget(ModeCFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    const float ModeCMin    = -10000.0f;
    const float ModeCMax    =  100000.0f;
    const int   SliderSteps = 1000;

    min_value_edit_ = new QLineEdit();
    min_value_edit_->setValidator(new TextFieldDoubleValidator(ModeCMin, ModeCMax, Precision));
    connect(min_value_edit_, &QLineEdit::textEdited, this, &ModeCFilterWidget::minValueEditedSlot);

    addNameValuePair("Mode C Code >=", min_value_edit_);

    max_value_edit_ = new QLineEdit();
    max_value_edit_->setValidator(new TextFieldDoubleValidator(ModeCMin, ModeCMax, Precision));
    connect(max_value_edit_, &QLineEdit::textEdited, this, &ModeCFilterWidget::maxValueEditedSlot);

    addNameValuePair("Mode C Code <=", max_value_edit_);

    const QString limit0 = QString::number(ModeCMin, 'f', Precision);
    const QString limit1 = QString::number(ModeCMax, 'f', Precision);

    range_edit_ = new RangeEditFloat(SliderSteps, Precision);
    range_edit_->setLimits(limit0, limit1);
    range_edit_->connectToFields(min_value_edit_, max_value_edit_);

    addNameValuePair("", range_edit_);

    null_check_ = new QCheckBox();
    connect(null_check_, &QCheckBox::clicked, this, &ModeCFilterWidget::nullWantedChangedSlot);
    addNameValuePair("NULL Values", null_check_);

    update();
}

ModeCFilterWidget::~ModeCFilterWidget() = default;

void ModeCFilterWidget::update()
{
    DBFilterWidget::update();

    traced_assert(min_value_edit_);
    traced_assert(max_value_edit_);
    traced_assert(null_check_);

    const QString r0 = QString::number(filter_.minValue(), 'f', Precision);
    const QString r1 = QString::number(filter_.maxValue(), 'f', Precision);

    min_value_edit_->setText(r0);
    max_value_edit_->setText(r1);

    null_check_->setChecked(filter_.nullWanted());
}

void ModeCFilterWidget::minValueEditedSlot(const QString& value)
{
    if (!value.size())
    {
        loginf << "skipping empty string";
        return;
    }

    bool ok;

    float value_float = value.toFloat(&ok);
    traced_assert(ok);

    loginf << "'" << value_float << "'";

    filter_.minValue(value_float);
}

void ModeCFilterWidget::maxValueEditedSlot(const QString& value)
{
    if (!value.size())
    {
        loginf << "skipping empty string";
        return;
    }

    bool ok;

    float value_float = value.toFloat(&ok);
    traced_assert(ok);

    loginf << "'" << value_float << "'";

    filter_.maxValue(value_float);
}

void ModeCFilterWidget::nullWantedChangedSlot()
{
     traced_assert(null_check_);
    bool wanted = null_check_->checkState() == Qt::Checked;

    loginf << "start" << wanted;

    filter_.nullWanted(wanted);
}
