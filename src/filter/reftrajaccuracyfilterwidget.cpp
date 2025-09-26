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

#include "reftrajaccuracyfilterwidget.h"
//#include "stringconv.h"
#include "textfielddoublevalidator.h"
#include "logger.h"
//#include "rangeedit.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

using namespace std;
//using namespace Utils;

RefTrajAccuracyFilterWidget::RefTrajAccuracyFilterWidget(RefTrajAccuracyFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    min_value_edit_ = new QLineEdit();
    min_value_edit_->setValidator(new TextFieldDoubleValidator(0, 10e6, Precision));
    connect(min_value_edit_, &QLineEdit::textEdited, this, &RefTrajAccuracyFilterWidget::minValueEditedSlot);

    addNameValuePair("Accuracy <=", min_value_edit_);

    update();
}

RefTrajAccuracyFilterWidget::~RefTrajAccuracyFilterWidget()
{
}

void RefTrajAccuracyFilterWidget::update()
{
    DBFilterWidget::update();

    traced_assert(min_value_edit_);

    const QString r0 = QString::number(filter_.minValue(), 'f', Precision);

    min_value_edit_->setText(r0);
}

void RefTrajAccuracyFilterWidget::minValueEditedSlot(const QString& value)
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

