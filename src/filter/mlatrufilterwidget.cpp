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

#include "mlatrufilterwidget.h"
#include "mlatrufilter.h"
//#include "stringconv.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>

using namespace std;
//using namespace Utils;

MLATRUFilterWidget::MLATRUFilterWidget(MLATRUFilter& filter)
    : DBFilterWidget(filter), filter_(filter)
{
    value_edit_ = new QLineEdit();
    //value_edit_->setValidator(new TextFieldDoubleValidator(0, 100000, 0));
    connect(value_edit_, &QLineEdit::textEdited, this, &MLATRUFilterWidget::valueEditedSlot);

    addNameValuePair("MLAT RUs IN", value_edit_);

    update();
}

MLATRUFilterWidget::~MLATRUFilterWidget() = default;

void MLATRUFilterWidget::update()
{
    DBFilterWidget::update();

    traced_assert(value_edit_);

    value_edit_->setText(filter_.rus().c_str());
}

void MLATRUFilterWidget::valueEditedSlot(const QString& value)
{
    string value_str = value.toStdString();

    loginf << "'" << value_str << "'";

    filter_.rus(value_str);
}
