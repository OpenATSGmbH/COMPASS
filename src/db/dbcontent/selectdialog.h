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

#include "dbcontent/dbcontentcombobox.h"
//#include "textfielddoublevalidator.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>

namespace dbContent
{

class SelectDBContentDialog : public QDialog
{
    Q_OBJECT

  public:
    SelectDBContentDialog()
    {
        setMinimumWidth(300);

        QVBoxLayout* main_layout = new QVBoxLayout();

        cat_edit_ = new QLineEdit("Category"); // TODO validate to uint´
        main_layout->addWidget(cat_edit_);

        object_box_ = new DBContentComboBox(false, false);
        main_layout->addWidget(object_box_);

        QDialogButtonBox* button_box =
            new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        connect(button_box, SIGNAL(accepted()), this, SLOT(accept()));
        connect(button_box, SIGNAL(rejected()), this, SLOT(reject()));

        main_layout->addWidget(button_box);

        setLayout(main_layout);

        setWindowTitle(tr("Select Name and DBContent"));
    }

    unsigned int category()
    {
        traced_assert(cat_edit_);
        return cat_edit_->text().toUInt();
    }

    std::string selectedObject()
    {
        traced_assert(object_box_);
        return object_box_->currentText().toStdString();
    }

  protected:
    QLineEdit* cat_edit_{nullptr};
    DBContentComboBox* object_box_{nullptr};
};

}
