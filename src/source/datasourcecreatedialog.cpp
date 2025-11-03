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

#include "datasourcecreatedialog.h"
#include "datasourcemanager.h"
#include "dstypeselectioncombobox.h"
#include "datasourcesconfigurationdialog.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "util/number.h"

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>

using namespace std;
using namespace Utils;

DataSourceCreateDialog::DataSourceCreateDialog(DataSourcesConfigurationDialog& dialog, DataSourceManager& ds_man)
    :  QDialog(&dialog), ds_man_(ds_man)
{
    setWindowTitle("Create New Data Source");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMaximumWidth(300);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* prop_lay = new QFormLayout();

    dstype_combo_ = new DSTypeSelectionComboBox();
    connect(dstype_combo_, &DSTypeSelectionComboBox::changedTypeSignal, this, &DataSourceCreateDialog::dsTypeEditedSlot);
    prop_lay->addRow("Data Source Type", dstype_combo_);

    sac_edit_ = new QLineEdit("0");
    sac_edit_->setValidator(new TextFieldDoubleValidator(0, 255, 0));
    connect(sac_edit_, &QLineEdit::textEdited, this, &DataSourceCreateDialog::sacEditedSlot);
    prop_lay->addRow("SAC", sac_edit_);

    sic_edit_ = new QLineEdit("0");
    sic_edit_->setValidator(new TextFieldDoubleValidator(0, 255, 0));
    connect(sic_edit_, &QLineEdit::textEdited, this, &DataSourceCreateDialog::sicEditedSlot);
    prop_lay->addRow("SIC", sic_edit_);

    main_layout->addLayout(prop_lay);

    main_layout->addStretch();

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &DataSourceCreateDialog::cancelClickedSlot);
    button_layout->addWidget(cancel_button_);

    button_layout->addStretch();

    done_button_ = new QPushButton("Done");
    connect(done_button_, &QPushButton::clicked, this, &DataSourceCreateDialog::doneClickedSlot);
    button_layout->addWidget(done_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    checkInput();
}

DataSourceCreateDialog::~DataSourceCreateDialog()
{

}

void DataSourceCreateDialog::dsTypeEditedSlot(const QString& value)
{
    ds_type_ = value.toStdString();
    loginf << "start" << ds_type_;

    checkInput();
}

void DataSourceCreateDialog::sacEditedSlot(const QString& value_str)
{
    sac_ = value_str.toUInt();

    loginf << "start" << sac_;

    checkInput();
}
void DataSourceCreateDialog::sicEditedSlot(const QString& value_str)
{
    sic_ = value_str.toUInt();

    loginf << "start" << sic_;

    checkInput();
}

void DataSourceCreateDialog::cancelClickedSlot()
{
    cancelled_ = true;

    emit doneSignal();
}

void DataSourceCreateDialog::doneClickedSlot()
{
    cancelled_ = false;

    emit doneSignal();
}

void DataSourceCreateDialog::checkInput()
{
    traced_assert(done_button_);

    if (!ds_type_.size())
    {
        done_button_->setDisabled(true);
        done_button_->setToolTip("Please set a data source type");
    }
    else if (ds_man_.hasConfigDataSource(Number::dsIdFrom(sac_, sic_)))
    {
        done_button_->setDisabled(true);
        done_button_->setToolTip("Data Source with SAC/SIC already exists");
    }
    else
    {
        done_button_->setDisabled(false);
        done_button_->setToolTip("");
    }
}

unsigned int DataSourceCreateDialog::sac() const
{
    return sac_;
}

unsigned int DataSourceCreateDialog::sic() const
{
    return sic_;
}

std::string DataSourceCreateDialog::dsType() const
{
    return ds_type_;
}

bool DataSourceCreateDialog::cancelled() const
{
    return cancelled_;
}
