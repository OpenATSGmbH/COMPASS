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

#include "gpsimportcsvtaskwidget.h"
#include "gpsimportcsvtask.h"
#include "logger.h"
#include "stringconv.h"

#include "textfielddoublevalidator.h"
#include "textfieldhexvalidator.h"
#include "textfieldoctvalidator.h"

#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QTextEdit>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>

#include <iostream>

using namespace std;
using namespace Utils;

GPSImportCSVTaskWidget::GPSImportCSVTaskWidget(GPSImportCSVTask& task, QWidget* parent, Qt::WindowFlags f)
    : TaskWidget(parent, f), task_(task)
{
    main_layout_ = new QHBoxLayout();

    tab_widget_ = new QTabWidget();

    main_layout_->addWidget(tab_widget_);

    addMainTab();
    addConfigTab();

    updateText();
    updateConfig();

    setLayout(main_layout_);
}

void GPSImportCSVTaskWidget::addMainTab()
{
    traced_assert(tab_widget_);

    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* tab_layout = new QVBoxLayout();

    // file stuff
    file_label_ = new QLabel(task_.importFilename().c_str());
    tab_layout->addWidget(file_label_);


    text_edit_ = new QTextEdit ();
    text_edit_->setReadOnly(true);
    tab_layout->addWidget(text_edit_);

    QWidget* main_tab_widget = new QWidget();
    main_tab_widget->setContentsMargins(0, 0, 0, 0);
    main_tab_widget->setLayout(tab_layout);
    tab_widget_->addTab(main_tab_widget, "Main");
}

void GPSImportCSVTaskWidget::addConfigTab()
{
    QFont font_bold;
    font_bold.setBold(true);

    QVBoxLayout* tab_layout = new QVBoxLayout();

    QGridLayout* grid = new QGridLayout();

    unsigned int row = 0;

    // sac
    grid->addWidget(new QLabel("SAC"), row, 0);

    sac_edit_ = new QLineEdit();
    sac_edit_->setValidator(new TextFieldDoubleValidator(0, 255, 0));
    connect(sac_edit_, &QLineEdit::textEdited, this, &GPSImportCSVTaskWidget::sacEditedSlot);
    grid->addWidget(sac_edit_, row, 1);

    // sic
    ++row;

    grid->addWidget(new QLabel("SIC"), row, 0);

    sic_edit_ = new QLineEdit();
    sic_edit_->setValidator(new TextFieldDoubleValidator(0, 255, 0));
    connect(sic_edit_, &QLineEdit::textEdited, this, &GPSImportCSVTaskWidget::sicEditedSlot);
    grid->addWidget(sic_edit_, row, 1);

    // name

    ++row;
    grid->addWidget(new QLabel("Name"), row, 0);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::textEdited, this, &GPSImportCSVTaskWidget::nameEditedSlot);
    grid->addWidget(name_edit_, row, 1);

    // tod offset
    ++row;
    grid->addWidget(new QLabel("Time of Day Offset"), row, 0);

    tod_offset_edit_ = new QLineEdit();
    tod_offset_edit_->setValidator(new TextFieldDoubleValidator(-24 * 3600, 24 * 3600, 3));
    connect(tod_offset_edit_, &QLineEdit::textEdited, this,
            &GPSImportCSVTaskWidget::todOffsetEditedSlot);
    grid->addWidget(tod_offset_edit_, row, 1);

    // mode 3a

    ++row;
    set_mode_3a_code_check_ = new QCheckBox("Mode 3/A Code (octal)");
    connect(set_mode_3a_code_check_, &QCheckBox::clicked, this, &GPSImportCSVTaskWidget::mode3ACheckedSlot);
    grid->addWidget(set_mode_3a_code_check_, row, 0);

    mode_3a_code_edit_ = new QLineEdit();
    mode_3a_code_edit_->setValidator(new TextFieldOctValidator(4));
    connect(mode_3a_code_edit_, &QLineEdit::textEdited, this,
            &GPSImportCSVTaskWidget::mode3AEditedSlot);
    grid->addWidget(mode_3a_code_edit_, row, 1);

    // target address

    ++row;
    set_target_address_check_ = new QCheckBox("Aircraft Address (hexadecimal)");
    connect(set_target_address_check_, &QCheckBox::clicked, this, &GPSImportCSVTaskWidget::targetAddressCheckedSlot);
    grid->addWidget(set_target_address_check_, row, 0);

    target_address_edit_ = new QLineEdit();
    target_address_edit_->setValidator(new TextFieldHexValidator(6));
    connect(target_address_edit_, &QLineEdit::textEdited, this,
            &GPSImportCSVTaskWidget::targetAddressEditedSlot);
    grid->addWidget(target_address_edit_, row, 1);

    // callsign

    ++row;
    set_callsign_check_ = new QCheckBox("Aircraft Identification");
    connect(set_callsign_check_, &QCheckBox::clicked, this, &GPSImportCSVTaskWidget::callsignCheckedSlot);
    grid->addWidget(set_callsign_check_, row, 0);

    callsign_edit_ = new QLineEdit();
    callsign_edit_->setMaxLength(8);
    connect(callsign_edit_, &QLineEdit::textEdited, this,
            &GPSImportCSVTaskWidget::callsignEditedSlot);
    grid->addWidget(callsign_edit_, row, 1);


    // line id
    ++row;

    grid->addWidget(new QLabel("Line ID"), row, 0);

    QComboBox* file_line_box = new QComboBox();
    file_line_box->addItems({"1", "2", "3", "4"});

    connect(file_line_box, &QComboBox::currentTextChanged,
            this, &GPSImportCSVTaskWidget::lineIDEditSlot);
    grid->addWidget(file_line_box, row, 1);

    // finish him
    tab_layout->addLayout(grid);

    tab_layout->addStretch();

    QWidget* tab_widget = new QWidget();
    tab_widget->setContentsMargins(0, 0, 0, 0);
    tab_widget->setLayout(tab_layout);

    tab_widget_->addTab(tab_widget, "Config");
}

GPSImportCSVTaskWidget::~GPSImportCSVTaskWidget() {}

void GPSImportCSVTaskWidget::selectFile(const std::string& filename)
{
    traced_assert(file_label_);
    file_label_->setText(task_.importFilename().c_str());

    updateText();
}

void GPSImportCSVTaskWidget::sacEditedSlot(const QString& value)
{
    traced_assert(sac_edit_);

    TextFieldDoubleValidator::displayValidityAsColor(sac_edit_);

    if (sac_edit_->hasAcceptableInput())
        task_.dsSAC(sac_edit_->text().toUInt());
}

void GPSImportCSVTaskWidget::sicEditedSlot(const QString& value)
{
    traced_assert(sic_edit_);

    TextFieldDoubleValidator::displayValidityAsColor(sic_edit_);

    if (sic_edit_->hasAcceptableInput())
        task_.dsSIC(sic_edit_->text().toUInt());
}

void GPSImportCSVTaskWidget::nameEditedSlot(const QString& value)
{
    traced_assert(name_edit_);
    task_.dsName(name_edit_->text().toStdString());
}

void GPSImportCSVTaskWidget::todOffsetEditedSlot(const QString& value)
{
    traced_assert(tod_offset_edit_);

    TextFieldDoubleValidator::displayValidityAsColor(tod_offset_edit_);

    if (tod_offset_edit_->hasAcceptableInput())
        task_.todOffset(value.toFloat());
}

void GPSImportCSVTaskWidget::mode3ACheckedSlot()
{
    loginf;

    traced_assert(set_mode_3a_code_check_);
    task_.setMode3aCode(set_mode_3a_code_check_->checkState() == Qt::Checked);
}

void GPSImportCSVTaskWidget::mode3AEditedSlot(const QString& value)
{
    loginf << "value " << value.toStdString();

    traced_assert(mode_3a_code_edit_);

    if (mode_3a_code_edit_->hasAcceptableInput())
        task_.mode3aCode(String::intFromOctalString(value.toStdString()));
}

void GPSImportCSVTaskWidget::targetAddressCheckedSlot()
{
    loginf;

    traced_assert(set_target_address_check_);
    task_.setTargetAddress(set_target_address_check_->checkState() == Qt::Checked);
}

void GPSImportCSVTaskWidget::targetAddressEditedSlot(const QString& value)
{
    loginf << "value " << value.toStdString();

    traced_assert(target_address_edit_);

    if (target_address_edit_->hasAcceptableInput())
        task_.targetAddress(String::intFromHexString(value.toStdString()));
}

void GPSImportCSVTaskWidget::callsignCheckedSlot()
{
    loginf;

    traced_assert(set_callsign_check_);
    task_.setCallsign(set_callsign_check_->checkState() == Qt::Checked);
}

void GPSImportCSVTaskWidget::callsignEditedSlot(const QString& value)
{
    loginf << "value '" << value.toStdString() << "'";

    QString upper_value = value.toUpper();

    callsign_edit_->setText(upper_value);

    task_.callsign(upper_value.toStdString());
}

void GPSImportCSVTaskWidget::lineIDEditSlot(const QString& text)
{
    loginf << "value '" << text.toStdString() << "'";

    bool ok;

    unsigned int line_id = text.toUInt(&ok);

    traced_assert(ok);

    traced_assert(line_id > 0 && line_id <= 4);

    task_.lineID(line_id-1);
}

void GPSImportCSVTaskWidget::updateConfig ()
{
    traced_assert(sac_edit_);
    sac_edit_->setText(QString::number(task_.dsSAC()));

    traced_assert(sic_edit_);
    sic_edit_->setText(QString::number(task_.dsSIC()));

    traced_assert(name_edit_);
    name_edit_->setText(task_.dsName().c_str());

    traced_assert(tod_offset_edit_);
    tod_offset_edit_->setText(QString::number(task_.todOffset()));

    traced_assert(set_mode_3a_code_check_);
    set_mode_3a_code_check_->setChecked(task_.setMode3aCode());

    traced_assert(mode_3a_code_edit_);
    mode_3a_code_edit_->setText(String::octStringFromInt(task_.mode3aCode()).c_str());

    traced_assert(set_target_address_check_);
    set_target_address_check_->setChecked(task_.setTargetAddress());
    traced_assert(target_address_edit_);
    target_address_edit_->setText(String::hexStringFromInt(task_.targetAddress()).c_str());

    traced_assert(set_callsign_check_);
    set_callsign_check_->setChecked(task_.setCallsign());
    traced_assert(callsign_edit_);
    callsign_edit_->setText(task_.callsign().c_str());
}

void GPSImportCSVTaskWidget::expertModeChangedSlot() {}

//void GPSImportCSVTaskWidget::runStarted()
//{
//    loginf;

//    //test_button_->setDisabled(true);
//}

//void GPSImportCSVTaskWidget::runDone()
//{
//    loginf;

//    //test_button_->setDisabled(false);
//}

void GPSImportCSVTaskWidget::updateText ()
{
    loginf;

    traced_assert(text_edit_);

    stringstream ss;

    if (task_.currentError().size())
        ss << "Errors:\n" << task_.currentError() << "\n";

    if (task_.currentText().size())
        ss << task_.currentText();

    text_edit_->setText(ss.str().c_str());

    //    if (task_.currentError().size())
    //    {
    //        context_edit_->setText(QString("Error: ")+task_.currentError().c_str());
    //        import_button_->setDisabled(true);
    //    }
    //    else
    //    {
    //        const nlohmann::json& data = task_.currentData();

    //        traced_assert(data.contains("view_point_context"));

    //        context_edit_->setText(data.at("view_point_context").dump(4).c_str());

    //        import_button_->setDisabled(false);
    //    }
}
