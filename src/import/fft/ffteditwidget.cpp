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

#include "ffteditwidget.h"
#include "fftmanager.h"
#include "configurationfft.h"
#include "dbfft.h"
#include "fftsconfigurationdialog.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "textfieldhexvalidator.h"
#include "textfieldoctvalidator.h"
#include "stringconv.h"

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QScrollArea>
#include <QFormLayout>

using namespace std;
using namespace Utils;

FFTEditWidget::FFTEditWidget(FFTManager& ds_man, FFTsConfigurationDialog& dialog)
    : ds_man_(ds_man), dialog_(dialog)
{
    setMaximumWidth(400);

    QScrollArea* scroll_area = new QScrollArea();
    scroll_area->setWidgetResizable(true);

    main_widget_ = new QWidget();

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setSizeConstraint(QLayout::SetMinimumSize);

    QGridLayout* properties_layout = new QGridLayout();

    unsigned int row = 0;

    // "Name", "Short Name", "DSType", "SAC", "SIC"

    //name_edit_
    properties_layout->addWidget(new QLabel("Name"), row, 0);

    name_edit_ = new QLineEdit();
    name_edit_->setReadOnly(true);
    name_edit_->setDisabled(true);
    //connect(name_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::nameEditedSlot);
    properties_layout->addWidget(name_edit_, row, 1);
    row++;

    // secondary stuff
    properties_layout->addWidget(new QLabel("Mode S Address (hexadecimal)"), row, 0);

    mode_s_address_edit_ = new QLineEdit();
    mode_s_address_edit_->setValidator(new TextFieldHexValidator(6));
    connect(mode_s_address_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::modeSAddressEditedSlot);
    properties_layout->addWidget(mode_s_address_edit_, row, 1);
    row++;

    properties_layout->addWidget(new QLabel("Mode 3/A Code (octal)"), row, 0);

    mode_3a_edit_ = new QLineEdit();
    mode_3a_edit_->setValidator(new TextFieldOctValidator(4));
    connect(mode_3a_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::modeAEditedSlot);
    properties_layout->addWidget(mode_3a_edit_, row, 1);
    row++;


    properties_layout->addWidget(new QLabel("Mode C Code [ft]"), row, 0);

    mode_c_edit_ = new QLineEdit();
    mode_c_edit_->setValidator(new TextFieldDoubleValidator(-10000, 100000, 2));
    connect(mode_c_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::modeCEditedSlot);
    properties_layout->addWidget(mode_c_edit_, row, 1);
    row++;

    // positions

    properties_layout->addWidget(new QLabel("Latitude"), row, 0);

    latitude_edit_ = new QLineEdit();
    latitude_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 12));
    connect(latitude_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::latitudeEditedSlot);
    properties_layout->addWidget(latitude_edit_, row, 1);
    row++;

    properties_layout->addWidget(new QLabel("Longitude"), row, 0);

    longitude_edit_ = new QLineEdit();
    longitude_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 12));
    connect(longitude_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::longitudeEditedSlot);
    properties_layout->addWidget(longitude_edit_, row, 1);
    row++;

    properties_layout->addWidget(new QLabel("Altitude [m]"), row, 0);

    altitude_edit_ = new QLineEdit();
    altitude_edit_->setValidator(new TextFieldDoubleValidator(-10000, 10000, 12));
    connect(altitude_edit_, &QLineEdit::textEdited, this, &FFTEditWidget::altitudeEditedSlot);
    properties_layout->addWidget(altitude_edit_, row, 1);
    row++;

    main_layout->addLayout(properties_layout);

    main_layout->addStretch();

    delete_button_ = new QPushButton("Delete");
    delete_button_->setToolTip("Deletes the data source in configuration");
    connect(delete_button_, &QPushButton::clicked, this, &FFTEditWidget::deleteSlot);
    main_layout->addWidget(delete_button_);

    updateContent();

    main_widget_->setLayout(main_layout);
    scroll_area->setWidget(main_widget_);

    QVBoxLayout* top_lay = new QVBoxLayout();
    top_lay->setSizeConstraint(QLayout::SetMinimumSize);
    top_lay->addWidget(scroll_area);

    setLayout(top_lay);
}

void FFTEditWidget::showFFT(const std::string& name)
{
    has_current_fft_ = true;
    current_name_ = name;
    current_fft_in_db_ = ds_man_.hasDBFFT(current_name_);

    loginf << "name " << name << " in db " << current_fft_in_db_;

    traced_assert(ds_man_.hasConfigFFT(current_name_));

    updateContent();
}

void FFTEditWidget::clear()
{
    loginf;

    has_current_fft_ = false;
    current_name_ = "";
    current_fft_in_db_ = false;

    updateContent();
}


void FFTEditWidget::modeSAddressEditedSlot(const QString& value_str)
{
    traced_assert(mode_s_address_edit_);

    if (mode_s_address_edit_->hasAcceptableInput())
    {
        if (!value_str.size())
        {
            if (current_fft_in_db_)
            {
                traced_assert(ds_man_.hasDBFFT(current_name_));
                ds_man_.dbFFT(current_name_).clearModeSAddress();
            }

            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds_man_.configFFT(current_name_).clearModeSAddress();
        }
        else
        {
            unsigned int value = String::intFromHexString(value_str.toStdString());

            if (current_fft_in_db_)
            {
                traced_assert(ds_man_.hasDBFFT(current_name_));
                ds_man_.dbFFT(current_name_).modeSAddress(value);
            }

            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds_man_.configFFT(current_name_).modeSAddress(value);
        }
    }
}
void FFTEditWidget::modeAEditedSlot(const QString& value_str)
{
    traced_assert(mode_3a_edit_);

    if (mode_3a_edit_->hasAcceptableInput())
    {
        if (!value_str.size())
        {
            if (current_fft_in_db_)
            {
                traced_assert(ds_man_.hasDBFFT(current_name_));
                ds_man_.dbFFT(current_name_).clearMode3ACode();
            }

            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds_man_.configFFT(current_name_).clearMode3ACode();
        }
        else
        {
            unsigned int value = String::intFromOctalString(value_str.toStdString());

            if (current_fft_in_db_)
            {
                traced_assert(ds_man_.hasDBFFT(current_name_));
                ds_man_.dbFFT(current_name_).mode3ACode(value);
            }

            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds_man_.configFFT(current_name_).mode3ACode(value);
        }
    }
}
void FFTEditWidget::modeCEditedSlot(const QString& value_str)
{
    float value = value_str.toFloat();

    loginf << "'" << value << "'";

    if (current_fft_in_db_)
    {
        traced_assert(ds_man_.hasDBFFT(current_name_));
        ds_man_.dbFFT(current_name_).modeCCode(value);
    }

    traced_assert(ds_man_.hasConfigFFT(current_name_));
    ds_man_.configFFT(current_name_).modeCCode(value);
}

void FFTEditWidget::latitudeEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    loginf << "'" << value << "'";

    if (current_fft_in_db_)
    {
        traced_assert(ds_man_.hasDBFFT(current_name_));
        ds_man_.dbFFT(current_name_).latitude(value);
    }

    traced_assert(ds_man_.hasConfigFFT(current_name_));
    ds_man_.configFFT(current_name_).latitude(value);
}

void FFTEditWidget::longitudeEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    loginf << "'" << value << "'";

    if (current_fft_in_db_)
    {
        traced_assert(ds_man_.hasDBFFT(current_name_));
        ds_man_.dbFFT(current_name_).longitude(value);
    }

    traced_assert(ds_man_.hasConfigFFT(current_name_));
    ds_man_.configFFT(current_name_).longitude(value);
}

void FFTEditWidget::altitudeEditedSlot(const QString& value_str)
{
    double value = value_str.toDouble();

    loginf << "'" << value << "'";

    if (current_fft_in_db_)
    {
        traced_assert(ds_man_.hasDBFFT(current_name_));
        ds_man_.dbFFT(current_name_).altitude(value);
    }

    traced_assert(ds_man_.hasConfigFFT(current_name_));
    ds_man_.configFFT(current_name_).altitude(value);
}


void FFTEditWidget::deleteSlot()
{
    loginf;

    traced_assert(has_current_fft_);
    //assert (!current_fft_in_db_);

    dialog_.beginResetModel();
    ds_man_.deleteFFT(current_name_);
    dialog_.endResetModel();

    clear();
}

void FFTEditWidget::updateContent()
{
    traced_assert(main_widget_);
    traced_assert(name_edit_);
    traced_assert(delete_button_);

    if (!has_current_fft_)
    {
        name_edit_->setText("");
        name_edit_->setDisabled(true);

        main_widget_->setHidden(true);
    }
    else
    {
        FFTBase* ds = nullptr;

        if (current_fft_in_db_) // db && cfg
        {
            traced_assert(ds_man_.hasDBFFT(current_name_));
            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds = dynamic_cast<FFTBase*>(&ds_man_.dbFFT(current_name_));
        }
        else
        {
            traced_assert(ds_man_.hasConfigFFT(current_name_));
            ds = dynamic_cast<FFTBase*>(&ds_man_.configFFT(current_name_));
        }
        traced_assert(ds);

        name_edit_->setText(ds->name().c_str());
        name_edit_->setDisabled(false);

        if (ds->hasModeSAddress())
            mode_s_address_edit_->setText(String::hexStringFromInt(ds->modeSAddress()).c_str());
        else
            mode_s_address_edit_->clear();

        if (ds->hasMode3ACode())
            mode_3a_edit_->setText(String::octStringFromInt(ds->mode3ACode()).c_str());
        else
            mode_3a_edit_->clear();

        if (ds->hasModeCCode())
            mode_c_edit_->setText(QString::number(ds->modeCCode()));
        else
            mode_c_edit_->clear();

        if (ds->hasPosition())
        {
            latitude_edit_->setText(QString::number(ds->latitude(), 'g', 12));
            longitude_edit_->setText(QString::number(ds->longitude(), 'g', 12));
            altitude_edit_->setText(QString::number(ds->altitude(), 'g', 12));
        }
        else
        {
            latitude_edit_->setText("0");
            longitude_edit_->setText("0");
            altitude_edit_->setText("0");
        }

        main_widget_->setHidden(false);
    }
}
