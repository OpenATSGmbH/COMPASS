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

#include "fft_edit_widget.h"
#include "fft.h"
#include "logger.h"
#include "textfielddoublevalidator.h"
#include "traced_assert.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

namespace context
{

FFTEditWidget::FFTEditWidget(std::function<void()> on_changed,
                             QWidget* parent)
    : QWidget(parent)
    , on_changed_(std::move(on_changed))
{
    auto* layout = new QVBoxLayout();

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::nameEditedSlot);
    form->addRow(new QLabel("Name"), name_edit_);

    latitude_edit_ = new QLineEdit();
    latitude_edit_->setValidator(new TextFieldDoubleValidator(-90, 90, 8));
    latitude_edit_->setPlaceholderText("not set");
    connect(latitude_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::positionEditedSlot);
    form->addRow(new QLabel("Latitude"), latitude_edit_);

    longitude_edit_ = new QLineEdit();
    longitude_edit_->setValidator(new TextFieldDoubleValidator(-180, 180, 8));
    longitude_edit_->setPlaceholderText("not set");
    connect(longitude_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::positionEditedSlot);
    form->addRow(new QLabel("Longitude"), longitude_edit_);

    altitude_edit_ = new QLineEdit();
    altitude_edit_->setValidator(new TextFieldDoubleValidator(-10000, 100000, 1));
    altitude_edit_->setPlaceholderText("not set");
    connect(altitude_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::positionEditedSlot);
    form->addRow(new QLabel("Altitude [ft]"), altitude_edit_);

    mode_s_edit_ = new QLineEdit();
    mode_s_edit_->setPlaceholderText("not set");
    connect(mode_s_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::modeSEditedSlot);
    form->addRow(new QLabel("Mode S Address"), mode_s_edit_);

    mode_3a_edit_ = new QLineEdit();
    mode_3a_edit_->setPlaceholderText("not set");
    connect(mode_3a_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::mode3AEditedSlot);
    form->addRow(new QLabel("Mode 3/A Code"), mode_3a_edit_);

    mode_c_edit_ = new QLineEdit();
    mode_c_edit_->setValidator(new TextFieldDoubleValidator(-10000, 100000, 1));
    mode_c_edit_->setPlaceholderText("not set");
    connect(mode_c_edit_, &QLineEdit::editingFinished, this, &FFTEditWidget::modeCEditedSlot);
    form->addRow(new QLabel("Mode C Code [ft]"), mode_c_edit_);

    layout->addLayout(form);
    layout->addStretch();

    setLayout(layout);
}

void FFTEditWidget::show(FFT& fft)
{
    loginf << "showing FFT '" << fft.name() << "'";

    current_fft_ = &fft;

    name_edit_->blockSignals(true);

    name_edit_->setText(QString::fromStdString(fft.name()));

    const auto& info = fft.info();

    // position
    if (fft.hasPosition())
    {
        latitude_edit_->setText(QString::number(fft.latitude(), 'g', 12));
        longitude_edit_->setText(QString::number(fft.longitude(), 'g', 12));
    }
    else
    {
        latitude_edit_->setText("");
        longitude_edit_->setText("");
    }

    if (fft.hasAltitude())
        altitude_edit_->setText(QString::number(fft.altitude(), 'f', 1));
    else
        altitude_edit_->setText("");

    // mode codes
    if (info.contains("mode_s_address"))
        mode_s_edit_->setText(QString::number(info.at("mode_s_address").get<unsigned int>()));
    else
        mode_s_edit_->setText("");

    if (info.contains("mode_3a_code"))
        mode_3a_edit_->setText(QString::number(info.at("mode_3a_code").get<unsigned int>()));
    else
        mode_3a_edit_->setText("");

    if (info.contains("mode_c_code"))
        mode_c_edit_->setText(QString::number(info.at("mode_c_code").get<float>(), 'f', 1));
    else
        mode_c_edit_->setText("");

    name_edit_->blockSignals(false);
}

std::string FFTEditWidget::currentFFTName() const
{
    traced_assert(current_fft_);
    return current_fft_->name();
}

void FFTEditWidget::clear()
{
    current_fft_ = nullptr;

    name_edit_->blockSignals(true);

    name_edit_->clear();
    latitude_edit_->clear();
    longitude_edit_->clear();
    altitude_edit_->clear();
    mode_s_edit_->clear();
    mode_3a_edit_->clear();
    mode_c_edit_->clear();

    name_edit_->blockSignals(false);
}

void FFTEditWidget::setReadOnly(bool read_only)
{
    read_only_ = read_only;

    name_edit_->setReadOnly(read_only);
    latitude_edit_->setReadOnly(read_only);
    longitude_edit_->setReadOnly(read_only);
    altitude_edit_->setReadOnly(read_only);
    mode_s_edit_->setReadOnly(read_only);
    mode_3a_edit_->setReadOnly(read_only);
    mode_c_edit_->setReadOnly(read_only);
}

void FFTEditWidget::nameEditedSlot()
{
    if (!current_fft_ || read_only_)
        return;

    std::string new_name = name_edit_->text().trimmed().toStdString();
    if (new_name.empty() || new_name == current_fft_->name())
        return;

    loginf << "renaming FFT '" << current_fft_->name() << "' to '" << new_name << "'";

    current_fft_->name(new_name);

    if (on_changed_)
        on_changed_();
}

void FFTEditWidget::positionEditedSlot()
{
    if (!current_fft_ || read_only_)
        return;

    auto& info = current_fft_->info();

    QString lat_text = latitude_edit_->text().trimmed();
    QString lon_text = longitude_edit_->text().trimmed();
    QString alt_text = altitude_edit_->text().trimmed();

    if (!lat_text.isEmpty() && !lon_text.isEmpty())
    {
        bool lat_ok = false, lon_ok = false;
        double lat = lat_text.toDouble(&lat_ok);
        double lon = lon_text.toDouble(&lon_ok);

        if (lat_ok && lon_ok)
        {
            info["latitude"] = lat;
            info["longitude"] = lon;
        }
    }
    else
    {
        info.erase("latitude");
        info.erase("longitude");
    }

    if (!alt_text.isEmpty())
    {
        bool ok = false;
        double alt = alt_text.toDouble(&ok);
        if (ok)
            info["altitude"] = alt;
    }
    else
    {
        info.erase("altitude");
    }

    if (on_changed_)
        on_changed_();
}

void FFTEditWidget::modeSEditedSlot()
{
    if (!current_fft_ || read_only_)
        return;

    auto& info = current_fft_->info();
    QString text = mode_s_edit_->text().trimmed();

    if (text.isEmpty())
        info.erase("mode_s_address");
    else
    {
        bool ok = false;
        unsigned int val = text.toUInt(&ok);
        if (ok)
            info["mode_s_address"] = val;
    }

    if (on_changed_)
        on_changed_();
}

void FFTEditWidget::mode3AEditedSlot()
{
    if (!current_fft_ || read_only_)
        return;

    auto& info = current_fft_->info();
    QString text = mode_3a_edit_->text().trimmed();

    if (text.isEmpty())
        info.erase("mode_3a_code");
    else
    {
        bool ok = false;
        unsigned int val = text.toUInt(&ok);
        if (ok)
            info["mode_3a_code"] = val;
    }

    if (on_changed_)
        on_changed_();
}

void FFTEditWidget::modeCEditedSlot()
{
    if (!current_fft_ || read_only_)
        return;

    auto& info = current_fft_->info();
    QString text = mode_c_edit_->text().trimmed();

    if (text.isEmpty())
        info.erase("mode_c_code");
    else
    {
        bool ok = false;
        float val = text.toFloat(&ok);
        if (ok)
            info["mode_c_code"] = val;
    }

    if (on_changed_)
        on_changed_();
}

} // namespace context
