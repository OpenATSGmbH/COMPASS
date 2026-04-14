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
#include "db_context_manager.h"
#include "fft.h"
#include "logger.h"
#include "traced_assert.h"
#include "textfielddoublevalidator.h"

#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

namespace context
{

FFTEditWidget::FFTEditWidget(DBContextManager& manager,
                             std::function<void()> on_changed,
                             QWidget* parent)
    : QWidget(parent)
    , manager_(manager)
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

void FFTEditWidget::showFFT(const std::string& name)
{
    loginf << "showing FFT '" << name << "'";

    if (!manager_.hasFFT(name))
    {
        clear();
        return;
    }

    has_current_ = true;
    current_name_ = name;

    auto* fft = manager_.fft(name);
    const auto& info = fft->info();

    name_edit_->blockSignals(true);

    name_edit_->setText(QString::fromStdString(fft->name()));

    // position
    if (fft->hasPosition())
    {
        latitude_edit_->setText(QString::number(fft->latitude(), 'g', 12));
        longitude_edit_->setText(QString::number(fft->longitude(), 'g', 12));
    }
    else
    {
        latitude_edit_->setText("");
        longitude_edit_->setText("");
    }

    if (fft->hasAltitude())
        altitude_edit_->setText(QString::number(fft->altitude(), 'f', 1));
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

void FFTEditWidget::clear()
{
    has_current_ = false;
    current_name_.clear();

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

void FFTEditWidget::saveCurrent()
{
    if (!has_current_)
        return;

    manager_.saveContext(manager_.activeContextName());
}

void FFTEditWidget::nameEditedSlot()
{
    if (!has_current_)
        return;

    auto* fft = manager_.fft(current_name_);
    if (!fft)
        return;

    std::string new_name = name_edit_->text().trimmed().toStdString();
    if (new_name.empty() || new_name == current_name_)
        return;

    if (manager_.hasFFT(new_name))
    {
        // revert to current name
        name_edit_->setText(QString::fromStdString(current_name_));
        return;
    }

    loginf << "renaming FFT '" << current_name_ << "' to '" << new_name << "'";

    fft->name(new_name);
    current_name_ = new_name;
    saveCurrent();

    if (on_changed_)
        on_changed_();
}

void FFTEditWidget::positionEditedSlot()
{
    if (!has_current_)
        return;

    auto* fft = manager_.fft(current_name_);
    if (!fft)
        return;

    auto& info = fft->info();

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

    saveCurrent();
}

void FFTEditWidget::modeSEditedSlot()
{
    if (!has_current_)
        return;

    auto* fft = manager_.fft(current_name_);
    if (!fft)
        return;

    auto& info = fft->info();
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

    saveCurrent();
}

void FFTEditWidget::mode3AEditedSlot()
{
    if (!has_current_)
        return;

    auto* fft = manager_.fft(current_name_);
    if (!fft)
        return;

    auto& info = fft->info();
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

    saveCurrent();
}

void FFTEditWidget::modeCEditedSlot()
{
    if (!has_current_)
        return;

    auto* fft = manager_.fft(current_name_);
    if (!fft)
        return;

    auto& info = fft->info();
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

    saveCurrent();
}

} // namespace context
