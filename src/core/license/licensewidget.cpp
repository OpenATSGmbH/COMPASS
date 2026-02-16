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

#include "licensewidget.h"
#include "license.h"

#include "timeconv.h"

#include <QLineEdit>
#include <QLabel>
#include <QHBoxLayout>
#include <QFormLayout>

/**
*/
LicenseWidget::LicenseWidget(bool with_extra_infos, QWidget* parent)
:   QWidget          (parent)
,   with_extra_infos_(with_extra_infos)
{
    auto layout = new QFormLayout;
    setLayout(layout);

    label_type_ = new QLabel;
    layout->addRow("Type: ", label_type_);

    label_id_ = new QLabel;
    layout->addRow("ID: ", label_id_);

    label_licensee_ = new QLabel;
    layout->addRow("Licensed to: ", label_licensee_);

    label_components_ = new QLabel;
    layout->addRow("Licensed Components: ", label_components_);

    label_activates_ = new QLabel;
    layout->addRow("Activation Date: ", label_activates_);

    label_expires_ = new QLabel;
    layout->addRow("Expiration Date: ", label_expires_);

    label_created_ = new QLabel;
    layout->addRow("Created: ", label_created_);

    if (with_extra_infos_)
    {
        label_info_ = new QLabel;
        layout->addRow("Info: ", label_info_);
    }
}

/**
*/
void LicenseWidget::showLicense(const license::License* license)
{
    bool read = license && license->state == license::License::State::Read;

    if (label_type_      ) label_type_->setText(read ? QString::fromStdString(license::License::typeToString(license->type)) : "-");
    if (label_id_        ) label_id_->setText(read ? QString::fromStdString(license->id) : "-");
    if (label_licensee_  ) label_licensee_->setText(read ? QString::fromStdString(license->licensee) : "-");
    if (label_components_) label_components_->setText(read ? QString::fromStdString(license->componentsAsString()) : "-");
    if (label_activates_ ) label_activates_->setText(read ? QString::fromStdString(Utils::Time::toDateString(license->date_activation)) : "-");
    if (label_expires_   ) label_expires_->setText(read ? QString::fromStdString(Utils::Time::toDateString(license->date_expiration)) : "-");
    if (label_created_   ) label_created_->setText(read ? QString::fromStdString(Utils::Time::toString(license->created)) : "-");
    if (label_info_      ) label_info_->setText(read ? QString::fromStdString(license->info) : "-");
}
