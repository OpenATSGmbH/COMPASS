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

#include <QWidget>

namespace license
{
    struct License;
}

class QLabel;

/**
 */
class LicenseWidget : public QWidget
{
public:
    LicenseWidget(bool with_extra_infos = false, QWidget* parent = nullptr);
    virtual ~LicenseWidget() = default;

    void showLicense(const license::License* license);

private:
    QLabel* label_type_       = nullptr;
    QLabel* label_id_         = nullptr;
    QLabel* label_licensee_   = nullptr;
    QLabel* label_components_ = nullptr;
    QLabel* label_activates_  = nullptr;
    QLabel* label_expires_    = nullptr;
    QLabel* label_created_    = nullptr;
    QLabel* label_info_       = nullptr;

    bool with_extra_infos_ = false;
};
