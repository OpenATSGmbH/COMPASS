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

#include <QDialog>

class LicenseWidget;
class QTreeWidget;
class QPushButton;

/**
*/
class LicenseManagerDialog : public QDialog
{
public:
    LicenseManagerDialog(QWidget* parent = nullptr, 
                         Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~LicenseManagerDialog();

    void selectLicense(const std::string& id);

private:
    void updateList();
    void updateLicenseWidget();
    void addLicense();
    void removeCurrentLicense();

    QTreeWidget*   license_list_          = nullptr;
    LicenseWidget* license_widget_        = nullptr;
    QPushButton*   remove_license_button_ = nullptr;

    int idx_id_    = -1;
    int state_idx_ = -1;
};
