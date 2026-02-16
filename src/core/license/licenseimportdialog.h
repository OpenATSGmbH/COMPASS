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

#include <memory>

#include <QDialog>

namespace license
{
    struct License;
}

class LicenseWidget;

class QPushButton;
class QLabel;

/**
*/
class LicenseImportDialog : public QDialog
{
public:
    LicenseImportDialog(QWidget* parent = nullptr, 
                        Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~LicenseImportDialog();

    const std::unique_ptr<license::License>& getLicense() const { return license_; }

private:
    void importLicenseKey(const QString& license_key);

    std::unique_ptr<license::License> license_;

    LicenseWidget* license_widget_ = nullptr;
    QPushButton*   import_button_  = nullptr;
    QLabel*        status_label_   = nullptr;
};
