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
#include <QColor>

class QLineEdit;
class QCheckBox;
class QPushButton;

class ImportSectorDialog : public QDialog
{
    Q_OBJECT

public slots:
    void colorSlot();

    void cancelSlot();
    void importSlot();

public:
    ImportSectorDialog(const std::string& layer_name, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    std::string layerName();
    bool exclude ();
    QColor color ();

protected:
    QLineEdit* layer_name_edit_edit_ {nullptr};
    QCheckBox* exclude_check_ {nullptr};
    QPushButton* color_button_ {nullptr};

    QColor color_;
};
