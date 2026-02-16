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

class FFTManager;
class FFTsConfigurationDialog;
class DSTypeSelectionComboBox;

class QLabel;
class QLineEdit;
class QPushButton;
class QGridLayout;

class FFTEditWidget : public QWidget
{
    Q_OBJECT

public slots:
    //void nameEditedSlot(const QString& value);

    void modeSAddressEditedSlot(const QString& value_str);
    void modeAEditedSlot(const QString& value_str);
    void modeCEditedSlot(const QString& value_str);

    void latitudeEditedSlot(const QString& value_str);
    void longitudeEditedSlot(const QString& value_str);
    void altitudeEditedSlot(const QString& value_str);

    void deleteSlot();

public:
    FFTEditWidget(FFTManager& ds_man, FFTsConfigurationDialog& dialog);

    void showFFT(const std::string& name);
    void clear();

    void updateContent();

protected:
    FFTManager& ds_man_;
    FFTsConfigurationDialog& dialog_;

    bool has_current_fft_ {false};
    std::string current_name_;
    bool current_fft_in_db_ {false};

    QWidget* main_widget_{nullptr};

    QLineEdit* name_edit_{nullptr};

    // secondary attributes
    QLineEdit* mode_s_address_edit_{nullptr};
    QLineEdit* mode_3a_edit_{nullptr};
    QLineEdit* mode_c_edit_{nullptr};

    // position
    QLineEdit* latitude_edit_{nullptr};
    QLineEdit* longitude_edit_{nullptr};
    QLineEdit* altitude_edit_{nullptr};

    QPushButton* delete_button_{nullptr};
};

