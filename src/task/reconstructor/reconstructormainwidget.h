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

#pragma once

#include <QWidget>

class ReconstructorBase;

class QComboBox;
class QSpinBox;
class QLineEdit;
class QCheckBox;

class PropertyValueEdit;

/**
*/
class ReconstructorMainWidget : public QWidget
{
public:
    explicit ReconstructorMainWidget(ReconstructorBase& reconstructor, 
                                     QWidget *parent = nullptr);
    virtual ~ReconstructorMainWidget();

    void updateValues();

protected:
    ReconstructorBase& reconstructor_;

    QLineEdit* ds_name_edit_      = nullptr; 
    QSpinBox*  ds_sac_box_        = nullptr;
    QSpinBox*  ds_sic_box_        = nullptr;
    QComboBox* ds_line_combo_     = nullptr;

    PropertyValueEdit* ds_time_begin_box_ = nullptr;
    PropertyValueEdit* ds_time_end_box_   = nullptr;

    QSpinBox*  slice_length_box_  = nullptr;
    QSpinBox*  slice_overlap_box_ = nullptr;

    QCheckBox* delete_refs_box_   = nullptr;
    QCheckBox* skip_reference_data_writing_box_   = nullptr;
};
