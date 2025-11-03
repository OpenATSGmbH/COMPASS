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

class ReconstructorBase;

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;

/**
*/
class ReferenceCalculatorWidget : public QWidget
{
public:
    explicit ReferenceCalculatorWidget(ReconstructorBase& reconstructor);
    virtual ~ReferenceCalculatorWidget();

    virtual void updateValues();

protected:
    QComboBox* rec_type_combo_       = nullptr;
    QComboBox* rec_type_combo_final_ = nullptr;

    ReconstructorBase& reconstructor_;

private:
    void updateEnabledStates();

    QDoubleSpinBox* Q_std_static_edit_            = nullptr;
    QDoubleSpinBox* Q_std_ground_edit_            = nullptr;
    QDoubleSpinBox* Q_std_air_edit_               = nullptr;
    QDoubleSpinBox* Q_std_unknown_edit_           = nullptr;
    QCheckBox*      dynamic_Q_box_                = nullptr;
    QDoubleSpinBox* repr_distance_box_            = nullptr;
    //QSpinBox*       min_chain_size_box_           = nullptr;
    QDoubleSpinBox* min_dt_box_                   = nullptr;
    QDoubleSpinBox* max_dt_box_                   = nullptr;
    QDoubleSpinBox* max_distance_box_             = nullptr;
    QCheckBox*      smooth_rts_box_               = nullptr;
    QDoubleSpinBox* smooth_scale_box_             = nullptr;
    QCheckBox*      resample_systracks_box_       = nullptr;
    QDoubleSpinBox* resample_systracks_dt_box_    = nullptr;
    QDoubleSpinBox* resample_systracks_maxdt_box_ = nullptr;
    QCheckBox*      resample_result_box_          = nullptr;
    QDoubleSpinBox* resample_Q_std_static_edit_   = nullptr;
    QDoubleSpinBox* resample_Q_std_ground_edit_   = nullptr;
    QDoubleSpinBox* resample_Q_std_air_edit_      = nullptr;
    QDoubleSpinBox* resample_Q_std_unknown_edit_  = nullptr;
    QDoubleSpinBox* resample_dt_box_              = nullptr;
    QCheckBox*      filter_max_stddev_box_        = nullptr;
    QDoubleSpinBox* filter_max_stddev_thres_box_  = nullptr;
};
