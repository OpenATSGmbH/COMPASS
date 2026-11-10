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

#include "requirement/base/probabilitybaseconfigwidget.h"

class ComparisonTypeComboBox;

class QLineEdit;
class QCheckBox;

class QFormLayout;

namespace EvaluationRequirement
{

class DubiousTrackConfig;

class DubiousTrackConfigWidget : public ProbabilityBaseConfigWidget
{
    Q_OBJECT

public slots:
    void toggleEvalOnlySingleDsId();
    void evalOnlySingleDsIdEditSlot(QString value);

    void minCompTimeEditSlot(QString value);
    void maxCompTimeEditSlot(QString value);

    void toggleMarkPrimaryOnlySlot();

    void toggleUseMinUpdatesSlot();
    void minUpdatesEditSlot(QString value);

    void toggleUseMinDurationSlot();
    void minDurationEditSlot(QString value);

    void toggleUseMaxGroundspeedSlot();
    void maxGroundspeedEditSlot(QString value);

    void toggleUseMaxAccelerationSlot();
    void maxAccelerationEditSlot(QString value);

    void toggleUseMaxTurnrateSlot();
    void maxTurnrateEditSlot(QString value);

    void toggleUseMaxROCDSlot();
    void maxROCDEditSlot(QString value);

    void dubiousProbEditSlot(QString value);

public:
    DubiousTrackConfigWidget(DubiousTrackConfig& cfg);

protected:
    QCheckBox* eval_only_single_ds_id_check_{nullptr};
    QLineEdit* single_ds_id_edit_{nullptr};


    QLineEdit* min_comp_time_edit_{nullptr};
    QLineEdit* max_comp_time_edit_{nullptr};

    QCheckBox* mark_primary_only_check_{nullptr};

    QCheckBox* use_min_updates_check_{nullptr};
    QLineEdit* min_updates_edit_{nullptr};

    QCheckBox* use_min_duration_check_{nullptr};
    QLineEdit* min_duration_edit_{nullptr};

    QCheckBox* use_max_groundspeed_check_{nullptr};
    QLineEdit* max_groundspeed_kts_edit_{nullptr};

    QCheckBox* use_max_acceleration_check_{nullptr};
    QLineEdit* max_acceleration_edit_{nullptr};

    QCheckBox* use_max_turnrate_check_{nullptr};
    QLineEdit* max_turnrate_edit_{nullptr};

    QCheckBox* use_max_rocd_check_{nullptr};
    QLineEdit* max_rocd_edit_{nullptr};

    QLineEdit* dubious_prob_edit_{nullptr};

    DubiousTrackConfig& config();

    void updateActive();
};

}
