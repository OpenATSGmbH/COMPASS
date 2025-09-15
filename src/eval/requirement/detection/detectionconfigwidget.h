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

#include "eval/requirement/base/probabilitybaseconfigwidget.h"

class QLineEdit;
class QCheckBox;

class QFormLayout;

namespace EvaluationRequirement
{
class DetectionConfig;

class DetectionConfigWidget : public ProbabilityBaseConfigWidget
{
    Q_OBJECT

public slots:
    void updateIntervalEditSlot(QString value);

    void toggleUseMinGapLengthSlot();
    void minGapLengthEditSlot(QString value);

    void toggleUseMaxGapLengthSlot();
    void maxGapLengthEditSlot(QString value);

    void toggleInvertProbSlot();

    void toggleUseMissToleranceSlot();
    void missToleranceEditSlot(QString value);

    void toggleHoldForAnyTargetSlot();
    
    void toggleIgnorePrimaryOnlySlot();
public:
    DetectionConfigWidget(DetectionConfig& cfg);

protected:
    QLineEdit* update_interval_edit_{nullptr};

    QCheckBox* use_min_gap_length_check_{nullptr};
    QLineEdit* min_gap_length_edit_{nullptr};

    QCheckBox* use_max_gap_length_check_{nullptr};
    QLineEdit* max_gap_length_edit_{nullptr};

    QCheckBox* use_invert_prob_check_{nullptr};

    QCheckBox* use_miss_tolerance_check_{nullptr};
    QLineEdit* miss_tolerance_edit_{nullptr};

    QCheckBox* hold_for_any_target_check_{nullptr};

    QCheckBox* ignore_primary_only_check_{nullptr};

    DetectionConfig& config();

    void updateActive();
};

}
