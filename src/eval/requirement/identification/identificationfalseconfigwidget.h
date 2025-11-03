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
class IdentificationFalseConfig;

class IdentificationFalseConfigWidget : public ProbabilityBaseConfigWidget
{
    Q_OBJECT

public slots:
    void toggleRequireCorrectnessOfAllSlot();

    void toggleUseModeASlot();
    void toggleUseMsTaSlot();
    void toggleUseMsTiSlot();

public:
    IdentificationFalseConfigWidget(IdentificationFalseConfig& cfg);

protected:
    QCheckBox* require_all_false_check_{nullptr};

    // mode a ssr code
    QCheckBox* use_mode_a_check_{nullptr};
    // 24-bit mode s address
    QCheckBox* use_ms_ta_check_{nullptr};
    // downlinked aircraft identification
    QCheckBox* use_ms_ti_check_{nullptr};


    IdentificationFalseConfig& config();
};

}
