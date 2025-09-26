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

#include "eval/requirement/mode_a/modeafalseconfigwidget.h"
#include "eval/requirement/mode_a/falseconfig.h"
//#include "textfielddoublevalidator.h"
//#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>

using namespace std;

namespace EvaluationRequirement
{

ModeAFalseConfigWidget::ModeAFalseConfigWidget(ModeAFalseConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of false Mode 3/A code");

    traced_assert(check_type_box_);
}


ModeAFalseConfig& ModeAFalseConfigWidget::config()
{
    ModeAFalseConfig* config = dynamic_cast<ModeAFalseConfig*>(&config_);
    traced_assert(config);

    return *config;
}
}
