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

#include "eval/requirement/mode_a/modeapresentconfigwidget.h"
#include "eval/requirement/mode_a/presentconfig.h"
//#include "textfielddoublevalidator.h"
//#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>

using namespace std;

namespace EvaluationRequirement
{

ModeAPresentConfigWidget::ModeAPresentConfigWidget(ModeAPresentConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of Mode 3/A code present");

    traced_assert(check_type_box_);
}

ModeAPresentConfig& ModeAPresentConfigWidget::config()
{
    ModeAPresentConfig* config = dynamic_cast<ModeAPresentConfig*>(&config_);
    traced_assert(config);

    return *config;
}


}
