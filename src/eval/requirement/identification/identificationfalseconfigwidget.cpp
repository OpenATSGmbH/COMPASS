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

#include "eval/requirement/identification/identificationfalseconfigwidget.h"
#include "eval/requirement/identification/falseconfig.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>

using namespace std;

namespace EvaluationRequirement
{

IdentificationFalseConfigWidget::IdentificationFalseConfigWidget(IdentificationFalseConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of false identification");

    traced_assert(check_type_box_);

    // correctness of all
    require_all_false_check_ = new QCheckBox ();
    require_all_false_check_->setChecked(config().requireAllFalse());
    require_all_false_check_->setToolTip(
                "If checked, all available secondary attributes be different than in "
                " the reference to count. If not checked, a single wrong secondary attribute is enough.");
    connect(require_all_false_check_, &QCheckBox::clicked,
            this, &IdentificationFalseConfigWidget::toggleRequireCorrectnessOfAllSlot);
    form_layout_->addRow("Require All False", require_all_false_check_);

    // mode a ssr code
    use_mode_a_check_ = new QCheckBox ();
    use_mode_a_check_->setChecked(config().useModeA());
    use_mode_a_check_->setToolTip("If the Mode 3/A code should be checked");
    connect(use_mode_a_check_, &QCheckBox::clicked,
            this, &IdentificationFalseConfigWidget::toggleUseModeASlot);
    form_layout_->addRow("Use Mode 3/A Code", use_mode_a_check_);

    // 24-bit mode s address
    use_ms_ta_check_ = new QCheckBox ();
    use_ms_ta_check_->setChecked(config().useMsTa());
    use_ms_ta_check_->setToolTip("If the Mode S target address should be checked");
    connect(use_ms_ta_check_, &QCheckBox::clicked,
            this, &IdentificationFalseConfigWidget::toggleUseMsTaSlot);
    form_layout_->addRow("Use Mode S Target Address", use_ms_ta_check_);


    // downlinked aircraft identification
    use_ms_ti_check_ = new QCheckBox ();
    use_ms_ti_check_->setChecked(config().useMsTi());
    use_ms_ti_check_->setToolTip("If the Mode S target identification should be checked");
    connect(use_ms_ti_check_, &QCheckBox::clicked,
            this, &IdentificationFalseConfigWidget::toggleUseMsTiSlot);
    form_layout_->addRow("Use Mode S Target Identification", use_ms_ti_check_);
}

void IdentificationFalseConfigWidget::toggleRequireCorrectnessOfAllSlot()
{
    loginf;

    traced_assert(require_all_false_check_);
    config().requireAllFalse(require_all_false_check_->checkState() == Qt::Checked);

}

void IdentificationFalseConfigWidget::toggleUseModeASlot()
{
    loginf;

    traced_assert(use_mode_a_check_);
    config().useModeA(use_mode_a_check_->checkState() == Qt::Checked);
}

void IdentificationFalseConfigWidget::toggleUseMsTaSlot()
{
    loginf;

    traced_assert(use_ms_ta_check_);
    config().useMsTa(use_ms_ta_check_->checkState() == Qt::Checked);

}

void IdentificationFalseConfigWidget::toggleUseMsTiSlot()
{
    loginf;

    traced_assert(use_ms_ti_check_);
    config().useMsTi(use_ms_ti_check_->checkState() == Qt::Checked);

}

IdentificationFalseConfig& IdentificationFalseConfigWidget::config()
{
    IdentificationFalseConfig* config = dynamic_cast<IdentificationFalseConfig*>(&config_);
    traced_assert(config);

    return *config;
}
}
