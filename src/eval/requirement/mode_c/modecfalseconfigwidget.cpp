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

#include "eval/requirement/mode_c/modecfalseconfigwidget.h"
#include "eval/requirement/mode_c/falseconfig.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

ModeCFalseConfigWidget::ModeCFalseConfigWidget(ModeCFalseConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of false Mode C code");

    traced_assert(check_type_box_);

    // max diff
    max_diff_edit_ = new QLineEdit(QString::number(config().maxDifference()));
    max_diff_edit_->setValidator(new QDoubleValidator(0.0, 1000.0, 4, this));
    prob_edit_->setToolTip("Maximum altitude difference between the test and the reference");
    connect(max_diff_edit_, &QLineEdit::textEdited,
            this, &ModeCFalseConfigWidget::maxDiffEditSlot);

    form_layout_->addRow("Maximum Difference [ft]", max_diff_edit_);
}

void ModeCFalseConfigWidget::maxDiffEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxDifference(val);
    else
        loginf << "invalid value";
}

ModeCFalseConfig& ModeCFalseConfigWidget::config()
{
    ModeCFalseConfig* config = dynamic_cast<ModeCFalseConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
