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

#include "eval/requirement/mode_c/modeccorrectconfigwidget.h"
#include "eval/requirement/mode_c/correctconfig.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

ModeCCorrectConfigWidget::ModeCCorrectConfigWidget(ModeCCorrectConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of correct Mode C code");

    traced_assert(check_type_box_);

    distance_value_edit_ = new QLineEdit(QString::number(config().maxDistanceFt()));
    distance_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    connect(distance_value_edit_, &QLineEdit::textEdited,
            this, &ModeCCorrectConfigWidget::distanceValueEditSlot);

    form_layout_->addRow("Maximum Offset [ft]", distance_value_edit_);


}

void ModeCCorrectConfigWidget::distanceValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxDistanceFt(val);
    else
        loginf << "invalid value";
}

ModeCCorrectConfig& ModeCCorrectConfigWidget::config()
{
    ModeCCorrectConfig* config = dynamic_cast<ModeCCorrectConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
