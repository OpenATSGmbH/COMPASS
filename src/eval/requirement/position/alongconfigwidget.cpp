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

#include "eval/requirement/position/alongconfigwidget.h"
#include "eval/requirement/position/alongconfig.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

PositionAlongConfigWidget::PositionAlongConfigWidget(PositionAlongConfig& cfg)
    : PositionBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of acceptable along-track position");

    traced_assert(check_type_box_);

    // max dist
    max_abs_value_edit_ = new QLineEdit(QString::number(config().maxAbsValue()));
    max_abs_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    max_abs_value_edit_->setToolTip(
                "Maximum absolute along-track position difference between the test and the reference");
    connect(max_abs_value_edit_, &QLineEdit::textEdited,
            this, &PositionAlongConfigWidget::maxAbsValueEditSlot);

    form_layout_->addRow("Maximum Absolute Value [m]", max_abs_value_edit_);
}

void PositionAlongConfigWidget::maxAbsValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxAbsValue(val);
    else
        loginf << "invalid value";
}

PositionAlongConfig& PositionAlongConfigWidget::config()
{
    PositionAlongConfig* config = dynamic_cast<PositionAlongConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
