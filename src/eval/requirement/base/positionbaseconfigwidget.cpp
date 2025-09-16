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

#include "positionbaseconfigwidget.h"
#include "positionbaseconfig.h"

#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QDoubleValidator>

namespace EvaluationRequirement 
{

/**
*/
PositionBaseConfigWidget::PositionBaseConfigWidget(PositionBaseConfig& cfg)
:   ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(form_layout_);

    ref_min_acc_edit_ = new QLineEdit(QString::number(config().refMinimumAccuracy()));
    ref_min_acc_edit_->setValidator(new QDoubleValidator(0.1, 1000.0, 1, this));
    ref_min_acc_edit_->setToolTip("Minimum Reference Accuracy");
    connect(ref_min_acc_edit_, &QLineEdit::textEdited, this,
            &PositionBaseConfigWidget::minRefAccuracyEditSlot);

    form_layout_->addRow("Minimum Reference Accuracy [m]", ref_min_acc_edit_);

}

/**
*/
PositionBaseConfig& PositionBaseConfigWidget::config()
{
    PositionBaseConfig* config = dynamic_cast<PositionBaseConfig*>(&config_);
    traced_assert(config);

    return *config;
}

/**
*/
void PositionBaseConfigWidget::minRefAccuracyEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().refMinimumAccuracy(val);
    else
        loginf << "invalid value";
}

} // namespace EvaluationRequirement
