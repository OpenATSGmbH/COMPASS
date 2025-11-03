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

#include "probabilitybaseconfigwidget.h"
#include "probabilitybaseconfig.h"
#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>

namespace EvaluationRequirement {

ProbabilityBaseConfigWidget::ProbabilityBaseConfigWidget(ProbabilityBaseConfig& cfg)
    : BaseConfigWidget(cfg)
{
    traced_assert(form_layout_);

    // prob
    prob_edit_ = new QLineEdit(QString::number(config().prob()));
    prob_edit_->setValidator(new QDoubleValidator(0.0000001, 1.0, 8, this));
    connect(prob_edit_, &QLineEdit::textEdited,
            this, &ProbabilityBaseConfigWidget::changedProbabilitySlot);

    form_layout_->addRow("Probability [1]", prob_edit_);

    // prob check type
    check_type_box_ = new ComparisonTypeComboBox();
    check_type_box_->setType(config().probCheckType());
    connect(check_type_box_, &ComparisonTypeComboBox::changedTypeSignal,
            this, &ProbabilityBaseConfigWidget::changedTypeSlot);
    form_layout_->addRow("Probability Check Type", check_type_box_);
}

void ProbabilityBaseConfigWidget::changedProbabilitySlot(const QString& value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    double val = value.toDouble(&ok);

    if (ok)
        config().prob(val);
    else
        loginf << "invalid value";
}

void ProbabilityBaseConfigWidget::changedTypeSlot()
{
    traced_assert(check_type_box_);
    logdbg << "value " << check_type_box_->getType();
    config().probCheckType(check_type_box_->getType());
}

ProbabilityBaseConfig& ProbabilityBaseConfigWidget::config()
{
    ProbabilityBaseConfig* config = dynamic_cast<ProbabilityBaseConfig*>(&config_);
    traced_assert(config);

    return *config;
}


} // namespace EvaluationRequirement
