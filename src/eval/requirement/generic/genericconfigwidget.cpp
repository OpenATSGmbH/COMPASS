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

#include "eval/requirement/generic/genericconfigwidget.h"
#include "eval/requirement/generic/genericconfig.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

GenericIntegerConfigWidget::GenericIntegerConfigWidget(GenericIntegerConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability");

    traced_assert(check_type_box_);
}


GenericIntegerConfig& GenericIntegerConfigWidget::config()
{
    GenericIntegerConfig* config = dynamic_cast<GenericIntegerConfig*>(&config_);
    traced_assert(config);

    return *config;
}


// ---------------------------------------------

GenericDoubleConfigWidget::GenericDoubleConfigWidget(GenericDoubleConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability");

    traced_assert(check_type_box_);

    threshold_value_edit_ = new QLineEdit(QString::number(config().threshold()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 1000.0, 4, this));
    prob_edit_->setToolTip("Maximum value difference between the test and the reference");
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &GenericDoubleConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Maximum Difference", threshold_value_edit_);
}


GenericDoubleConfig& GenericDoubleConfigWidget::config()
{
    GenericDoubleConfig* config = dynamic_cast<GenericDoubleConfig*>(&config_);
    traced_assert(config);

    return *config;
}

void GenericDoubleConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    double val = value.toDouble(&ok);

    if (ok)
        config().threshold(val);
    else
        loginf << "invalid value";
}

}
