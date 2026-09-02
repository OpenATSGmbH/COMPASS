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

#include "eval/requirement/latency/adsblatencyconfigwidget.h"
#include "eval/requirement/latency/adsblatencyconfig.h"

#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

ADSBLatencyConfigWidget::ADSBLatencyConfigWidget(ADSBLatencyConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of acceptable latency");

    traced_assert(check_type_box_);

    // max value
    max_value_edit_ = new QLineEdit(QString::number(config().maxValue()));
    max_value_edit_->setValidator(new QDoubleValidator(0.01, 60.0, 3, this));
    max_value_edit_->setToolTip("Maximum acceptable latency (Time of Report Transmission"
                                " minus Time of Message Reception for Position)");
    connect(max_value_edit_, &QLineEdit::textEdited,
            this, &ADSBLatencyConfigWidget::maxValueEditSlot);

    form_layout_->addRow("Maximum Value [s]", max_value_edit_);
}

void ADSBLatencyConfigWidget::maxValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxValue(val);
    else
        loginf << "invalid value";
}

ADSBLatencyConfig& ADSBLatencyConfigWidget::config()
{
    ADSBLatencyConfig* config = dynamic_cast<ADSBLatencyConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
