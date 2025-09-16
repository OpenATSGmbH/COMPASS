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

#include "eval/requirement/position/latencyconfigwidget.h"
#include "eval/requirement/position/latencyconfig.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"
#include "stringconv.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>

using namespace Utils;

namespace EvaluationRequirement
{

PositionLatencyConfigWidget::PositionLatencyConfigWidget(PositionLatencyConfig& cfg)
    : PositionBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of acceptable position latency");

    traced_assert(check_type_box_);

    // max dist
    max_abs_value_edit_ = new QLineEdit(String::timeStringFromDouble(config().maxAbsValue()).c_str());
    max_abs_value_edit_->setToolTip("Maximum absolute latency");
    connect(max_abs_value_edit_, &QLineEdit::textEdited,
            this, &PositionLatencyConfigWidget::maxAbsValueEditSlot);

    form_layout_->addRow("Maximum Absolute Value [HH:MM:SS.SSS]", max_abs_value_edit_);
}

void PositionLatencyConfigWidget::maxAbsValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = String::timeFromString(value.toStdString(), &ok);

    if (ok)
    {
        config().maxAbsValue(val);
    }
    else
        loginf << "invalid value";
}

PositionLatencyConfig& PositionLatencyConfigWidget::config()
{
    PositionLatencyConfig* config = dynamic_cast<PositionLatencyConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
