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

#include "eval/requirement/position/radarrangeconfigwidget.h"
#include "eval/requirement/position/radarrangeconfig.h"
//#include "textfielddoublevalidator.h"
//#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

PositionRadarRangeConfigWidget::PositionRadarRangeConfigWidget(PositionRadarRangeConfig& cfg)
    : PositionBaseConfigWidget(cfg)
{
    // max dist
    threshold_value_edit_ = new QLineEdit(QString::number(config().thresholdValue()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    threshold_value_edit_->setToolTip("Minimum/Maximum allowed distance from test target report to reference");
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &PositionRadarRangeConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Threshold Value [m]", threshold_value_edit_);
}

void PositionRadarRangeConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    double val = value.toDouble(&ok);

    if (ok)
        config().thresholdValue(val);
    else
        loginf << "invalid value";
}

PositionRadarRangeConfig& PositionRadarRangeConfigWidget::config()
{
    PositionRadarRangeConfig* config = dynamic_cast<PositionRadarRangeConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
