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


#include "requirement/position/radartimestampconfigwidget.h"
#include "requirement/position/radartimestampconfig.h"
#include "logger.h"
#include "traced_assert.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

PositionRadarTimeStampConfigWidget::PositionRadarTimeStampConfigWidget(PositionRadarTimeStampConfig& cfg)
    : PositionBaseValueConfigWidget(cfg)
{
    threshold_value_edit_ = new QLineEdit(QString::number(config().thresholdValue()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 3600.0, 4, this));
    threshold_value_edit_->setToolTip("The absolute time stamp error, estimated from the along-track offsets to the reference, must be smaller");
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &PositionRadarTimeStampConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Maximum Absolute Error [s]", threshold_value_edit_);
}

void PositionRadarTimeStampConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    double val = value.toDouble(&ok);

    if (ok)
        config().thresholdValue(val);
    else
        loginf << "invalid value";
}

PositionRadarTimeStampConfig& PositionRadarTimeStampConfigWidget::config()
{
    PositionRadarTimeStampConfig* config = dynamic_cast<PositionRadarTimeStampConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
