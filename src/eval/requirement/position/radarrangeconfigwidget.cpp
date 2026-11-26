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

#include "requirement/position/radarrangeconfigwidget.h"
#include "requirement/position/radarrangeconfig.h"
//#include "textfielddoublevalidator.h"
//#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLineEdit>
#include <QComboBox>
#include <QLabel>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

PositionRadarRangeConfigWidget::PositionRadarRangeConfigWidget(PositionRadarRangeConfig& cfg)
    : PositionBaseValueConfigWidget(cfg)
{
    statistic_box_ = new QComboBox();
    for (const auto& info : RadarHelpers::rangeStatistics())
        statistic_box_->addItem(QString::fromStdString(info.label), (unsigned int)info.statistic);
    statistic_box_->setCurrentIndex(statistic_box_->findData((unsigned int)config().statistic()));
    statistic_box_->setToolTip("Statistic of the differences between the test target reports and the reference that is checked");
    connect(statistic_box_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &PositionRadarRangeConfigWidget::statisticChangedSlot);

    form_layout_->addRow("Statistic", statistic_box_);

    threshold_value_edit_ = new QLineEdit(QString::number(config().thresholdValue()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 4, this));
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &PositionRadarRangeConfigWidget::thresholdValueEditSlot);

    threshold_label_ = new QLabel();
    form_layout_->addRow(threshold_label_, threshold_value_edit_);

    updateThresholdTexts();
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

void PositionRadarRangeConfigWidget::statisticChangedSlot(int index)
{
    config().statistic((RadarHelpers::Statistic)statistic_box_->itemData(index).toUInt());
    updateThresholdTexts();
}

void PositionRadarRangeConfigWidget::updateThresholdTexts()
{
    const auto& info = config().statisticInfo();

    threshold_label_->setText(QString::fromStdString(info.threshold_label));
    threshold_value_edit_->setToolTip(QString::fromStdString(info.threshold_description));
}

PositionRadarRangeConfig& PositionRadarRangeConfigWidget::config()
{
    PositionRadarRangeConfig* config = dynamic_cast<PositionRadarRangeConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}

