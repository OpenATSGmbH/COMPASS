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

#include "eval/requirement/position/distanceconfigwidget.h"
#include "eval/requirement/position/distanceconfig.h"
//#include "textfielddoublevalidator.h"
#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

PositionDistanceConfigWidget::PositionDistanceConfigWidget(PositionDistanceConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of correct/false position");

    traced_assert(check_type_box_);

//    Probability [1]: Probability of correct position
//    • Probability Check Type: ≥
//    • Threshold Value [m]:
//    • Threshold Value Check Type: ≤,
//    • Failed Values are of Interest: Checked,

    // max dist
    threshold_value_edit_ = new QLineEdit(QString::number(config().thresholdValue()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    threshold_value_edit_->setToolTip("Minimum/Maximum allowed distance from test target report to reference");
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &PositionDistanceConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Threshold Value [m]", threshold_value_edit_);


    // prob check type
    threshold_value_check_type_box_ = new ComparisonTypeComboBox();
    threshold_value_check_type_box_->setType(config().thresholdValueCheckType());
    threshold_value_check_type_box_->setToolTip("Distance comparison operator with the given threshold");
    connect(threshold_value_check_type_box_, &ComparisonTypeComboBox::changedTypeSignal,
            this, &PositionDistanceConfigWidget::changedThresholdValueCheckTypeSlot);
    form_layout_->addRow("Threshold Value Check Type", threshold_value_check_type_box_);

    // failed values of interest
    failed_values_of_interest_check_ = new QCheckBox ();
    failed_values_of_interest_check_->setChecked(config().failedValuesOfInterest());
    failed_values_of_interest_check_->setToolTip("If the distances of interest are the ones not passing the check");
    connect(failed_values_of_interest_check_, &QCheckBox::clicked,
            this, &PositionDistanceConfigWidget::toggleFailedValuesOfInterestSlot);

    form_layout_->addRow("Failed Values are of Interest", failed_values_of_interest_check_);
}

void PositionDistanceConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().thresholdValue(val);
    else
        loginf << "invalid value";
}

void PositionDistanceConfigWidget::changedThresholdValueCheckTypeSlot()
{
    traced_assert(threshold_value_check_type_box_);
    loginf << "value "
           << threshold_value_check_type_box_->getType();
    config().thresholdValueCheckType(threshold_value_check_type_box_->getType());
}

void PositionDistanceConfigWidget::toggleFailedValuesOfInterestSlot()
{
    loginf;

    traced_assert(failed_values_of_interest_check_);
    config().failedValuesOfInterest(failed_values_of_interest_check_->checkState() == Qt::Checked);
}

PositionDistanceConfig& PositionDistanceConfigWidget::config()
{
    PositionDistanceConfig* config = dynamic_cast<PositionDistanceConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
