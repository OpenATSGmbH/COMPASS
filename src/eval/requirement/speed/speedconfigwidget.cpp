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

#include "eval/requirement/speed/speedconfigwidget.h"
#include "eval/requirement/speed/speedconfig.h"
//#include "textfielddoublevalidator.h"
#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

SpeedConfigWidget::SpeedConfigWidget(SpeedConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    // threshold
    threshold_value_edit_ = new QLineEdit(QString::number(config().thresholdValue()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &SpeedConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Speed Offset Value [m/s]", threshold_value_edit_);

    // use percent threshold if higher
    use_percent_if_higher_check_ = new QCheckBox ();
    use_percent_if_higher_check_->setChecked(config().usePercentIfHigher());
    connect(use_percent_if_higher_check_, &QCheckBox::clicked,
            this, &SpeedConfigWidget::toggleUsePercentIfHigherSlot);

    form_layout_->addRow("Use Percent Threshold if Higher", use_percent_if_higher_check_);

    // percent threshold
    threshold_percent_edit_ = new QLineEdit(QString::number(config().thresholdPercent()));
    threshold_percent_edit_->setValidator(new QDoubleValidator(0.0, 100.0, 2, this));
    connect(threshold_percent_edit_, &QLineEdit::textEdited,
            this, &SpeedConfigWidget::thresholdPercentEditSlot);

    form_layout_->addRow("Threshold Percent [%]", threshold_percent_edit_);

    // prob check type
    threshold_value_check_type_box_ = new ComparisonTypeComboBox();
    threshold_value_check_type_box_->setType(config().thresholdValueCheckType());
    connect(threshold_value_check_type_box_, &ComparisonTypeComboBox::changedTypeSignal,
            this, &SpeedConfigWidget::changedThresholdValueCheckTypeSlot);
    form_layout_->addRow("Threshold Value Check Type", threshold_value_check_type_box_);

    // failed values of interest
    failed_values_of_interest_check_ = new QCheckBox ();
    failed_values_of_interest_check_->setChecked(config().failedValuesOfInterest());
    connect(failed_values_of_interest_check_, &QCheckBox::clicked,
            this, &SpeedConfigWidget::toggleFailedValuesOfInterestSlot);

    form_layout_->addRow("Failed Values are of Interest", failed_values_of_interest_check_);

    updateActive();
}

void SpeedConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().thresholdValue(val);
    else
        loginf << "invalid value";
}

void SpeedConfigWidget::toggleUsePercentIfHigherSlot()
{
    loginf;

    traced_assert(use_percent_if_higher_check_);
    config().usePercentIfHigher(use_percent_if_higher_check_->checkState() == Qt::Checked);

    updateActive();
}

void SpeedConfigWidget::thresholdPercentEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().thresholdPercent(val);
    else
        loginf << "invalid value";
}

void SpeedConfigWidget::changedThresholdValueCheckTypeSlot()
{
    traced_assert(threshold_value_check_type_box_);
    loginf << "value "
           << threshold_value_check_type_box_->getType();
    config().thresholdValueCheckType(threshold_value_check_type_box_->getType());
}

void SpeedConfigWidget::toggleFailedValuesOfInterestSlot()
{
    loginf;

    traced_assert(failed_values_of_interest_check_);
    config().failedValuesOfInterest(failed_values_of_interest_check_->checkState() == Qt::Checked);
}

SpeedConfig& SpeedConfigWidget::config()
{
    SpeedConfig* config = dynamic_cast<SpeedConfig*>(&config_);
    traced_assert(config);

    return *config;
}

void SpeedConfigWidget::updateActive()
{
    traced_assert(threshold_percent_edit_);
    threshold_percent_edit_->setEnabled(config().usePercentIfHigher());
}


}
