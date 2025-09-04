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

#include "eval/requirement/trackangle/trackangleconfigwidget.h"
#include "eval/requirement/trackangle/trackangleconfig.h"
//#include "textfielddoublevalidator.h"
#include "eval/requirement/base/comparisontypecombobox.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

TrackAngleConfigWidget::TrackAngleConfigWidget(TrackAngleConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    // threshold
    threshold_value_edit_ = new QLineEdit(QString::number(config().threshold()));
    threshold_value_edit_->setValidator(new QDoubleValidator(0.0, 360.0, 2, this));
    connect(threshold_value_edit_, &QLineEdit::textEdited,
            this, &TrackAngleConfigWidget::thresholdValueEditSlot);

    form_layout_->addRow("Track Angle Offset Value [deg]", threshold_value_edit_);

    // use minimum speed
    use_minimum_speed_check_ = new QCheckBox ();
    use_minimum_speed_check_->setChecked(config().useMinimumSpeed());
    connect(use_minimum_speed_check_, &QCheckBox::clicked,
            this, &TrackAngleConfigWidget::toggleUseMinimumSpeedSlot);

    form_layout_->addRow("Use Minimum Speed", use_minimum_speed_check_);

    // track angle difference threshold
    minimum_speed_edit_ = new QLineEdit(QString::number(config().minimumSpeed()));
    minimum_speed_edit_->setValidator(new QDoubleValidator(0.0, 10000.0, 2, this));
    connect(minimum_speed_edit_, &QLineEdit::textEdited,
            this, &TrackAngleConfigWidget::minimumSpeedEditSlot);

    form_layout_->addRow("Minimum Speed [m/s]", minimum_speed_edit_);

    // prob check type
    threshold_value_check_type_box_ = new ComparisonTypeComboBox();
    threshold_value_check_type_box_->setType(config().thresholdValueCheckType());
    connect(threshold_value_check_type_box_, &ComparisonTypeComboBox::changedTypeSignal,
            this, &TrackAngleConfigWidget::changedThresholdValueCheckTypeSlot);
    form_layout_->addRow("Threshold Value Check Type", threshold_value_check_type_box_);

    // failed values of interest
    failed_values_of_interest_check_ = new QCheckBox ();
    failed_values_of_interest_check_->setChecked(config().failedValuesOfInterest());
    connect(failed_values_of_interest_check_, &QCheckBox::clicked,
            this, &TrackAngleConfigWidget::toggleFailedValuesOfInterestSlot);

    form_layout_->addRow("Failed Values are of Interest", failed_values_of_interest_check_);

    updateActive();
}

void TrackAngleConfigWidget::thresholdValueEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().threshold(val);
    else
        loginf << "invalid value";
}

void TrackAngleConfigWidget::toggleUseMinimumSpeedSlot()
{
    loginf;

    traced_assert(use_minimum_speed_check_);
    config().useMinimumSpeed(use_minimum_speed_check_->checkState() == Qt::Checked);

    updateActive();
}

void TrackAngleConfigWidget::minimumSpeedEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minimumSpeed(val);
    else
        loginf << "invalid value";
}

void TrackAngleConfigWidget::changedThresholdValueCheckTypeSlot()
{
    traced_assert(threshold_value_check_type_box_);
    loginf << "value "
           << threshold_value_check_type_box_->getType();
    config().thresholdValueCheckType(threshold_value_check_type_box_->getType());
}

void TrackAngleConfigWidget::toggleFailedValuesOfInterestSlot()
{
    loginf;

    traced_assert(failed_values_of_interest_check_);
    config().failedValuesOfInterest(failed_values_of_interest_check_->checkState() == Qt::Checked);
}

TrackAngleConfig& TrackAngleConfigWidget::config()
{
    TrackAngleConfig* config = dynamic_cast<TrackAngleConfig*>(&config_);
    traced_assert(config);

    return *config;
}

void TrackAngleConfigWidget::updateActive()
{
    traced_assert(minimum_speed_edit_);
    minimum_speed_edit_->setEnabled(config().useMinimumSpeed());
}


}
