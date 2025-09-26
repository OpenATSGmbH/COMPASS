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

#include "eval/requirement/dubious/dubioustargetconfigwidget.h"
#include "eval/requirement/dubious/dubioustargetconfig.h"

#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement
{

DubiousTargetConfigWidget::DubiousTargetConfigWidget(DubiousTargetConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    // min comp time
    min_comp_time_edit_ = new QLineEdit(QString::number(config().minimumComparisonTime()));
    min_comp_time_edit_->setValidator(new QDoubleValidator(0, 30, 3, this));
    connect(min_comp_time_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::minCompTimeEditSlot);

    form_layout_->addRow("Minimum Comparison Time [s]", min_comp_time_edit_);

    // max comp time
    max_comp_time_edit_ = new QLineEdit(QString::number(config().maximumComparisonTime()));
    max_comp_time_edit_->setValidator(new QDoubleValidator(0, 300, 0, this));
    connect(max_comp_time_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::maxCompTimeEditSlot);

    form_layout_->addRow("Maximum Comparison Time [s]", max_comp_time_edit_);


    // mark primary-only tracks
    mark_primary_only_check_ = new QCheckBox ();
    mark_primary_only_check_->setChecked(config().markPrimaryOnly());
    connect(mark_primary_only_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleMarkPrimaryOnlySlot);

    form_layout_->addRow("Mark Primary-Only", mark_primary_only_check_);


    // use min updates
    use_min_updates_check_ = new QCheckBox ();
    use_min_updates_check_->setChecked(config().useMinUpdates());
    connect(use_min_updates_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMinUpdatesSlot);

    form_layout_->addRow("Use Minimum Updates", use_min_updates_check_);

    // min updates
    min_updates_edit_ = new QLineEdit(QString::number(config().minUpdates()));
    min_updates_edit_->setValidator(new QDoubleValidator(0, 100, 0, this));
    connect(min_updates_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::minUpdatesEditSlot);

    form_layout_->addRow("Minimum Updates [1]", min_updates_edit_);


    // use min duration
    use_min_duration_check_ = new QCheckBox ();
    use_min_duration_check_->setChecked(config().useMinDuration());
    connect(use_min_duration_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMinDurationSlot);

    form_layout_->addRow("Use Minimum Duration", use_min_duration_check_);

    // min duration
    min_duration_edit_ = new QLineEdit(QString::number(config().minDuration()));
    min_duration_edit_->setValidator(new QDoubleValidator(0, 100, 0, this));
    connect(min_duration_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::minDurationEditSlot);

    form_layout_->addRow("Minimum Duration [s]", min_duration_edit_);


    // use max groundspeed
    use_max_groundspeed_check_ = new QCheckBox ();
    use_max_groundspeed_check_->setChecked(config().useMaxGroundspeed());
    connect(use_max_groundspeed_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMaxGroundspeedSlot);

    form_layout_->addRow("Use Maximum Groundspeed", use_max_groundspeed_check_);

    // max groundspeed
    max_groundspeed_kts_edit_ = new QLineEdit(QString::number(config().maxGroundspeedKts()));
    max_groundspeed_kts_edit_->setValidator(new QDoubleValidator(0, 10000, 0, this));
    connect(max_groundspeed_kts_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::maxGroundspeedEditSlot);

    form_layout_->addRow("Maximum Groundspeed [kts]", max_groundspeed_kts_edit_);

    // use max accel
    use_max_acceleration_check_ = new QCheckBox ();
    use_max_acceleration_check_->setChecked(config().useMaxAcceleration());
    connect(use_max_acceleration_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMaxAccelerationSlot);

    form_layout_->addRow("Use Maximum Acceleration", use_max_acceleration_check_);

    // max accel
    max_acceleration_edit_ = new QLineEdit(QString::number(config().maxAcceleration()));
    max_acceleration_edit_->setValidator(new QDoubleValidator(0, 1000, 0, this));
    connect(max_acceleration_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::maxAccelerationEditSlot);

    form_layout_->addRow("Maximum Acceleration [m/s^2]", max_acceleration_edit_);

    // use max turnrate
    use_max_turnrate_check_ = new QCheckBox ();
    use_max_turnrate_check_->setChecked(config().useMaxTurnrate());
    connect(use_max_turnrate_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMaxTurnrateSlot);

    form_layout_->addRow("Use Maximum Turnrate", use_max_turnrate_check_);

    // max turnrate
    max_turnrate_edit_ = new QLineEdit(QString::number(config().maxTurnrate()));
    max_turnrate_edit_->setValidator(new QDoubleValidator(0, 360, 0, this));
    connect(max_turnrate_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::maxTurnrateEditSlot);

    form_layout_->addRow("Maximum Turnrate [°/s]", max_turnrate_edit_);

    // use max rocd
    use_max_rocd_check_ = new QCheckBox ();
    use_max_rocd_check_->setChecked(config().useMaxROCD());
    connect(use_max_rocd_check_, &QCheckBox::clicked,
            this, &DubiousTargetConfigWidget::toggleUseMaxROCDSlot);

    form_layout_->addRow("Use Maximum ROCD", use_max_rocd_check_);

    // max rocd
    max_rocd_edit_ = new QLineEdit(QString::number(config().maxROCD()));
    max_rocd_edit_->setValidator(new QDoubleValidator(0, 10000, 0, this));
    connect(max_rocd_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::maxROCDEditSlot);

    form_layout_->addRow("Maximum ROCD [ft/s]", max_rocd_edit_);

    // dubious prob
    dubious_prob_edit_ = new QLineEdit(QString::number(config().dubiousProb()));
    dubious_prob_edit_->setValidator(new QDoubleValidator(0, 1, 4, this));
    connect(dubious_prob_edit_, &QLineEdit::textEdited,
            this, &DubiousTargetConfigWidget::dubiousProbEditSlot);

    form_layout_->addRow("Dubious Probability [1]", dubious_prob_edit_);

    updateActive();
}

void DubiousTargetConfigWidget::minCompTimeEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minimumComparisonTime(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::maxCompTimeEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maximumComparisonTime(val);
    else
        loginf << "invalid value";
}


void DubiousTargetConfigWidget::toggleMarkPrimaryOnlySlot()
{
    loginf;

    traced_assert(mark_primary_only_check_);
    config().markPrimaryOnly(mark_primary_only_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::toggleUseMinUpdatesSlot()
{
    loginf;

    traced_assert(use_min_updates_check_);
    config().useMinUpdates(use_min_updates_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::minUpdatesEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    unsigned int val = value.toUInt(&ok);

    if (ok)
        config().minUpdates(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::toggleUseMinDurationSlot()
{
    loginf;

    traced_assert(use_min_duration_check_);
    config().useMinDuration(use_min_duration_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::minDurationEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minDuration(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::toggleUseMaxGroundspeedSlot()
{
    loginf;

    traced_assert(use_max_groundspeed_check_);
    config().useMaxGroundspeed(use_max_groundspeed_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::maxGroundspeedEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxGroundspeedKts(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::toggleUseMaxAccelerationSlot()
{
    loginf;

    traced_assert(use_max_acceleration_check_);
    config().useMaxAcceleration(use_max_acceleration_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::maxAccelerationEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxAcceleration(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::toggleUseMaxTurnrateSlot()
{
    loginf;

    traced_assert(use_max_turnrate_check_);
    config().useMaxTurnrate(use_max_turnrate_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::maxTurnrateEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxTurnrate(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::toggleUseMaxROCDSlot()
{
    loginf;

    traced_assert(use_max_rocd_check_);
    config().useMaxROCD(use_max_rocd_check_->checkState() == Qt::Checked);

    updateActive();
}

void DubiousTargetConfigWidget::maxROCDEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxROCD(val);
    else
        loginf << "invalid value";
}

void DubiousTargetConfigWidget::dubiousProbEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().dubiousProb(val);
    else
        loginf << "invalid value";
}


DubiousTargetConfig& DubiousTargetConfigWidget::config()
{
    DubiousTargetConfig* config = dynamic_cast<DubiousTargetConfig*>(&config_);
    traced_assert(config);

    return *config;
}

void DubiousTargetConfigWidget::updateActive()
{
    traced_assert(min_updates_edit_);
    min_updates_edit_->setEnabled(config().useMinUpdates());

    traced_assert(min_duration_edit_);
    min_duration_edit_->setEnabled(config().useMinDuration());

    traced_assert(max_groundspeed_kts_edit_);
    max_groundspeed_kts_edit_->setEnabled(config().useMaxGroundspeed());

    traced_assert(max_acceleration_edit_);
    max_acceleration_edit_->setEnabled(config().useMaxAcceleration());

    traced_assert(max_turnrate_edit_);
    max_turnrate_edit_->setEnabled(config().useMaxTurnrate());

    traced_assert(max_rocd_edit_);
    max_rocd_edit_->setEnabled(config().useMaxROCD());
}




}
