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

#include "intervalbaseconfigwidget.h"
#include "intervalbaseconfig.h"

#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

namespace EvaluationRequirement 
{

/**
*/
IntervalBaseConfigWidget::IntervalBaseConfigWidget(IntervalBaseConfig& cfg)
:   ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(form_layout_);

    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of code detection or miss (inverted probability)");

    traced_assert(check_type_box_);

    uint32_t config_flags = config().configFlags();

    // ui
    update_interval_edit_ = new QLineEdit(QString::number(config().updateInterval()));
    update_interval_edit_->setValidator(new QDoubleValidator(0.1, 30.0, 2, this));
    update_interval_edit_->setToolTip("Update interval of the test data");
    connect(update_interval_edit_, &QLineEdit::textEdited,
            this, &IntervalBaseConfigWidget::updateIntervalEditSlot);

    form_layout_->addRow("Update Interval [s]", update_interval_edit_);

    // min gap
    if (config_flags & IntervalBaseConfig::UseMinGapLen)
    {
        use_min_gap_length_check_ = new QCheckBox ();
        use_min_gap_length_check_->setChecked(config().useMinGapLength());
        use_min_gap_length_check_->setToolTip("If minimum gap length should be used");
        connect(use_min_gap_length_check_, &QCheckBox::clicked,
                this, &IntervalBaseConfigWidget::toggleUseMinGapLengthSlot);

        form_layout_->addRow("Use Minimum Gap Length", use_min_gap_length_check_);

        min_gap_length_edit_ = new QLineEdit(QString::number(config().minGapLength()));
        min_gap_length_edit_->setValidator(new QDoubleValidator(0.01, 300.0, 3, this));
        min_gap_length_edit_->setToolTip("Minimum gap length to be considered");
        connect(min_gap_length_edit_, &QLineEdit::textEdited,
                this, &IntervalBaseConfigWidget::minGapLengthEditSlot);

        form_layout_->addRow("Minimum Gap Length [s]", min_gap_length_edit_);
    }

    // max gap
    if (config_flags & IntervalBaseConfig::UseMaxGapLen)
    {
        use_max_gap_length_check_ = new QCheckBox ();
        use_max_gap_length_check_->setChecked(config().useMaxGapLength());
        use_max_gap_length_check_->setToolTip("If maximum gap length should be used");
        connect(use_max_gap_length_check_, &QCheckBox::clicked,
                this, &IntervalBaseConfigWidget::toggleUseMaxGapLengthSlot);

        form_layout_->addRow("Use Maximum Gap Length", use_max_gap_length_check_);

        max_gap_length_edit_ = new QLineEdit(QString::number(config().maxGapLength()));
        max_gap_length_edit_->setValidator(new QDoubleValidator(0.01, 300.0, 3, this));
        max_gap_length_edit_->setToolTip("Maximum gap length to be considered");
        connect(max_gap_length_edit_, &QLineEdit::textEdited,
                this, &IntervalBaseConfigWidget::maxGapLengthEditSlot);

        form_layout_->addRow("Maximum Gap Length [s]", max_gap_length_edit_);
    }

    // miss tolerance
    if (config_flags & IntervalBaseConfig::UseMissTol)
    {
        use_miss_tolerance_check_ = new QCheckBox ();
        use_miss_tolerance_check_->setChecked(config().useMissTolerance());
        use_miss_tolerance_check_->setToolTip("If miss tolerance should be used");
        connect(use_miss_tolerance_check_, &QCheckBox::clicked,
                this, &IntervalBaseConfigWidget::toggleUseMissToleranceSlot);

        form_layout_->addRow("Use Miss Tolerance", use_miss_tolerance_check_);

        miss_tolerance_edit_ = new QLineEdit(QString::number(config().missTolerance()));
        miss_tolerance_edit_->setValidator(new QDoubleValidator(0.01, 1.0, 3, this));
        miss_tolerance_edit_->setToolTip("Acceptable time delta for miss detection");
        connect(miss_tolerance_edit_, &QLineEdit::textEdited,
                this, &IntervalBaseConfigWidget::missToleranceEditSlot);

        form_layout_->addRow("Miss Tolerance [s]", miss_tolerance_edit_);
    }

    // hold_for_any_target_check_
    if (config_flags & IntervalBaseConfig::UseAnyTarget)
    {
        hold_for_any_target_check_ = new QCheckBox ();
        hold_for_any_target_check_->setChecked(config().holdForAnyTarget());
        hold_for_any_target_check_->setToolTip("If requirement must hold for any target (all single targets)");
        connect(hold_for_any_target_check_, &QCheckBox::clicked,
                this, &IntervalBaseConfigWidget::toggleHoldForAnyTargetSlot);

        form_layout_->addRow("Must hold for any target", hold_for_any_target_check_);
    }

    updateActive();
}

/**
*/
IntervalBaseConfig& IntervalBaseConfigWidget::config()
{
    IntervalBaseConfig* config = dynamic_cast<IntervalBaseConfig*>(&config_);
    traced_assert(config);

    return *config;
}

/**
*/
void IntervalBaseConfigWidget::updateActive()
{
    if (min_gap_length_edit_)
        min_gap_length_edit_->setEnabled(config().useMinGapLength());

    if (max_gap_length_edit_)
        max_gap_length_edit_->setEnabled(config().useMaxGapLength());

    if (miss_tolerance_edit_)
        miss_tolerance_edit_->setEnabled(config().useMissTolerance());
}

/**
*/
void IntervalBaseConfigWidget::updateIntervalEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().updateInterval(val);
    else
        loginf << "invalid value";
}

/**
*/
void IntervalBaseConfigWidget::toggleUseMinGapLengthSlot()
{
    loginf;

    if (use_min_gap_length_check_)
    {
        config().useMinGapLength(use_min_gap_length_check_->checkState() == Qt::Checked);
        updateActive();
    }
}
void IntervalBaseConfigWidget::minGapLengthEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minGapLength(val);
    else
        loginf << "invalid value";
}

/**
*/
void IntervalBaseConfigWidget::toggleUseMaxGapLengthSlot()
{
    loginf;

    if (use_max_gap_length_check_)
    {
        config().useMaxGapLength(use_max_gap_length_check_->checkState() == Qt::Checked);
        updateActive();
    }
}
void IntervalBaseConfigWidget::maxGapLengthEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxGapLength(val);
    else
        loginf << "axvalid value";
}

/**
*/
void IntervalBaseConfigWidget::toggleUseMissToleranceSlot()
{
    loginf;

    if (use_miss_tolerance_check_)
    {
        config().useMissTolerance(use_miss_tolerance_check_->checkState() == Qt::Checked);
        updateActive();
    }
}
void IntervalBaseConfigWidget::missToleranceEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().missTolerance(val);
    else
        loginf << "invalid value";
}

/**
*/
void IntervalBaseConfigWidget::toggleHoldForAnyTargetSlot()
{
    loginf;

    if (hold_for_any_target_check_)
    {
        config().holdForAnyTarget(hold_for_any_target_check_->checkState() == Qt::Checked);
    }
}

} // namespace EvaluationRequirement
