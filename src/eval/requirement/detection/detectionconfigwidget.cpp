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

#include "eval/requirement/detection/detectionconfigwidget.h"
#include "eval/requirement/detection/detectionconfig.h"

#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

DetectionConfigWidget::DetectionConfigWidget(DetectionConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of detection or miss (inverted probability)");

    traced_assert(check_type_box_);

    // ui
    update_interval_edit_ = new QLineEdit(QString::number(config().updateInterval()));
    update_interval_edit_->setValidator(new QDoubleValidator(0.1, 30.0, 2, this));
    update_interval_edit_->setToolTip("Update interval of the test data");
    connect(update_interval_edit_, &QLineEdit::textEdited,
            this, &DetectionConfigWidget::updateIntervalEditSlot);

    form_layout_->addRow("Update Interval [s]", update_interval_edit_);

    // min gap
    use_min_gap_length_check_ = new QCheckBox ();
    use_min_gap_length_check_->setChecked(config().useMinGapLength());
    use_min_gap_length_check_->setToolTip("If minimum gap length should be used");
    connect(use_min_gap_length_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleUseMinGapLengthSlot);

    form_layout_->addRow("Use Minimum Gap Length", use_min_gap_length_check_);

    min_gap_length_edit_ = new QLineEdit(QString::number(config().minGapLength()));
    min_gap_length_edit_->setValidator(new QDoubleValidator(0.01, 300.0, 3, this));
    min_gap_length_edit_->setToolTip("Minimum gap length to be considered");
    connect(min_gap_length_edit_, &QLineEdit::textEdited,
            this, &DetectionConfigWidget::minGapLengthEditSlot);

    form_layout_->addRow("Minimum Gap Length [s]", min_gap_length_edit_);

    // max gap
    use_max_gap_length_check_ = new QCheckBox ();
    use_max_gap_length_check_->setChecked(config().useMaxGapLength());
    use_max_gap_length_check_->setToolTip("If maximum gap length should be used");
    connect(use_max_gap_length_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleUseMaxGapLengthSlot);

    form_layout_->addRow("Use Maximum Gap Length", use_max_gap_length_check_);

    max_gap_length_edit_ = new QLineEdit(QString::number(config().maxGapLength()));
    max_gap_length_edit_->setValidator(new QDoubleValidator(0.01, 300.0, 3, this));
    max_gap_length_edit_->setToolTip("Maximum gap length to be considered");
    connect(max_gap_length_edit_, &QLineEdit::textEdited,
            this, &DetectionConfigWidget::maxGapLengthEditSlot);

    form_layout_->addRow("Maximum Gap Length [s]", max_gap_length_edit_);

    // invert prob
    use_invert_prob_check_ = new QCheckBox ();
    use_invert_prob_check_->setChecked(config().invertProb());
    use_invert_prob_check_->setToolTip("If calculated probability should be inverted");
    connect(use_invert_prob_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleInvertProbSlot);

    form_layout_->addRow("Invert Probability", use_invert_prob_check_);

    // miss tolerance
    use_miss_tolerance_check_ = new QCheckBox ();
    use_miss_tolerance_check_->setChecked(config().useMissTolerance());
    use_miss_tolerance_check_->setToolTip("If miss tolerance should be used");
    connect(use_miss_tolerance_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleUseMissToleranceSlot);

    form_layout_->addRow("Use Miss Tolerance", use_miss_tolerance_check_);

    miss_tolerance_edit_ = new QLineEdit(QString::number(config().missTolerance()));
    miss_tolerance_edit_->setValidator(new QDoubleValidator(0.01, 1.0, 3, this));
    miss_tolerance_edit_->setToolTip("Acceptable time delta for miss detection");
    connect(miss_tolerance_edit_, &QLineEdit::textEdited,
            this, &DetectionConfigWidget::missToleranceEditSlot);

    form_layout_->addRow("Miss Tolerance [s]", miss_tolerance_edit_);

    // hold_for_any_target_check_
    hold_for_any_target_check_ = new QCheckBox ();
    hold_for_any_target_check_->setChecked(config().holdForAnyTarget());
    hold_for_any_target_check_->setToolTip("If requirement must hold for any target (all single targets)");
    connect(hold_for_any_target_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleHoldForAnyTargetSlot);

    form_layout_->addRow("Hold for any target", hold_for_any_target_check_);


    // ignore_primary_only
    ignore_primary_only_check_ = new QCheckBox ();
    ignore_primary_only_check_->setChecked(config().ignorePrimaryOnly());
    ignore_primary_only_check_->setToolTip("Requirement result is ignored if target is primary only (has no"
                                           " secondary attributes, also not in reference)");
    connect(ignore_primary_only_check_, &QCheckBox::clicked,
            this, &DetectionConfigWidget::toggleIgnorePrimaryOnlySlot);

    form_layout_->addRow("Ignore Primary Only", ignore_primary_only_check_);

    updateActive();
}


void DetectionConfigWidget::updateIntervalEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().updateInterval(val);
    else
        loginf << "invalid value";
}

// min
void DetectionConfigWidget::toggleUseMinGapLengthSlot()
{
    loginf;

    traced_assert(use_min_gap_length_check_);
    config().useMinGapLength(use_min_gap_length_check_->checkState() == Qt::Checked);

    updateActive();
}
void DetectionConfigWidget::minGapLengthEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minGapLength(val);
    else
        loginf << "invalid value";
}

// max
void DetectionConfigWidget::toggleUseMaxGapLengthSlot()
{
    loginf;

    traced_assert(use_max_gap_length_check_);
    config().useMaxGapLength(use_max_gap_length_check_->checkState() == Qt::Checked);

    updateActive();
}
void DetectionConfigWidget::maxGapLengthEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().maxGapLength(val);
    else
        loginf << "axvalid value";
}

// invert prob
void DetectionConfigWidget::toggleInvertProbSlot()
{
    loginf;

    traced_assert(use_invert_prob_check_);
    config().invertProb(use_invert_prob_check_->checkState() == Qt::Checked);

    updateActive();
}

// miss tol
void DetectionConfigWidget::toggleUseMissToleranceSlot()
{
    loginf;

    traced_assert(use_miss_tolerance_check_);
    config().useMissTolerance(use_miss_tolerance_check_->checkState() == Qt::Checked);

    updateActive();
}
void DetectionConfigWidget::missToleranceEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().missTolerance(val);
    else
        loginf << "invalid value";
}

void DetectionConfigWidget::toggleHoldForAnyTargetSlot()
{
    loginf;

    traced_assert(hold_for_any_target_check_);
    config().holdForAnyTarget(hold_for_any_target_check_->checkState() == Qt::Checked);
}

DetectionConfig& DetectionConfigWidget::config()
{
    DetectionConfig* config = dynamic_cast<DetectionConfig*>(&config_);
    traced_assert(config);

    return *config;
}


void DetectionConfigWidget::toggleIgnorePrimaryOnlySlot()
{
    loginf;

    traced_assert(ignore_primary_only_check_);
    config().ignorePrimaryOnly(ignore_primary_only_check_->checkState() == Qt::Checked);
}

void DetectionConfigWidget::updateActive()
{
    traced_assert(min_gap_length_edit_);
    min_gap_length_edit_->setEnabled(config().useMinGapLength());

    traced_assert(max_gap_length_edit_);
    max_gap_length_edit_->setEnabled(config().useMaxGapLength());

    traced_assert(miss_tolerance_edit_);
    miss_tolerance_edit_->setEnabled(config().useMissTolerance());
}

}
