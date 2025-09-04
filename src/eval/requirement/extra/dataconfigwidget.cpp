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

#include "eval/requirement/extra/dataconfigwidget.h"
#include "eval/requirement/extra/dataconfig.h"
//#include "textfielddoublevalidator.h"
#include "logger.h"

#include <QLineEdit>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleValidator>

using namespace std;

namespace EvaluationRequirement
{

ExtraDataConfigWidget::ExtraDataConfigWidget(ExtraDataConfig& cfg)
    : ProbabilityBaseConfigWidget(cfg)
{
    traced_assert(prob_edit_);
    prob_edit_->setToolTip("Probability of extra data");

    traced_assert(check_type_box_);

    // min_duration
    min_duration_edit_ = new QLineEdit(QString::number(config().minDuration()));
    min_duration_edit_->setValidator(new QDoubleValidator(0.1, 3000.0, 1, this));
    min_duration_edit_->setToolTip("Minimum track duration, requirement result is ignored if less");
    connect(min_duration_edit_, &QLineEdit::textEdited,
            this, &ExtraDataConfigWidget::minDurationEditSlot);

    form_layout_->addRow("Minimum Duration [s]", min_duration_edit_);

    // min_num_updates
    min_num_updates_edit_ = new QLineEdit(QString::number(config().minNumUpdates()));
    min_num_updates_edit_->setValidator(new QDoubleValidator(0, 300, 0, this));
    min_num_updates_edit_->setToolTip("Minimum number of extra target reports, requirement result is ignored if less");
    connect(min_num_updates_edit_, &QLineEdit::textEdited,
            this, &ExtraDataConfigWidget::minNumUpdatesEditSlot);

    form_layout_->addRow("Minimum Number of Updates [1]", min_num_updates_edit_);

    // ignore_primary_only
    ignore_primary_only_check_ = new QCheckBox ();
    ignore_primary_only_check_->setChecked(config().ignorePrimaryOnly());
    ignore_primary_only_check_->setToolTip("Requirement result is ignored if target is primary only (has no"
                                           " secondary attributes, also not in reference)");
    connect(ignore_primary_only_check_, &QCheckBox::clicked,
            this, &ExtraDataConfigWidget::toggleIgnorePrimaryOnlySlot);

    form_layout_->addRow("Ignore Primary Only", ignore_primary_only_check_);
}

void ExtraDataConfigWidget::minDurationEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    float val = value.toFloat(&ok);

    if (ok)
        config().minDuration(val);
    else
        loginf << "invalid value";
}

void ExtraDataConfigWidget::minNumUpdatesEditSlot(QString value)
{
    loginf << "value " << value.toStdString();

    bool ok;
    unsigned int val = value.toUInt(&ok);

    if (ok)
        config().minNumUpdates(val);
    else
        loginf << "invalid value";
}

void ExtraDataConfigWidget::toggleIgnorePrimaryOnlySlot()
{
    loginf;

    traced_assert(ignore_primary_only_check_);
    config().ignorePrimaryOnly(ignore_primary_only_check_->checkState() == Qt::Checked);
}

ExtraDataConfig& ExtraDataConfigWidget::config()
{
    ExtraDataConfig* config = dynamic_cast<ExtraDataConfig*>(&config_);
    traced_assert(config);

    return *config;
}

}
