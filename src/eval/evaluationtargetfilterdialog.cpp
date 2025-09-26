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

#include "evaluationtargetfilterdialog.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "evaluationtargetfilter.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QLineEdit>
#include <QCheckBox>
#include <QFormLayout>

using namespace std;



EvaluationTargetFilterDialog::EvaluationTargetFilterDialog(
    EvaluationTargetFilter& target_filter,
    dbContent::TargetModel& model, QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), target_filter_(target_filter), model_(model)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setWindowTitle("Filter Evaluation Targets");

    setModal(true);

    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    // config stuff
    QFormLayout* config_layout = new QFormLayout();

    // shorts
    remove_short_check_ = new QCheckBox();
    remove_short_check_->setChecked(target_filter_.removeShortTargets());
    connect(remove_short_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeShortTargetsSlot);

    config_layout->addRow("Remove Short Targets", remove_short_check_);

    remove_st_min_updates_edit_ = new QLineEdit();
    remove_st_min_updates_edit_->setText(QString::number(target_filter_.removeShortTargetsMinUpdates()));
    connect(remove_st_min_updates_edit_, &QLineEdit::editingFinished,
            this, &EvaluationTargetFilterDialog::removeSTMinUpdatesEditedSlot);
    config_layout->addRow(tr("\tShort Targets Minimum Updates [1]"), remove_st_min_updates_edit_);

    remove_st_min_duration_edit_ = new QLineEdit();
    remove_st_min_duration_edit_->setText(QString::number(target_filter_.removeShortTargetsMinDuration()));
    connect(remove_st_min_duration_edit_, &QLineEdit::editingFinished,
            this, &EvaluationTargetFilterDialog::removeSTMinDurationEditedSlot);
    config_layout->addRow(tr("\tShort Targets Duration [s]"), remove_st_min_duration_edit_);

    // psr only
    remove_psr_only_targets_check_ = new QCheckBox();
    remove_psr_only_targets_check_->setChecked(target_filter_.removePsrOnlyTargets());
    connect(remove_psr_only_targets_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removePSROnlyTargetsSlot);

    config_layout->addRow("Remove Primary-Only Targets", remove_psr_only_targets_check_);

    // ma
    remove_mode_ac_only_check_ = new QCheckBox();
    remove_mode_ac_only_check_->setChecked(target_filter_.removeModeACOnlys());
    connect(remove_mode_ac_only_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeModeACOnlyTargetsSlot);
    config_layout->addRow("Remove Mode A/C Code onlys", remove_mode_ac_only_check_);

    remove_mode_a_check_ = new QCheckBox();
    remove_mode_a_check_->setChecked(target_filter_.filterModeACodes());
    connect(remove_mode_a_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeModeASlot);

    config_layout->addRow("Remove By Mode A Code", remove_mode_a_check_);

    remove_mode_a_blacklist_check_ = new QCheckBox();
    remove_mode_a_blacklist_check_->setChecked(target_filter_.filterModeACodeBlacklist());
    remove_mode_a_blacklist_check_->setToolTip("If checked, values are a blacklist - if not checked, a whitelist");
    connect(remove_mode_a_blacklist_check_, &QCheckBox::clicked,
            this, &EvaluationTargetFilterDialog::removeModeABlackListSlot);

    config_layout->addRow("Mode A Code Blacklist", remove_mode_a_blacklist_check_);


    remove_mode_a_edit_ = new QTextEdit();
    remove_mode_a_edit_->setText(target_filter_.filterModeACodeValues().c_str());
    connect(remove_mode_a_edit_, &QTextEdit::textChanged, this,
            &EvaluationTargetFilterDialog::removeModeAValuesSlot);

    config_layout->addRow("\tMode A Code (oct,oct1-oct2)", remove_mode_a_edit_);

    // mc

    remove_mode_c_check_ = new QCheckBox();
    remove_mode_c_check_->setChecked(target_filter_.removeModeCValues());
    connect(remove_mode_c_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeModeCSlot);

    config_layout->addRow("Remove By Mode C Code", remove_mode_c_check_);

    remove_mode_c_min_edit_ = new QTextEdit();
    remove_mode_c_min_edit_->setText(QString::number(target_filter_.removeModeCMinValue()));
    connect(remove_mode_c_min_edit_, &QTextEdit::textChanged, this,
            &EvaluationTargetFilterDialog::removeModeCMinValueSlot);

    config_layout->addRow("\tMode Min Value [ft]", remove_mode_c_min_edit_);

    // ta
    remove_ta_check_ = new QCheckBox();
    remove_ta_check_->setChecked(target_filter_.filterTargetAddresses());
    connect(remove_ta_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeTASlot);

    config_layout->addRow("Remove By Target Address", remove_ta_check_);

    remove_ta_blacklist_check_ = new QCheckBox();
    remove_ta_blacklist_check_->setChecked(target_filter_.filterTargetAddressesBlacklist());
    remove_ta_blacklist_check_->setToolTip("If checked, values are a blacklist - if not checked, a whitelist");
    connect(remove_ta_blacklist_check_, &QCheckBox::clicked,
            this, &EvaluationTargetFilterDialog::removeTABlackListSlot);

    config_layout->addRow("Target Addresses Blacklist", remove_ta_blacklist_check_);

    remove_ta_edit_ = new QTextEdit();
    remove_ta_edit_->setText(target_filter_.filterTargetAddressValues().c_str());
    connect(remove_ta_edit_, &QTextEdit::textChanged, this,
            &EvaluationTargetFilterDialog::removeTAValuesSlot);

    config_layout->addRow("\tTarget Addresses (hex)", remove_ta_edit_);

    // dbconts
    remove_dbcont_check_ = new QCheckBox();
    remove_dbcont_check_->setChecked(target_filter_.removeNotDetectedDBContents());
    connect(remove_dbcont_check_, &QCheckBox::clicked, this,
            &EvaluationTargetFilterDialog::removeDBContentsSlot);

    config_layout->addRow("Remove By Non-Detection in DBContent", remove_dbcont_check_);

    for (auto& dbcont_it : COMPASS::instance().dbContentManager())
    {
        if (dbcont_it.second->containsStatusContent())
            continue;

        QCheckBox* tmp = new QCheckBox();
        tmp->setChecked(target_filter_.removeNotDetectedDBContent(dbcont_it.first));
        tmp->setProperty("dbcontent_name", dbcont_it.first.c_str());
        connect(tmp, &QCheckBox::clicked, this,
                &EvaluationTargetFilterDialog::removeSpecificDBContentsSlot);

        config_layout->addRow(("\tNon-Detection of "+dbcont_it.first).c_str(), tmp);
    }

    main_layout->addLayout(config_layout);

    main_layout->addStretch();

    // buttons

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &EvaluationTargetFilterDialog::cancelSlot);
    button_layout->addWidget(cancel_button_);

    button_layout->addStretch();

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &EvaluationTargetFilterDialog::runSlot);
    button_layout->addWidget(run_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}

void EvaluationTargetFilterDialog::removeShortTargetsSlot(bool checked)
{
    target_filter_.removeShortTargets(checked);
}

void EvaluationTargetFilterDialog::removeSTMinUpdatesEditedSlot()
{
    traced_assert(remove_st_min_updates_edit_);
    QString text = remove_st_min_updates_edit_->text();

    bool ok;
    unsigned int value = text.toUInt(&ok);

    if (ok)
        target_filter_.removeShortTargetsMinUpdates(value);
    else
        logwrn << "conversion of text '"
               << text.toStdString() << "' failed";
}

void EvaluationTargetFilterDialog::removeSTMinDurationEditedSlot()
{
    traced_assert(remove_st_min_duration_edit_);
    QString text = remove_st_min_duration_edit_->text();

    bool ok;
    float value = text.toFloat(&ok);

    if (ok)
        target_filter_.removeShortTargetsMinDuration(value);
    else
        logwrn << "conversion of text '"
               << text.toStdString() << "' failed";
}

void EvaluationTargetFilterDialog::removePSROnlyTargetsSlot(bool checked)
{
    target_filter_.removePsrOnlyTargets(checked);
}

void EvaluationTargetFilterDialog::removeModeACOnlyTargetsSlot(bool checked)
{
    target_filter_.removeModeACOnlys(checked);
}

void EvaluationTargetFilterDialog::removeModeASlot(bool checked)
{
    target_filter_.filterModeACodes(checked);
}

void EvaluationTargetFilterDialog::removeModeABlackListSlot(bool checked)
{
    target_filter_.filterModeACodeBlacklist(checked);
}

void EvaluationTargetFilterDialog::removeModeAValuesSlot()
{
    traced_assert(remove_mode_a_edit_);
    target_filter_.filterModeACodeValues(remove_mode_a_edit_->document()->toPlainText().toStdString());
}

void EvaluationTargetFilterDialog::removeModeCSlot(bool checked)
{
    target_filter_.removeModeCValues(checked);
}

void EvaluationTargetFilterDialog::removeModeCMinValueSlot()
{
    traced_assert(remove_mode_c_min_edit_);
    target_filter_.removeModeCMinValue(remove_mode_c_min_edit_->document()->toPlainText().toFloat());
}

void EvaluationTargetFilterDialog::removeTASlot(bool checked)
{
    target_filter_.filterTargetAddresses(checked);
}

void EvaluationTargetFilterDialog::removeTABlackListSlot(bool checked)
{
    target_filter_.filterTargetAddressesBlacklist(checked);
}

void EvaluationTargetFilterDialog::removeTAValuesSlot()
{
    traced_assert(remove_ta_edit_);
    target_filter_.filterTargetAddressValues(remove_ta_edit_->document()->toPlainText().toStdString());
}

void EvaluationTargetFilterDialog::removeDBContentsSlot(bool checked)
{
    target_filter_.removeNotDetectedDBContents(checked);
}

void EvaluationTargetFilterDialog::removeSpecificDBContentsSlot(bool checked)
{
    QCheckBox* tmp = dynamic_cast<QCheckBox*>(sender());
    traced_assert(tmp);

    QVariant data = tmp->property("dbcontent_name");
    traced_assert(data.isValid());

    string dbcontent_name = data.toString().toStdString();

    target_filter_.removeNotDetectedDBContents(dbcontent_name,checked);
}

void EvaluationTargetFilterDialog::runSlot()
{
    model_.setUseByFilter();

    close();
}

void EvaluationTargetFilterDialog::cancelSlot()
{
    close();
}


