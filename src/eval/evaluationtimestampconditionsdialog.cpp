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

#include "evaluationtimestampconditionsdialog.h"
#include "util/timeconv.h"
#include "timewindowcollectionwidget.h"
#include "evaluationmanager.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFormLayout>
#include <QCheckBox>
#include <QLineEdit>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QScrollArea>

using namespace Utils;

EvaluationTimestampConditionsDialog::EvaluationTimestampConditionsDialog(EvaluationManager& eval_man, QWidget* parent)
    : QDialog(parent), eval_man_(eval_man)
{
    setWindowTitle("Edit Evaluation Timestamp Conditions");

    setModal(true);

    setMinimumSize(QSize(600, 400));

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout;
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    // time
    use_time_check_ = new QCheckBox ();
    connect(use_time_check_, &QCheckBox::clicked, this, &EvaluationTimestampConditionsDialog::toggleUseTimeSlot);
    form_layout->addRow("Use Timestamp Filter", use_time_check_);

    time_begin_edit_ = new QDateTimeEdit(QDateTime::currentDateTime());
    time_begin_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT.c_str());
    connect(time_begin_edit_, &QDateTimeEdit::dateTimeChanged, this, &EvaluationTimestampConditionsDialog::timeBeginEditedSlot);
    form_layout->addRow("Timestamp Begin", time_begin_edit_);

    time_end_edit_ = new QDateTimeEdit(QDateTime::currentDateTime());
    time_end_edit_->setDisplayFormat(Time::QT_DATETIME_FORMAT.c_str());
    connect(time_end_edit_, &QDateTimeEdit::dateTimeChanged, this, &EvaluationTimestampConditionsDialog::timeEndEditedSlot);
    form_layout->addRow("Timestamp End", time_end_edit_);

    tw_widget_ = new TimeWindowCollectionWidget(eval_man.excludedTimeWindows());
    form_layout->addRow("Excluded Time Windows", tw_widget_);


    main_layout->addLayout(form_layout);

    main_layout->addStretch();

    // buttons

    QHBoxLayout* button_layout = new QHBoxLayout();

    button_layout->addStretch();

    QPushButton* run_button = new QPushButton("OK");
    connect(run_button, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(run_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    updateValues();
}

EvaluationTimestampConditionsDialog::~EvaluationTimestampConditionsDialog()
{
    tw_widget_ = nullptr;
}

bool EvaluationTimestampConditionsDialog::somethingChangedFlag() const
{
    traced_assert(tw_widget_);
    return something_changed_flag_ || tw_widget_->somethingChangedFlag();
}

void EvaluationTimestampConditionsDialog::updateValues()
{
    update_active_ = true;

    // time filter
    traced_assert(use_time_check_);
    use_time_check_->setChecked(eval_man_.useTimestampFilter());

    traced_assert(time_begin_edit_);
    //time_begin_edit_->setText(String::timeStringFromDouble(eval_man_.loadTimestampBegin()).c_str());
    time_begin_edit_->setDateTime(QDateTime::fromString(Time::toString(eval_man_.loadTimestampBegin()).c_str(),
                                                        Time::QT_DATETIME_FORMAT.c_str()));

    traced_assert(time_end_edit_);
    //time_end_edit_->setText(String::timeStringFromDouble(eval_man_.loadTimestampEnd()).c_str());
    time_end_edit_->setDateTime(QDateTime::fromString(Time::toString(eval_man_.loadTimestampEnd()).c_str(),
                                                      Time::QT_DATETIME_FORMAT.c_str()));

    update_active_ = false;

}

/**
 */
void EvaluationTimestampConditionsDialog::toggleUseTimeSlot()
{
    traced_assert(use_time_check_);
    eval_man_.useTimestampFilter(use_time_check_->checkState() == Qt::Checked);

    something_changed_flag_ = true;
}

/**
 */
void EvaluationTimestampConditionsDialog::timeBeginEditedSlot (const QDateTime& datetime)
{
    if (update_active_)
        return;

    loginf << "value "
           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

    eval_man_.loadTimestampBegin(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()));

    something_changed_flag_ = true;
}

/**
 */
void EvaluationTimestampConditionsDialog::timeEndEditedSlot (const QDateTime& datetime)
{
    if (update_active_)
        return;

    loginf << "value "
           << datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString();

    eval_man_.loadTimestampEnd(Time::fromString(datetime.toString(Time::QT_DATETIME_FORMAT.c_str()).toStdString()));

    something_changed_flag_ = true;
}
