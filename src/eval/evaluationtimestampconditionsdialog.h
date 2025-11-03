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

#pragma once

#include "timewindowcollectionwidget.h"

#include <QDialog>

class EvaluationManager;

class QCheckBox;
class QDateTimeEdit;

class EvaluationTimestampConditionsDialog : public QDialog
{
    Q_OBJECT

private slots:
    void toggleUseTimeSlot();
    void timeBeginEditedSlot (const QDateTime& datetime);
    void timeEndEditedSlot (const QDateTime& datetime);

public:
    EvaluationTimestampConditionsDialog(EvaluationManager& eval_man, QWidget* parent=nullptr);
    virtual ~EvaluationTimestampConditionsDialog();

    bool somethingChangedFlag() const;

protected:
    EvaluationManager& eval_man_;

    QCheckBox* use_time_check_{nullptr};
    QDateTimeEdit* time_begin_edit_{nullptr};
    QDateTimeEdit* time_end_edit_{nullptr};

    TimeWindowCollectionWidget* tw_widget_{nullptr};

    bool update_active_ {false};
    bool something_changed_flag_ {false};

    void updateValues();

};

