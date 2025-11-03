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

#include <QDialog>

#include <map>

class QCheckBox;
class QLineEdit;
class QTextEdit;

namespace dbContent {

class TargetModel;

}

class EvaluationTargetFilter;

class EvaluationTargetFilterDialog  : public QDialog
{
    Q_OBJECT

public slots:
    // shorts
    void removeShortTargetsSlot(bool checked);
    void removeSTMinUpdatesEditedSlot();
    void removeSTMinDurationEditedSlot();
    // psr
    void removePSROnlyTargetsSlot(bool checked);
    // ma
    void removeModeACOnlyTargetsSlot(bool checked);
    void removeModeASlot(bool checked);
    void removeModeABlackListSlot(bool checked);
    void removeModeAValuesSlot();
    // mc
    void removeModeCSlot(bool checked);
    void removeModeCMinValueSlot();
    //ta
    void removeTASlot(bool checked);
    void removeTABlackListSlot(bool checked);
    void removeTAValuesSlot();
    // dbcont
    void removeDBContentsSlot(bool checked);
    void removeSpecificDBContentsSlot(bool checked);

    void runSlot();
    void cancelSlot();

public:
    EvaluationTargetFilterDialog(EvaluationTargetFilter& target_filter, dbContent::TargetModel& model,
                               QWidget* parent=nullptr, Qt::WindowFlags f = Qt::WindowFlags());

protected:
    EvaluationTargetFilter& target_filter_;
    dbContent::TargetModel& model_;

    QCheckBox* remove_short_check_ {nullptr};

    QLineEdit* remove_st_min_updates_edit_ {nullptr};
    QLineEdit* remove_st_min_duration_edit_ {nullptr};

    QCheckBox* remove_psr_only_targets_check_ {nullptr};

    QCheckBox* remove_mode_ac_only_check_ {nullptr};
    QCheckBox* remove_mode_a_check_ {nullptr};
    QCheckBox* remove_mode_a_blacklist_check_ {nullptr};
    QTextEdit* remove_mode_a_edit_ {nullptr};

    QCheckBox* remove_mode_c_check_ {nullptr};
    QTextEdit* remove_mode_c_min_edit_ {nullptr};

    QCheckBox* remove_ta_check_ {nullptr};
    QCheckBox* remove_ta_blacklist_check_ {nullptr};
    QTextEdit* remove_ta_edit_ {nullptr};

    QCheckBox* remove_dbcont_check_ {nullptr};
    std::map<std::string, QCheckBox*> remove_dbcont_checks_;

    QPushButton* run_button_{nullptr};
    QPushButton* cancel_button_{nullptr};

};


