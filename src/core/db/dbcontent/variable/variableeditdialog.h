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

class UnitSelectionWidget;
class StringRepresentationComboBox;

class QLineEdit;
class QCheckBox;
class QComboBox;
class QListWidget;
class QPushButton;
class QTextEdit;

namespace dbContent
{

class Variable;
class VariableDataTypeComboBox;

class VariableEditDialog : public QDialog
{
    Q_OBJECT

public slots:
    void nameChangedSlot(const QString& name);
    void shortNameChangedSlot(const QString& name);
    void commentChangedSlot();
    void dbColumnChangedSlot(const QString& name);

    void originChangedSlot(const QString& origin);
    void dataItemChangedSlot(const QString& data_item);
    void inputsChangedSlot(const QString& inputs);
    void sourceVariableChangedSlot(const QString& name);
    void moveCandidateUpSlot();
    void moveCandidateDownSlot();
    void removeCandidateSlot();
    void addCandidateSlot();

    void doneSlot();

public:
    VariableEditDialog(Variable& variable, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    bool variableEdited() const;

    Variable &variable() const;

protected:
    Variable& variable_;

    bool expert_mode_ {false}; // COMPASS expert mode

    QLineEdit* name_edit_ {nullptr};
    QLineEdit* short_name_edit_ {nullptr};
    QTextEdit* description_edit_ {nullptr};

    // origin model, see readme_dbcontent.md: the widgets of the other origins are disabled
    QComboBox* origin_combo_ {nullptr};
    QLineEdit* data_item_edit_ {nullptr};
    QLineEdit* inputs_edit_ {nullptr};
    QListWidget* candidates_list_ {nullptr};
    QPushButton* candidate_up_button_ {nullptr};
    QPushButton* candidate_down_button_ {nullptr};
    QPushButton* candidate_remove_button_ {nullptr};
    QComboBox* candidate_add_combo_ {nullptr};
    QPushButton* candidate_add_button_ {nullptr};
    QLineEdit* source_variable_edit_ {nullptr};

    void updateOriginWidgets();
    void updateCandidates(int current_row = -1);
    void moveCandidate(int from, int to);
    VariableDataTypeComboBox* type_combo_ {nullptr};
    UnitSelectionWidget* unit_sel_ {nullptr};
    StringRepresentationComboBox* representation_box_ {nullptr};
    QLineEdit* db_column_edit_ {nullptr};

    QPushButton* done_button_ {nullptr};

    bool variable_edited_ {false};
};

}
