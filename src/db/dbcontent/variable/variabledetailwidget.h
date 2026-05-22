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

#include <QWidget>

class QLineEdit;
class QTextEdit;
class UnitSelectionWidget;
class StringRepresentationComboBox;

class DBContentManager;

namespace dbContent
{

class Variable;
class VariableDataTypeComboBox;

/**
 * @brief Right-side detail editor for a single Variable.
 *
 * Used inside DBContentEditDialog's stacked widget. Mirrors the form layout
 * of VariableEditDialog but is non-modal: bind to a Variable via show(),
 * unbind via clear(). Read-only outside expert mode.
 */
class VariableDetailWidget : public QWidget
{
    Q_OBJECT

public slots:
    void nameChangedSlot(const QString& name);
    void shortNameChangedSlot(const QString& name);
    void descriptionChangedSlot();
    void sourceChangedSlot();
    void dbColumnChangedSlot(const QString& name);

public:
    VariableDetailWidget(DBContentManager& dbcont_man, QWidget* parent = nullptr);

    void show(Variable& variable);
    void clear();

private:
    DBContentManager& dbcont_man_;

    bool has_current_entry_ {false};
    Variable* variable_ {nullptr};

    QLineEdit* name_edit_ {nullptr};
    QLineEdit* short_name_edit_ {nullptr};
    QTextEdit* description_edit_ {nullptr};
    QTextEdit* source_edit_ {nullptr};
    VariableDataTypeComboBox* type_combo_ {nullptr};
    UnitSelectionWidget* unit_sel_ {nullptr};
    StringRepresentationComboBox* representation_box_ {nullptr};
    QLineEdit* db_column_edit_ {nullptr};
};

}
