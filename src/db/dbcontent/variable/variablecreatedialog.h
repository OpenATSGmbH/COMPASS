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

#include "dbcontent/variable/variable.h"

#include <QDialog>

class UnitSelectionWidget;
class StringRepresentationComboBox;

class QLineEdit;
class QCheckBox;
class QPushButton;
class QTextEdit;

namespace dbContent
{

class Variable;
class VariableDataTypeComboBox;

class VariableCreateDialog : public QDialog
{
    Q_OBJECT

public slots:
    void nameChangedSlot(const QString& name);
    void shortNameChangedSlot(const QString& name);
    void commentChangedSlot();
    void dbColumnChangedSlot(const QString& name);

public:
    VariableCreateDialog(DBContent& object, const std::string name="",
                            const std::string description="",
                            QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());

    std::string name() const;

    std::string shortName() const;

    std::string dataTypeStr() const;

    std::string dimension() const;

    std::string unit() const;

    std::string  representationStr() const;

    std::string description() const;

    std::string dbColumnName() const;

protected:
    DBContent& object_;

    QLineEdit* name_edit_ {nullptr};
    QLineEdit* short_name_edit_ {nullptr};
    QTextEdit* description_edit_ {nullptr};
    dbContent::VariableDataTypeComboBox* type_combo_ {nullptr};
    UnitSelectionWidget* unit_sel_ {nullptr};
    StringRepresentationComboBox* representation_box_ {nullptr};
    QLineEdit* db_column_edit_ {nullptr};

    QPushButton* cancel_button_ {nullptr};
    QPushButton* ok_button_ {nullptr};

    std::string name_;
    bool name_ok_ {false};
    std::string short_name_;

    PropertyDataType data_type_ {PropertyDataType::BOOL};
    std::string data_type_str_ {"BOOL"};
    std::string dimension_;
    std::string unit_;

    Variable::Representation representation_ {Variable::Representation::STANDARD};
    std::string representation_str_ {"STANDARD"};

    std::string description_;
    std::string db_column_name_;
    bool db_column_name_ok_ {false};

    void checkSettings();
};

}
