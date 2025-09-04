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

#include "dbcontent/variable/variableeditdialog.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variabledatatypecombobox.h"
#include "stringrepresentationcombobox.h"
#include "unitselectionwidget.h"
//#include "stringconv.h"
#include "dbcontent/dbcontent.h"
#include "compass.h"
#include "logger.h"

#include <QLineEdit>
#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QTextEdit>

#include <boost/algorithm/string.hpp>

using namespace std;

namespace dbContent
{

VariableEditDialog::VariableEditDialog(Variable& variable, QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), variable_(variable)
{
    expert_mode_ = COMPASS::instance().expertMode();

    setWindowFlags(Qt::Window | Qt::WindowTitleHint); //  | Qt::CustomizeWindowHint

    if (expert_mode_)
        setWindowTitle("Edit DBContent Variable");
    else
        setWindowTitle("Show DBContent Variable");

    setModal(true);

    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout;
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    //    QLineEdit* name_edit_ {nullptr};

    name_edit_ = new QLineEdit(variable_.name().c_str());
    name_edit_->setReadOnly(!expert_mode_);
    connect(name_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::nameChangedSlot);
    form_layout->addRow("Name", name_edit_);

    //    QLineEdit* short_name_edit_ {nullptr};
    short_name_edit_ = new QLineEdit(variable_.shortName().c_str());
    short_name_edit_->setReadOnly(!expert_mode_);
    connect(short_name_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::shortNameChangedSlot);
    form_layout->addRow("Short Name", short_name_edit_);

    //    QTextEdit* description_edit_ {nullptr};
    description_edit_ = new QTextEdit();
    description_edit_->document()->setPlainText(variable_.description().c_str());
    short_name_edit_->setReadOnly(!expert_mode_);
    description_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);

    connect(description_edit_, &QTextEdit::textChanged, this,
            &VariableEditDialog::commentChangedSlot);

    form_layout->addRow("Comment", description_edit_);

    //    VariableDataTypeComboBox* type_combo_ {nullptr};
    type_combo_ = new VariableDataTypeComboBox(variable_.dataTypeRef(), variable_.dataTypeStringRef());
    type_combo_->setEnabled(expert_mode_);
    form_layout->addRow("Data Type", type_combo_);

    //    UnitSelectionWidget* unit_sel_ {nullptr};
    unit_sel_ = new UnitSelectionWidget(variable_.dimension(), variable_.unit());
    unit_sel_->setEnabled(expert_mode_);
    form_layout->addRow("Unit", unit_sel_);

    //    StringRepresentationComboBox* representation_box_ {nullptr};
    representation_box_ = new StringRepresentationComboBox(variable_.representationRef(),
                                                           variable_.representationStringRef());
    representation_box_->setEnabled(expert_mode_);
    form_layout->addRow("Representation", representation_box_);

    //    QLineEdit* db_column_edit_ {nullptr};
    db_column_edit_ = new QLineEdit(variable_.dbColumnName().c_str());
    db_column_edit_->setReadOnly(!expert_mode_);
    connect(db_column_edit_, &QLineEdit::textChanged, this, &VariableEditDialog::dbColumnChangedSlot);
    form_layout->addRow("DBColumn", db_column_edit_);

    main_layout->addLayout(form_layout);

    done_button_ = new QPushButton("Done");
    connect(done_button_, &QPushButton::clicked, this, &VariableEditDialog::doneSlot);
    main_layout->addWidget(done_button_);

    setLayout(main_layout);
}

bool VariableEditDialog::variableEdited() const
{
    return variable_edited_;
}

Variable &VariableEditDialog::variable() const
{
    return variable_;
}

void VariableEditDialog::nameChangedSlot(const QString& name)
{
    loginf << "name '" << name.trimmed().toStdString() << "'";

    traced_assert(name_edit_);
    string new_name = name.trimmed().toStdString();

    if (new_name == variable_.name())
        return;

    if (!new_name.size())
    {
        name_edit_->setStyleSheet(COMPASS::instance().lineEditInvalidStyle());
        return;
    }

    if (variable_.object().hasVariable(new_name))
    {
        logwrn << "name '" << new_name << "' already in use";

        name_edit_->setStyleSheet(COMPASS::instance().lineEditInvalidStyle());
        name_edit_->setToolTip(("Variable name '"+new_name+"' already in use").c_str());
        return;
    }

    // ok, rename

    name_edit_->setStyleSheet("");
    name_edit_->setToolTip("");

    loginf << "renaming '" << variable_.name() << "' to '" << new_name << "'";
    variable_.object().renameVariable(std::string(variable_.name()), new_name); // copy, otherwise wrong

    if (new_name.size())
    {
        string db_column_name = name.toStdString();

        std::replace_if(db_column_name.begin(), db_column_name.end(), [](char ch) {
                return !(isalnum(ch) || ch == '_');
            }, '_');

        boost::algorithm::to_lower(db_column_name);

        db_column_edit_->setText(db_column_name.c_str());
        dbColumnChangedSlot(db_column_name.c_str());
    }

    variable_edited_ = true;
}

void VariableEditDialog::shortNameChangedSlot(const QString& name)
{
    loginf << "name '" << name.trimmed().toStdString() << "'";

    variable_.shortName(name.trimmed().toStdString());

    variable_edited_ = true;
}

void VariableEditDialog::commentChangedSlot()
{
    traced_assert(description_edit_);

    variable_.description(description_edit_->document()->toPlainText().trimmed().toStdString());

    variable_edited_ = true;
}

void VariableEditDialog::dbColumnChangedSlot(const QString& name)
{
    traced_assert(db_column_edit_);
    string new_name = name.trimmed().toStdString();

    if (new_name == variable_.dbColumnName())
        return;

    if (!new_name.size())
    {
        db_column_edit_->setStyleSheet(COMPASS::instance().lineEditInvalidStyle());
        return;
    }

    if (variable_.object().hasVariableDBColumnName(new_name))
    {
        logwrn << "name '" << new_name << "' already in use";

        db_column_edit_->setStyleSheet(COMPASS::instance().lineEditInvalidStyle());
        db_column_edit_->setToolTip(("Variable DB Column name '"+new_name+"' already in use").c_str());
        return;
    }

    db_column_edit_->setStyleSheet("");
    db_column_edit_->setToolTip("");

    loginf << "changing '" << variable_.dbColumnName()
           << "' to '" << new_name << "'";
    variable_.dbColumnName(new_name);

    variable_edited_ = true;
}

void VariableEditDialog::doneSlot()
{
    loginf;

    accept();
}

}
