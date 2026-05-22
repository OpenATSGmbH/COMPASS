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

#include "dbcontent/variable/variabledetailwidget.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variabledatatypecombobox.h"
#include "stringrepresentationcombobox.h"
#include "unitselectionwidget.h"
#include "logger.h"

#include <QFormLayout>
#include <QLineEdit>
#include <QTextEdit>
#include <QVBoxLayout>

#include <boost/algorithm/string.hpp>

#include <algorithm>

using namespace std;

namespace dbContent
{

// Sentinels to keep widgets bound to something valid before the first show().
namespace
{
    PropertyDataType g_dummy_type {PropertyDataType::STRING};
    std::string g_dummy_type_str {"STRING"};
    std::string g_dummy_dimension {};
    std::string g_dummy_unit {};
    Variable::Representation g_dummy_representation {Variable::Representation::STANDARD};
    std::string g_dummy_representation_str {"STANDARD"};
}

VariableDetailWidget::VariableDetailWidget(DBContentManager& dbcont_man, QWidget* parent)
    : QWidget(parent), dbcont_man_(dbcont_man)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout();
    form_layout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::textChanged, this, &VariableDetailWidget::nameChangedSlot);
    form_layout->addRow("Name", name_edit_);

    short_name_edit_ = new QLineEdit();
    connect(short_name_edit_, &QLineEdit::textChanged, this, &VariableDetailWidget::shortNameChangedSlot);
    form_layout->addRow("Short Name", short_name_edit_);

    description_edit_ = new QTextEdit();
    description_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);
    connect(description_edit_, &QTextEdit::textChanged, this, &VariableDetailWidget::descriptionChangedSlot);
    form_layout->addRow("Comment", description_edit_);

    source_edit_ = new QTextEdit();
    source_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);
    connect(source_edit_, &QTextEdit::textChanged, this, &VariableDetailWidget::sourceChangedSlot);
    form_layout->addRow("Source", source_edit_);

    type_combo_ = new VariableDataTypeComboBox(g_dummy_type, g_dummy_type_str);
    form_layout->addRow("Data Type", type_combo_);

    unit_sel_ = new UnitSelectionWidget(dbcont_man_.compass().unitManager(),
                                        g_dummy_dimension, g_dummy_unit);
    form_layout->addRow("Unit", unit_sel_);

    representation_box_ = new StringRepresentationComboBox(g_dummy_representation, g_dummy_representation_str);
    form_layout->addRow("Representation", representation_box_);

    db_column_edit_ = new QLineEdit();
    connect(db_column_edit_, &QLineEdit::textChanged, this, &VariableDetailWidget::dbColumnChangedSlot);
    form_layout->addRow("DBColumn", db_column_edit_);

    main_layout->addLayout(form_layout);
    main_layout->addStretch();

    setLayout(main_layout);

    clear();
}

void VariableDetailWidget::show(Variable& variable)
{
    bool expert_mode = dbcont_man_.compass().expertMode();

    has_current_entry_ = true;
    variable_ = &variable;

    {
        QSignalBlocker block(name_edit_);
        name_edit_->setText(variable.name().c_str());
    }
    name_edit_->setReadOnly(!expert_mode);
    name_edit_->setEnabled(true);

    {
        QSignalBlocker block(short_name_edit_);
        short_name_edit_->setText(variable.shortName().c_str());
    }
    short_name_edit_->setReadOnly(!expert_mode);
    short_name_edit_->setEnabled(true);

    {
        QSignalBlocker block(description_edit_);
        description_edit_->document()->setPlainText(variable.description().c_str());
    }
    description_edit_->setReadOnly(!expert_mode);
    description_edit_->setEnabled(true);

    {
        QSignalBlocker block(source_edit_);
        source_edit_->document()->setPlainText(variable.source().c_str());
    }
    source_edit_->setReadOnly(!expert_mode);
    source_edit_->setEnabled(true);

    {
        QSignalBlocker block(db_column_edit_);
        db_column_edit_->setText(variable.dbColumnName().c_str());
    }
    db_column_edit_->setReadOnly(!expert_mode);
    db_column_edit_->setEnabled(true);

    type_combo_->setType(variable.dataTypeRef(), variable.dataTypeStringRef());
    type_combo_->setEnabled(expert_mode);

    unit_sel_->update(variable.dimension(), variable.unit());
    unit_sel_->setEnabled(expert_mode);

    representation_box_->setRepresentation(variable.representationRef(),
                                           variable.representationStringRef());
    representation_box_->setEnabled(expert_mode);
}

void VariableDetailWidget::clear()
{
    has_current_entry_ = false;
    variable_ = nullptr;

    {
        QSignalBlocker block(name_edit_);
        name_edit_->clear();
    }
    name_edit_->setEnabled(false);

    {
        QSignalBlocker block(short_name_edit_);
        short_name_edit_->clear();
    }
    short_name_edit_->setEnabled(false);

    {
        QSignalBlocker block(description_edit_);
        description_edit_->clear();
    }
    description_edit_->setEnabled(false);

    {
        QSignalBlocker block(source_edit_);
        source_edit_->clear();
    }
    source_edit_->setEnabled(false);

    {
        QSignalBlocker block(db_column_edit_);
        db_column_edit_->clear();
    }
    db_column_edit_->setEnabled(false);

    type_combo_->setEnabled(false);
    unit_sel_->setEnabled(false);
    representation_box_->setEnabled(false);
}

void VariableDetailWidget::nameChangedSlot(const QString& name)
{
    if (!has_current_entry_)
        return;

    string new_name = name.trimmed().toStdString();

    if (new_name == variable_->name())
        return;

    if (!new_name.size())
    {
        name_edit_->setStyleSheet(dbcont_man_.compass().lineEditInvalidStyle());
        return;
    }

    if (variable_->object().hasVariable(new_name))
    {
        name_edit_->setStyleSheet(dbcont_man_.compass().lineEditInvalidStyle());
        name_edit_->setToolTip(("Variable name '" + new_name + "' already in use").c_str());
        return;
    }

    name_edit_->setStyleSheet("");
    name_edit_->setToolTip("");

    variable_->object().renameVariable(string(variable_->name()), new_name);

    string db_column_name = name.toStdString();
    std::replace_if(db_column_name.begin(), db_column_name.end(),
                    [](char ch) { return !(isalnum(ch) || ch == '_'); }, '_');
    boost::algorithm::to_lower(db_column_name);

    db_column_edit_->setText(db_column_name.c_str());
    dbColumnChangedSlot(db_column_name.c_str());
}

void VariableDetailWidget::shortNameChangedSlot(const QString& name)
{
    if (!has_current_entry_)
        return;

    variable_->shortName(name.trimmed().toStdString());
}

void VariableDetailWidget::descriptionChangedSlot()
{
    if (!has_current_entry_)
        return;

    variable_->description(description_edit_->document()->toPlainText().trimmed().toStdString());
}

void VariableDetailWidget::sourceChangedSlot()
{
    if (!has_current_entry_)
        return;

    variable_->source(source_edit_->document()->toPlainText().trimmed().toStdString());
}

void VariableDetailWidget::dbColumnChangedSlot(const QString& name)
{
    if (!has_current_entry_)
        return;

    string new_name = name.trimmed().toStdString();

    if (new_name == variable_->dbColumnName())
        return;

    if (!new_name.size())
    {
        db_column_edit_->setStyleSheet(dbcont_man_.compass().lineEditInvalidStyle());
        return;
    }

    if (variable_->object().hasVariableDBColumnName(new_name))
    {
        db_column_edit_->setStyleSheet(dbcont_man_.compass().lineEditInvalidStyle());
        db_column_edit_->setToolTip(("Variable DB Column name '" + new_name + "' already in use").c_str());
        return;
    }

    db_column_edit_->setStyleSheet("");
    db_column_edit_->setToolTip("");

    variable_->dbColumnName(new_name);
}

}
