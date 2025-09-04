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

#include "dbcontent/variable/metavariabledetailwidget.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variableselectionwidget.h"
#include "compass.h"
#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QFormLayout>
#include <QPushButton>

using namespace std;

namespace dbContent
{

MetaVariableDetailWidget::MetaVariableDetailWidget(DBContentManager& dbcont_man, QWidget *parent)
    : QWidget(parent), dbcont_man_(dbcont_man)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QFormLayout* form_layout = new QFormLayout();

    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::editingFinished,
            this, &MetaVariableDetailWidget::nameEditedSlot);
    name_edit_->setDisabled(true);
    form_layout->addRow("Name", name_edit_);

    description_edit_ = new QTextEdit();
    description_edit_->setReadOnly(true);
    description_edit_->setWordWrapMode(QTextOption::WrapMode::WrapAnywhere);

    form_layout->addRow("Comment", description_edit_);

    for (auto dbcont_it = dbcont_man_.begin(); dbcont_it != dbcont_man_.end(); ++dbcont_it)
    {
        VariableSelectionWidget* var_sel = new VariableSelectionWidget(true);
        var_sel->showDBContentOnly(dbcont_it->first);
        var_sel->showExistingInDBOnly(false);
        
        //var_sel->setProperty("DBContent", dbcont_it->first.c_str());

        connect(var_sel, &VariableSelectionWidget::selectionChanged,
                this, &MetaVariableDetailWidget::variableChangedSlot);

        var_sel->setDisabled(true);

        selection_widgets_[dbcont_it->first] = var_sel;
        form_layout->addRow(dbcont_it->first.c_str(), var_sel);
    }

    main_layout->addLayout(form_layout);


    delete_button_ = new QPushButton("Delete");
    connect(delete_button_, &QPushButton::clicked,
            this, &MetaVariableDetailWidget::deleteVariableSlot);
    delete_button_->setDisabled(true);

    main_layout->addWidget(delete_button_);

    setLayout(main_layout);
}


void MetaVariableDetailWidget::show (MetaVariable& meta_var)
{
    bool expert_mode = COMPASS::instance().expertMode();

    loginf << "var '" << meta_var.name() << "' expert_mode " << expert_mode;

    has_current_entry_ = true;
    meta_var_ = &meta_var;

    name_edit_->setText(meta_var.name().c_str());
    name_edit_->setDisabled(false);
    name_edit_->setReadOnly(!expert_mode);

    description_edit_->document()->setPlainText(meta_var.description().c_str());

    for (auto& sel_it : selection_widgets_)
    {
        logdbg << "var '" << meta_var.name() << "' exists in " << sel_it.first
               << " " << meta_var.existsIn(sel_it.first);

        if (meta_var.existsIn(sel_it.first))
            sel_it.second->selectedVariable(meta_var.getFor(sel_it.first));
        else
            sel_it.second->selectEmptyVariable();

        sel_it.second->setDisabled(false);
        sel_it.second->setReadOnly(!expert_mode);

    }

    delete_button_->setDisabled(!expert_mode);
}

void MetaVariableDetailWidget::clear()
{
    has_current_entry_ = false;
    meta_var_ = nullptr;

    name_edit_->setText("");
    name_edit_->setDisabled(true);

    description_edit_->document()->setPlainText("");

    for (auto& sel_it : selection_widgets_)
    {
        sel_it.second->selectEmptyVariable();
        sel_it.second->setDisabled(true);
    }

    delete_button_->setDisabled(true);
}

void MetaVariableDetailWidget::nameEditedSlot()
{
    if (!has_current_entry_)
        return;

    traced_assert(name_edit_);

    string new_name = name_edit_->text().toStdString();

    loginf << "name '" << new_name << "'";

    traced_assert(has_current_entry_);
    traced_assert(meta_var_);

    dbcont_man_.renameMetaVariable(meta_var_->name(), new_name);
}

void MetaVariableDetailWidget::variableChangedSlot()
{
    loginf;

    if (!has_current_entry_)
        return;

    VariableSelectionWidget* sel_widget = dynamic_cast<VariableSelectionWidget*>(QObject::sender());
    traced_assert(sel_widget);
    traced_assert(sel_widget->hasVariable());

    traced_assert(meta_var_);
    meta_var_->set(sel_widget->selectedVariable());
}

void MetaVariableDetailWidget::deleteVariableSlot()
{
    loginf;

    traced_assert(has_current_entry_);
    traced_assert(meta_var_);

    dbcont_man_.deleteMetaVariable(meta_var_->name());
}

}
