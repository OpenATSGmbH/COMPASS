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

#include "db_context_delete_dialog.h"
#include "db_context_manager.h"
#include "logger.h"
#include "traced_assert.h"

#include <QCheckBox>
#include <QLabel>
#include <QIcon>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

namespace context
{

DBContextDeleteDialog::DBContextDeleteDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    setWindowTitle("Delete Data Contexts");
    setObjectName("ctx_delete_dialog");
    setMinimumWidth(350);
    setModal(true);

    auto* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("Select contexts to delete:"));
    layout->addSpacing(5);

    // checkboxes for each context
    for (const auto& name : manager_.contextNames())
    {
        auto* cb = new QCheckBox(QString::fromStdString(name));
        cb->setObjectName("ctx_delete_check_" + QString::fromStdString(name));
        cb->setChecked(false);
        connect(cb, &QCheckBox::toggled, this, &DBContextDeleteDialog::updateDeleteButton);
        checkboxes_[name] = cb;
        layout->addWidget(cb);
    }

    layout->addSpacing(5);

    // Select All / Select None
    auto* sel_layout = new QHBoxLayout();

    auto* select_all_btn = new QPushButton("Select All");
    select_all_btn->setObjectName("ctx_delete_select_all_button");
    select_all_btn->setIcon(QIcon());
    connect(select_all_btn, &QPushButton::clicked, this, &DBContextDeleteDialog::selectAllSlot);
    sel_layout->addWidget(select_all_btn);

    auto* select_none_btn = new QPushButton("Select None");
    select_none_btn->setObjectName("ctx_delete_select_none_button");
    select_none_btn->setIcon(QIcon());
    connect(select_none_btn, &QPushButton::clicked, this, &DBContextDeleteDialog::selectNoneSlot);
    sel_layout->addWidget(select_none_btn);

    sel_layout->addStretch();
    layout->addLayout(sel_layout);

    layout->addSpacing(10);

    // Cancel / Delete
    auto* button_layout = new QHBoxLayout();

    auto* cancel_btn = new QPushButton("Cancel");
    cancel_btn->setObjectName("ctx_delete_cancel_button");
    cancel_btn->setIcon(QIcon());
    connect(cancel_btn, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_btn);

    button_layout->addStretch();

    delete_button_ = new QPushButton("Delete");
    delete_button_->setObjectName("ctx_delete_apply_button");
    delete_button_->setIcon(QIcon());
    delete_button_->setEnabled(false);
    delete_button_->setToolTip("Select at least one context to delete");
    connect(delete_button_, &QPushButton::clicked, this, &DBContextDeleteDialog::deleteSlot);
    button_layout->addWidget(delete_button_);

    layout->addLayout(button_layout);

    setLayout(layout);
}

void DBContextDeleteDialog::selectAllSlot()
{
    for (auto& [name, cb] : checkboxes_)
        cb->setChecked(true);
}

void DBContextDeleteDialog::selectNoneSlot()
{
    for (auto& [name, cb] : checkboxes_)
        cb->setChecked(false);
}

void DBContextDeleteDialog::updateDeleteButton()
{
    unsigned int selected = 0;
    for (const auto& [name, cb] : checkboxes_)
        if (cb->isChecked()) ++selected;

    unsigned int total = checkboxes_.size();

    // enable only if at least one selected AND at least one will remain
    if (selected == 0)
    {
        delete_button_->setEnabled(false);
        delete_button_->setToolTip("Select at least one context to delete");
    }
    else if (selected >= total)
    {
        delete_button_->setEnabled(false);
        delete_button_->setToolTip("At least one context must remain");
    }
    else
    {
        delete_button_->setEnabled(true);
        delete_button_->setToolTip("");
    }
}

void DBContextDeleteDialog::deleteSlot()
{
    // collect names to delete
    std::vector<std::string> to_delete;
    for (const auto& [name, cb] : checkboxes_)
        if (cb->isChecked()) to_delete.push_back(name);

    if (to_delete.empty())
        return;

    // confirmation
    QString msg = "Delete " + QString::number(to_delete.size()) + " context(s)?\n\n";
    for (const auto& name : to_delete)
        msg += "  - " + QString::fromStdString(name) + "\n";

    QMessageBox confirm(this);
    confirm.setObjectName("ctx_delete_confirm_dialog");
    confirm.setWindowTitle("Confirm Delete");
    confirm.setText(msg);
    auto* yes_btn = confirm.addButton("Yes", QMessageBox::YesRole);
    auto* no_btn = confirm.addButton("No", QMessageBox::NoRole);
    yes_btn->setObjectName("ctx_delete_confirm_yes_button");
    no_btn->setObjectName("ctx_delete_confirm_no_button");
    yes_btn->setIcon(QIcon());
    no_btn->setIcon(QIcon());
    confirm.exec();
    if (confirm.clickedButton() != yes_btn)
        return;

    // check if active context is being deleted
    bool active_deleted = false;
    for (const auto& name : to_delete)
    {
        if (name == manager_.activeContextName())
            active_deleted = true;
    }

    // delete
    bool errors = false;
    for (const auto& name : to_delete)
    {
        loginf << "deleting context '" << name << "'";
        try
        {
            manager_.deleteContext(name);
        }
        catch (const std::exception& e)
        {
            logerr << "Deleting context " << name << " failed: " << e.what();
            errors = true;
        }
    }

    if (errors)
    {
        QMessageBox::critical(this, "Error",
                                    "One or more contexts could not be deleted");
        reject();
    }

    if (active_deleted && !manager_.contextNames().empty())
        manager_.setActiveContext(manager_.contextNames().front());

    accept();
}

} // namespace context
