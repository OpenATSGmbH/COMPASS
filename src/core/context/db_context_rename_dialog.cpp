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

#include "db_context_rename_dialog.h"
#include "db_context_manager.h"
#include "logger.h"

#include <QLabel>
#include <QLineEdit>
#include <QIcon>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

namespace context
{

DBContextRenameDialog::DBContextRenameDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    setWindowTitle("Rename Data Context");
    setMinimumWidth(400);
    setModal(true);

    auto* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("Rename context '" +
        QString::fromStdString(manager_.activeContextName()) + "':"));
    layout->addSpacing(10);

    auto* name_layout = new QHBoxLayout();
    name_layout->addWidget(new QLabel("New name:"));
    name_edit_ = new QLineEdit();
    name_edit_->setText(QString::fromStdString(manager_.activeContextName()));
    name_edit_->selectAll();
    name_layout->addWidget(name_edit_);
    layout->addLayout(name_layout);

    layout->addSpacing(10);

    auto* button_layout = new QHBoxLayout();

    auto* cancel_btn = new QPushButton("Cancel");
    cancel_btn->setIcon(QIcon());
    connect(cancel_btn, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_btn);

    button_layout->addStretch();

    rename_button_ = new QPushButton("Rename");
    rename_button_->setIcon(QIcon());
    rename_button_->setEnabled(false);
    rename_button_->setDefault(true);
    connect(rename_button_, &QPushButton::clicked, this, &DBContextRenameDialog::renameSlot);
    button_layout->addWidget(rename_button_);

    layout->addLayout(button_layout);

    setLayout(layout);

    connect(name_edit_, &QLineEdit::textChanged, this, &DBContextRenameDialog::updateRenameButton);
    connect(name_edit_, &QLineEdit::returnPressed, this, &DBContextRenameDialog::renameSlot);

    updateRenameButton();
}

void DBContextRenameDialog::updateRenameButton()
{
    std::string name = name_edit_->text().trimmed().toStdString();

    if (name.empty())
    {
        rename_button_->setEnabled(false);
        rename_button_->setToolTip("Enter a new name for the context");
        return;
    }

    if (name == manager_.activeContextName())
    {
        rename_button_->setEnabled(false);
        rename_button_->setToolTip("Name is the same as the current name");
        return;
    }

    if (manager_.hasContext(name))
    {
        rename_button_->setEnabled(false);
        rename_button_->setToolTip("A context with this name already exists");
        return;
    }

    rename_button_->setEnabled(true);
    rename_button_->setToolTip("");
}

void DBContextRenameDialog::renameSlot()
{
    if (!rename_button_->isEnabled())
        return;

    std::string old_name = manager_.activeContextName();
    std::string new_name = name_edit_->text().trimmed().toStdString();

    try
    {
        manager_.renameContext(old_name, new_name);
    }
    catch (const std::exception& e)
    {
        logerr << "renameContext failed: " << e.what();
        QMessageBox::critical(this, "Rename Context Failed", "Could not rename the context.");
        return;
    }

    loginf << "renamed context '" << old_name << "' to '" << new_name << "'";

    accept();
}

} // namespace context
