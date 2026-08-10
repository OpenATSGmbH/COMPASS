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

#include "db_context_create_dialog.h"
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

DBContextCreateDialog::DBContextCreateDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    setWindowTitle("Create Data Context");
    setObjectName("ctx_create_dialog");
    setMinimumWidth(400);
    setModal(true);

    auto* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("Create a new data context:"));
    layout->addSpacing(10);

    auto* name_layout = new QHBoxLayout();
    name_layout->addWidget(new QLabel("Context name:"));
    name_edit_ = new QLineEdit();
    name_edit_->setObjectName("ctx_create_name_edit");
    name_edit_->setPlaceholderText("e.g. Test_2026");
    name_layout->addWidget(name_edit_);
    layout->addLayout(name_layout);

    layout->addSpacing(10);

    auto* button_layout = new QHBoxLayout();

    auto* cancel_button = new QPushButton("Cancel");
    cancel_button->setObjectName("ctx_create_cancel_button");
    cancel_button->setIcon(QIcon());
    connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button);

    button_layout->addStretch();

    create_button_ = new QPushButton("Create");
    create_button_->setObjectName("ctx_create_apply_button");
    create_button_->setIcon(QIcon());
    create_button_->setEnabled(false);
    create_button_->setDefault(true);
    connect(create_button_, &QPushButton::clicked, this, &DBContextCreateDialog::createSlot);
    button_layout->addWidget(create_button_);

    layout->addLayout(button_layout);

    setLayout(layout);

    connect(name_edit_, &QLineEdit::textChanged, this, &DBContextCreateDialog::updateCreateButton);
    connect(name_edit_, &QLineEdit::returnPressed, this, &DBContextCreateDialog::createSlot);
}

void DBContextCreateDialog::updateCreateButton()
{
    std::string name = name_edit_->text().trimmed().toStdString();

    if (name.empty())
    {
        create_button_->setEnabled(false);
        create_button_->setToolTip("Enter a name for the new context");
        return;
    }

    if (manager_.hasContext(name))
    {
        create_button_->setEnabled(false);
        create_button_->setToolTip("A context with this name already exists");
        return;
    }

    create_button_->setEnabled(true);
    create_button_->setToolTip("");
}

void DBContextCreateDialog::createSlot()
{
    if (!create_button_->isEnabled())
        return;

    std::string name = name_edit_->text().trimmed().toStdString();

    try
    {
        manager_.createContext(name);
    }
    catch (const std::exception& e)
    {
        logerr << "Creating context failed: " << e.what();
        QMessageBox::critical(this, "Error", "Creating new context failed.");
        reject();
    }

    manager_.setActiveContext(name);
    created_name_ = name;

    loginf << "created and activated context '" << name << "'";

    accept();
}

} // namespace context
