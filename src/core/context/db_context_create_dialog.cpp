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
    setMinimumWidth(400);
    setModal(true);

    auto* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("No data context exists. Please create one to continue."));
    layout->addSpacing(10);

    auto* name_layout = new QHBoxLayout();
    name_layout->addWidget(new QLabel("Context name:"));
    name_edit_ = new QLineEdit();
    name_edit_->setPlaceholderText("e.g. Test_2026");
    name_layout->addWidget(name_edit_);
    layout->addLayout(name_layout);

    layout->addSpacing(10);

    auto* button_layout = new QHBoxLayout();
    button_layout->addStretch();

    auto* create_button = new QPushButton("Create");
    create_button->setDefault(true);
    connect(create_button, &QPushButton::clicked, this, &DBContextCreateDialog::createSlot);
    button_layout->addWidget(create_button);

    layout->addLayout(button_layout);

    setLayout(layout);

    connect(name_edit_, &QLineEdit::returnPressed, this, &DBContextCreateDialog::createSlot);
}

void DBContextCreateDialog::createSlot()
{
    std::string name = name_edit_->text().trimmed().toStdString();

    if (name.empty())
    {
        QMessageBox::warning(this, "Error", "Please enter a context name.");
        return;
    }

    if (manager_.hasContext(name))
    {
        QMessageBox::warning(this, "Error",
            "Context '" + QString::fromStdString(name) + "' already exists.");
        return;
    }

    manager_.createContext(name);
    manager_.setActiveContext(name);
    created_name_ = name;

    loginf << "created and activated context '" << name << "'";

    accept();
}

} // namespace context
