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

#include "db_context_copy_dialog.h"
#include "db_context_manager.h"
#include "logger.h"

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QIcon>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

namespace context
{

DBContextCopyDialog::DBContextCopyDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    setWindowTitle("Copy Data Context");
    setMinimumWidth(400);
    setModal(true);

    auto* layout = new QVBoxLayout();

    // source context
    auto* source_layout = new QHBoxLayout();
    source_layout->addWidget(new QLabel("Source context:"));
    source_combo_ = new QComboBox();
    for (const auto& name : manager_.contextNames())
        source_combo_->addItem(QString::fromStdString(name));

    // select current active context by default
    if (manager_.hasActiveContext())
    {
        int idx = source_combo_->findText(QString::fromStdString(manager_.activeContextName()));
        if (idx >= 0)
            source_combo_->setCurrentIndex(idx);
    }
    source_layout->addWidget(source_combo_);
    layout->addLayout(source_layout);

    layout->addSpacing(5);

    // new name
    auto* name_layout = new QHBoxLayout();
    name_layout->addWidget(new QLabel("New name:"));
    name_edit_ = new QLineEdit();
    connect(name_edit_, &QLineEdit::textChanged, this, &DBContextCopyDialog::updateCopyButton);
    name_layout->addWidget(name_edit_);
    layout->addLayout(name_layout);

    layout->addSpacing(10);

    // buttons
    auto* button_layout = new QHBoxLayout();

    auto* cancel_btn = new QPushButton("Cancel");
    cancel_btn->setIcon(QIcon());
    connect(cancel_btn, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_btn);

    button_layout->addStretch();

    copy_button_ = new QPushButton("Copy");
    copy_button_->setIcon(QIcon());
    copy_button_->setEnabled(false);
    connect(copy_button_, &QPushButton::clicked, this, &DBContextCopyDialog::copySlot);
    button_layout->addWidget(copy_button_);

    layout->addLayout(button_layout);

    setLayout(layout);

    connect(name_edit_, &QLineEdit::returnPressed, this, &DBContextCopyDialog::copySlot);
}

void DBContextCopyDialog::updateCopyButton()
{
    std::string name = name_edit_->text().trimmed().toStdString();

    if (name.empty())
    {
        copy_button_->setEnabled(false);
        copy_button_->setToolTip("Enter a name for the new context");
        return;
    }

    if (manager_.hasContext(name))
    {
        copy_button_->setEnabled(false);
        copy_button_->setToolTip("A context with this name already exists");
        return;
    }

    copy_button_->setEnabled(true);
    copy_button_->setToolTip("");
}

void DBContextCopyDialog::copySlot()
{
    if (!copy_button_->isEnabled())
        return;

    std::string source = source_combo_->currentText().toStdString();
    std::string dest = name_edit_->text().trimmed().toStdString();

    try
    {
        manager_.duplicateContext(source, dest);
    }
    catch (const std::exception& e)
    {
        logerr << "duplicateContext failed: " << e.what();
        QMessageBox::critical(this, "Copy Context Failed", "Could not copy the context.");
        return;
    }

    loginf << "copied context '" << source << "' as '" << dest << "'";

    accept();
}

} // namespace context
