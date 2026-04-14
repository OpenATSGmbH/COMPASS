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

#include "db_context_select_dialog.h"
#include "db_context_manager.h"
#include "logger.h"

#include <QLabel>
#include <QListWidget>
#include <QIcon>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMessageBox>

namespace context
{

DBContextSelectDialog::DBContextSelectDialog(DBContextManager& manager, QWidget* parent)
    : QDialog(parent)
    , manager_(manager)
{
    setWindowTitle("Select Data Context");
    setMinimumWidth(400);
    setMinimumHeight(300);
    setModal(true);

    auto* layout = new QVBoxLayout();

    layout->addWidget(new QLabel("Please select a data context to continue:"));
    layout->addSpacing(5);

    list_widget_ = new QListWidget();
    for (const auto& name : manager_.contextNames())
        list_widget_->addItem(QString::fromStdString(name));

    if (list_widget_->count() > 0)
        list_widget_->setCurrentRow(0);

    connect(list_widget_, &QListWidget::itemDoubleClicked,
            this, &DBContextSelectDialog::itemDoubleClickedSlot);

    layout->addWidget(list_widget_);

    auto* button_layout = new QHBoxLayout();

    auto* cancel_button = new QPushButton("Cancel");
    cancel_button->setIcon(QIcon());
    connect(cancel_button, &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button);

    button_layout->addStretch();

    auto* select_button = new QPushButton("Select");
    select_button->setIcon(QIcon());
    select_button->setDefault(true);
    connect(select_button, &QPushButton::clicked, this, &DBContextSelectDialog::selectSlot);
    button_layout->addWidget(select_button);

    layout->addLayout(button_layout);

    setLayout(layout);
}

void DBContextSelectDialog::selectSlot()
{
    auto* item = list_widget_->currentItem();
    if (!item)
    {
        QMessageBox msg(this);
        msg.setWindowTitle("Error");
        msg.setText("Please select a context.");
        auto* ok_btn = msg.addButton("OK", QMessageBox::AcceptRole);
        ok_btn->setIcon(QIcon());
        msg.exec();
        return;
    }

    selected_name_ = item->text().toStdString();
    manager_.setActiveContext(selected_name_);

    loginf << "selected context '" << selected_name_ << "'";

    accept();
}

void DBContextSelectDialog::itemDoubleClickedSlot()
{
    selectSlot();
}

} // namespace context
