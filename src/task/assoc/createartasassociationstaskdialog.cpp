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

#include "createartasassociationstaskdialog.h"
#include "createartasassociationstask.h"
#include "createartasassociationstaskwidget.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

CreateARTASAssociationsTaskDialog::CreateARTASAssociationsTaskDialog(CreateARTASAssociationsTask& task)
: QDialog(), task_(task)
{
    setWindowTitle("Calculate ARTAS Target Report Usage");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(800, 600));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    task_widget_ = new CreateARTASAssociationsTaskWidget(task_, this);
    main_layout->addWidget(task_widget_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &CreateARTASAssociationsTaskDialog::cancelClickedSlot);
    button_layout->addWidget(cancel_button_);

    button_layout->addStretch();

    run_button_ = new QPushButton("Run");
    connect(run_button_, &QPushButton::clicked, this, &CreateARTASAssociationsTaskDialog::runClickedSlot);
    button_layout->addWidget(run_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    update();
    updateButtons();

    connect(&task_, &CreateARTASAssociationsTask::dataSourceChanged, this, &CreateARTASAssociationsTaskDialog::updateButtons);
}

void CreateARTASAssociationsTaskDialog::updateButtons()
{
    traced_assert(run_button_);

    auto err = task_.checkError();

    QString tt = "";
    if (err == CreateARTASAssociationsTask::Error::NoDataSource)
        tt = "Chosen data source invalid or empty";
    else if (err == CreateARTASAssociationsTask::Error::NoDataForLineID)
        tt = "Chosen line empty";

    run_button_->setEnabled(err == CreateARTASAssociationsTask::Error::NoError);
    run_button_->setToolTip(tt);
}

void CreateARTASAssociationsTaskDialog::runClickedSlot()
{
    emit runSignal();
}

void CreateARTASAssociationsTaskDialog::cancelClickedSlot()
{
    emit cancelSignal();
}
