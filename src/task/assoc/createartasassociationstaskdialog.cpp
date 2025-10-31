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

    CreateARTASAssociationsTaskWidget* task_widget_ = new CreateARTASAssociationsTaskWidget(task_, this);
    main_layout->addWidget(task_widget_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel_button = new QPushButton("Cancel");
    connect(cancel_button,  &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button);

    button_layout->addStretch();

    QPushButton* run_button = new QPushButton("Run");
    connect(run_button, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(run_button);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    traced_assert(run_button);

    auto err = task_.checkError();

    QString tt = "";
    if (err == CreateARTASAssociationsTask::Error::NoDataSource)
        tt = "Chosen data source invalid or empty";
    else if (err == CreateARTASAssociationsTask::Error::NoDataForLineID)
        tt = "Chosen line empty";

    run_button->setDefault(true);

    run_button->setEnabled(err == CreateARTASAssociationsTask::Error::NoError);
    run_button->setToolTip(tt);

}

