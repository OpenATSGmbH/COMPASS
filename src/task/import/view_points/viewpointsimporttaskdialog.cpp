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

#include "viewpointsimporttaskdialog.h"
#include "viewpointsimporttaskwidget.h"
#include "viewpointsimporttask.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

ViewPointsImportTaskDialog::ViewPointsImportTaskDialog(ViewPointsImportTask& task)
    : QDialog(), task_(task)
{
    setWindowTitle("Import View Points");
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setModal(true);

    setMinimumSize(QSize(400, 400));

    QFont font_bold;
    font_bold.setBold(true);

    QFont font_big;
    font_big.setPointSize(16);

    QVBoxLayout* main_layout = new QVBoxLayout();

    task_widget_ = new ViewPointsImportTaskWidget(task_, this);
    main_layout->addWidget(task_widget_);

    QHBoxLayout* button_layout = new QHBoxLayout();

    cancel_button_ = new QPushButton("Cancel");
    connect(cancel_button_, &QPushButton::clicked, this, &ViewPointsImportTaskDialog::cancelClickedSlot);
    button_layout->addWidget(cancel_button_);


    button_layout->addStretch();

    import_button_ = new QPushButton("Import");
    connect(import_button_, &QPushButton::clicked, this, &ViewPointsImportTaskDialog::importClickedSlot);
    button_layout->addWidget(import_button_);

    main_layout->addLayout(button_layout);

    setLayout(main_layout);

    update();
}

void ViewPointsImportTaskDialog::updateText ()
{
    traced_assert(task_widget_);
    task_widget_->updateText();
}

void ViewPointsImportTaskDialog::updateButtons()
{
    traced_assert(import_button_);

    import_button_->setDisabled(!task_.canRun());
}


void ViewPointsImportTaskDialog::importClickedSlot()
{
    emit importSignal();
}

void ViewPointsImportTaskDialog::cancelClickedSlot()
{
    emit cancelSignal();
}
