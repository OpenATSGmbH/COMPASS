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

#include "radarplotpositioncalculatortaskdialog.h"
#include "radarplotpositioncalculatortask.h"
#include "radarplotpositioncalculatortaskwidget.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

RadarPlotPositionCalculatorTaskDialog::RadarPlotPositionCalculatorTaskDialog(RadarPlotPositionCalculatorTask& task)
    : task_(task)
{
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

    setWindowTitle("Calculate Radar Plot Positions");

    setModal(true);

    setMinimumSize(QSize(300, 200));

    QVBoxLayout* main_layout = new QVBoxLayout();

    RadarPlotPositionCalculatorTaskWidget* widget = new RadarPlotPositionCalculatorTaskWidget(task_);
    main_layout->addWidget(widget);

    main_layout->addStretch();

    QHBoxLayout* button_layout = new QHBoxLayout();

    QPushButton* cancel_button = new QPushButton("Cancel");
    connect(cancel_button,  &QPushButton::clicked, this, &QDialog::reject);
    button_layout->addWidget(cancel_button);

    button_layout->addStretch();

    QPushButton* ok_button_ = new QPushButton("OK");
    connect(ok_button_, &QPushButton::clicked, this, &QDialog::accept);
    button_layout->addWidget(ok_button_);

    ok_button_->setDefault(true);

    ok_button_->setDisabled(!task_.canRun());

    main_layout->addLayout(button_layout);

    setLayout(main_layout);
}



