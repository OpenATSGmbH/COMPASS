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

#include "radarplotpositioncalculatortaskwidget.h"
//#include "compass.h"
//#include "dbcontent/dbcontentcombobox.h"
//#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/variable/variable.h"
//#include "dbcontent/variable/variableselectionwidget.h"
#include "logger.h"
#include "projectionmanager.h"
#include "projectionmanagerwidget.h"
#include "radarplotpositioncalculatortask.h"
//#include "stringconv.h"
//#include "taskmanager.h"

#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

//using namespace Utils::String;
using namespace dbContent;

RadarPlotPositionCalculatorTaskWidget::RadarPlotPositionCalculatorTaskWidget(
    RadarPlotPositionCalculatorTask& task, QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f), task_(task)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    main_layout->addWidget(ProjectionManager::instance().widget());

    main_layout->addStretch();

    setLayout(main_layout);
}

RadarPlotPositionCalculatorTaskWidget::~RadarPlotPositionCalculatorTaskWidget()
{
    ProjectionManager::instance().deleteWidget();
}

