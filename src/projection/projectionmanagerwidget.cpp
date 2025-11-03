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

#include "projectionmanagerwidget.h"
#include "logger.h"
#include "projection.h"
#include "projectionmanager.h"
//#include "stringconv.h"

#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QVBoxLayout>
#include "traced_assert.h"



ProjectionManagerWidget::ProjectionManagerWidget(ProjectionManager& proj_man, QWidget* parent,
                                                 Qt::WindowFlags f)
    : QWidget(parent, f), projection_manager_(proj_man)
{
    setContentsMargins(0, 0, 0, 0);

    QVBoxLayout* main_layout = new QVBoxLayout();
    main_layout->setContentsMargins(0, 0, 0, 0);

    QGridLayout* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);

    grid->addWidget(new QLabel("Projection Method"), 0, 0);

    projection_box_ = new QComboBox();

    for (auto& proj_it : projection_manager_.projections())
        projection_box_->addItem(proj_it.first.c_str());

    if (projection_manager_.hasCurrentProjection())
        projection_box_->setCurrentText(projection_manager_.currentProjectionName().c_str());

    connect(projection_box_, SIGNAL(currentIndexChanged(const QString&)), this,
            SLOT(selectedObjectParserSlot(const QString&)));

    grid->addWidget(projection_box_, 0, 1);

    main_layout->addLayout(grid);

    setLayout(main_layout);
}

ProjectionManagerWidget::~ProjectionManagerWidget()
{
    logdbg;
}

void ProjectionManagerWidget::selectedObjectParserSlot(const QString& name)
{
    loginf << "name " << name.toStdString();

    traced_assert(projection_manager_.hasProjection(name.toStdString()));
    projection_manager_.currentProjectionName(name.toStdString());
}
