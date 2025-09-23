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

#include "reconstructortaskclassificationwidget.h"
#include "reconstructortask.h"
#include "logger.h"

#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QCheckBox>
#include <QSpinBox>

using namespace std;
using namespace Utils;

ReconstructorTaskClassificationWidget::ReconstructorTaskClassificationWidget(ReconstructorBase& reconstructor, QWidget *parent)
    : QWidget{parent}, reconstructor_(reconstructor)
{
    QFormLayout* layout = new QFormLayout;

    min_aircraft_modec_edit_ = new QSpinBox();
    min_aircraft_modec_edit_->setRange(-10000, 10000);
    connect(min_aircraft_modec_edit_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ReconstructorTaskClassificationWidget::minAircraftModeCEditedSlot);

    layout->addRow(new QLabel("Minimum Aircraft Altitude [ft]"), min_aircraft_modec_edit_);

    vehicle_acids_edit_ = new QTextEdit();
    connect(vehicle_acids_edit_, &QTextEdit::textChanged, this,
            &ReconstructorTaskClassificationWidget::vehicleACIDsChangedSlot);

    layout->addRow(new QLabel("Vehicle ACIDs"), vehicle_acids_edit_);

    vehicle_acads_edit_ = new QTextEdit();
    connect(vehicle_acads_edit_, &QTextEdit::textChanged, this,
            &ReconstructorTaskClassificationWidget::vehicleACADsChangedSlot);

    layout->addRow(new QLabel("Vehicle ACADs"), vehicle_acads_edit_);

    setLayout(layout);

    updateValues();
}


void ReconstructorTaskClassificationWidget::updateValues()
{
    min_aircraft_modec_edit_->setValue(reconstructor_.settings().min_aircraft_modec_);
    vehicle_acids_edit_->setText(reconstructor_.settings().vehicle_acids_.c_str());
    vehicle_acads_edit_->setText(reconstructor_.settings().vehicle_acads_.c_str());
}

void ReconstructorTaskClassificationWidget::minAircraftModeCEditedSlot (int value)
{
    loginf << "value '" << value << "'";

    reconstructor_.settings().min_aircraft_modec_ = value;
}

void ReconstructorTaskClassificationWidget::vehicleACIDsChangedSlot()
{
    traced_assert(vehicle_acids_edit_);
    reconstructor_.settings().setVehicleACIDs(vehicle_acids_edit_->document()->toPlainText().toStdString());
}
void ReconstructorTaskClassificationWidget::vehicleACADsChangedSlot()
{
    traced_assert(vehicle_acads_edit_);
    reconstructor_.settings().setVehicleACADs(vehicle_acads_edit_->document()->toPlainText().toStdString());
}
