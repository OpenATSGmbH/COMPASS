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



#pragma once

#include <QWidget>

class ReconstructorBase;

class QTextEdit;
class QSpinBox;

class ReconstructorTaskClassificationWidget : public QWidget
{
    Q_OBJECT

  signals:

  public slots:

    void minAircraftModeCEditedSlot (int value);

    void vehicleACIDsChangedSlot();
    void vehicleACADsChangedSlot();

  public:
    ReconstructorTaskClassificationWidget(ReconstructorBase& reconstructor, QWidget *parent = nullptr);
    virtual ~ReconstructorTaskClassificationWidget() {}

    void updateValues();

  protected:
    ReconstructorBase& reconstructor_;

    QSpinBox* min_aircraft_modec_edit_{nullptr};

    QTextEdit* vehicle_acids_edit_ {nullptr};
    QTextEdit* vehicle_acads_edit_ {nullptr};
};

