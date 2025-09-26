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

#include <memory>

class SimpleReconstructor;
class ReconstructorMainWidget;
class DataSourcesUseWidget;
class ReconstructorSectorWidget;
class SimpleReconstructorAssociationWidget;
class ReconstructorTaskClassificationWidget;
class ReferenceCalculatorWidget;
class ReconstructorTaskAnalysisWidget;

class QLineEdit;
class QCheckBox;

class SimpleReconstructorWidget : public QWidget
{
    Q_OBJECT

  public slots:
    void updateSlot(); // called if something was changed in reconstructor

  public:
    SimpleReconstructorWidget(SimpleReconstructor& reconstructor);
    virtual ~SimpleReconstructorWidget();

    void update();

  private:
    SimpleReconstructor& reconstructor_;

    std::unique_ptr<ReconstructorMainWidget> main_widget_;
    std::unique_ptr<DataSourcesUseWidget> use_widget_;
    std::unique_ptr<ReconstructorSectorWidget> sectors_widget_;
    std::unique_ptr<SimpleReconstructorAssociationWidget> assoc_widget_;
    std::unique_ptr<ReconstructorTaskClassificationWidget> classif_widget_;
    std::unique_ptr<ReferenceCalculatorWidget> calc_widget_;
    std::unique_ptr<ReconstructorTaskAnalysisWidget> debug_widget_;
};

