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


#include "simplereconstructorwidget.h"

#include "simplereconstructor.h"
#include "reconstructormainwidget.h"
#include "simplereconstructorassociationwidget.h"
#include "referencecalculatorwidget.h"
#include "datasourcesusewidget.h"
#include "reconstructorsectorwidget.h"
#include "reconstructortask.h"
#include "reconstructortaskanalysiswidget.h"
#include "reconstructortaskclassificationwidget.h"
#include "compass.h"

#include <QCheckBox>
#include <QTabWidget>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

using namespace std;

SimpleReconstructorWidget::SimpleReconstructorWidget(SimpleReconstructor& reconstructor)
    : reconstructor_(reconstructor)
{
    QVBoxLayout* main_layout = new QVBoxLayout();

    QTabWidget* tab_widget = new QTabWidget();

    // main widget
    main_widget_.reset(new ReconstructorMainWidget(reconstructor, tab_widget));
    tab_widget->addTab(main_widget_.get(), "Main");

    // use ds widget

    std::function<bool(const std::string&)> get_use_dstype_func =
        [this] (const std::string& ds_type) { return reconstructor_.task().useDStype(ds_type); };
    std::function<void(const std::string&,bool)> set_use_dstype_func =
        [this] (const std::string& ds_type, bool value) { return reconstructor_.task().useDSType(ds_type,value); };
    std::function<bool(unsigned int)> get_use_ds_func =
        [this] (unsigned int ds_id) { return reconstructor_.task().useDataSource(ds_id); };
    std::function<void(unsigned int,bool)> set_use_ds_func =
        [this] (unsigned int ds_id, bool value) { return reconstructor_.task().useDataSource(ds_id, value); };
    std::function<bool(unsigned int,unsigned int)> get_use_ds_line_func =
        [this] (unsigned int ds_id, unsigned int line_id)
    { return reconstructor_.task().useDataSourceLine(ds_id, line_id); };
    std::function<void(unsigned int,unsigned int, bool)> set_use_ds_line_func =
        [this] (unsigned int ds_id, unsigned int line_id, bool value)
    { return reconstructor_.task().useDataSourceLine(ds_id, line_id, value); };

    use_widget_.reset(new DataSourcesUseWidget(
        get_use_dstype_func, set_use_dstype_func,
        get_use_ds_func, set_use_ds_func,
        get_use_ds_line_func, set_use_ds_line_func));
    use_widget_->disableDataSources(reconstructor_.task().disabledDataSources());

    tab_widget->addTab(use_widget_.get(), "Data Sources");

    sectors_widget_.reset(new ReconstructorSectorWidget(reconstructor_.task()));
    tab_widget->addTab(sectors_widget_.get(), "Sectors");

    assoc_widget_.reset(new SimpleReconstructorAssociationWidget(reconstructor_, *this));
    tab_widget->addTab(assoc_widget_.get(), "Association");

    classif_widget_.reset(new ReconstructorTaskClassificationWidget(reconstructor_));

    tab_widget->addTab(classif_widget_.get(), "Classification");

    calc_widget_.reset(new ReferenceCalculatorWidget(reconstructor_));
    tab_widget->addTab(calc_widget_.get(), "Reference Calculation");

    debug_widget_.reset(new ReconstructorTaskAnalysisWidget(reconstructor_.task(), false));

    if (!COMPASS::isAppImage() || COMPASS::instance().expertMode())
        tab_widget->addTab(debug_widget_.get(), "Analysis");

    update();

    setContentsMargins(0, 0, 0, 0);

    main_layout->addWidget(tab_widget);

    setLayout(main_layout);
}

SimpleReconstructorWidget::~SimpleReconstructorWidget()
{
    use_widget_ = nullptr;
    assoc_widget_ = nullptr;
    calc_widget_ = nullptr;
    debug_widget_ = nullptr;
}

void SimpleReconstructorWidget::update()
{
    traced_assert(use_widget_);
    use_widget_->updateContent();
    use_widget_->disableDataSources(reconstructor_.task().disabledDataSources());

    traced_assert(sectors_widget_);
    sectors_widget_->update();

    traced_assert(assoc_widget_);
    assoc_widget_->updateValues();

    traced_assert(calc_widget_);
    calc_widget_->update();

    if (debug_widget_)
        debug_widget_->updateValues();
}


void SimpleReconstructorWidget::updateSlot()
{
    update();
}
