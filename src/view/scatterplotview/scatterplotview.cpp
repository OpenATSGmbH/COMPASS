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

#include "scatterplotview.h"

#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/metavariable.h"
//#include "scatterplotviewconfigwidget.h"
#include "scatterplotviewdatasource.h"
#include "scatterplotviewdatawidget.h"
#include "scatterplotviewwidget.h"
#include "logger.h"
#include "latexvisitor.h"
#include "viewpointgenerator.h"

#include <QApplication>

using namespace std;
using namespace dbContent;

const std::string ScatterPlotView::ParamUseConnectionLines = "use_connection_lines";

/**
*/
ScatterPlotView::ScatterPlotView(const std::string& class_id, 
                                 const std::string& instance_id,
                                 ViewContainer* w, 
                                 ViewManager& view_manager)
:   VariableView(class_id, instance_id, w, view_manager)
{
    registerParameter(ParamUseConnectionLines, &settings_.use_connection_lines, Settings().use_connection_lines);

    const std::vector<PropertyDataType> valid_types = { PropertyDataType::BOOL,
                                                        PropertyDataType::CHAR,
                                                        PropertyDataType::UCHAR,
                                                        PropertyDataType::INT,
                                                        PropertyDataType::UINT,
                                                        PropertyDataType::LONGINT,
                                                        PropertyDataType::ULONGINT,
                                                        PropertyDataType::FLOAT,
                                                        PropertyDataType::DOUBLE,
                                                        PropertyDataType::TIMESTAMP };

    addVariable("data_var_x", "X", "x", META_OBJECT_NAME, DBContent::meta_var_longitude_.name(), true, true, false, valid_types);
    addVariable("data_var_y", "Y", "y", META_OBJECT_NAME, DBContent::meta_var_latitude_.name() , true, true, false, valid_types);

    // create sub done in init
}

/**
*/
ScatterPlotView::~ScatterPlotView()
{
    loginf;

    if (data_source_)
    {
        delete data_source_;
        data_source_ = nullptr;
    }

    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }

    loginf << "done";
}

/**
*/
bool ScatterPlotView::init_impl()
{
    createSubConfigurables();

    traced_assert(data_source_);

//    connect(data_source_, &ScatterPlotViewDataSource::loadingStartedSignal, widget_->getDataWidget(),
//            &ScatterPlotViewDataWidget::loadingStartedSlot);
//    connect(data_source_, &ScatterPlotViewDataSource::updateDataSignal, widget_->getDataWidget(),
//            &ScatterPlotViewDataWidget::updateDataSlot);

//    connect(widget_->configWidget(), &ScatterPlotViewConfigWidget::exportSignal,
//            widget_->getDataWidget(), &ScatterPlotViewDataWidget::exportDataSlot);
//    connect(widget_->getDataWidget(), &ScatterPlotViewDataWidget::exportDoneSignal,
//            widget_->configWidget(), &ScatterPlotViewConfigWidget::exportDoneSlot);

//    connect(data_source_, &ScatterPlotViewDataSource::loadingStartedSignal, widget_->configWidget(),
//            &ScatterPlotViewConfigWidget::loadingStartedSlot);

    //    connect(this, &ScatterPlotView::showOnlySelectedSignal, widget_->getDataWidget(),
    //            &ScatterPlotViewDataWidget::showOnlySelectedSlot);
    //    connect(this, &ScatterPlotView::usePresentationSignal, widget_->getDataWidget(),
    //            &ScatterPlotViewDataWidget::usePresentationSlot);
    //    connect(this, &ScatterPlotView::showAssociationsSignal, widget_->getDataWidget(),
    //            &ScatterPlotViewDataWidget::showAssociationsSlot);

    //    widget_->getDataWidget()->showOnlySelectedSlot(show_only_selected_);
    //    widget_->getDataWidget()->usePresentationSlot(use_presentation_);
    //    widget_->getDataWidget()->showAssociationsSlot(show_associations_);

    return true;
}

/**
*/
void ScatterPlotView::generateSubConfigurable(const std::string& class_id,
                                            const std::string& instance_id)
{
    logdbg << "class_id " << class_id << " instance_id "
           << instance_id;
    if (class_id == "ScatterPlotViewDataSource")
    {
        traced_assert(!data_source_);
        data_source_ = new ScatterPlotViewDataSource(class_id, instance_id, this);
    }
    else if (class_id == "ScatterPlotViewWidget")
    {
        widget_ = new ScatterPlotViewWidget(class_id, instance_id, this, this, central_widget_);
        setWidget(widget_);
    }
    else
        throw std::runtime_error("ScatterPlotView: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

/**
*/
void ScatterPlotView::checkSubConfigurables()
{
    if (!data_source_)
    {
        generateSubConfigurable("ScatterPlotViewDataSource", "ScatterPlotViewDataSource0");
    }

    if (!widget_)
    {
        generateSubConfigurable("ScatterPlotViewWidget", "ScatterPlotViewWidget0");
    }
}

/**
*/
ScatterPlotViewDataWidget* ScatterPlotView::getDataWidget()
{
    traced_assert(widget_);
    return widget_->getViewDataWidget();
}

/**
*/
VariableSet ScatterPlotView::getBaseSet(const std::string& dbcontent_name)
{
    traced_assert(data_source_);

    VariableSet set = data_source_->getSet()->getFor(dbcontent_name);

    if (dbcontent_name == "CAT063") // add sensor sec/sic special case
    {
        DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

        traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat063_sensor_sac_));
        traced_assert(dbcont_man.canGetVariable(dbcontent_name, DBContent::var_cat063_sensor_sic_));

        set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat063_sensor_sac_));
        set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat063_sensor_sic_));
    }

    return set;
}

/**
*/
void ScatterPlotView::accept(LatexVisitor& v)
{
    v.visit(this);
}

/**
*/
bool ScatterPlotView::useConnectionLines()
{
    return settings_.use_connection_lines;
}

/**
*/
void ScatterPlotView::useConnectionLines(bool value, bool redraw)
{
    setParameter(settings_.use_connection_lines, value);

    if (redraw)
        updateView(VU_Redraw);
}

/**
*/
void ScatterPlotView::updateSelection()
{
    loginf;
    traced_assert(widget_);

    widget_->getViewDataWidget()->redrawData(true);

    //    if (show_only_selected_)
    //        widget_->getDataWidget()->updateToSelection();
    //    else
    //        widget_->getDataWidget()->resetModels();  // just updates the checkboxes
}

/**
*/
void ScatterPlotView::unshowViewPoint(const ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->unshowViewPoint(vp);
}

/**
*/
void ScatterPlotView::showViewPoint(const ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->showViewPoint(vp);
}

/**
 */
void ScatterPlotView::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableView::viewInfoJSON_impl(info);

    info[ ParamUseConnectionLines ] = settings_.use_connection_lines;
}

/**
 */
std::set<std::string> ScatterPlotView::acceptedAnnotationFeatureTypes() const
{
    std::set<std::string> types;
    types.insert(ViewPointGenFeatureScatterSeries::FeatureName);

    return types;
}
