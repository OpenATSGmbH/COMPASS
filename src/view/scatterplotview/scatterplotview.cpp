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
#include "viewcontainer.h"

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
// ScatterPlotView::ScatterPlotView(const std::string& class_name,
//                                  const std::string& instance_name,
//                                  ViewContainer* w,
//                                  ViewManager& view_manager)
// :   VariableView(class_name, instance_name, w, view_manager)

ScatterPlotView::ScatterPlotView(nlohmann::json& config, ViewContainer* parent)
:   VariableView(config, parent)
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
void ScatterPlotView::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    const auto& instance_name = Configuration::getInstanceName(child_json);
    logdbg << "class_name " << class_name << " instance_name "
           << instance_name;
    if (class_name == "ScatterPlotViewDataSource")
    {
        traced_assert(!data_source_);
        data_source_ = new ScatterPlotViewDataSource(child_json, this);
    }
    else
        throw std::runtime_error("ScatterPlotView: generateSubConfigurable: unknown class_name " +
                                 class_name);
}

/**
*/
void ScatterPlotView::checkSubConfigurables()
{
    if (!data_source_)
    {
        generateSubConfigurableFromConfig("ScatterPlotViewDataSource", "ScatterPlotViewDataSource0");
    }

    if (!widget_)
    {
        widget_ = new ScatterPlotViewWidget(this, central_widget_);
        setWidget(widget_);
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
        DBContentManager& dbcont_man = compass().dbContentManager();

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
void ScatterPlotView::unshowViewPoint(ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->unshowViewPoint(vp);
}

/**
*/
void ScatterPlotView::showViewPoint(ViewableDataConfig* vp)
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
