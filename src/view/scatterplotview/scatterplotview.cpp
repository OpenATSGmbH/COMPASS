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
#include "viewmanager.h"
#include "viewcontainer.h"

#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/metavariable.h"
//#include "scatterplotviewconfigwidget.h"
#include "scatterplotviewdatasource.h"
#include "scatterplotviewmodel.h"
#include "scatterplotviewwidget.h"
#include "logger.h"
#include "latexvisitor.h"
#include "viewpointgenerator.h"

#include <QApplication>
#include <QColor>

using namespace std;
using namespace dbContent;

const std::string ScatterPlotView::ParamUseConnectionLines = "use_connection_lines";

const int ScatterPlotView::ConnectLinesDataCountMax = 100000;

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

    addVariable("data_var_x", "X", "x", META_OBJECT_NAME, dbcontent_vars::meta_var_longitude_.name(), true, true, false, valid_types);
    addVariable("data_var_y", "Y", "y", META_OBJECT_NAME, dbcontent_vars::meta_var_latitude_.name() , true, true, false, valid_types);

    model_.reset(new ScatterPlotViewModel(viewManager().dbContentManager(),
                                          viewManager().dbContextManager(),
                                          viewManager().appState()));
    model_->setObjectName(QString::fromStdString(instanceName()));
    model_->setVariables(variable(0), variable(1));
    model_->setReloadNeededFunc([ this ] () { return reloadNeeded(); });

    setModel(model_.get());

    connect(model_.get(), &ViewModel::displayDataChangedSignal, this, &ScatterPlotView::displayDataChanged);

    //note: triggers a view update, otherwise a manual update would be needed
    connect(model_.get(), &ViewModel::selectionChangedSignal, this, &View::emitSelectionChange);

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

    return true;
}

/**
*/
GenerateResult ScatterPlotView::generateSubConfigurable_impl(nlohmann::json& child_json)
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
        return GenerateResult::Unknown;

    return GenerateResult::Generated;
}

/**
*/
void ScatterPlotView::checkSubConfigurables()
{
    if (!data_source_)
    {
        createSubConfigurableFromConfig("ScatterPlotViewDataSource", "ScatterPlotViewDataSource0");
    }

    if (!widget_)
    {
        widget_ = new ScatterPlotViewWidget(this, central_widget_);
        setWidget(widget_);
    }
}

/**
*/
ScatterPlotRenderOptions ScatterPlotView::renderOptions() const
{
    ScatterPlotRenderOptions options;
    options.use_connection_lines = settings_.use_connection_lines;

    return options;
}

/**
*/
void ScatterPlotView::displayDataChanged()
{
    auto data = model_->displayData();

    if (!data || !data->series.useConnectionLines().has_value())
        return;

    useConnectionLines(data->series.useConnectionLines().value(), false);
    updateComponents();
}

/**
 * Before the model computes the display data of the load.
*/
void ScatterPlotView::loadingDone()
{
    // disable connection lines?
    if (useConnectionLines() && model_->loadedDataCount() > (unsigned int)ConnectLinesDataCountMax)
    {
        loginf << "loaded data items >" << ConnectLinesDataCountMax << ", disabling connection lines";

        //no redraw, happens later anyway
        useConnectionLines(false, false);
    }

    VariableView::loadingDone();
}

/**
*/
VariableSet ScatterPlotView::getBaseSet(const std::string& dbcontent_name)
{
    traced_assert(data_source_);

    VariableSet set = data_source_->getSet()->getFor(dbcontent_name);

    if (dbcontent_name == "CAT063") // add sensor sec/sic special case
    {
        DBContentManager& dbcont_man = viewManager().dbContentManager();

        traced_assert(dbcont_man.canGetVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sac_));
        traced_assert(dbcont_man.canGetVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sic_));

        set.add(dbcont_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sac_));
        set.add(dbcont_man.getVariable(dbcontent_name, dbcontent_vars::var_cat063_sensor_sic_));
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
void ScatterPlotView::updateSelection_impl()
{
    loginf;
    traced_assert(widget_);

    widget_->redrawData(true);

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

