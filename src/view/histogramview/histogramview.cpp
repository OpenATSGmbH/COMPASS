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

#include "histogramview.h"

#include <QApplication>

#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/metavariable.h"
#include "histogramviewconfigwidget.h"
#include "histogramviewdatasource.h"
#include "histogramviewmodel.h"
#include "histogramviewwidget.h"
#include "viewmanager.h"
#include "logger.h"
#include "latexvisitor.h"
#include "viewpointgenerator.h"

using namespace dbContent;

const std::string HistogramView::ParamUseLogScale = "use_log_scale";

/**
 */
HistogramView::Settings::Settings()
:   use_log_scale(false)
{
}

/**
 */
HistogramView::HistogramView(nlohmann::json& config, ViewContainer* parent)
:   VariableView(config, parent)
{
    registerParameter(ParamUseLogScale, &settings_.use_log_scale, Settings().use_log_scale);

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

    addVariable("data_var", "", "data_var", META_OBJECT_NAME, dbcontent_vars::meta_var_timestamp_.name(), true, true, false, valid_types);

    model_.reset(new HistogramViewModel(viewManager().dbContentManager(),
                                        viewManager().dbContextManager(),
                                        viewManager().appState()));
    model_->setObjectName(QString::fromStdString(instanceName()));
    model_->setVariable(variable(0));

    setModel(model_.get());

    connect(model_.get(), &ViewModel::displayDataChangedSignal, this, &HistogramView::displayDataChanged);

    //note: triggers a view update, otherwise a manual update would be needed
    connect(model_.get(), &ViewModel::selectionChangedSignal, this, &View::emitSelectionChange);

    // create sub done in init
}

/**
 */
HistogramView::~HistogramView()
{
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
}

/**
 */
bool HistogramView::init_impl()
{
    createSubConfigurables();

    traced_assert(data_source_);

    return true;
}

/**
 */
GenerateResult HistogramView::generateSubConfigurable_impl(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    const auto& instance_name = Configuration::getInstanceName(child_json);
    logdbg << "class_name " << class_name << " instance_name "
           << instance_name;
    if (class_name == "HistogramViewDataSource")
    {
        traced_assert(!data_source_);
        data_source_ = new HistogramViewDataSource(child_json, this);
    }
    else
        return GenerateResult::Unknown;

    return GenerateResult::Generated;
}

/**
 */
void HistogramView::checkSubConfigurables()
{
    if (!data_source_)
    {
        createSubConfigurableFromConfig("HistogramViewDataSource", "HistogramViewDataSource0");
    }

    if (!widget_)
    {
        widget_ = new HistogramViewWidget(this, central_widget_);
        setWidget(widget_);
    }
}

/**
 */
HistogramRenderOptions HistogramView::renderOptions() const
{
    HistogramRenderOptions options;
    options.use_log_scale = settings_.use_log_scale;

    return options;
}

/**
 */
void HistogramView::displayDataChanged()
{
    auto data = model_->displayData();

    if (!data || !data->histogram.useLogScale().has_value())
        return;

    useLogScale(data->histogram.useLogScale().value(), false);
    updateComponents();
}

/**
 */
VariableSet HistogramView::getBaseSet(const std::string& dbcontent_name)
{
    traced_assert(data_source_);
    return data_source_->getSet()->getFor(dbcontent_name);
}

/**
 */
void HistogramView::accept(LatexVisitor& v)
{
    v.visit(this);
}

/**
 */
std::set<std::string> HistogramView::acceptedAnnotationFeatureTypes() const
{
    std::set<std::string> types;
    types.insert(ViewPointGenFeatureHistogram::FeatureName);

    return types;
}

/**
 */
bool HistogramView::useLogScale() const
{
    return settings_.use_log_scale;
}

/**
 */
void HistogramView::useLogScale(bool use_log_scale, bool notify_changes)
{
    setParameter(settings_.use_log_scale, use_log_scale);

    if (notify_changes)
    {
        updateView(VU_Redraw);
    }
}

/**
 */
void HistogramView::updateSelection_impl()
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
ViewInfos HistogramView::viewInfos_impl() const
{
    return model_->viewInfo();
}

/**
 */
void HistogramView::viewInfoJSON_impl(nlohmann::json& info) const
{
    //!call base!
    VariableView::viewInfoJSON_impl(info);

    info[ "use_log_scale"] = settings_.use_log_scale;
}

