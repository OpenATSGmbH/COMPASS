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

#include "tableview.h"
#include "viewcontainer.h"

#include <QApplication>

#include "compass.h"
#include "tableviewconfigwidget.h"
#include "tableviewdatasource.h"
#include "tableviewdatawidget.h"
#include "tableviewwidget.h"
#include "logger.h"
#include "latexvisitor.h"

const std::string ParamShowSelected    = "show_only_selected";
const std::string ParamUsePresentation = "use_presentation";
const std::string ParamIgnoreTargetReports = "ignore_non_target_reports";
const std::string SubConfigDataSource  = "TableViewDataSource";
const std::string SubConfigViewWidget  = "TableViewWidget";

/**
*/
TableView::Settings::Settings()
:   show_only_selected_(false)
,   use_presentation_  (true)
{
}

/**
*/
TableView::TableView(nlohmann::json& config, ViewContainer* parent)
:   View(config, parent)
{
    registerParameter(ParamShowSelected, &settings_.show_only_selected_, Settings().show_only_selected_);
    registerParameter(ParamUsePresentation, &settings_.use_presentation_, Settings().use_presentation_);
    registerParameter(ParamIgnoreTargetReports, &settings_.ignore_non_target_reports_,
                      Settings().ignore_non_target_reports_);
}

/**
*/
TableView::~TableView()
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
bool TableView::init_impl()
{
    createSubConfigurables();

    traced_assert(data_source_);

    connect(widget_->getViewConfigWidget(), &TableViewConfigWidget::exportSignal,
            widget_->getViewDataWidget(), &TableViewDataWidget::exportDataSlot);
    connect(widget_->getViewDataWidget(), &TableViewDataWidget::exportDoneSignal,
            widget_->getViewConfigWidget(), &TableViewConfigWidget::exportDoneSlot);

    // connect(this, &TableView::showOnlySelectedSignal,
    //         widget_->getViewDataWidget(), &TableViewDataWidget::showOnlySelectedSlot);
    // connect(this, &TableView::usePresentationSignal,
    //         widget_->getViewDataWidget(), &TableViewDataWidget::usePresentationSlot);

    widget_->getViewDataWidget()->updateToSettingsChange();

    return true;
}

/**
*/
void TableView::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    logdbg << "class_name " << class_name;
    if (class_name == SubConfigDataSource)
    {
        traced_assert(!data_source_);
        data_source_ = new TableViewDataSource(child_json, this);

        //notify view that it needs to reload
        connect(data_source_, &TableViewDataSource::reloadNeeded, [ this ] { notifyViewUpdateNeeded(VU_Reload); });
    }
    else
    {
        throw std::runtime_error("TableView: generateSubConfigurable: unknown class_name " +
                                 class_name);
    }
}

/**
*/
void TableView::checkSubConfigurables()
{
    if (!data_source_)
    {
        generateSubConfigurableFromConfig(SubConfigDataSource, SubConfigDataSource + "0");
    }

    if (!widget_)
    {
        widget_ = new TableViewWidget(this, central_widget_);
        setWidget(widget_);
    }
}

TableViewDataWidget* TableView::getDataWidget()
{
    traced_assert(widget_);
    return widget_->getViewDataWidget();
}

dbContent::VariableSet TableView::getSet(const std::string& dbcontent_name)
{
    traced_assert(data_source_);

    return data_source_->getSet()->getFor(dbcontent_name);
}

bool TableView::usePresentation() const 
{ 
    return settings_.use_presentation_;
}

void TableView::usePresentation(bool value)
{
    loginf << "start" << value;

    setParameter(settings_.use_presentation_, value);

    widget_->getViewDataWidget()->updateToSettingsChange();
}

bool TableView::showOnlySelected() const 
{ 
    return settings_.show_only_selected_;
}

void TableView::showOnlySelected(bool value)
{
    loginf << "start" << value;

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    setParameter(settings_.show_only_selected_, value);

    widget_->getViewDataWidget()->updateToSettingsChange();

    QApplication::restoreOverrideCursor();
}

bool TableView::ignoreNonTargetReports() const
{
    return settings_.ignore_non_target_reports_;
}

void TableView::ignoreNonTargetReports(bool value)
{
    loginf << "start" << value;

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    setParameter(settings_.ignore_non_target_reports_, value);

    widget_->getViewDataWidget()->updateToSettingsChange();

    QApplication::restoreOverrideCursor();
}

void TableView::accept(LatexVisitor& v)
{
    v.visit(this);
}

void TableView::updateSelection()
{
    loginf;
    traced_assert(widget_);

    if (settings_.show_only_selected_)
        widget_->getViewDataWidget()->updateToSelection();
    else
        widget_->getViewDataWidget()->resetModels();  // just updates the checkboxes
}

void TableView::unshowViewPointSlot (ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->unshowViewPoint(vp);
}

void TableView::showViewPointSlot (ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->showViewPoint(vp);
    traced_assert(widget_);
}

void TableView::onConfigurationChanged_impl(const std::vector<std::string>& changed_params)
{
    if (changed_params.size())
        widget_->getViewDataWidget()->updateToSettingsChange();
}

/**
 */
void TableView::viewInfoJSON_impl(nlohmann::json& info) const
{
    info[ "variables"          ] = data_source_->getSet()->definitions();
    info[ ParamShowSelected    ] = settings_.show_only_selected_;
    info[ ParamUsePresentation ] = settings_.use_presentation_;
    info[ ParamIgnoreTargetReports ] = settings_.ignore_non_target_reports_;
}

const TableView::Settings& TableView::settings() const
{
    return settings_;
}
