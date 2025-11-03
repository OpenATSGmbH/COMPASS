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
TableView::TableView(const std::string& class_id, 
                         const std::string& instance_id,
                         ViewContainer* w, 
                         ViewManager& view_manager)
:   View(class_id, instance_id, w, view_manager)
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
void TableView::generateSubConfigurable(const std::string& class_id,
                                          const std::string& instance_id)
{
    logdbg << "class_id " << class_id << " instance_id "
           << instance_id;
    if (class_id == SubConfigDataSource)
    {
        traced_assert(!data_source_);
        data_source_ = new TableViewDataSource(class_id, instance_id, this);

        //notify view that it needs to reload
        connect(data_source_, &TableViewDataSource::reloadNeeded, [ this ] { notifyViewUpdateNeeded(VU_Reload); });
    }
    else if (class_id == SubConfigViewWidget)
    {
        widget_ = new TableViewWidget(class_id, instance_id, this, this, central_widget_);
        setWidget(widget_);
    }
    else
    {
        throw std::runtime_error("TableView: generateSubConfigurable: unknown class_id " +
                                 class_id);
    }
}

/**
*/
void TableView::checkSubConfigurables()
{
    if (!data_source_)
    {
        generateSubConfigurable(SubConfigDataSource, SubConfigDataSource + "0");
    }

    if (!widget_)
    {
        generateSubConfigurable(SubConfigViewWidget, SubConfigViewWidget + "0");
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

void TableView::unshowViewPointSlot (const ViewableDataConfig* vp)
{
    loginf;

    traced_assert(vp);
    traced_assert(data_source_);
    data_source_->unshowViewPoint(vp);
}

void TableView::showViewPointSlot (const ViewableDataConfig* vp)
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
