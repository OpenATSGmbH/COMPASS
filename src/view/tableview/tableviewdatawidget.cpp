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

#include "tableviewdatawidget.h"
#include "tableviewwidget.h"
#include "tableview.h"
#include "allbuffertablewidget.h"
#include "compass.h"
//#include "buffer.h"
#include "buffertablewidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
//#include "tableviewdatasource.h"
#include "logger.h"

#include <QHBoxLayout>
#include <QMessageBox>
#include <QTabWidget>
#include <QTableView>
#include <QHeaderView>

TableViewDataWidget::TableViewDataWidget(TableViewWidget* view_widget, 
                                             QWidget* parent, 
                                             Qt::WindowFlags f)
:   ViewDataWidget(view_widget, parent, f)
{
    view_ = view_widget->getView();
    traced_assert(view_);

    data_source_ = view_->getDataSource();
    traced_assert(data_source_);

    QHBoxLayout* layout = new QHBoxLayout();
    layout->setMargin(0);

    tab_widget_ = new QTabWidget();
    layout->addWidget(tab_widget_);

    for (auto& obj_it : COMPASS::instance().dbContentManager())
    {
        if (!all_buffer_table_widget_)
        {
            all_buffer_table_widget_ = new AllBufferTableWidget(*view_, *data_source_);
            tab_widget_->addTab(all_buffer_table_widget_, "All");
            connect(all_buffer_table_widget_, &AllBufferTableWidget::exportDoneSignal, this,
                    &TableViewDataWidget::exportDoneSlot);
        }

        BufferTableWidget* buffer_table =
            new BufferTableWidget(*obj_it.second, *view_, *data_source_);
        tab_widget_->addTab(buffer_table, obj_it.first.c_str());
        buffer_tables_[obj_it.first] = buffer_table;
        connect(buffer_table, &BufferTableWidget::exportDoneSignal, this,
                &TableViewDataWidget::exportDoneSlot);
    }

    setLayout(layout);
}

TableViewDataWidget::~TableViewDataWidget()
{
}

void TableViewDataWidget::clearData_impl()
{
    logdbg << "begin";

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->clear();

    for (auto buffer_table : buffer_tables_)
        buffer_table.second->clear();

    logdbg << "end";
}

void TableViewDataWidget::clearIntermediateRedrawData_impl()
{
    //nothing to do here
}

void TableViewDataWidget::loadingStarted_impl()
{
    loginf;
    //nothing to do yet
}

void TableViewDataWidget::updateData_impl(bool requires_reset)
{
    logdbg << "begin";

    //nothing to do yet

    logdbg << "end";
}

void TableViewDataWidget::loadingDone_impl()
{
    logdbg << "begin";

    //default behavior
    ViewDataWidget::loadingDone_impl();

    for (auto& buf_widget : buffer_tables_)
        showTab(buf_widget.second, buf_widget.second->hasData());

    logdbg << "end";
}

ViewDataWidget::DrawState TableViewDataWidget::redrawData_impl(bool recompute)
{
    logdbg << "start - recompute = " << recompute;

    traced_assert(all_buffer_table_widget_);
    all_buffer_table_widget_->show(viewData());

    for (auto& buf_it : viewData())
    {
        traced_assert(buffer_tables_.count(buf_it.first) > 0);
        buffer_tables_.at(buf_it.first)->show(buf_it.second);
    }

    selectFirstSelectedRow();

    logdbg << "end";

    return (all_buffer_table_widget_->rowCount() > 0 ? DrawState::DrawnContent : DrawState::Drawn);
}

void TableViewDataWidget::liveReload_impl()
{
    //implement live reload behavior here
}

void TableViewDataWidget::exportDataSlot()
{
    logdbg << "start";
    traced_assert(tab_widget_);

    AllBufferTableWidget* all_buffer_widget =
        dynamic_cast<AllBufferTableWidget*>(tab_widget_->currentWidget());

    BufferTableWidget* buffer_widget =
        dynamic_cast<BufferTableWidget*>(tab_widget_->currentWidget());

    if (all_buffer_widget && !buffer_widget)
    {
        all_buffer_widget->exportSlot();
        return;
    }

    if (!all_buffer_widget && !buffer_widget)
    {
        QMessageBox msgBox;
        msgBox.setText("Export can not be used without loaded data.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        exportDoneSignal(true);
        return;
    }

    buffer_widget->exportSlot();
}

void TableViewDataWidget::exportDoneSlot(bool cancelled) 
{ 
    emit exportDoneSignal(cancelled); 
}

void TableViewDataWidget::updateToSettingsChange()
{
    loginf;

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSettingsChange();

    for (auto& buf_wgt : buffer_tables_)
    {
        buf_wgt.second->updateToSettingsChange();
        showTab(buf_wgt.second, buf_wgt.second->hasData());
    }
}

void TableViewDataWidget::showTab(QWidget* widget_ptr, bool value)
{
    if (tab_widget_)
    {
        traced_assert(widget_ptr);
        int index = tab_widget_->indexOf(widget_ptr);
        traced_assert(index >= 0);

#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
        tab_widget_->setTabVisible(index, value);
#else
        tab_widget_->setTabEnabled(index, value); // setTabVisitable only for >=Qt 5.15
        tab_widget_->setStyleSheet("QTabBar::tab::disabled {min-width: 0px;max-width: 0px;color:rgba(0,0,0,0);background-color: rgba(0,0,0,0);}");
#endif
    }
}

void TableViewDataWidget::resetModels()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->resetModel();

    for (auto& table_widget_it : buffer_tables_)
        table_widget_it.second->resetModel();
}

void TableViewDataWidget::updateToSelection()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSelection();

    for (auto& table_widget_it : buffer_tables_)
        table_widget_it.second->updateToSelection();
}

void TableViewDataWidget::selectFirstSelectedRow()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->selectSelectedRows();
}

AllBufferTableWidget* TableViewDataWidget::getAllBufferTableWidget ()
{
    traced_assert(all_buffer_table_widget_);
    return all_buffer_table_widget_;
}

void TableViewDataWidget::toolChanged_impl(int mode)
{
    //nothing to do here
}

void TableViewDataWidget::viewInfoJSON_impl(nlohmann::json& info) const
{
    nlohmann::json table_infos = nlohmann::json::array();

    auto addTable = [ & ] (const std::string& db_content, 
                           const QTableView* table,
                           bool show_only_selected,
                           bool use_presentation, bool ignore_non_target_reports)
    {
        nlohmann::json table_info;

        table_info[ "content"            ] = db_content;
        table_info[ "show_only_selected" ] = show_only_selected;
        table_info[ "use_presentation"   ] = use_presentation;
        table_info[ "ignore_non_target_reports"   ] = ignore_non_target_reports;
        table_info[ "count"              ] = table->model()->rowCount();

        //get properties
        std::vector<std::string> properties;
        for(int i = 0; i < table->model()->columnCount(); i++)
            properties.push_back(table->model()->headerData(i, Qt::Horizontal).toString().toStdString());
        
        table_info[ "properties" ] = properties;

        //get line zero
        std::vector<std::string> line0; 
        if (table->model()->rowCount() > 0)
        {
            for(int i = 0; i < table->model()->columnCount(); i++)
            {
                auto index = table->model()->index(0, i);
                line0.push_back(table->model()->data(index, Qt::DisplayRole).toString().toStdString());
            }
        }

        table_info[ "line0" ] = line0;

        table_infos.push_back(table_info);
    };

    addTable("All", 
             all_buffer_table_widget_->table(), 
             view_->showOnlySelected(),
             view_->usePresentation(), view_->ignoreNonTargetReports());

    for (const auto& it : buffer_tables_)
    {
        addTable(it.first, 
                 it.second->table(), 
                 view_->showOnlySelected(),
                 view_->usePresentation(), view_->ignoreNonTargetReports());
    }

    info[ "tables" ] = table_infos;
}
