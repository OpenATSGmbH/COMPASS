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
#include "allbuffertablemodel.h"
#include "compass.h"
#include "buffer.h"
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "color_provider.h"
#include "dbcontentlayer.h"
#include "layertreemodel.h"
#include "tableleafpayload.h"
#include "logger.h"
#include "number.h"
#include "stringconv.h"
#include "viewasyncprocessor.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>
#include <QHBoxLayout>
#include <QMessageBox>
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

    all_buffer_table_widget_ = new AllBufferTableWidget(*view_, *data_source_);
    layout->addWidget(all_buffer_table_widget_);
    connect(all_buffer_table_widget_, &AllBufferTableWidget::exportDoneSignal, this,
            &TableViewDataWidget::exportDoneSlot);

    setLayout(layout);
}

TableViewDataWidget::~TableViewDataWidget() = default;

void TableViewDataWidget::attachLayerPanel(DBContentRootItem* root, LayerTreeModel* layer_model)
{
    db_content_root_ = root;
    layer_model_     = layer_model;

    // Single fire at end of any visibility change (group toggles cascade
    // through multiple leaves but emit hiddenChangedSignal once).
    connect(layer_model_, &LayerTreeModel::hiddenChangedSignal,
            this, [this]() {
                pushLayerStateToModel();
                redrawData(true);
            });

    // Color-mode change (from the data source widget) → resolveSeriesColor
    // outputs differ → rebuild payloads + icons. AllBufferTableModel::
    // setLayerColors emits dataChanged so the icon column refreshes without
    // a full redraw.
    connect(&view_->compass(), &COMPASS::colorModeChangedSignal,
            this, [this](unsigned int) { rebuildLayerTree(); });

    rebuildLayerTree();
}

void TableViewDataWidget::clearData_impl()
{
    logdbg << "begin";

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->clear();

    layer_agg_valid_ = false;

    logdbg << "end";
}

void TableViewDataWidget::clearIntermediateRedrawData_impl()
{
    //nothing to do here
}

void TableViewDataWidget::loadingStarted_impl()
{
    loginf;

    //new data incoming - the cached layer aggregation goes stale; the async prepare
    //commit refills it
    layer_agg_valid_ = false;
}

void TableViewDataWidget::updateFromSource_impl(const DBContentDataSet& /*source*/,
                                                const std::vector<std::string>& /*names*/, bool /*reset*/, bool /*last*/)
{
    // nothing to do - the table model reads viewData() (source-fed) at redraw
}

// Offline the heavy per-redraw work - the layer scan, the row index build and the
// sort - runs on a worker; commitLoadedData applies the results on completion, and
// the base defers dataLoaded / viewRefreshed until then. Used by both the load-done
// path and interactive recompute redraws (style, color mode, filter changes). Live
// stays synchronous: the feed mutates the same buffers every tick. Returns false
// when the synchronous path should run instead.
bool TableViewDataWidget::launchAsyncPrepare()
{
    unsigned int num_records = 0;
    for (auto& buf_it : viewData())
        num_records += buf_it.second->size();

    bool async = view_->compass().appMode() != AppMode::LiveRunning && num_records > 0;

    if (!async)
        return false;

    traced_assert(all_buffer_table_widget_);

    auto* model = all_buffer_table_widget_->allBufferTableModel();
    traced_assert(model);

    auto scan_input = std::make_shared<view_layer_scan::ScanInput>(
        view_layer_scan::makeScanInput(viewData(), view_->compass()));
    auto prep_input = std::make_shared<AllBufferTableModel::PrepareInput>(
        model->makePrepareInput(viewData(), /*for_load*/ true));

    auto agg      = std::make_shared<std::map<std::string, view_layer_scan::LayerAgg>>();
    auto prepared = std::make_shared<AllBufferTableModel::PreparedData>();

    asyncProcessor().launch(
        "table view row data with " + std::to_string(num_records) + " records",
        [ scan_input, prep_input, agg, prepared ] ()
        {
            *agg      = view_layer_scan::aggregateLayers(*scan_input);
            *prepared = AllBufferTableModel::prepareData(*prep_input);
        },
        [ this, agg, prepared ] ()
        {
            commitLoadedData(*agg, std::move(*prepared));
        });

    return true;
}

void TableViewDataWidget::loadingDone_impl()
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : viewData())
        num_records += buf_it.second->size();

    loginf << "begin with " << num_records << " records";

    if (launchAsyncPrepare())
        return;

    // Rebuild the layer tree first so the panel reflects current data; the
    // base redraw below will then consult the allowed-layer-ids set we push
    // into the model from rebuildLayerTree.
    rebuildLayerTree();

    //default behavior
    ViewDataWidget::loadingDone_impl();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

/**
 * Main-thread commit of the asynchronously prepared load data.
 */
void TableViewDataWidget::commitLoadedData(const std::map<std::string, view_layer_scan::LayerAgg>& agg,
                                           AllBufferTableModel::PreparedData&& prepared)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    traced_assert(all_buffer_table_widget_);

    setUpdatesEnabled(false);

    //keep the worker-computed aggregation: color mode changes restyle the panel from
    //it without another full row scan (see rebuildLayerTree)
    layer_agg_cache_ = agg;
    layer_agg_valid_ = true;

    applyLayerTree(layer_agg_cache_);

    auto selected_ranges = std::move(prepared.selected_ranges);

    all_buffer_table_widget_->showPrepared(std::move(prepared));

    if (anyLayerHidden())
    {
        // the prepared row data is unfiltered; re-filter against the freshly pushed
        // allowed set (rare - only when the stored panel state hides layers)
        loginf << "hidden layers stored, re-filtering row data";

        all_buffer_table_widget_->allBufferTableModel()->rebuild();
        all_buffer_table_widget_->selectSelectedRows();
    }
    else
    {
        all_buffer_table_widget_->selectSelectedRows(selected_ranges);
    }

    setUpdatesEnabled(true);

    setDrawState(all_buffer_table_widget_->rowCount() > 0 ? DrawState::DrawnContent
                                                          : DrawState::Drawn);
    emit displayChanged();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << all_buffer_table_widget_->rowCount() << " rows in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

ViewDataWidget::DrawState TableViewDataWidget::redrawData_impl(bool recompute)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = 0;
    for (auto& buf_it : viewData())
        num_records += buf_it.second->size();

    loginf << "start - recompute " << recompute << " records " << num_records;

    traced_assert(all_buffer_table_widget_);

    //offline, the row data prepare runs on a worker and commitLoadedData applies it;
    //the view keeps showing the previous rows until the commit, and viewRefreshed is
    //held back by the pending processing guard
    if (launchAsyncPrepare())
        return (all_buffer_table_widget_->rowCount() > 0 ? DrawState::DrawnContent : DrawState::Drawn);

    setUpdatesEnabled(false);

    all_buffer_table_widget_->show(viewData());

    selectFirstSelectedRow();

    setUpdatesEnabled(true);

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "done with " << num_records << " records in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);

    return (all_buffer_table_widget_->rowCount() > 0 ? DrawState::DrawnContent : DrawState::Drawn);
}

void TableViewDataWidget::liveReload_impl()
{
    //implement live reload behavior here
}

void TableViewDataWidget::exportDataSlot()
{
    logdbg;

    if (!all_buffer_table_widget_ || all_buffer_table_widget_->rowCount() == 0)
    {
        QMessageBox msgBox(this);
        msgBox.setText("Export can not be used without loaded data.");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();

        emit exportDoneSignal(true);
        return;
    }

    all_buffer_table_widget_->exportSlot();
}

void TableViewDataWidget::exportDoneSlot(bool canceled)
{
    emit exportDoneSignal(canceled);
}

void TableViewDataWidget::updateToSettingsChange()
{
    loginf;

    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSettingsChange();
}

void TableViewDataWidget::resetModels()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->resetModel();
}

void TableViewDataWidget::updateToSelection()
{
    if (all_buffer_table_widget_)
        all_buffer_table_widget_->updateToSelection();
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

    info[ "tables" ] = table_infos;
}

void TableViewDataWidget::rebuildLayerTree()
{
    if (!db_content_root_ || !layer_model_)
        return;

    // the aggregation only depends on the loaded buffers - reuse it when the data is
    // unchanged: a color mode change restyles the panel from the same aggregates, and
    // the recompute is a full row scan (measured as a seconds-long main thread stall)
    if (layer_agg_valid_)
    {
        applyLayerTree(layer_agg_cache_);
        return;
    }

    // Count records per (ds_type, ds_name, line, dbcontent) layer by scanning
    // ds_id + line_id columns on each loaded buffer (shared scan helper; on the
    // asynchronous load path the scan runs on a worker instead, see loadingDone_impl).
    layer_agg_cache_ = view_layer_scan::aggregateLayers(
        view_layer_scan::makeScanInput(viewData(), view_->compass()));
    layer_agg_valid_ = true;

    applyLayerTree(layer_agg_cache_);
}

/**
 * The tree/payload part of rebuildLayerTree, from precomputed layer aggregates.
 */
void TableViewDataWidget::applyLayerTree(const std::map<std::string, view_layer_scan::LayerAgg>& agg)
{
    if (!db_content_root_ || !layer_model_)
        return;

    auto& compass = view_->compass();

    // Build payloads + LeafEntry list.
    std::vector<std::unique_ptr<TableLeafPayload>> new_payloads;
    std::vector<DBContentRootItem::LeafEntry>      entries;
    new_payloads.reserve(agg.size());
    entries.reserve(agg.size());

    for (const auto& kv : agg)
    {
        const std::string&              full_key = kv.first;
        const view_layer_scan::LayerAgg& a       = kv.second;

        QColor color = context::resolveSeriesColor(
            a.ds_type, a.ds_name, a.line_index, a.dbcontent, compass);

        new_payloads.emplace_back(std::make_unique<TableLeafPayload>(
            full_key, a.count, color));

        entries.push_back({a.ds_type, a.ds_name, a.line, a.dbcontent,
                           new_payloads.back().get()});
    }

    // Scoped subtree refresh - keeps the header widths untouched. Also
    // re-applies the stored hidden state to the fresh leaves.
    layer_model_->refreshSubtree(db_content_root_, [&]() {
        payloads_ = std::move(new_payloads);
        return db_content_root_->buildChildrenFrom(entries);
    });
    db_content_root_->recomputeColorsRecursive();

    // Push initial allowed set to the model so the first redraw filters correctly.
    pushLayerStateToModel();

    emit layerTreeRebuiltSignal();
}

/**
 */
bool TableViewDataWidget::anyLayerHidden() const
{
    for (const auto& payload : payloads_)
        if (payload && !payload->visible())
            return true;

    return false;
}

void TableViewDataWidget::pushLayerStateToModel()
{
    if (!all_buffer_table_widget_)
        return;

    auto* model = all_buffer_table_widget_->allBufferTableModel();
    if (!model)
        return;

    if (payloads_.empty())
    {
        model->setAllowedLayerIds(std::nullopt);
        model->setLayerColors({});
        return;
    }

    std::set<std::string> allowed;
    std::map<std::string, QColor> layer_colors;

    auto& dbcont_man = view_->compass().dbContentManager();
    for (const auto& payload : payloads_)
    {
        if (payload->visible())
            allowed.insert(payload->persistenceId());

        // Only target-report DBContents get a colored icon. Matches scatter's
        // leafColorFor rule: non-TR DBContents render as blank space.
        const std::string dbcontent = [&]() {
            // persistenceId is "<ds_type>:<ds_name>:L<n>:<dbcontent>"
            const std::string& k = payload->persistenceId();
            auto last = k.rfind(':');
            return last == std::string::npos ? std::string{} : k.substr(last + 1);
        }();

        QColor color;
        if (dbcont_man.existsDBContent(dbcontent) &&
            dbcont_man.dbContent(dbcontent).containsTargetReports())
            color = payload->color();

        layer_colors.emplace(payload->persistenceId(), color);
    }

    model->setAllowedLayerIds(std::move(allowed));
    model->setLayerColors(std::move(layer_colors));
}
