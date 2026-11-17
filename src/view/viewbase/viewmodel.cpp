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

#include "viewmodel.h"
#include "viewasyncprocessor.h"
#include "viewlayertreemodel.h"
#include "viewlayersnapshot.h"
#include "appstate.h"
#include "color_provider.h"
#include "buffer.h"
#include "dbcontentdataset.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"
#include "traced_assert.h"

#include <QScopedValueRollback>

/**
 */
ViewModel::ViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state)
:   dbcont_man_(dbcont_man)
,   ctx_man_   (ctx_man   )
,   app_state_ (app_state )
{
}

/**
 */
ViewModel::~ViewModel()
{
    // async_processor_ joins its outstanding workers in its destructor; their commits
    // (which may reference this model) never run
}

/**
 * The model's asynchronous processor, created on first use. Its completions drive the
 * deferred load-done handling via notifyProcessingFinished().
 */
ViewAsyncProcessor& ViewModel::asyncProcessor()
{
    if (!async_processor_)
    {
        async_processor_.reset(new ViewAsyncProcessor);

        connect(async_processor_.get(), &ViewAsyncProcessor::startedSignal,
                this, &ViewModel::processingStartedSignal);
        connect(async_processor_.get(), &ViewAsyncProcessor::finishedSignal,
                this, &ViewModel::notifyProcessingFinished);
    }

    return *async_processor_;
}

/**
 */
bool ViewModel::hasPendingAsyncWork() const
{
    return async_processor_ && async_processor_->hasPending();
}

/**
 */
void ViewModel::shutdownAsyncProcessor()
{
    if (!async_processor_)
        return;

    async_processor_->invalidate();     // commits of finishing tasks are discarded
    async_processor_->waitForPending(); // block until the work functions returned
}

/**
 */
bool ViewModel::hasPendingProcessing() const
{
    return pending_loading_done_ || pending_data_loaded_ || pending_recompute_ ||
           hasPendingAsyncWork();
}

/**
 * All asynchronous work completed (committed or discarded): run the deferred load-done
 * parts and report the model as finished. Connected to asyncProcessor()'s finishedSignal.
 */
void ViewModel::notifyProcessingFinished()
{
    if (hasPendingAsyncWork())
        return; // more work already outstanding, wait for it

    if (pending_loading_done_)
    {
        pending_loading_done_ = false;

        recompute();

        if (hasPendingAsyncWork())
        {
            // the recompute launched follow-up work; data loaded once that completed
            pending_data_loaded_ = true;
            return;
        }
    }
    else if (pending_recompute_)
    {
        // a recompute was requested while work was outstanding - run the
        // coalesced recompute now; if it launches new work, completion is reported
        // once that work finished
        pending_recompute_ = false;

        recompute();

        if (hasPendingAsyncWork())
            return;

        emit processingFinishedSignal();
        return;
    }
    else if (!pending_data_loaded_)
    {
        // finished outside a deferred load-done cycle (e.g. an interactive rebuild)
        emit processingFinishedSignal();
        return;
    }

    pending_data_loaded_ = false;

    finishDataLoad();

    emit processingFinishedSignal();
}

/**
 * Reacts on loading started.
 */
void ViewModel::loadingStarted()
{
    //a deferred load-done of a superseded cycle is dropped, as is a coalesced
    //recompute - the load computes everything anyway
    pending_loading_done_ = false;
    pending_data_loaded_  = false;
    pending_recompute_    = false;

    clearData();
}

/**
 * Reacts on loading ended. With asynchronous processing outstanding the recompute is
 * deferred: it runs once the pending work completed (it reads the processed results), and
 * dataLoadedSignal is emitted once the recompute's own launched work completed.
 * See notifyProcessingFinished().
 */
void ViewModel::loadingDone()
{
    if (hasPendingAsyncWork())
    {
        loginf << "deferring load done work, view processing pending";
        pending_loading_done_ = true;
        return;
    }

    recompute();

    if (hasPendingAsyncWork())
    {
        //the recompute launched asynchronous work; data loaded once that completed
        pending_data_loaded_ = true;
        return;
    }

    finishDataLoad();
}

/**
 */
void ViewModel::finishDataLoad()
{
    endDataReload();

    //signal that the view data has been loaded
    emit dataLoadedSignal();
}

/**
 * Mirrors the source's buffers and invokes the derived hook, which finalizes on last=true.
 */
void ViewModel::updateFromSource(const DBContentDataSet& source,
                                 const std::vector<std::string>& /*names*/, bool reset, bool last)
{
    logdbg;

    data_                   = source.buffers();
    data_fulfills_read_set_ = source.fulfillsReadSet();

    // set before the derived class rebuilds from the data (reload end and live tick alike)
    setLayerDataComplete(last);

    updateFromSource_impl(reset, last);
}

/**
 */
void ViewModel::clearData()
{
    logdbg;

    // results of in-flight asynchronous work are stale now; deferred load-done state
    // of the outgoing cycle is dropped. Invalidate alone is not enough: the running
    // work function keeps reading the buffers released below and writing the data
    // cleared by clearData_impl(), racing the main thread into heap corruption
    // (double free / freed-Buffer crash on close_db, 2026-08-17)
    // - so block until the work functions returned, as the destructor does, BEFORE
    // releasing anything they touch.
    shutdownAsyncProcessor();
    pending_loading_done_ = false;
    pending_data_loaded_  = false;

    setLayerDataComplete(false);

    // no reload follows => the data rows go (forgotten, see readme F7), before the payloads are released;
    // a reload keeps them stale until its data arrives
    if (layer_model_ && !layer_model_->dataReloading())
        layer_model_->clearDataSubtree();

    data_                   = {};
    data_fulfills_read_set_ = true;

    count_null_.reset();
    count_nan_.reset();

    clearData_impl();
}

/**
 */
void ViewModel::recompute()
{
    //a recompute while asynchronous work is outstanding is coalesced: the running
    //work's inputs are already stale, so remember the request and run one recompute
    //when the work has finished (notifyProcessingFinished) instead of stacking jobs
    if (hasPendingAsyncWork())
    {
        pending_recompute_ = true;
        return;
    }

    recompute_impl();

    //check for nan values in the data
    if (nanCount().has_value() && nanCount().value() > 0)
    {
        logerr << nanCount().value() << " inf value(s) detected in view " << objectName().toStdString();
    }
}

/**
 */
bool ViewModel::hasData() const
{
    return !data_.empty();
}

/**
 */
void ViewModel::resetCounts()
{
    count_null_.reset();
    count_nan_.reset();
}

/**
 */
void ViewModel::addNullCount(size_t n)
{
    if (!count_null_.has_value())
        count_null_ = 0;

    count_null_.value() += n;
}

/**
 */
void ViewModel::addNanCount(size_t n)
{
    if (!count_nan_.has_value())
        count_nan_ = 0;

    count_nan_.value() += n;
}

/**
 */
void ViewModel::invertSelection()
{
    loginf;

    for (auto& buf_it : mutableViewData())
    {
        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
        {
            if (selected_vec.isNull(cnt))
                selected_vec.set(cnt, true);
            else
                selected_vec.set(cnt, !selected_vec.get(cnt));
        }
    }

    emit selectionChangedSignal();
}

/**
 */
void ViewModel::clearSelection()
{
    loginf;

    for (auto& buf_it : mutableViewData())
    {
        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        for (unsigned int cnt=0; cnt < buf_it.second->size(); ++cnt)
            selected_vec.set(cnt, false);
    }

    emit selectionChangedSignal();
}

/**
 */
void ViewModel::beginDataReload()
{
    if (layer_model_)
        layer_model_->beginDataReload();
}

/**
 */
void ViewModel::endDataReload()
{
    if (!layer_model_ || !layer_model_->dataReloading())
        return;

    logwrn << "data rows not updated by the reload, removing them";

    layer_model_->clearDataSubtree();
}

/**
 * Partial data updates are skipped during a reload, see ViewLayerTreeModel::updateDataSubtree().
 */
void ViewModel::setLayerDataComplete(bool complete)
{
    if (layer_model_)
        layer_model_->setDataComplete(complete);
}

/**
 */
void ViewModel::databaseClosed()
{
    //hidden-layer memory is per database session - forget it on close
    if (layer_model_)
        layer_model_->state().clear();
}

/**
 * Notice: the first rebuild runs here, the model may come from a rebuilt config widget.
 */
void ViewModel::attachLayerPanel(ViewLayerTreeModel* layer_model)
{
    traced_assert(layer_model);

    // e.g. a rebuilt config widget attaches again
    for (const auto& connection : layer_connections_)
        disconnect(connection);
    layer_connections_.clear();

    layer_model_ = layer_model;

    auto visibilityChanged = [ this ] ()
    {
        if (!updating_layer_tree_)
            layerVisibilityChanged();
    };

    layer_connections_.push_back(connect(layer_model_, &LayerTreeModel::hiddenChangedSignal, this, visibilityChanged));
    layer_connections_.push_back(connect(&app_state_, &AppState::colorModeChangedSignal,
                                         this, [ this ] (unsigned int) { colorModeChanged(); }));

    // the panel belongs to the config widget, which can go before the model
    layer_connections_.push_back(connect(layer_model_, &QObject::destroyed, this, [ this ] () { layer_model_ = nullptr; }));

    rebuildLayerTree();

    emit layerPanelAttachedSignal();
}

/**
 */
void ViewModel::updateLayerTree(LayerSnapshots layers, bool content_final)
{
    if (layer_model_)
    {
        std::vector<DBContentRootItem::LeafEntry>    entries;
        std::vector<DBContentRootItem::TopLeafEntry> top_leaves;
        entries.reserve(layers.size());

        for (const auto& layer : layers)
        {
            const auto& id = layer->id();

            if (layer->isTopLeaf())
                top_leaves.push_back({ id.dbcontent, layer.get() });
            else
                entries.push_back({ id.ds_type, id.ds_name, id.line, id.dbcontent, layer.get() });
        }

        // hidden signals of the update itself (e.g. configured states of new leaves) => no visibility hook
        QScopedValueRollback<bool> updating(updating_layer_tree_, true);

        layer_model_->updateDataSubtree(entries, top_leaves, content_final);
    }

    // the leaves point at the new snapshots now
    layer_snapshots_ = std::move(layers);
}

/**
 */
void ViewModel::refreshLayerTree()
{
    LayerSnapshots layers = std::move(layer_snapshots_);
    updateLayerTree(std::move(layers));
}

/**
 */
const ViewModel::LayerAggregates& ViewModel::layerAggregates()
{
    // the aggregation only depends on the loaded buffers - reused e.g. when a color mode change
    // restyles the panel, the scan is a full row scan (seconds at millions of rows)
    if (!layer_agg_valid_)
    {
        layer_agg_cache_ = view_layer_scan::aggregateLayers(
            view_layer_scan::makeScanInput(viewData(), dbcont_man_, ctx_man_));
        layer_agg_valid_ = true;
    }

    return layer_agg_cache_;
}

/**
 */
void ViewModel::setLayerAggregates(LayerAggregates aggregates)
{
    layer_agg_cache_ = std::move(aggregates);
    layer_agg_valid_ = true;
}

/**
 */
ViewModel::LayerSnapshots ViewModel::snapshotsFromAggregates(const LayerAggregates& aggregates) const
{
    LayerSnapshots layers;
    layers.reserve(aggregates.size());

    for (const auto& [ key, agg ] : aggregates)
    {
        const QColor color = context::resolveSeriesColor(agg.ds_type, agg.ds_name, agg.line_index, agg.dbcontent, ctx_man_);

        layers.push_back(std::make_unique<ViewLayerSnapshot>(key, agg, agg.count, 0, color));
    }

    return layers;
}

/**
 */
void ViewModel::snapshotHiddenLayerIds()
{
    hidden_layer_ids_ = layer_model_ ? layer_model_->state().hiddenIds() : std::set<std::string>{};
}
