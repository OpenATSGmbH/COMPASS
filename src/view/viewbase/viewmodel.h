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

#pragma once

#include "viewlayerscan.h"
#include "json_fwd.hpp"

#include <QObject>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/optional.hpp>

class Buffer;
class DBContentDataSet;
class DBContentManager;
class AppState;
class ViewLayerTreeModel;
class ViewLayerSnapshot;
class ViewAsyncProcessor;
struct PlotMetadata;

namespace context
{
    class DBContextManager;
}

/**
 * Base class of the view models. A model is not a widget. It holds the buffers of a view and builds
 * the display data of its render widget from them, or from an annotation.
 *
 * Inputs:
 *  - Buffers (updateFromSource()), processed in recompute(), which can run asynchronously.
 *  - An annotation (setAnnotation()), no processing.
 * Output: the display data, see the view specific class. displayDataChangedSignal() reports a new one.
 *
 * Asynchronous processing: a model runs heavy computations via asyncProcessor() (work on a worker
 * thread, commit on the main thread). loadingDone() then defers its recompute (work pending at entry)
 * or dataLoadedSignal() (work launched by the recompute) until the work has finished.
 */
class ViewModel : public QObject
{
    Q_OBJECT
public:
    typedef std::map<std::string, std::shared_ptr<Buffer>> BufferData;

    ViewModel(DBContentManager& dbcont_man, context::DBContextManager& ctx_man, AppState& app_state);
    virtual ~ViewModel(); // out-of-line: async_processor_ joins its workers here

    // inputs

    void loadingStarted();
    void updateFromSource(const DBContentDataSet& source,
                          const std::vector<std::string>& names, bool reset, bool last);
    void loadingDone();
    /// Clears the buffers and the display data. Emits no display data change, the owner clears the display.
    void clearData();
    /// Computes the display data again. A recompute while processing is outstanding runs once the
    /// processing has finished.
    void recompute();

    /// Shows the annotation in place of the buffers, from the next recompute() on.
    /// Returns false for a malformed annotation, error then holds the reason.
    virtual bool setAnnotation(const nlohmann::json& feature, const PlotMetadata& metadata,
                               std::string* error = nullptr) = 0;
    virtual void clearAnnotation() = 0;

    /// The layer panel model of the view (owned by the config widget's panel): its data rows follow the
    /// model's layer snapshots (updateLayerTree()), visibility and color mode changes reach the hooks.
    void attachLayerPanel(ViewLayerTreeModel* layer_model);
    ViewLayerTreeModel* layerPanelModel() const { return layer_model_; }

    /// a reload follows: the layer tree keeps its data rows (stale, detached from their payloads)
    /// until the reloaded data updates them, instead of being emptied by the clear
    void beginDataReload();
    void databaseClosed();

    // state

    bool hasData() const;
    /// the display data shows an annotation
    virtual bool hasAnnotation() const = 0;
    /// processing is outstanding, processingFinishedSignal() is emitted once done
    bool hasPendingProcessing() const;

    const BufferData& viewData() const { return data_; }

    const boost::optional<size_t>& nullCount() const { return count_null_; }
    const boost::optional<size_t>& nanCount() const { return count_nan_; }

    virtual void viewInfoJSON(nlohmann::json& info) const {}

    // selection

    /// inverts / clears the selection flags of the buffers
    void invertSelection();
    void clearSelection();

signals:
    void displayDataChangedSignal();
    void selectionChangedSignal();
    void processingStartedSignal();
    void processingFinishedSignal();
    /// the loaded data is processed
    void dataLoadedSignal();
    /// a layer panel got attached (after the first rebuild of its data rows)
    void layerPanelAttachedSignal();

protected:
    typedef std::vector<std::unique_ptr<ViewLayerSnapshot>> LayerSnapshots;
    typedef std::map<std::string, view_layer_scan::LayerAgg> LayerAggregates;

    /// the buffers changed (full data on the load finalize, last)
    virtual void updateFromSource_impl(bool reset, bool last) = 0;
    /// clears all data computed from the buffers, and the display data
    virtual void clearData_impl() = 0;
    /// Computes the display data and emits displayDataChangedSignal(), from the main thread.
    /// Can launch the computation on asyncProcessor(), then the commit emits the signal.
    virtual void recompute_impl() = 0;

    ViewAsyncProcessor& asyncProcessor(); // created on first use
    bool hasPendingAsyncWork() const;

    // Invalidates and joins outstanding asynchronous work. MUST be called from the
    // destructor of any derived model whose work functions capture `this`: the base
    // destructor joins too, but only after the derived part is already destroyed, so
    // a still-running worker would read a half-dead object.
    void shutdownAsyncProcessor();

    BufferData& mutableViewData() { return data_; } // because of selection

    // whether the mirrored data honors the engine's read-set contract (see
    // DBContentDataSet::fulfillsReadSet); false for live feed data
    bool dataFulfillsReadSet() const { return data_fulfills_read_set_; }

    void resetCounts();
    void addNullCount(size_t n);
    void addNanCount(size_t n);

    DBContentManager& dbContentManager() const { return dbcont_man_; }
    context::DBContextManager& dbContextManager() const { return ctx_man_; }
    AppState& appState() const { return app_state_; }

    // ---- layer panel (see attachLayerPanel()) ----------

    /// rebuilds the snapshots and calls updateLayerTree(), on attach and by default on color mode changes
    virtual void rebuildLayerTree() { refreshLayerTree(); }
    /// the checked state of layer tree items changed (not for the changes of the tree update itself)
    virtual void layerVisibilityChanged() {}
    /// the global color mode changed, default: rebuildLayerTree()
    virtual void colorModeChanged() { rebuildLayerTree(); }

    bool hasLayerPanel() const { return layer_model_ != nullptr; }

    /// the snapshots become current, the layer tree's data rows are updated in place (if attached),
    /// the previous snapshots are released afterwards; content_final, see ViewLayerTreeModel::updateDataSubtree()
    /// Notice: main thread only
    void updateLayerTree(LayerSnapshots layers, bool content_final = false);
    /// updates the data rows from the current snapshots again
    void refreshLayerTree();
    const LayerSnapshots& layerSnapshots() const { return layer_snapshots_; }

    /// layer aggregation of the loaded data (row scan, main thread), cached until invalidated
    const LayerAggregates& layerAggregates();
    /// aggregation computed elsewhere (e.g. on a worker), cached
    void setLayerAggregates(LayerAggregates aggregates);
    void invalidateLayerAggregates() { layer_agg_valid_ = false; }

    /// snapshots of aggregated layers, colors resolved for the current color mode
    LayerSnapshots snapshotsFromAggregates(const LayerAggregates& aggregates) const;

    /// main thread: snapshot of the layer panel's hidden ids (state().hiddenIds()) for processing
    /// which may run on a worker - workers read hiddenLayerIds(), never the model
    void snapshotHiddenLayerIds();
    const std::set<std::string>& hiddenLayerIds() const { return hidden_layer_ids_; }

private:
    void notifyProcessingFinished();
    void setLayerDataComplete(bool complete);
    /// stale data rows left after the data was loaded (the model updated nothing) are removed
    void endDataReload();
    void finishDataLoad();

    DBContentManager&          dbcont_man_;
    context::DBContextManager& ctx_man_;
    AppState&                  app_state_;

    ViewLayerTreeModel* layer_model_         = nullptr; // attached layer panel model, NOT owned
    std::vector<QMetaObject::Connection> layer_connections_; // of the attached layer panel
    LayerSnapshots      layer_snapshots_;               // the data rows' payloads
    bool                updating_layer_tree_ = false;   // hidden signals of the own tree update => no hook
    LayerAggregates     layer_agg_cache_;               // see layerAggregates()
    bool                layer_agg_valid_     = false;

    std::set<std::string> hidden_layer_ids_; // see snapshotHiddenLayerIds()

    BufferData data_;

    bool data_fulfills_read_set_ = true; // mirrored from the source in updateFromSource

    boost::optional<size_t> count_null_ = 0;
    boost::optional<size_t> count_nan_  = 0;

    // a recompute requested while asynchronous work was outstanding, run
    // coalesced once the work finishes (see recompute / notifyProcessingFinished)
    bool pending_recompute_ = false;

    // deferred load-done state (see loadingDone / notifyProcessingFinished)
    bool pending_loading_done_ = false; // recompute deferred on pending work
    bool pending_data_loaded_  = false; // dataLoadedSignal deferred on work launched by the recompute

    std::unique_ptr<ViewAsyncProcessor> async_processor_; // created on first use
};
