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

#include "viewcomponent.h"
#include "viewlayerscan.h"
#include "appmode.h"
#include "json_fwd.hpp"

#include <map>
#include <memory>
#include <set>
#include <vector>
#include <string>

#include <boost/optional.hpp>

#include <QWidget>

class ViewWidget;
class ViewToolSwitcher;
class Buffer;
class ViewLayerTreeModel;
class ViewLayerSnapshot;
class DBContentItemProvider;
class DBContentDataSet;
class ViewAsyncProcessor;
class QLayout;

namespace dbContent
{
    class Variable;
}

/**
 * Base class for view data widgets, which are held in the data area of the ViewWidget.
 * Used to display data in a view specific way, e.g. as a graph.
 * Derive and reimplement as needed.
 */
class ViewDataWidget : public QWidget, public ViewComponent 
{
    Q_OBJECT
public:
    enum class DrawState
    {
        NotDrawn,    // not drawn yet
        Drawn,       // drawn, but no content
        DrawnContent // drawn with content
    };

    typedef std::map<std::string, std::shared_ptr<Buffer>> BufferData;

    ViewDataWidget(ViewWidget* view_widget, QWidget* parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~ViewDataWidget(); // out-of-line: async_processor_ joins its workers here

    void setToolSwitcher(ViewToolSwitcher* tool_switcher);

    void loadingStarted();
    void loadingDone();
    void updateFromSource(const DBContentDataSet& source,
                          const std::vector<std::string>& names, bool reset, bool last);
    void clearData();
    DrawState redrawData(bool recompute, bool notify = false);
    void liveReload();

    /// a reload follows: the layer tree keeps its data rows (stale, detached from their payloads)
    /// until the reloaded data updates them, instead of being emptied by the clear
    void beginDataReload();

    // Optional item provider, registered by a derived widget which OWNS it (the
    // provider's lifetime is tied to the derived widget's, whose teardown may be
    // order-sensitive). The base only borrows it: it drives the provider's input
    // from the current source (updateFromSource) and resets it on clearData().
    // The derived widget must setItemProvider(nullptr) before destroying it.
    // Views without a provider are unaffected.
    // Every provider reset starts a reload of the layer tree's data rows (beginDataReload()).
    void setItemProvider(DBContentItemProvider* provider);
    bool hasItemProvider() const { return item_provider_ != nullptr; }
    DBContentItemProvider* itemProvider() { return item_provider_; }

    bool hasData() const;
    bool hasAnnotations() const;
    bool hasContent() const;
    bool hasVisibleContent() const;
    bool isDrawn() const;
    bool isContentDrawn() const;

    // the widget still processes data asynchronously (a worker task is outstanding or
    // load-done work is deferred on one); processingFinishedSignal is emitted once done.
    // Drives ViewManager's deferred load-done edge via the owning View.
    bool hasPendingProcessing() const;

    void databaseOpened();
    void databaseClosed();

    /// The view's layer panel tree model, if any. Lets the base class manage
    /// per-database-session layer state (e.g. forget hidden layers on close).
    /// Default: the attached one (see attachLayerPanel()).
    virtual ViewLayerTreeModel* layerTreeModel() { return layer_model_; }

    /// The layer panel model of the view (owned by the config widget's panel): its data rows follow the
    /// view's layer snapshots (updateLayerTree()), visibility and color mode changes reach the hooks.
    void attachLayerPanel(ViewLayerTreeModel* layer_model);

    virtual void isExporting(bool ok) { is_exporting_ = ok; }
    bool isExporting() const { return is_exporting_; }

    virtual bool hasScreenshotContent() const;

    unsigned int loadedDataCount();

    bool isVariableSetLoaded() const;

    virtual void appModeSwitch(AppMode app_mode) {} //reacts on switching the application mode
    virtual void configChanged() {}                 //reacts on configuration changes
    virtual void onInit() {}                        //reacts on view init (both data and config widget created)

    nlohmann::json viewInfoJSON() const override final;

    virtual QImage renderData();

    // called before renderData() so subclasses can settle async resources
    virtual void prepareForRender() {}

    QColor colorForGroupName(const std::string& group_name); // creates new one of required
    const std::map<std::string, QColor>& dbContentColors() const;

    const boost::optional<size_t>& nullCount() const { return count_null_; }
    const boost::optional<size_t>& nanCount() const { return count_nan_; }

    static const double      MarkerSizePx;
    static const double      MarkerSizeSelectedPx;

    static const std::string Color_CAT001;
    static const std::string Color_CAT010;
    static const std::string Color_CAT020;
    static const std::string Color_CAT021;
    static const std::string Color_CAT048;
    static const std::string Color_RefTraj;
    static const std::string Color_CAT062;

    static const QColor      ColorSelected;

public slots:
    /// inverts / clears the selection flags of the view's data, notifies the view
    virtual void invertSelectionSlot();
    virtual void clearSelectionSlot();

signals:
    void displayChanged();
    void dataLoaded();
    void redrawStarted();
    void redrawDone();
    void updateStarted();
    void updateDone();
    // asynchronous processing started (the processor went busy); forwarded by the View
    void processingStartedSignal();
    // asynchronous processing finished (see hasPendingProcessing); forwarded by the View
    void processingFinishedSignal();

protected:
    // Asynchronous processing support. A widget runs heavy per-load computations via
    // asyncProcessor() (work on a worker thread, commit on the main thread) or its own
    // mechanism (the geo view's geometry builds), and reports outstanding work through
    // hasPendingAsyncWork_impl(). loadingDone() then defers loadingDone_impl() (work
    // pending at entry) or the dataLoaded emission (work launched inside the impl)
    // until notifyProcessingFinished() runs - wired automatically for asyncProcessor(),
    // called explicitly by widgets with their own mechanism.
    ViewAsyncProcessor& asyncProcessor();                       // created on first use
    virtual bool hasPendingAsyncWork_impl() const;              // default: asyncProcessor tasks
    void notifyProcessingFinished();

    virtual void toolChanged_impl(int tool_id) = 0;        //implements reactions on tool switches
    virtual void loadingStarted_impl() = 0;                //implements behavior at starting a reload
    virtual void loadingDone_impl();                       //implements behavior at finishing a reload
    virtual void updateFromSource_impl(const DBContentDataSet& source,
                                       const std::vector<std::string>& names, bool reset, bool last) = 0; //implements behavior at receiving a source update
    virtual void clearData_impl() = 0;                     //implements clearing all view data
    virtual void clearIntermediateRedrawData_impl() = 0;   //implements clearing of any data collected during redraw
    virtual DrawState redrawData_impl(bool recompute) = 0; //implements redrawing the display (and possibly needed computations), and returns the new draw state
    virtual void liveReload_impl() = 0;                    //implements data reload during live running mode (reload not handled via a real db reload)
    virtual bool hasAnnotations_impl() const = 0;          //implements checking if the view has any annotations
    virtual void databaseOpened_impl() {}                  //implements behavior on opening a database
    virtual void databaseClosed_impl() {}                  //implements behavior on closing a database

    virtual void viewInfoJSON_impl(nlohmann::json& info) const {}

    void addNullCount(size_t n);
    void addNanCount(size_t n);

    // clears intermediate redraw data + counts (the compute-phase reset of redrawData);
    // protected so asynchronous recomputes can run it in their worker phase
    void clearIntermediateRedrawData();

    void endTool();
    ViewToolSwitcher* toolSwitcher() { return tool_switcher_; } // null before the view widget set it

    void setDrawState(DrawState state) { draw_state_ = state; }
    DrawState drawState() const { return draw_state_; }

    // Invalidates and joins outstanding asynchronous work. MUST be called from the
    // destructor of any derived widget whose work functions capture `this`: the base
    // destructor joins too, but only after the derived part is already destroyed, so
    // a still-running worker would read a half-dead object.
    void shutdownAsyncProcessor();

    const BufferData& viewData() const { return data_; }
    BufferData& viewData() { return data_; } //exposed because of selection

    // whether the mirrored data honors the engine's read-set contract (see
    // DBContentDataSet::fulfillsReadSet); false for live feed data
    bool dataFulfillsReadSet() const { return data_fulfills_read_set_; }

    ViewWidget* getWidget() { return view_widget_; }

    /// Takes the widget out of the layout (if given) and destroys it via deleteLater, not in place:
    /// charts own a QGraphicsScene with their axes, and this can run from an asynchronous commit
    /// while Qt still has pending events for it (destroying in place crashed inside QtCharts:
    /// QGraphicsScene::clear -> ~QChart -> deleteAllAxes). Hidden right away.
    static void releaseWidgetLater(QWidget* widget, QLayout* layout);

    // ---- layer panel (views with layer snapshots, see attachLayerPanel()) ----------

    typedef std::vector<std::unique_ptr<ViewLayerSnapshot>> LayerSnapshots;
    typedef std::map<std::string, view_layer_scan::LayerAgg> LayerAggregates;

    /// rebuilds the snapshots and calls updateLayerTree(), on attach and by default on color mode changes
    virtual void rebuildLayerTree() { refreshLayerTree(); }
    /// the checked state of layer tree items changed (not for the changes of the tree update itself)
    virtual void layerVisibilityChanged() {}
    /// the global color mode changed, default: rebuildLayerTree()
    virtual void colorModeChanged() { rebuildLayerTree(); }
    /// the layer panel got attached (after the first rebuild)
    virtual void layerPanelAttached() {}

    bool hasLayerPanel() const { return layer_model_ != nullptr; }
    ViewLayerTreeModel* layerPanelModel() const { return layer_model_; }

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
    /// main thread: the panel's current hidden ids differ from the snapshot
    bool hiddenLayerIdsChanged() const;

private:
    friend class ViewLoadStateWidget;

    void toolChanged(int mode, const QCursor& cursor);
    void setLayerDataComplete(bool complete);
    /// stale data rows left after the data was loaded (the view updated nothing) are removed
    void endDataReload();

    ViewWidget*       view_widget_   = nullptr;
    ViewToolSwitcher* tool_switcher_ = nullptr;

    DBContentItemProvider*  item_provider_ = nullptr;   // optional, NOT owned (see setItemProvider)
    QMetaObject::Connection provider_reset_connection_; // reset => reload of the layer tree's data rows

    ViewLayerTreeModel* layer_model_         = nullptr; // attached layer panel model, NOT owned
    std::vector<QMetaObject::Connection> layer_connections_; // of the attached layer panel
    LayerSnapshots      layer_snapshots_;               // the data rows' payloads
    bool                updating_layer_tree_ = false;   // hidden signals of the own tree update => no hook
    LayerAggregates     layer_agg_cache_;               // see layerAggregates()
    bool                layer_agg_valid_     = false;

    std::set<std::string> hidden_layer_ids_; // see snapshotHiddenLayerIds()

    BufferData data_;
    DrawState  draw_state_ = DrawState::NotDrawn;

    bool data_fulfills_read_set_ = true; // mirrored from the source in updateFromSource

    // a recompute redraw requested while asynchronous work was outstanding, run
    // coalesced once the work finishes (see redrawData / notifyProcessingFinished)
    bool pending_recompute_redraw_ = false;

    bool is_exporting_ = false;

    std::map<std::string, QColor> dbc_colors_;

    boost::optional<size_t> count_null_ = 0;
    boost::optional<size_t> count_nan_  = 0;

    // deferred load-done state (see loadingDone / notifyProcessingFinished)
    bool pending_loading_done_ = false; // loadingDone_impl deferred on pending work
    bool pending_data_loaded_  = false; // dataLoaded deferred on work launched in the impl

    std::unique_ptr<ViewAsyncProcessor> async_processor_; // created on first use
};
