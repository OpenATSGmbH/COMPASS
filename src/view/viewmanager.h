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

#include "configurable.h"
#include "dbcontent/variable/variableset.h"
#include "appmode.h"
#include "viewpresets.h"

#include <QObject>

#include "boost/date_time/posix_time/ptime.hpp"
#include "boost/date_time/posix_time/posix_time_duration.hpp"

#include <memory>
#include <vector>
#include <set>

class COMPASS;
class Buffer;
class DBContentDataSet;
class LiveDataFeed;
struct LoadRequest;
class ViewContainer;
class ViewContainerWidget;
//class ViewManagerWidget;
class View;
class ViewableDataConfig;
class ViewPointsWidget;
class ViewPointsReportGenerator;

class QWidget;
class QTabWidget;
class LoadController;
class LiveController;

/**
*/
class ViewManager : public QObject, public Configurable
{
    Q_OBJECT

  signals:
    void selectionChangedSignal();
    // data has been distributed to the views (offline finalize + each live tick);
    // the data-sources status widget connects here to refresh (acts only in live)
    void dataDistributedSignal(bool reset);
    // display-load lifecycle bookends (offline load + the live session as one cycle).
    // UI/view chrome consumers (MainWindow, TargetListWidget, RT waits) connect here,
    // never to DBContentManager, whose loads may be issuer-private batch loads.
    void loadingStartedSignal();
    void loadingDoneSignal();
    void unshowViewPointSignal (ViewableDataConfig* vp);
    void showViewPointSignal (ViewableDataConfig* vp);
    void reloadStateChanged();
    void automaticUpdatesChanged();
    void presetEdited(ViewPresets::EditAction ea);

  public slots:
    void selectionChangedSlot();

    void databaseOpenedSlot();
    void databaseClosedSlot();

    void loadingStartedSlot();

    // driven by the current source's dataChangedSignal: pulls source->buffers()
    // and pushes them to the views. names empty = synthetic finalize event (skipped;
    // completion runs on loadingDoneSlot). reset = re-create all shown info.
    void sourceDataChangedSlot(const std::vector<std::string>& names, bool reset, bool last);
    void loadingDoneSlot(); // emitted when all dbconts have finished loading

    // a view finished its asynchronous processing (view processingFinishedSignal):
    // fires the deferred load-done edge once no view is pending anymore
    void viewProcessingStartedSlot();
    void viewProcessingFinishedSlot();

    void appModeSwitchSlot (AppMode app_mode_previous, AppMode app_mode_current);

    // ASTERIX live watchdog trigger -> one live tick (delegates to the live session)
    void forceLiveUpdate();

  public:
    struct Config
    {
        bool automatic_reload = true;
        bool automatic_redraw = true;
    };

    ViewManager(nlohmann::json& config, COMPASS& compass);
    virtual ~ViewManager();

    COMPASS& compass() { return compass_; }

    void init(QTabWidget* main_tab_widget);
    void close();

    void clearDataInViews();

    // reload the view content: builds the standard view LoadRequest, makes the
    // resulting operation the current source, and runs it through DBContentManager
    void reload(bool blocking = false, bool measure_performance = false);

    // the currently displayed data set (LoadOperation offline, LiveDataFeed live);
    // null when nothing is loaded. The single owner of the displayed buffers.
    std::shared_ptr<DBContentDataSet> currentSource() const { return current_source_; }

    // the displayed buffers (the current source's, or empty when nothing is loaded).
    // The accessor lives here, on the owner - not on DBContentManager, which holds no dataset.
    const std::map<std::string, std::shared_ptr<Buffer>>& currentBuffers() const;

    // live-session latency, delegated to the live session (the geo view reads it via the
    // DBContentManager façade). The feed + the two controllers stay private, not exposed.
    bool hasMaxLatency() const;
    boost::posix_time::time_duration maxLatency() const;

    // selection carried to the next load (set by FilterManager / view-point selection)
    void storeSelectedRecNums(const std::vector<unsigned long>& selected);
    void clearSelectedRecNums();

    void registerView(View* view);
    void unregisterView(View* view);
    bool isRegistered(View* view);

    ViewContainerWidget* addNewContainerWidget();

    // void deleteContainer (std::string instance_name);
    void removeContainer(std::string instance_name);
    void deleteContainerWidget(std::string instance_name);
    void removeContainerWidget(std::string instance_name);

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;

    void viewShutdown(View* view, const std::string& err = "");

    std::map<std::string, ViewContainer*> getContainers() { return containers_; }
    std::map<std::string, ViewContainerWidget*> getContainerWidgets() { return container_widgets_; }
    std::map<std::string, View*> getViews() { return views_; }
    View* latestView();
    ViewContainerWidget* latestViewContainer();
    
    dbContent::VariableSet getReadSet(const std::string& dbcontent_name);

    //@TODO: needed because of view container widget hack in ui_test_find.h
    //remove if no longer needed!
    ViewContainerWidget* containerWidget(const std::string& container_widget_name)
    {
        auto it = container_widgets_.find(container_widget_name);
        if (it == container_widgets_.end())
            return nullptr;

        return it->second;
    }

    ViewPointsWidget* viewPointsWidget() const;
    ViewPointsReportGenerator& viewPointsGenerator();

    void loadViewPoints();
    std::pair<bool, std::string> loadViewPoints(nlohmann::json json_obj);
    void clearViewPoints();
    void addViewPoints(const std::vector <nlohmann::json>& viewpoints);

    void setCurrentViewPoint (ViewableDataConfig* viewable,
                              bool load_blocking = false);
    /// Same, but takes shared ownership. Use this whenever the caller cannot
    /// guarantee that the viewable outlives the load it starts - the
    /// set_view_point command is destroyed before its load completes, which
    /// would leave current_viewable_ (and the views' current_view_point_)
    /// dangling in doViewPointAfterLoad().
    void setCurrentViewPoint (std::shared_ptr<ViewableDataConfig> viewable,
                              bool load_blocking = false);
    void unsetCurrentViewPoint ();
    void doViewPointAfterLoad ();

    /// For each view container, if the currently-selected tab's view does
    /// not accept any annotation feature type present in `viewable`, switch
    /// the container to the first view (in container order) that does. No
    /// switch occurs when the current view is already compatible, when no
    /// container holds a compatible view, or when the viewable has no
    /// annotation features.
    void activateCompatibleViewTabs(const ViewableDataConfig* viewable);

    void selectTimeWindow(boost::posix_time::ptime ts_min, boost::posix_time::ptime ts_max);

    void showMainViewContainerAddView();

    std::map<std::string, std::string> viewClassList() const;

    unsigned int newViewNumber(const std::string& class_name);
    std::string newViewInstanceId(const std::string& class_name);
    std::string newViewName(const std::string& class_name);

    // disables propagation of data to the views. used when loading is performed for processing purposes

    bool isProcessingData() const;

    // any view still processing data asynchronously (e.g. geographic view geometry
    // builds). Part of the "busy" condition: gates a second load, closing the
    // database, applying a view point and rendering a report figure.
    bool hasPendingViewProcessing() const;

    void resetToStartupConfiguration();

    bool isInitialized() const;

    bool viewPresetsEnabled() const;
    ViewPresets& viewPresets() { return presets_; }
    const ViewPresets& viewPresets() const { return presets_; }

    void notifyReloadStateChanged();
    bool reloadNeeded() const;
    void enableAutomaticReload(bool enable);
    void enableAutomaticRedraw(bool enable);
    bool automaticReloadEnabled() const;
    bool automaticRedrawEnabled() const;

    void updateFeatures();

    template<class T>
    std::vector<T*> viewsOfType()
    {
        std::vector<T*> views;
        for (const auto& v : views_)
        {
            T* vt = dynamic_cast<T*>(v.second);
            if (vt != nullptr)
                views.push_back(vt);
        }

        return views;
    }

protected:
    virtual void checkSubConfigurables();

    void enableStoredReadSets();
    void disableStoredReadSets();

    // runs a request as the new display source (selection carry-over + view clear +
    // setCurrentSource); used by reload() and by the paused-display load
    void issueLoad(const LoadRequest& req, bool blocking = false);

    // loads the DB contents into the display when live is paused; posted from
    // appModeSwitchSlot so it runs after the app-mode switch has been fully delivered
    void loadPausedDisplay();

    // subscribes to the source's dataChangedSignal and takes ownership; set by
    // issueLoad() (offline/paused op) and appModeSwitchSlot (live feed)
    void setCurrentSource(std::shared_ptr<DBContentDataSet> source);

    // live-session bookends (distinct from the offline op-driven loadingStarted/DoneSlot):
    // view chrome + external loadingStarted/DoneSignal, no offline dialog/progress/viewpoint
    void beginLiveSession();
    void endLiveSession();

    // the deferred tail of loadingDoneSlot: dialog close + external done bookend
    void finishLoadingDone();

    // selection carry-over across (re)loads (owned here - a view concern):
    // captureSelection() reads selected_ from the current source before a reload swaps
    // it away; applyCarriedSelection() restores it onto freshly arrived buffers.
    void captureSelection();
    void applyCarriedSelection(const std::vector<std::string>& names);
    void restoreSelectionInto(const std::string& dbcontent_name, Buffer& buffer);

    COMPASS& compass_;

    ViewPointsWidget* view_points_widget_{nullptr};

    Config config_;

    bool initialized_     = false;
    bool processing_data_ = false;
    bool reload_needed_   = false;

    // the load-done edge (dialog close + loadingDoneSignal) is held back because a
    // view still processes data asynchronously; released in viewProcessingFinishedSlot.
    // Cleared when a new load starts (the deferred done of a superseded load is dropped).
    bool loading_done_deferred_ = false;

    // views whose asynchronous processing the view-phase progress is still waiting on:
    // each advances the progress when it reports finished, so the dialog reflects
    // finished views rather than dispatched ones
    std::set<View*> pending_views_;

    // Diagnostic state for loading lifecycle:
    //   loading_done_dispatched_: set true after a loadingDoneSlot body completes,
    //     reset on loadingStartedSlot. Used to detect late sourceDataChangedSlot calls.
    //   current_dispatch_: name of slot currently inside its per-view loop, or
    //     empty when idle. Logged when re-entry happens.
    bool loading_done_dispatched_ = false;
    std::string current_dispatch_;

    QTabWidget* main_tab_widget_{nullptr};

    std::map<std::string, ViewContainer*> containers_;
    std::map<std::string, ViewContainerWidget*> container_widgets_;
    std::map<std::string, View*> views_;

    std::shared_ptr<DBContentDataSet> current_source_; // subscribed data source

    std::unique_ptr<LoadController> load_controller_; // view-load dialog/cursor/progress
    std::unique_ptr<LiveController> live_controller_; // live session: feed + tick

    std::map<std::string, std::set<unsigned long>> carried_selection_; // selected rec nums carried between loads

    std::unique_ptr<ViewPointsReportGenerator> view_points_report_gen_;

    ViewableDataConfig* current_viewable_ {nullptr};
    /// Set when the viewable was handed over with shared ownership, to keep it
    /// alive for as long as current_viewable_ points at it. Empty for the
    /// ViewPoints owned by ViewPointsTableModel, which are passed raw.
    std::shared_ptr<ViewableDataConfig> current_viewable_owned_;
    bool view_point_data_selected_ {false};

    unsigned int container_count_{0};

    std::map<std::string, std::string> view_class_list_; // class name -> name (without appended number)

    bool use_tmp_stored_readset_ {false};
    std::map<std::string, dbContent::VariableSet> tmp_stored_readset_;

    ViewPresets presets_;
};
