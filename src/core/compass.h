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
#include "json_fwd.hpp"
#include "appmode.h"
#include "result.h"
#include "logmodel.h"
#include "message.h"

#include <QObject>

#include <functional>
#include <memory>
#include <map>
#include <vector>

class DBInterface;
class DBContentManager;
class SimpleConfig;
class IMainWindow;
class IViewProvider;
class IFilterProvider;
class IEvalProvider;
class ITaskProvider;
class LicenseManager;
class ConfigJSON;
class ConfigurationManager;
class JobManager;
class UnitManager;
class ProjectionManager;
class RTCommandManager;
class IDBVariableResolver;
class QWidget;

namespace context
{
    class DBContext;
    class DBContextManager;
}

namespace rtcommand
{
    class RTCommandRunner;
}

/**
 * Access point to the managers of the application.
 * - the managers are created, owned and wired by the composition root (CompassRoot);
 *   COMPASS holds pointers to them and stays their configuration parent
 * - database lifecycle, application mode and global settings
 */
class COMPASS : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void databaseOpenedSignal();
    void databaseClosedSignal();
    // emitted right before appModeSwitchSignal, for steps that must run before all its slots
    void appModeAboutToSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void appModeSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void liveOverloadChangedSignal(bool overloaded);
    void colorModeChangedSignal(unsigned int mode);
    void darkModeChangedSignal(bool dark);

    // relayed from ViewManager, for code below the view layer
    void viewDataDistributedSignal(bool reset);
    void viewLoadingDoneSignal();

public:
    /**
     * Pointers to the managers, owned and set by the composition root (CompassRoot).
     */
    struct Managers
    {
        DBInterface*                db_interface       = nullptr;
        DBContentManager*           dbcontent_manager  = nullptr;
        context::DBContextManager*  context_manager    = nullptr;
        LicenseManager*             license_manager    = nullptr;
        JobManager*                 job_manager        = nullptr;
        UnitManager*                unit_manager       = nullptr;
        ProjectionManager*          projection_manager = nullptr;
        RTCommandManager*           rt_cmd_manager     = nullptr;
        rtcommand::RTCommandRunner* rt_cmd_runner      = nullptr;

        // the managers of the upper layers, as seen by code below their layer
        IViewProvider*              view_provider      = nullptr;
        IFilterProvider*            filter_provider    = nullptr;
        IEvalProvider*              eval_provider      = nullptr;
        ITaskProvider*              task_provider      = nullptr;
    };

    explicit COMPASS(ConfigurationManager& config_manager);
    virtual ~COMPASS();

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;
    std::string getPath() const override final;

    // composition root interface
    void registerManagerCreator(const std::string& class_name,
                                const std::string& instance_id,
                                const std::function<void(nlohmann::json&)>& create);
    void createManagers();

    Managers& managers() { return managers_; }

    void setMainWindow(IMainWindow* main_window);
    void setLiveImportBacklogSizeFunc(const std::function<unsigned int()>& func);
    void setShutdownSequence(const std::function<void()>& sequence);

    bool openDBFile(const std::string& filename);
    bool createNewDBFile(const std::string& filename);
    bool createInMemDBFile(const std::string& future_filename = "");
    bool createNewDBFileFromMemory();
    bool exportDBFile(const std::string& filename);
    bool closeDB();

    bool dbOpened();
    bool dbInMem() const;
    bool canCreateDBFileFromMemory() const;

    DBInterface& dbInterface();
    DBContentManager& dbContentManager();
    SimpleConfig& config();
    rtcommand::RTCommandRunner& rtCmdRunner();
    context::DBContextManager& dbContextManager();
    bool hasActiveContext() const;
    context::DBContext& context();
    const context::DBContext& context() const;
    LicenseManager& licenseManager();
    JobManager& jobManager();
    UnitManager& unitManager();
    ProjectionManager& projectionManager();
    RTCommandManager& rtCommandManager();
    ConfigurationManager& configManager() { return config_manager_; }

    // view, filter, evaluation and task manager for code below their layer
    // (the upper layers get them from their own manager, the app layer from CompassRoot)
    IViewProvider& viewProvider();
    IFilterProvider& filterProvider();
    IEvalProvider& evalProvider();
    ITaskProvider& taskProvider();

    const DBInterface& dbInterface() const;
    const DBContentManager& dbContentManager() const;
    const SimpleConfig& config() const;
    const rtcommand::RTCommandRunner& rtCmdRunner() const;
    const LicenseManager& licenseManager() const;
    const JobManager& jobManager() const;
    const UnitManager& unitManager() const;
    const ProjectionManager& projectionManager() const;
    const RTCommandManager& rtCommandManager() const;
    const ConfigurationManager& configManager() const { return config_manager_; }

    void shutdown();

    void setAppState(AppState state);

    IMainWindow& mainWindow();

    // View lookup facades - allow rtcommand helpers to navigate the view
    // tree without depending on the view layer (answered by IViewProvider)
    QWidget* viewContainerWidget(const std::string& name);
    QWidget* latestViewContainerWidget();
    QWidget* latestViewWidget();

    std::string lastUsedPath();
    void lastUsedPath(const std::string& last_path);

    std::string versionString(bool open_ats = true, 
                              bool license_type = true) const;
    std::string licenseeString(bool licensed_to = true) const;

    std::string lastDbFilename() const;
    std::vector<std::string> dbFileList() const;
    void clearDBFileList();
    void addDBFileToList(const std::string filename);

    AppMode appMode() const;
    void appMode(const AppMode& app_mode);
    std::string appModeStr() const;

    AppState appState() const { return app_state_; }

    static bool isAppImage() { return is_app_image_; }

    static const std::map<AppMode, std::string>& appModes2Strings();

    bool expertMode() const;
    void expertMode(bool expert_mode);

    // false while automated processing (CLI / runtime commands) runs - code must not open
    // modal GUI dialogs when this is false, but resolve non-interactively instead
    bool allowUserInteractions() const { return allow_user_interactions_; }
    void allowUserInteractions(bool value) { allow_user_interactions_ = value; }

    bool isShutDown() const;
    bool isRunning() const;

    bool pdflatexFound() const;

    bool disableLiveToOfflineSwitch() const;
    bool disableMenuConfigSave() const;
    bool disableGeographicViewRotate() const;
    bool disableAddRemoveViews() const;
    bool dbExportInProgress() const;

    unsigned int autoLiveRunningResumeAskTime() const; // min
    unsigned int autoLiveRunningResumeAskWaitTime() const; // min
    bool disableConfirmResetViews() const;

    unsigned int maxLiveDataAgeCache() const { return max_live_data_age_cache_; }
    unsigned int maxLiveDataAgeDb() const { return max_live_data_age_db_; }

    // live overload: while set, live ticks are held (the views keep the last distributed data)
    bool liveOverloaded() const { return live_overloaded_; }
    void liveOverloaded(bool overloaded);

    unsigned int liveImportBacklogSize() const; // ASTERIX packets in processing
    unsigned int liveOverloadMaxPackets() const { return live_overload_max_packets_; }
    double liveOverloadMaxLatency() const { return live_overload_max_latency_s_; }
    bool liveOverloadShowDebugInfo() const { return live_overload_show_debug_info_; }

    // debug: force a live overload (runtime only, not persisted; no UI, set from code)
    bool liveOverloadForced() const { return live_overload_forced_; }
    void liveOverloadForced(bool forced) { live_overload_forced_ = forced; }

    bool darkMode() const;
    void darkMode(bool value);

    /// Color mode used by the Data Sources panel: 0=DSType, 1=DBContent, 2=DataSource, 3=DataSourceLine
    unsigned int colorMode() const { return color_mode_; }
    void colorMode(unsigned int value);

    const char* lineEditInvalidStyle();

    LogStream logInfo(const std::string& component,
                      boost::optional<unsigned int> error_code={}, 
                      nlohmann::json json_blob={});
    LogStream logWarn(const std::string& component,
                      boost::optional<unsigned int> error_code={}, 
                      nlohmann::json json_blob={});
    LogStream logError(const std::string& component,
                       boost::optional<unsigned int> error_code={}, 
                       nlohmann::json json_blob={});

    unsigned int minAppWidth() const;

    unsigned int minAppHeight() const;

    float appFontScale() const;

    bool disableNativeDialogs() const;

    LogStore& logStore();

    const nlohmann::json& unspecificACIDs() const { return unspecific_acids_; }
    bool isUnspecificACID(const std::string& acid) const;

    bool sensorStatusTimeHack() const;
    void sensorStatusTimeHack(bool value);

protected:
    Result openDBFileInternal(const std::string& filename);
    Result createNewDBFileInternal(const std::string& filename);
    Result createInMemDBFileInternal(const std::string& future_filename);
    Result createNewDBFileFromMemoryInternal();
    Result exportDBFileInternal(const std::string& filename);
    Result closeDBInternal();

    virtual void checkSubConfigurables() override;

    ConfigurationManager& config_manager_;

    bool db_opened_{false};
    bool db_inmem_{false};
    bool expert_mode_ {false};
    bool allow_user_interactions_ {true};
    bool pdflatex_found_ {false};

    bool dark_mode_ {false};

    unsigned int color_mode_ {0}; // 0=DSType, 1=DBContent, 2=DataSource, 3=DataSourceLine

    AppMode app_mode_ {AppMode::Offline};
    AppState app_state_ = AppState::Starting;

    bool disable_live_to_offline_switch_ {false};
    bool disable_menu_config_save_ {false};
    bool disable_geographicview_rotate_ {false};
    bool disable_add_remove_views_ {false};
    bool disable_confirm_reset_views_ {false};

    static const bool is_app_image_;

    unsigned int max_live_data_age_cache_ {5};
    unsigned int max_live_data_age_db_ {60};

    bool         live_overloaded_            {false};
    unsigned int live_overload_max_packets_  {4};    // import backlog above which live ticks are held
    double       live_overload_max_latency_s_{-1.0}; // latency in seconds above which live ticks are held (<= 0 = off)
    bool         live_overload_show_debug_info_{false}; // show backlog/latency in the geographic view (debug)
    bool         live_overload_forced_       {false};   // debug: live ticks held until switched off (not persisted)

    unsigned int auto_live_running_resume_ask_time_ {60}; // minutes
    unsigned int auto_live_running_resume_ask_wait_time_ {1}; // minutes

    unsigned int min_app_width_ {1600};
    unsigned int min_app_height_ {800};

    float app_font_scale_{1.0};

    bool disable_native_dialogs_ {false};

    std::unique_ptr<SimpleConfig> simple_config_;

    Managers managers_; // owned by the composition root

    LogStore log_store_;

    std::unique_ptr<ConfigJSON> config_json_;

    std::string last_db_filename_;
    std::string inmem_future_filename_;
    nlohmann::json db_file_list_;

    std::string last_path_;

    bool db_export_in_progress_ {false};

    nlohmann::json unspecific_acids_;

    bool sensor_status_time_hack_{false};

    IMainWindow* main_window_ = nullptr; // owned by the composition root

    std::function<unsigned int()> live_import_backlog_size_func_; // ASTERIX packets in processing
    std::function<void()>         shutdown_sequence_;             // ordered shutdown of all managers

    struct ManagerCreator
    {
        std::string                          class_name;
        std::string                          instance_id;
        std::function<void(nlohmann::json&)> create;
        bool                                 created = false;
    };

    std::vector<ManagerCreator> manager_creators_; // sub-configurable managers, in creation order
    bool                        managers_created_ = false;
};
