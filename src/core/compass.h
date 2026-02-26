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

#include <memory>
#include <map>
#include <vector>

class DBInterface;
class DBContentManager;
class DataSourceManager;
class FilterManager;
class TaskManager;
class ViewManager;
class SimpleConfig;
class EvaluationManager;
class MainWindow;
class FFTManager;
class LicenseManager;
class ConfigJSON;
class ConfigurationManager;
class JobManager;
class UnitManager;
class ProjectionManager;
class RTCommandManager;

namespace rtcommand
{
    class RTCommandRunner;
}

class COMPASS : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void databaseOpenedSignal();
    void databaseClosedSignal();
    void appModeSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);

public:
    explicit COMPASS(ConfigurationManager& config_manager);
    virtual ~COMPASS();

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;
    std::string getPath() const override final;

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
    DataSourceManager& dataSourceManager();
    FilterManager& filterManager();
    TaskManager& taskManager();
    ViewManager& viewManager();
    SimpleConfig& config();
    EvaluationManager& evaluationManager();
    rtcommand::RTCommandRunner& rtCmdRunner();
    FFTManager& fftManager();
    LicenseManager& licenseManager();
    JobManager& jobManager();
    UnitManager& unitManager();
    ProjectionManager& projectionManager();
    RTCommandManager& rtCommandManager();
    ConfigurationManager& configManager() { return config_manager_; }

    void init();
    void shutdown();

    void setAppState(AppState state);

    MainWindow& mainWindow();

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

    bool darkMode() const;
    void darkMode(bool value);

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
    bool pdflatex_found_ {false};

    bool dark_mode_ {false};

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

    unsigned int auto_live_running_resume_ask_time_ {60}; // minutes
    unsigned int auto_live_running_resume_ask_wait_time_ {1}; // minutes

    unsigned int min_app_width_ {1600};
    unsigned int min_app_height_ {800};

    float app_font_scale_{1.0};

    bool disable_native_dialogs_ {false};

    std::unique_ptr<SimpleConfig> simple_config_;
    std::unique_ptr<DBInterface> db_interface_;
    std::unique_ptr<DBContentManager> dbcontent_manager_;
    std::unique_ptr<DataSourceManager> ds_manager_;
    std::unique_ptr<FilterManager> filter_manager_;
    std::unique_ptr<TaskManager> task_manager_;
    std::unique_ptr<ViewManager> view_manager_;
    std::unique_ptr<EvaluationManager> eval_manager_;
    std::unique_ptr<FFTManager> fft_manager_;
    std::unique_ptr<LicenseManager> license_manager_;
    std::unique_ptr<JobManager> job_manager_;
    std::unique_ptr<UnitManager> unit_manager_;
    std::unique_ptr<ProjectionManager> projection_manager_;
    std::unique_ptr<RTCommandManager> rt_cmd_manager_;

    std::unique_ptr<rtcommand::RTCommandRunner> rt_cmd_runner_;

    LogStore log_store_;

    std::unique_ptr<ConfigJSON> config_json_;

    std::string last_db_filename_;
    std::string inmem_future_filename_;
    nlohmann::json db_file_list_;

    std::string last_path_;

    bool db_export_in_progress_ {false};

    MainWindow* main_window_ {nullptr};

    nlohmann::json unspecific_acids_;

    bool sensor_status_time_hack_{false};
};
