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
#include "appstate.h"
#include "rtcommand_host.h"
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
class LicenseManager;
class ConfigJSON;
class ConfigurationManager;
class ProjectionManager;
class RTCommandManager;
class IDBVariableResolver;
class QWidget;

namespace context
{
    class DBContext;
    class DBContextManager;
}

/**
 * Application facade of the app level.
 * - the managers are created, owned and wired by the composition root (CompassRoot);
 *   COMPASS holds pointers to them and stays their configuration parent
 * - database session, owner of the app state (its settings are config parameters of COMPASS)
 * - host of the runtime command execution (rtcommand::IRTCommandHost)
 */
class COMPASS : public QObject, public Configurable, public rtcommand::IRTCommandHost
{
    Q_OBJECT

signals:
    void databaseOpenedSignal();
    void databaseClosedSignal();

    // relayed from AppState, for the libraries above core
    // emitted right before appModeSwitchSignal, for steps that must run before all its slots
    void appModeAboutToSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void appModeSwitchSignal (AppMode app_mode_previous, AppMode app_mode_current);
    void liveOverloadChangedSignal(bool overloaded);
    void colorModeChangedSignal(unsigned int mode);
    void darkModeChangedSignal(bool dark);

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
        ProjectionManager*          projection_manager = nullptr;
        RTCommandManager*           rt_cmd_manager     = nullptr;

        // the view manager, for the view lookups of the runtime command host
        IViewProvider*              view_provider      = nullptr;
    };

    explicit COMPASS(ConfigurationManager& config_manager);
    virtual ~COMPASS();

    std::string getPath() const override final;

    // composition root interface
    void registerManagerCreator(const std::string& class_name,
                                const std::string& instance_id,
                                const std::function<void(nlohmann::json&)>& create);
    void createManagers();

    Managers& managers() { return managers_; }

    void setMainWindow(IMainWindow* main_window);
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
    context::DBContextManager& dbContextManager();
    bool hasActiveContext() const;
    context::DBContext& context();
    const context::DBContext& context() const;
    LicenseManager& licenseManager();
    ProjectionManager& projectionManager();
    RTCommandManager& rtCommandManager();
    ConfigurationManager& configManager() { return config_manager_; }

    IViewProvider& viewProvider(); // view lookups of the runtime command host

    const DBInterface& dbInterface() const;
    const DBContentManager& dbContentManager() const;
    const SimpleConfig& config() const;
    const LicenseManager& licenseManager() const;
    const ProjectionManager& projectionManager() const;
    const RTCommandManager& rtCommandManager() const;
    const ConfigurationManager& configManager() const { return config_manager_; }

    void shutdown();

    // state and global settings of the application
    AppState& appState() override { return app_state_; }
    const AppState& appState() const { return app_state_; }

    IMainWindow& mainWindow();

    // View lookup facades - allow rtcommand helpers to navigate the view
    // tree without depending on the view layer (answered by IViewProvider)
    QWidget* viewContainerWidget(const std::string& name) override;
    QWidget* latestViewContainerWidget() override;
    QWidget* latestViewWidget() override;

    // runtime command host
    std::pair<rtcommand::FindObjectErrCode, Configurable*> findConfigurable(const std::string& path) override;
    bool viewsBusy() override;

    std::vector<std::string> dbFileList() const;
    void clearDBFileList();
    void addDBFileToList(const std::string filename);

    LogStore& logStore();

protected:
    virtual GenerateResult generateSubConfigurable_impl(nlohmann::json& child_json) override;

    Result openDBFileInternal(const std::string& filename);
    Result createNewDBFileInternal(const std::string& filename);
    Result createInMemDBFileInternal(const std::string& future_filename);
    Result createNewDBFileFromMemoryInternal();
    Result exportDBFileInternal(const std::string& filename);
    Result closeDBInternal();

    virtual void checkSubConfigurables() override;

    ConfigurationManager& config_manager_;

    AppState app_state_; // its settings are config parameters of COMPASS

    std::unique_ptr<SimpleConfig> simple_config_;

    Managers managers_; // owned by the composition root

    LogStore log_store_;

    std::unique_ptr<ConfigJSON> config_json_;

    std::string inmem_future_filename_;
    nlohmann::json db_file_list_;

    IMainWindow* main_window_ = nullptr; // owned by the composition root

    std::function<void()> shutdown_sequence_; // ordered shutdown of all managers

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
