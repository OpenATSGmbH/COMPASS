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

#include "compass.h"
#include "imainwindow.h"
#include "iviewprovider.h"
#include "dialogs.h"
#include "config.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "jobmanager.h"
#include "unitmanager.h"
#include "projectionmanager.h"
#include "rtcommand_manager.h"
#include "logger.h"
#include "files.h"
#include "rtcommand_runner.h"
#include "rtcommand_manager.h"
#include "rtcommand.h"
#include "rtcommand_registry.h"
#include "timeconv.h"
#include "db_context_manager.h"
#include "async.h"
#include "licensemanager.h"
#include "configurationmanager.h"
#include "result.h"
#include "dbinstance.h"
#include "system.h"
#include "msghandler.h"
#include "stringconv.h"

#include <QMessageBox>
#include <QApplication>

#include <osgDB/Registry>

#include "global.h"

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;
using namespace nlohmann;
using namespace Utils;

COMPASS::COMPASS(ConfigurationManager& config_manager)
    : Configurable(config_manager.getRootConfigJSON("COMPASS", "COMPASS0").json(), nullptr),
      config_manager_(config_manager),
      log_store_(!AppState::isAppImage())
{
    config_manager_.registerJsonRootConfigurable(*this);
    msghandler::MessageHandler::init(log_store_);
    logdbg;

    std::cout << "APPIMAGE: " << (AppState::isAppImage() ? "yes" : "no") << std::endl;

    // relay the app state signals for the libraries above core (connected first => run before
    // the slots connected to AppState directly)
    connect(&app_state_, &AppState::appModeAboutToSwitchSignal, this, &COMPASS::appModeAboutToSwitchSignal);
    connect(&app_state_, &AppState::appModeSwitchSignal, this, &COMPASS::appModeSwitchSignal);
    connect(&app_state_, &AppState::liveOverloadChangedSignal, this, &COMPASS::liveOverloadChangedSignal);
    connect(&app_state_, &AppState::colorModeChangedSignal, this, &COMPASS::colorModeChangedSignal);
    connect(&app_state_, &AppState::darkModeChangedSignal, this, &COMPASS::darkModeChangedSignal);

    simple_config_.reset(new SimpleConfig("config.json"));

    if (simple_config_->existsId("version"))
        app_state_.appVersion(simple_config_->getString("version"));

    registerParameter("last_db_filename", &app_state_.settings().last_db_filename, std::string());
    registerParameter("db_file_list", &db_file_list_, json::array());

    vector<string> cleaned_file_list;
    // clean missing files

    for (auto& filename : db_file_list_.get<std::vector<string>>())
    {
        if (Files::fileExists(filename))
            cleaned_file_list.push_back(filename);
    }
    db_file_list_ = cleaned_file_list;

    // settings of the app state, as config parameters of COMPASS
    auto& settings = app_state_.settings();

    registerParameter("dark_mode", &settings.dark_mode, false);

    registerParameter("color_mode", &settings.color_mode, 0u);

    registerParameter("last_path", &settings.last_path, {});

    if (!Files::directoryExists(settings.last_path))
        settings.last_path = QDir::homePath().toStdString();

    registerParameter("disable_live_to_offline_switch", &settings.disable_live_to_offline_switch, false);
    registerParameter("disable_menu_config_save", &settings.disable_menu_config_save, false);

    registerParameter("disable_geographicview_rotate", &settings.disable_geographicview_rotate, false);

    registerParameter("disable_add_remove_views", &settings.disable_add_remove_views, false);

    registerParameter("max_live_data_age_cache", &settings.max_live_data_age_cache, 5u);
    registerParameter("max_live_data_age_db", &settings.max_live_data_age_db, 60u);

    registerParameter("live_overload_max_packets", &settings.live_overload_max_packets, 4u);
    registerParameter("live_overload_max_latency_s", &settings.live_overload_max_latency_s, -1.0);
    registerParameter("live_overload_show_debug_info", &settings.live_overload_show_debug_info, false);

    registerParameter("auto_live_running_resume_ask_time", &settings.auto_live_running_resume_ask_time, 60u);
    registerParameter("auto_live_running_resume_ask_wait_time", &settings.auto_live_running_resume_ask_wait_time, 1u);

    registerParameter("disable_confirm_reset_views", &settings.disable_confirm_reset_views, false);

    registerParameter("min_app_width", &settings.min_app_width, settings.min_app_width);
    if (settings.min_app_width > 1600)
        settings.min_app_width = 1600;

    registerParameter("min_app_height", &settings.min_app_height, settings.min_app_height);
    if (settings.min_app_height > 800)
        settings.min_app_height = 800;

    registerParameter("app_font_scale", &settings.app_font_scale, settings.app_font_scale);

    registerParameter("disable_native_dialogs", &settings.disable_native_dialogs, settings.disable_native_dialogs);


    registerParameter("unspecific_acids", &settings.unspecific_acids, {"00000000","????????","        "});
    //registerParameter("unspecific_mode_3as", &unspecific_mode_3as_, {});

    traced_assert(settings.auto_live_running_resume_ask_time > 0);
    traced_assert(settings.auto_live_running_resume_ask_wait_time > 0);
    traced_assert(settings.auto_live_running_resume_ask_time > settings.auto_live_running_resume_ask_wait_time);

    logdbg << "end";
}

/**
 * Registers the creation of a manager above the core layer (sub-configurable of COMPASS).
 * Notice: must be called before createManagers(), in the order the managers are created when missing.
 */
void COMPASS::registerManagerCreator(const std::string& class_name,
                                     const std::string& instance_id,
                                     const std::function<void(nlohmann::json&)>& create)
{
    traced_assert(!managers_created_);
    traced_assert(create);

    ManagerCreator creator;
    creator.class_name  = class_name;
    creator.instance_id = instance_id;
    creator.create      = create;

    manager_creators_.push_back(creator);
}

/**
 * Creates the sub-configurable managers from the configuration, via the registered creators.
 * Notice: the composition root registers the creators before and starts and wires the managers after.
 */
void COMPASS::createManagers()
{
    traced_assert(!managers_created_);

    createSubConfigurables();

    managers_created_ = true;

    qRegisterMetaType<AppMode>("AppMode");

    rtcommand::init_basic_commands(rtcommand::RTCommandRegistry::instance());

    app_state_.appMode(app_state_.appMode());
}

/**
 */
void COMPASS::setMainWindow(IMainWindow* main_window)
{
    main_window_ = main_window;
}

/**
 */
void COMPASS::setShutdownSequence(const std::function<void()>& sequence)
{
    shutdown_sequence_ = sequence;
}

COMPASS::~COMPASS()
{
    logdbg;

    if (!app_state_.isShutDown())
    {
        logerr << "not shut down";
        shutdown();
    }

    config_manager_.unregisterJsonRootConfigurable(*this);

    // the composition root destroys all managers before COMPASS (they are sub-configurables)
    traced_assert(!managers_.db_interface);
    traced_assert(!managers_.dbcontent_manager);
    traced_assert(!managers_.context_manager);
    traced_assert(!managers_.license_manager);
    traced_assert(!managers_.projection_manager);
    traced_assert(!managers_.rt_cmd_manager);
    traced_assert(!managers_.view_provider);

    logdbg << "end";
}

std::string COMPASS::getPath() const
{
    return "";
}

void COMPASS::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    logdbg << "class_name " << class_name;
    if (class_name == "DataSourceManager")
    {
        // legacy - DataSourceManager replaced by DBContextManager, ignore config
        loginf << "ignoring legacy DataSourceManager config";
    }
    else if (class_name == "FFTManager")
    {
        // legacy - FFTManager replaced by DBContextManager, ignore config
        loginf << "ignoring legacy FFTManager config";
    }
    else
    {
        // managers, created by the composition root
        for (auto& creator : manager_creators_)
        {
            if (creator.class_name != class_name)
                continue;

            traced_assert(!creator.created);
            creator.create(child_json);
            creator.created = true;

            return;
        }

        throw std::runtime_error("COMPASS: generateSubConfigurable: unknown class_name " + class_name);
    }
}

/**
 * Creates the managers missing in the configuration, in the registration order of their creators.
 */
void COMPASS::checkSubConfigurables()
{
    for (auto& creator : manager_creators_)
    {
        if (creator.created)
            continue;

        generateSubConfigurableFromConfig(creator.class_name, creator.instance_id);
        traced_assert(creator.created);
    }
}

LogStore& COMPASS::logStore()
{
    return log_store_;
}

bool COMPASS::openDBFile(const std::string& filename)
{
    loginf << "opening file '" << filename << "'";

    traced_assert(!app_state_.dbOpened());

    QApplication::setOverrideCursor(Qt::WaitCursor);

    auto result = openDBFileInternal(filename);

    app_state_.lastUsedPath(Files::getDirectoryFromPath(filename));

    QApplication::restoreOverrideCursor();

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));
    else
        addDBFileToList(filename);

    return result.ok();
}

Result COMPASS::openDBFileInternal(const std::string& filename)
{
    traced_assert(managers_.db_interface);

    if (dbOpened())
        return Result::failed("Database already open");

    app_state_.settings().last_db_filename = filename;
    app_state_.dbInMem(false);

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBFile(filename, false);
        traced_assert(managers_.db_interface->ready());

        app_state_.dbOpened(true);

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }  
    catch (std::exception& e)
    {
        res = Result::failed(e.what());

        app_state_.dbOpened(false);
    }

    return res;
}

bool COMPASS::createNewDBFile(const std::string& filename)
{
    loginf << "creating new file '" << filename << "'";

    traced_assert(!app_state_.dbOpened());

    auto result = createNewDBFileInternal(filename);

    app_state_.lastUsedPath(Files::getDirectoryFromPath(filename));

    if (!result.ok())
    {
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));
    }
    else
    {
        addDBFileToList(filename);
    }

    return result.ok();
}

Result COMPASS::createNewDBFileInternal(const std::string& filename)
{
    traced_assert(managers_.db_interface);

    if (dbOpened())
    {
        log_store_.logError("COMPASS") << "Database '" << filename
                            << "' creation failed: Database already open";

        return Result::failed("Database already open");
    }

    app_state_.settings().last_db_filename = filename;
    app_state_.dbInMem(false);

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBFile(filename, true);
        traced_assert(managers_.db_interface->ready());

        app_state_.dbOpened(true);

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }
    catch(const std::exception& e)
    {
        res = Result::failed(e.what());

        app_state_.dbOpened(false);
    }

    return res;
}

bool COMPASS::createInMemDBFile(const std::string& future_filename)
{
    loginf << "future filename '" << future_filename << "'";

    traced_assert(!app_state_.dbOpened());

    auto result = createInMemDBFileInternal(future_filename);

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));

    return result.ok();
}

Result COMPASS::createInMemDBFileInternal(const std::string& future_filename)
{
    traced_assert(managers_.db_interface);

    if (dbOpened())
        return Result::failed("Database already open");

    inmem_future_filename_ = future_filename;
    app_state_.settings().last_db_filename = DBInstance::InMemFilename;
    app_state_.dbInMem(true);

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBInMemory();
        traced_assert(managers_.db_interface->ready());

        app_state_.dbOpened(true);

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }
    catch(const std::exception& e)
    {
        res = Result::failed(e.what());

        app_state_.dbOpened(false);
    }

    return res;
}

bool COMPASS::createNewDBFileFromMemory()
{
    loginf << "filename '" << inmem_future_filename_ << "'";

    traced_assert(canCreateDBFileFromMemory());

    QMessageBox* msg_box = new QMessageBox(Dialogs::statusDialogParent());

    msg_box->setWindowTitle("Exporting Database");
    msg_box->setText("Please wait ...");
    msg_box->setStandardButtons(QMessageBox::NoButton);
    msg_box->setWindowModality(Qt::ApplicationModal);
    // do not steal os focus from other applications when popping up
    msg_box->setAttribute(Qt::WA_ShowWithoutActivating, true);
    msg_box->show();

    Async::waitAndProcessEventsFor(50);

    auto result = createNewDBFileFromMemoryInternal();

    //@TODO: filename should be set as last path?

    msg_box->close();
    delete msg_box;

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));
    else
        addDBFileToList(app_state_.settings().last_db_filename);
    
    return result.ok();
}

Result COMPASS::createNewDBFileFromMemoryInternal()
{
    //check first
    if (!canCreateDBFileFromMemory())
        return Result::failed("Cannot create database from memory in current state");

    Result res;
    
    //export mem db to file
    res = exportDBFileInternal(inmem_future_filename_);
    if (!res.ok())
        return res;

    //close mem db
    res = closeDBInternal();
    if (!res.ok())
        return res;

    //open exportted file db
    res = openDBFileInternal(inmem_future_filename_);
    if (!res.ok())
        return res;
    
    return Result::succeeded();
}

bool COMPASS::exportDBFile(const std::string& filename)
{
    loginf << "exporting as file '" << filename << "'";

    traced_assert(app_state_.dbOpened());
    traced_assert(!app_state_.dbExportInProgress());

    QMessageBox* msg_box = new QMessageBox(Dialogs::statusDialogParent());

    msg_box->setWindowTitle("Exporting Database");
    msg_box->setText("Please wait ...");
    msg_box->setStandardButtons(QMessageBox::NoButton);
    msg_box->setWindowModality(Qt::ApplicationModal);
    // do not steal os focus from other applications when popping up
    msg_box->setAttribute(Qt::WA_ShowWithoutActivating, true);
    msg_box->show();

    Async::waitAndProcessEventsFor(50);

    auto result = exportDBFileInternal(filename);

    app_state_.lastUsedPath(Files::getDirectoryFromPath(filename));
    
    msg_box->close();
    delete msg_box;

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));

    return result.ok();
}

Result COMPASS::exportDBFileInternal(const std::string& filename)
{
    traced_assert(managers_.db_interface);

    if (!app_state_.dbOpened())
        return Result::failed("No database opened");
    if (app_state_.dbExportInProgress())
        return Result::failed("Export already in progress");
    
    app_state_.dbExportInProgress(true);

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->exportDBFile(filename);
    }
    catch(const std::exception& ex)
    {
        res = Result::failed(ex.what());
    }

    app_state_.dbExportInProgress(false);

    return res;
}

bool COMPASS::closeDB()
{
    loginf << "closing db file '" << app_state_.settings().last_db_filename << "'";

    traced_assert(app_state_.dbOpened());

    auto result = closeDBInternal();

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));

    return result.ok();
}

Result COMPASS::closeDBInternal()
{
    if (!app_state_.dbOpened())
        return Result::failed("No database opened");

    Result res = Result::succeeded();

    try
    {
        managers_.dbcontent_manager->saveTargets();
        managers_.context_manager->saveCountsToDB();
        managers_.context_manager->saveAsterixInfoToDB();

        // a job still in flight would keep working on the interface closed below
        managers_.dbcontent_manager->waitUntilEngineIdle();

        managers_.db_interface->closeDB();
        traced_assert(!managers_.db_interface->ready());

        app_state_.dbOpened(false);
        app_state_.dbInMem(false);

        emit databaseClosedSignal();
        log_store_.setLogCallbacks({}, {});
    }
    catch(const std::exception& ex)
    {
        res = Result::failed(ex.what());
    }
    
    return res;
}

DBInterface& COMPASS::dbInterface()
{
    traced_assert(managers_.db_interface);
    return *managers_.db_interface;
}

DBContentManager& COMPASS::dbContentManager()
{
    traced_assert(managers_.dbcontent_manager);
    return *managers_.dbcontent_manager;
}

QWidget* COMPASS::viewContainerWidget(const std::string& name)
{
    return viewProvider().viewContainerWidget(name);
}

QWidget* COMPASS::latestViewContainerWidget()
{
    return viewProvider().latestViewContainerWidget();
}

QWidget* COMPASS::latestViewWidget()
{
    return viewProvider().latestViewWidget();
}

/**
 * Finds a configurable below COMPASS (the root configurable) by its path.
 */
std::pair<rtcommand::FindObjectErrCode, Configurable*> COMPASS::findConfigurable(const std::string& path)
{
    return findSubConfigurablePath(path);
}

/**
 * True while a view dispatch is in flight or view processing is pending, so runtime commands
 * are deferred to the event loop instead of waiting here (see RTCommandRunnerStash).
 * Notice: a shutdown tears the views down anyway, and its event loop is going away -
 * deferring against it would leave the runner thread retrying forever.
 */
bool COMPASS::viewsBusy()
{
    if (app_state_.isShutDown())
        return false;

    IViewProvider& view_man = viewProvider();

    return view_man.hasPendingViewProcessing() || view_man.isProcessingData();
}

SimpleConfig& COMPASS::config()
{
    traced_assert(simple_config_);
    return *simple_config_;
}

context::DBContextManager& COMPASS::dbContextManager()
{
    traced_assert(managers_.context_manager);
    return *managers_.context_manager;
}

bool COMPASS::hasActiveContext() const
{
    return managers_.context_manager && managers_.context_manager->hasActiveContext();
}

context::DBContext& COMPASS::context()
{
    return managers_.context_manager->activeContext();
}

const context::DBContext& COMPASS::context() const
{
    return managers_.context_manager->activeContext();
}

LicenseManager& COMPASS::licenseManager()
{
    traced_assert(managers_.license_manager);
    return *managers_.license_manager;
}

ProjectionManager& COMPASS::projectionManager()
{
    traced_assert(managers_.projection_manager);
    return *managers_.projection_manager;
}

RTCommandManager& COMPASS::rtCommandManager()
{
    traced_assert(managers_.rt_cmd_manager);
    return *managers_.rt_cmd_manager;
}

/**
 */
IViewProvider& COMPASS::viewProvider()
{
    traced_assert(managers_.view_provider);
    return *managers_.view_provider;
}

const DBInterface& COMPASS::dbInterface() const
{
    traced_assert(managers_.db_interface);
    return *managers_.db_interface;
}

const DBContentManager& COMPASS::dbContentManager() const
{
    traced_assert(managers_.dbcontent_manager);
    return *managers_.dbcontent_manager;
}

const SimpleConfig& COMPASS::config() const
{
    traced_assert(simple_config_);
    return *simple_config_;
}

const LicenseManager& COMPASS::licenseManager() const
{
    traced_assert(managers_.license_manager);
    return *managers_.license_manager;
}

const ProjectionManager& COMPASS::projectionManager() const
{
    traced_assert(managers_.projection_manager);
    return *managers_.projection_manager;
}

const RTCommandManager& COMPASS::rtCommandManager() const
{
    traced_assert(managers_.rt_cmd_manager);
    return *managers_.rt_cmd_manager;
}

bool COMPASS::dbOpened()
{
    return app_state_.dbOpened();
}

bool COMPASS::dbInMem() const
{
    return app_state_.dbInMem();
}

bool COMPASS::canCreateDBFileFromMemory() const
{
    return app_state_.dbOpened() && app_state_.dbInMem() && managers_.db_interface && managers_.db_interface->canCreateDBFileFromMemory() && !inmem_future_filename_.empty();
}

/**
 * Shuts down all managers in the order given by the composition root's shutdown sequence.
 */
void COMPASS::shutdown()
{
    loginf << "shutdown";

    if (app_state_.isShutDown())
    {
        logerr << "already shut down";
        return;
    }

    app_state_.phase(AppPhase::Shutdown);

    traced_assert(shutdown_sequence_);
    shutdown_sequence_();

    loginf << "done. goodbye.";
}

/**
 */
IMainWindow& COMPASS::mainWindow()
{
    traced_assert(main_window_);
    return *main_window_;
}

std::vector<std::string> COMPASS::dbFileList() const
{
    return db_file_list_.get<std::vector<string>>();
}

void COMPASS::clearDBFileList()
{
    db_file_list_.clear();
}

void COMPASS::addDBFileToList(const std::string filename)
{
    vector<string> tmp_list = db_file_list_.get<std::vector<string>>();

    if (find(tmp_list.begin(), tmp_list.end(), filename) == tmp_list.end())
    {
        loginf << "adding filename '" << filename << "'";

        tmp_list.push_back(filename);

        sort(tmp_list.begin(), tmp_list.end());

        db_file_list_ = tmp_list;
    }
}


