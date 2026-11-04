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

const bool COMPASS::is_app_image_ = {Utils::System::appDir() != nullptr};

COMPASS::COMPASS(ConfigurationManager& config_manager)
    : Configurable(config_manager.getRootConfigJSON("COMPASS", "COMPASS0").json(), nullptr),
      config_manager_(config_manager),
      log_store_(!is_app_image_)
{
    config_manager_.registerJsonRootConfigurable(*this);
    msghandler::MessageHandler::init(log_store_);
    logdbg;

    std::cout << "APPIMAGE: " << (is_app_image_ ? "yes" : "no") << std::endl;

    pdflatex_found_ = System::exec("which pdflatex").size(); // empty if none

    simple_config_.reset(new SimpleConfig("config.json"));

    registerParameter("last_db_filename", &last_db_filename_, std::string());
    registerParameter("db_file_list", &db_file_list_, json::array());

    vector<string> cleaned_file_list;
    // clean missing files

    for (auto& filename : db_file_list_.get<std::vector<string>>())
    {
        if (Files::fileExists(filename))
            cleaned_file_list.push_back(filename);
    }
    db_file_list_ = cleaned_file_list;

    registerParameter("dark_mode", &dark_mode_, false);

    registerParameter("color_mode", &color_mode_, 0u);

    registerParameter("last_path", &last_path_, {});

    if (!Files::directoryExists(last_path_))
        last_path_ = QDir::homePath().toStdString();

    registerParameter("disable_live_to_offline_switch", &disable_live_to_offline_switch_, false);
    registerParameter("disable_menu_config_save", &disable_menu_config_save_, false);

    registerParameter("disable_geographicview_rotate", &disable_geographicview_rotate_, false);

    registerParameter("disable_add_remove_views", &disable_add_remove_views_, false);

    registerParameter("max_live_data_age_cache", &max_live_data_age_cache_, 5u);
    registerParameter("max_live_data_age_db", &max_live_data_age_db_, 60u);

    registerParameter("live_overload_max_packets", &live_overload_max_packets_, 4u);
    registerParameter("live_overload_max_latency_s", &live_overload_max_latency_s_, -1.0);
    registerParameter("live_overload_show_debug_info", &live_overload_show_debug_info_, false);

    registerParameter("auto_live_running_resume_ask_time", &auto_live_running_resume_ask_time_, 60u);
    registerParameter("auto_live_running_resume_ask_wait_time", &auto_live_running_resume_ask_wait_time_, 1u);

    registerParameter("disable_confirm_reset_views", &disable_confirm_reset_views_, false);

    registerParameter("min_app_width", &min_app_width_, min_app_width_);
    if (min_app_width_ > 1600)
        min_app_width_ = 1600;
    
    registerParameter("min_app_height", &min_app_height_, min_app_height_);
    if (min_app_height_ > 800)
        min_app_height_ = 800;

    registerParameter("app_font_scale", &app_font_scale_, app_font_scale_);

    registerParameter("disable_native_dialogs", &disable_native_dialogs_, disable_native_dialogs_);


    registerParameter("unspecific_acids", &unspecific_acids_, {"00000000","????????","        "});
    //registerParameter("unspecific_mode_3as", &unspecific_mode_3as_, {});

    traced_assert(auto_live_running_resume_ask_time_ > 0);
    traced_assert(auto_live_running_resume_ask_wait_time_ > 0);
    traced_assert(auto_live_running_resume_ask_time_ > auto_live_running_resume_ask_wait_time_);

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

    rtcommand::RTCommandHelp::init();

    appMode(app_mode_);
}

/**
 */
void COMPASS::setMainWindow(IMainWindow* main_window)
{
    main_window_ = main_window;
}

/**
 */
void COMPASS::setLiveImportBacklogSizeFunc(const std::function<unsigned int()>& func)
{
    live_import_backlog_size_func_ = func;
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

    if (app_state_ != AppState::Shutdown)
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
    traced_assert(!managers_.job_manager);
    traced_assert(!managers_.unit_manager);
    traced_assert(!managers_.projection_manager);
    traced_assert(!managers_.rt_cmd_manager);
    traced_assert(!managers_.rt_cmd_runner);
    traced_assert(!managers_.view_provider);
    traced_assert(!managers_.filter_provider);
    traced_assert(!managers_.eval_provider);
    traced_assert(!managers_.task_provider);

    logdbg << "end";
}

void COMPASS::setAppState(AppState state)
{
    app_state_ = state;

    //notify someone about changed app state?
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

bool COMPASS::sensorStatusTimeHack() const
{
    return sensor_status_time_hack_;
}
void COMPASS::sensorStatusTimeHack(bool value)
{
    loginf << "value " << value;

    sensor_status_time_hack_ = value;
}

bool COMPASS::disableNativeDialogs() const
{
    return disable_native_dialogs_;
}

float COMPASS::appFontScale() const
{
    return app_font_scale_;
}

bool COMPASS::isUnspecificACID(const std::string& acid) const
{
    // whitespace-only identifications carry no information
    if (Utils::String::trim(acid).empty())
        return true;

    // compare against the raw configured forms (e.g. "00000000", "????????")
    for (const auto& it : unspecific_acids_)
    {
        if (it.is_string() && it.get<std::string>() == acid)
            return true;
    }

    return false;
}

unsigned int COMPASS::minAppHeight() const
{
    return min_app_height_;
}

unsigned int COMPASS::minAppWidth() const
{
    return min_app_width_;
}

bool COMPASS::openDBFile(const std::string& filename)
{
    loginf << "opening file '" << filename << "'";

    traced_assert(!db_opened_);

    QApplication::setOverrideCursor(Qt::WaitCursor);

    auto result = openDBFileInternal(filename);

    lastUsedPath(Files::getDirectoryFromPath(filename));

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

    last_db_filename_ = filename;
    db_inmem_ = false;

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBFile(filename, false);
        traced_assert(managers_.db_interface->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }  
    catch (std::exception& e)
    {
        res = Result::failed(e.what());

        db_opened_ = false;
    }

    return res;
}

bool COMPASS::createNewDBFile(const std::string& filename)
{
    loginf << "creating new file '" << filename << "'";

    traced_assert(!db_opened_);

    auto result = createNewDBFileInternal(filename);

    lastUsedPath(Files::getDirectoryFromPath(filename));

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
        logError("COMPASS") << "Database '" << filename
                            << "' creation failed: Database already open";

        return Result::failed("Database already open");
    }

    last_db_filename_ = filename;
    db_inmem_ = false;

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBFile(filename, true);
        traced_assert(managers_.db_interface->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }
    catch(const std::exception& e)
    {
        res = Result::failed(e.what());

        db_opened_ = false;
    }

    return res;
}

bool COMPASS::createInMemDBFile(const std::string& future_filename)
{
    loginf << "future filename '" << future_filename << "'";

    traced_assert(!db_opened_);

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
    last_db_filename_ = DBInstance::InMemFilename;
    db_inmem_ = true;

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->openDBInMemory();
        traced_assert(managers_.db_interface->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { managers_.db_interface->saveTaskLogInfo(id, j); },
            [this]() { return managers_.db_interface->loadTaskLogInfo(); });
        emit databaseOpenedSignal();
    }
    catch(const std::exception& e)
    {
        res = Result::failed(e.what());

        db_opened_ = false;
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
        addDBFileToList(last_db_filename_);
    
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

    traced_assert(db_opened_);
    traced_assert(!db_export_in_progress_);

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

    lastUsedPath(Files::getDirectoryFromPath(filename));
    
    msg_box->close();
    delete msg_box;

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));

    return result.ok();
}

Result COMPASS::exportDBFileInternal(const std::string& filename)
{
    traced_assert(managers_.db_interface);

    if (!db_opened_)
        return Result::failed("No database opened");
    if (db_export_in_progress_)
        return Result::failed("Export already in progress");
    
    db_export_in_progress_ = true;

    Result res = Result::succeeded();

    try
    {
        managers_.db_interface->exportDBFile(filename);
    }
    catch(const std::exception& ex)
    {
        res = Result::failed(ex.what());
    }

    db_export_in_progress_ = false;

    return res;
}

bool COMPASS::closeDB()
{
    loginf << "closing db file '" << last_db_filename_ << "'";

    traced_assert(db_opened_);

    auto result = closeDBInternal();

    if (!result.ok())
        QMessageBox::critical(QApplication::activeWindow(), "Error", QString::fromStdString(result.error()));

    return result.ok();
}

Result COMPASS::closeDBInternal()
{
    if (!db_opened_)
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

        db_opened_ = false;
        db_inmem_ = false;

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

rtcommand::RTCommandRunner& COMPASS::rtCmdRunner()
{
    traced_assert(managers_.rt_cmd_runner);
    return *managers_.rt_cmd_runner;
}

JobManager& COMPASS::jobManager()
{
    traced_assert(managers_.job_manager);
    return *managers_.job_manager;
}

UnitManager& COMPASS::unitManager()
{
    traced_assert(managers_.unit_manager);
    return *managers_.unit_manager;
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

/**
 */
IFilterProvider& COMPASS::filterProvider()
{
    traced_assert(managers_.filter_provider);
    return *managers_.filter_provider;
}

/**
 */
IEvalProvider& COMPASS::evalProvider()
{
    traced_assert(managers_.eval_provider);
    return *managers_.eval_provider;
}

/**
 */
ITaskProvider& COMPASS::taskProvider()
{
    traced_assert(managers_.task_provider);
    return *managers_.task_provider;
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

const rtcommand::RTCommandRunner& COMPASS::rtCmdRunner() const
{
    traced_assert(managers_.rt_cmd_runner);
    return *managers_.rt_cmd_runner;
}

const LicenseManager& COMPASS::licenseManager() const
{
    traced_assert(managers_.license_manager);
    return *managers_.license_manager;
}

const JobManager& COMPASS::jobManager() const
{
    traced_assert(managers_.job_manager);
    return *managers_.job_manager;
}

const UnitManager& COMPASS::unitManager() const
{
    traced_assert(managers_.unit_manager);
    return *managers_.unit_manager;
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
    return db_opened_;
}

bool COMPASS::dbInMem() const
{
    return db_inmem_;
}

bool COMPASS::canCreateDBFileFromMemory() const
{
    return db_opened_ && db_inmem_ && managers_.db_interface && managers_.db_interface->canCreateDBFileFromMemory() && !inmem_future_filename_.empty();
}

/**
 * Shuts down all managers in the order given by the composition root's shutdown sequence.
 */
void COMPASS::shutdown()
{
    loginf << "shutdown";

    if (app_state_ == AppState::Shutdown)
    {
        logerr << "already shut down";
        return;
    }

    app_state_ = AppState::Shutdown;

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

std::string COMPASS::lastUsedPath()
{
    loginf << "return '" << last_path_ << "'";
    return last_path_;
}

void COMPASS::lastUsedPath(const std::string& last_path)
{
    loginf << "set '" << last_path << "'";
    last_path_ = last_path;
}

bool COMPASS::darkMode() const
{
    return dark_mode_;
}

void COMPASS::darkMode(bool value)
{
    if (dark_mode_ == value)
        return;

    dark_mode_ = value;
    emit darkModeChangedSignal(dark_mode_);
}

void COMPASS::colorMode(unsigned int value)
{
    if (color_mode_ == value)
        return;

    color_mode_ = value;
    emit colorModeChangedSignal(color_mode_);
}

const char* COMPASS::lineEditInvalidStyle()
{
    if (dark_mode_)
        return "QLineEdit { background: rgb(255, 50, 50); selection-background-color:"
                          " rgb(255, 100, 100); }";
    else
        return "QLineEdit { background: rgb(255, 100, 100); selection-background-color:"
                          " rgb(255, 200, 200); }";
}

LogStream COMPASS::logInfo(const std::string& component,
                           boost::optional<unsigned int> error_code, nlohmann::json json_blob) 
{
    return log_store_.logInfo(component, error_code, json_blob);
}

LogStream COMPASS::logWarn(const std::string& component,
                           boost::optional<unsigned int> error_code, nlohmann::json json_blob) 
{
    return log_store_.logWarn(component, error_code, json_blob);
}

LogStream COMPASS::logError(const std::string& component,
                            boost::optional<unsigned int> error_code, nlohmann::json json_blob) 
{
    return log_store_.logError(component, error_code, json_blob);
}

bool COMPASS::disableConfirmResetViews() const
{
    return disable_confirm_reset_views_;
}

unsigned int COMPASS::autoLiveRunningResumeAskWaitTime() const
{
    return auto_live_running_resume_ask_wait_time_;
}

unsigned int COMPASS::autoLiveRunningResumeAskTime() const
{
    return auto_live_running_resume_ask_time_;
}

bool COMPASS::dbExportInProgress() const
{
    return db_export_in_progress_;
}

bool COMPASS::disableAddRemoveViews() const
{
    return disable_add_remove_views_;
}

bool COMPASS::disableGeographicViewRotate() const
{
    return disable_geographicview_rotate_;
}

bool COMPASS::disableMenuConfigSave() const
{
    return disable_menu_config_save_;
}

bool COMPASS::pdflatexFound() const
{
    return pdflatex_found_;
}

bool COMPASS::disableLiveToOfflineSwitch() const
{
    return disable_live_to_offline_switch_;
}

bool COMPASS::isShutDown() const
{
    return (app_state_ == AppState::Shutdown);
}

bool COMPASS::isRunning() const
{
    return (app_state_ == AppState::Running);
}

bool COMPASS::expertMode() const
{
    return expert_mode_;
}

void COMPASS::expertMode(bool expert_mode)
{
    loginf << "setting expert mode " << expert_mode;

    expert_mode_ = expert_mode;
}

AppMode COMPASS::appMode() const
{
    return app_mode_;
}

void COMPASS::appMode(const AppMode& app_mode)
{
    if (app_mode_ != app_mode)
    {
        AppMode last_app_mode = app_mode_;

        app_mode_ = app_mode;

        loginf << "app_mode_current " << toString(app_mode_)
               << " previous " << toString(last_app_mode);

        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        // Live entry / pause / resume / exit all reduce to the same two steps: switch
        // ASTERIX ingestion (importer first, so no trailing inserts land after a pause),
        // then flip the display. Within a live session ViewManager keeps the LiveDataFeed as
        // the current source across running+paused, so pause = freeze and resume = unfreeze;
        // no reload or cache prime is needed - the feed already holds the data.
        // The importer is connected to appModeAboutToSwitchSignal => it runs before all
        // slots of appModeSwitchSignal.
        emit appModeAboutToSwitchSignal(last_app_mode, app_mode_);
        emit appModeSwitchSignal(last_app_mode, app_mode_);

        QApplication::restoreOverrideCursor();
    }
}

/**
 */
void COMPASS::liveOverloaded(bool overloaded)
{
    if (live_overloaded_ == overloaded)
        return;

    live_overloaded_ = overloaded;

    loginf << "live overload " << (overloaded ? "detected" : "cleared");

    emit liveOverloadChangedSignal(overloaded);
}

/**
 */
unsigned int COMPASS::liveImportBacklogSize() const
{
    traced_assert(live_import_backlog_size_func_);
    return live_import_backlog_size_func_();
}

std::string COMPASS::appModeStr() const
{
    if (!appModes2Strings().count(app_mode_))
    {
        std::cout << "COMPASS: appModeStr: unkown type " << (unsigned int) app_mode_ << std::endl;
        logerr << "unkown type " << (unsigned int) app_mode_;
    }

    traced_assert(appModes2Strings().count(app_mode_) > 0);
    return appModes2Strings().at(app_mode_);
}

const std::map<AppMode, std::string>& COMPASS::appModes2Strings()
{
    static const auto* map = new std::map<AppMode, std::string>
    {{AppMode::Offline, "Offline Mode"},
        {AppMode::LiveRunning, "Live Mode: Running"},
        {AppMode::LivePaused, "Live Mode: Paused"}};

    return *map;
}

std::string COMPASS::lastDbFilename() const
{
    return last_db_filename_;
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

std::string COMPASS::versionString(bool open_ats, 
                                   bool license_type) const
{
    traced_assert(simple_config_->existsId("version"));
    std::string version = simple_config_->getString("version");

    std::string version_str;
    if (open_ats) 
        version_str += "OpenATS ";

    version_str += "COMPASS v" + version;

    if (license_type)
    {
        const auto& license_manager = *managers_.license_manager;
        auto vl = license_manager.activeLicense();

        if (vl)
            version_str += " " + license::License::typeToString(vl->type);
        else
            version_str += " " + license::License::typeToString(license::License::Type::Free);
    }

    return version_str;
}

std::string COMPASS::licenseeString(bool licensed_to) const
{
    const auto& license_manager = *managers_.license_manager;
    auto vl = license_manager.activeLicense();

    if (!vl)
        return "";

    return (licensed_to ? "Licensed to " : "") + vl->licensee;
}


