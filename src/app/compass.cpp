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
#include "dialogs.h"
#include "config.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "filtermanager.h"
#include "jobmanager.h"
#include "unitmanager.h"
#include "projectionmanager.h"
#include "rtcommand_manager.h"
#include "logger.h"
#include "taskmanager.h"
#include "viewmanager.h"
#include "viewcontainerwidget.h"
#include "view.h"
#include "viewwidget.h"
#include "evaluationmanager.h"
#include "mainwindow.h"
#include "files.h"
#include "asteriximporttask.h"
#include "reconstructortask.h"
#include "rtcommand_runner.h"
#include "rtcommand_manager.h"
#include "rtcommand.h"
#include "util/timeconv.h"
#include "db_context_manager.h"
#include "util/async.h"
#include "licensemanager.h"
#include "dbcontentmanagervariableresolver.h"
#include "configurationmanager.h"
#include "result.h"
#include "dbinstance.h"
#include "logwidget.h"
#include "util/system.h"
#include "util/msghandler.h"
#include "util/stringconv.h"

#include <QMessageBox>
#include <QApplication>

#include <osgDB/Registry>

#include "global.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geo_view_api.h"
#endif

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

    createSubConfigurables();

    traced_assert(job_manager_);
    job_manager_->start();
    traced_assert(rt_cmd_manager_);
    rt_cmd_manager_->start();

    traced_assert(db_interface_);
    traced_assert(dbcontent_manager_);
    traced_assert(filter_manager_);
    traced_assert(task_manager_);
    traced_assert(view_manager_);
    traced_assert(eval_manager_);
    traced_assert(license_manager_);

    rt_cmd_runner_.reset(new rtcommand::RTCommandRunner(*this));

    // DBContextManager - not a Configurable, created directly
    context_manager_.reset(new context::DBContextManager(*this));

    // database opening & closing - DBContextManager connects FIRST

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     context_manager_.get(), &context::DBContextManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     context_manager_.get(), &context::DBContextManager::databaseClosedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     dbcontent_manager_.get(), &DBContentManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     dbcontent_manager_.get(), &DBContentManager::databaseClosedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     filter_manager_.get(), &FilterManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     filter_manager_.get(), &FilterManager::databaseClosedSlot);

    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     filter_manager_.get(), &FilterManager::dataSourcesChangedSlot);
    QObject::connect(context_manager_.get(), &context::DBContextManager::dataSourcesChangedSignal,
                     filter_manager_.get(), &FilterManager::dataSourcesChangedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     view_manager_.get(), &ViewManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     view_manager_.get(), &ViewManager::databaseClosedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     eval_manager_.get(), &EvaluationManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     eval_manager_.get(), &EvaluationManager::databaseClosedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     task_manager_.get(), &TaskManager::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     task_manager_.get(), &TaskManager::databaseClosedSlot);

    QObject::connect(this, &COMPASS::databaseOpenedSignal,
                     &log_store_, &LogStore::databaseOpenedSlot);
    QObject::connect(this, &COMPASS::databaseClosedSignal,
                     &log_store_, &LogStore::databaseClosedSlot);

    // data sources changed
    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     eval_manager_.get(), &EvaluationManager::dataSourcesChangedSlot);
    QObject::connect(context_manager_.get(), &context::DBContextManager::dataSourcesChangedSignal,
                     eval_manager_.get(), &EvaluationManager::dataSourcesChangedSlot);

    // sectors changed (direct edits within active context)
    connect (context_manager_.get(), &context::DBContextManager::sectorsChangedSignal,
             &task_manager_->reconstructReferencesTask(), &ReconstructorTask::sectorsChangedSlot);
    // context changed (DB open, context switch - sectors come with the new context)
    connect (context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
             &task_manager_->reconstructReferencesTask(), &ReconstructorTask::sectorsChangedSlot);

    // view data flows from the current source (ViewManager owns it); the load bookends
    // (loadingStarted/DoneSignal) are ViewManager's too - driven off the operation it owns.
    // The data-sources status widget connects directly to ViewManager::dataDistributedSignal.

    // appmode
    connect (this, &COMPASS::appModeSwitchSignal,
             filter_manager_.get(), &FilterManager::appModeSwitchSlot);
    // retained no-op hook (live bookends now synthesized in ViewManager::appModeSwitchSlot)
    connect (this, &COMPASS::appModeSwitchSignal,
             dbcontent_manager_.get(), &DBContentManager::appModeSwitchSlot);

    // features
    connect (license_manager_.get(), &LicenseManager::changed, task_manager_.get(), &TaskManager::updateFeatures);
    connect (license_manager_.get(), &LicenseManager::changed, view_manager_.get(), &ViewManager::updateFeatures);

    qRegisterMetaType<AppMode>("AppMode");

    rtcommand::RTCommandHelp::init();

    appMode(app_mode_);

    logdbg << "end";
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

    traced_assert(!dbcontent_manager_);
    traced_assert(!db_interface_);
    traced_assert(!filter_manager_);
    traced_assert(!task_manager_);
    traced_assert(!view_manager_);
    traced_assert(!eval_manager_);
    traced_assert(!license_manager_);

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
    if (class_name == "DBInterface")
    {
        traced_assert(!db_interface_);
        db_interface_.reset(new DBInterface(child_json, *this));
        traced_assert(db_interface_);
    }
    else if (class_name == "DBContentManager")
    {
        traced_assert(!dbcontent_manager_);
        dbcontent_manager_.reset(new DBContentManager(child_json, *this));
        traced_assert(dbcontent_manager_);
    }
    else if (class_name == "DataSourceManager")
    {
        // legacy - DataSourceManager replaced by DBContextManager, ignore config
        loginf << "ignoring legacy DataSourceManager config";
    }
    else if (class_name == "FilterManager")
    {
        traced_assert(!filter_manager_);
        filter_manager_.reset(new FilterManager(child_json, *this));
        traced_assert(filter_manager_);
    }
    else if (class_name == "TaskManager")
    {
        traced_assert(!task_manager_);
        task_manager_.reset(new TaskManager(child_json, *this));
        traced_assert(task_manager_);
    }
    else if (class_name == "ViewManager")
    {
        traced_assert(!view_manager_);
        view_manager_.reset(new ViewManager(child_json, *this));
        traced_assert(view_manager_);
    }
    else if (class_name == "EvaluationManager")
    {
        traced_assert(!eval_manager_);
        eval_manager_.reset(new EvaluationManager(child_json, *this));
        traced_assert(eval_manager_);
    }
    else if (class_name == "FFTManager")
    {
        // legacy - FFTManager replaced by DBContextManager, ignore config
        loginf << "ignoring legacy FFTManager config";
    }
    else if (class_name == "LicenseManager")
    {
        traced_assert(!license_manager_);
        license_manager_.reset(new LicenseManager(child_json, *this));
        traced_assert(license_manager_);
    }
    else if (class_name == "JobManager")
    {
        traced_assert(!job_manager_);
        job_manager_.reset(new JobManager(child_json, this));
    }
    else if (class_name == "UnitManager")
    {
        traced_assert(!unit_manager_);
        unit_manager_.reset(new UnitManager(child_json, this));
    }
    else if (class_name == "ProjectionManager")
    {
        traced_assert(!projection_manager_);
        projection_manager_.reset(new ProjectionManager(child_json, this));
    }
    else if (class_name == "RTCommandManager")
    {
        traced_assert(!rt_cmd_manager_);
        rt_cmd_manager_.reset(new RTCommandManager(child_json, this));
    }
    else
        throw std::runtime_error("COMPASS: generateSubConfigurable: unknown class_name " + class_name);
}

void COMPASS::checkSubConfigurables()
{
    if (!license_manager_)
    {
        generateSubConfigurableFromConfig("LicenseManager", "LicenseManager0");
        traced_assert(license_manager_);
    }
    if (!db_interface_)
    {
        generateSubConfigurableFromConfig("DBInterface", "DBInterface0");
        traced_assert(db_interface_);
    }
    if (!dbcontent_manager_)
    {
        generateSubConfigurableFromConfig("DBContentManager", "DBContentManager0");
        traced_assert(dbcontent_manager_);
    }
    if (!filter_manager_)
    {
        generateSubConfigurableFromConfig("FilterManager", "FilterManager0");
        traced_assert(filter_manager_);
    }
    if (!task_manager_)
    {
        generateSubConfigurableFromConfig("TaskManager", "TaskManager0");
        traced_assert(task_manager_);
    }
    if (!view_manager_)
    {
        generateSubConfigurableFromConfig("ViewManager", "ViewManager0");
        traced_assert(view_manager_);
    }
    if (!eval_manager_)
    {
        generateSubConfigurableFromConfig("EvaluationManager", "EvaluationManager0");
        traced_assert(eval_manager_);
    }
    if (!job_manager_)
        generateSubConfigurableFromConfig("JobManager", "JobManager0");
    if (!unit_manager_)
        generateSubConfigurableFromConfig("UnitManager", "UnitManager0");
    if (!projection_manager_)
        generateSubConfigurableFromConfig("ProjectionManager", "ProjectionManager0");
    if (!rt_cmd_manager_)
        generateSubConfigurableFromConfig("RTCommandManager", "RTCommandManager0");

    // wire up variable resolver for projection manager (breaks core→db dep)
    if (projection_manager_ && dbcontent_manager_ && !var_resolver_)
    {
        var_resolver_ = std::make_unique<DBContentManagerVariableResolver>(*dbcontent_manager_);
        projection_manager_->varResolver(*var_resolver_);
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
    traced_assert(db_interface_);

    if (dbOpened())
        return Result::failed("Database already open");

    last_db_filename_ = filename;
    db_inmem_ = false;

    Result res = Result::succeeded();

    try
    {
        db_interface_->openDBFile(filename, false);
        traced_assert(db_interface_->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { db_interface_->saveTaskLogInfo(id, j); },
            [this]() { return db_interface_->loadTaskLogInfo(); });
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
    traced_assert(db_interface_);

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
        db_interface_->openDBFile(filename, true);
        traced_assert(db_interface_->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { db_interface_->saveTaskLogInfo(id, j); },
            [this]() { return db_interface_->loadTaskLogInfo(); });
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
    traced_assert(db_interface_);

    if (dbOpened())
        return Result::failed("Database already open");

    inmem_future_filename_ = future_filename;
    last_db_filename_ = DBInstance::InMemFilename;
    db_inmem_ = true;

    Result res = Result::succeeded();

    try
    {
        db_interface_->openDBInMemory();
        traced_assert(db_interface_->ready());

        db_opened_ = true;

        log_store_.setLogCallbacks(
            [this](unsigned int id, const nlohmann::json& j) { db_interface_->saveTaskLogInfo(id, j); },
            [this]() { return db_interface_->loadTaskLogInfo(); });
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
    traced_assert(db_interface_);

    if (!db_opened_)
        return Result::failed("No database opened");
    if (db_export_in_progress_)
        return Result::failed("Export already in progress");
    
    db_export_in_progress_ = true;

    Result res = Result::succeeded();

    try
    {
        db_interface_->exportDBFile(filename);
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
        dbcontent_manager_->saveTargets();
        context_manager_->saveCountsToDB();
        context_manager_->saveAsterixInfoToDB();

        // a job still in flight would keep working on the interface closed below
        dbcontent_manager_->waitUntilEngineIdle();

        db_interface_->closeDB();
        traced_assert(!db_interface_->ready());

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
    traced_assert(db_interface_);
    return *db_interface_;
}

DBContentManager& COMPASS::dbContentManager()
{
    traced_assert(dbcontent_manager_);
    return *dbcontent_manager_;
}

FilterManager& COMPASS::filterManager()
{
    traced_assert(filter_manager_);
    return *filter_manager_;
}

TaskManager& COMPASS::taskManager()
{
    traced_assert(task_manager_);
    return *task_manager_;
}

ViewManager& COMPASS::viewManager()
{
    traced_assert(view_manager_);
    return *view_manager_;
}

QWidget* COMPASS::viewContainerWidget(const std::string& name)
{
    traced_assert(view_manager_);
    return view_manager_->containerWidget(name);
}

QWidget* COMPASS::latestViewContainerWidget()
{
    traced_assert(view_manager_);
    return view_manager_->latestViewContainer();
}

QWidget* COMPASS::latestViewWidget()
{
    traced_assert(view_manager_);
    auto* view = view_manager_->latestView();
    if (!view)
        return nullptr;
    return view->getCentralWidget()->findChild<ViewWidget*>();
}

SimpleConfig& COMPASS::config()
{
    traced_assert(simple_config_);
    return *simple_config_;
}

EvaluationManager& COMPASS::evaluationManager()
{
    traced_assert(eval_manager_);
    return *eval_manager_;
}

context::DBContextManager& COMPASS::dbContextManager()
{
    traced_assert(context_manager_);
    return *context_manager_;
}

bool COMPASS::hasActiveContext() const
{
    return context_manager_ && context_manager_->hasActiveContext();
}

context::DBContext& COMPASS::context()
{
    return context_manager_->activeContext();
}

const context::DBContext& COMPASS::context() const
{
    return context_manager_->activeContext();
}

LicenseManager& COMPASS::licenseManager()
{
    traced_assert(license_manager_);
    return *license_manager_;
}

rtcommand::RTCommandRunner& COMPASS::rtCmdRunner()
{
    traced_assert(rt_cmd_runner_);
    return *rt_cmd_runner_;
}

JobManager& COMPASS::jobManager()
{
    traced_assert(job_manager_);
    return *job_manager_;
}

UnitManager& COMPASS::unitManager()
{
    traced_assert(unit_manager_);
    return *unit_manager_;
}

ProjectionManager& COMPASS::projectionManager()
{
    traced_assert(projection_manager_);
    return *projection_manager_;
}

RTCommandManager& COMPASS::rtCommandManager()
{
    traced_assert(rt_cmd_manager_);
    return *rt_cmd_manager_;
}

const DBInterface& COMPASS::dbInterface() const
{
    traced_assert(db_interface_);
    return *db_interface_;
}

const DBContentManager& COMPASS::dbContentManager() const
{
    traced_assert(dbcontent_manager_);
    return *dbcontent_manager_;
}

const FilterManager& COMPASS::filterManager() const
{
    traced_assert(filter_manager_);
    return *filter_manager_;
}

const TaskManager& COMPASS::taskManager() const
{
    traced_assert(task_manager_);
    return *task_manager_;
}

const ViewManager& COMPASS::viewManager() const
{
    traced_assert(view_manager_);
    return *view_manager_;
}

const SimpleConfig& COMPASS::config() const
{
    traced_assert(simple_config_);
    return *simple_config_;
}

const EvaluationManager& COMPASS::evaluationManager() const
{
    traced_assert(eval_manager_);
    return *eval_manager_;
}

const rtcommand::RTCommandRunner& COMPASS::rtCmdRunner() const
{
    traced_assert(rt_cmd_runner_);
    return *rt_cmd_runner_;
}

const LicenseManager& COMPASS::licenseManager() const
{
    traced_assert(license_manager_);
    return *license_manager_;
}

const JobManager& COMPASS::jobManager() const
{
    traced_assert(job_manager_);
    return *job_manager_;
}

const UnitManager& COMPASS::unitManager() const
{
    traced_assert(unit_manager_);
    return *unit_manager_;
}

const ProjectionManager& COMPASS::projectionManager() const
{
    traced_assert(projection_manager_);
    return *projection_manager_;
}

const RTCommandManager& COMPASS::rtCommandManager() const
{
    traced_assert(rt_cmd_manager_);
    return *rt_cmd_manager_;
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
    return db_opened_ && db_inmem_ && db_interface_ && db_interface_->canCreateDBFileFromMemory() && !inmem_future_filename_.empty();
}

void COMPASS::init()
{
    traced_assert(task_manager_);
    task_manager_->init();
}

void COMPASS::shutdown()
{
    loginf << "shutdown";

    if (app_state_ == AppState::Shutdown)
    {
        logerr << "already shut down";
        return;
    }

    app_state_ = AppState::Shutdown;

    // asynchronous view processing (geo geometry builds) may still read manager state -
    // wait it out before any manager is torn down
    if (view_manager_ && view_manager_->hasPendingViewProcessing())
    {
        loginf << "waiting for pending view processing";
        Utils::Async::pumpUntil([this] { return !view_manager_->hasPendingViewProcessing(); });
    }

    traced_assert(task_manager_);
    task_manager_->shutdown();
    task_manager_ = nullptr;

    traced_assert(db_interface_);

    context_manager_ = nullptr;

    traced_assert(dbcontent_manager_);
    if (db_interface_->ready())
        dbcontent_manager_->saveTargets();
    dbcontent_manager_ = nullptr;

    if (job_manager_)
        job_manager_->shutdown();

    traced_assert(eval_manager_);
    eval_manager_->close();
    eval_manager_ = nullptr;

    traced_assert(view_manager_);
    view_manager_->close();
    view_manager_ = nullptr;

#if USE_EXPERIMENTAL_SOURCE == true
    // Release cached textures, retired GL contexts, and drain osgEarth's
    // job pool - must happen while OSG globals are still alive.
    geo_view::shutdown();
#endif

    //osgDB::Registry::instance(true);

    traced_assert(filter_manager_);
    filter_manager_ = nullptr;

    if (db_interface_->ready())
        db_interface_->closeDB();

    db_interface_ = nullptr;

    traced_assert(license_manager_);
    license_manager_ = nullptr;

    //shut down command manager at the end
    if (rt_cmd_manager_)
        rt_cmd_manager_->shutdown();

    loginf << "done. goodbye.";
}

MainWindow& COMPASS::mainWindow()
{
    if (!main_window_)
    {
        main_window_ = new MainWindow(*this);
        
        QObject::connect(view_manager_.get(), &ViewManager::loadingStartedSignal,
                        main_window_, &MainWindow::loadingStarted);
        QObject::connect(view_manager_.get(), &ViewManager::loadingDoneSignal,
                        main_window_, &MainWindow::loadingDone);
    }

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
        taskManager().asterixImporterTask().appModeSwitchSlot(last_app_mode, app_mode_);
        emit appModeSwitchSignal(last_app_mode, app_mode_);

        QApplication::restoreOverrideCursor();
    }
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
        const auto& license_manager = *license_manager_;
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
    const auto& license_manager = *license_manager_;
    auto vl = license_manager.activeLicense();

    if (!vl)
        return "";

    return (licensed_to ? "Licensed to " : "") + vl->licensee;
}


