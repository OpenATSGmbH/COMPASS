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

#include "compassroot.h"
#include "mainwindow.h"

#include "evaluationmanager.h"
#include "taskmanager.h"
#include "asteriximporttask.h"
#include "reconstructortask.h"
#include "viewmanager.h"
#include "filtermanager.h"

#include "compass.h"
#include "dbinterface.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontentmanagervariableresolver.h"
#include "db_context_manager.h"
#include "licensemanager.h"
#include "jobmanager.h"
#include "unitmanager.h"
#include "projectionmanager.h"
#include "rtcommand_manager.h"
#include "rtcommand_runner.h"
#include "async.h"

#include "logmodel.h"
#include "logger.h"
#include "traced_assert.h"

#include "global.h"

#if USE_EXPERIMENTAL_SOURCE == true
#include "geo_view_api.h"
#endif

/**
 * Creates COMPASS and all managers, and wires them.
 */
CompassRoot::CompassRoot(ConfigurationManager& config_manager)
{
    compass_.reset(new COMPASS(config_manager));

    registerManagerCreators();

    compass_->createManagers();

    traced_assert(license_manager_);
    traced_assert(db_interface_);
    traced_assert(dbcontent_manager_);
    traced_assert(filter_manager_);
    traced_assert(task_manager_);
    traced_assert(view_manager_);
    traced_assert(eval_manager_);
    traced_assert(job_manager_);
    traced_assert(unit_manager_);
    traced_assert(projection_manager_);
    traced_assert(rt_cmd_manager_);

    startManagers();

    compass_->setLiveImportBacklogSizeFunc([ this ] () { return compass_->taskManager().asterixImporterTask().numPacketsInProcessing(); });
    compass_->setShutdownSequence([ this ] () { shutdown(); });

    connectManagers();
}

/**
 * Notice: all managers are destroyed before COMPASS, since they are its sub-configurables.
 */
CompassRoot::~CompassRoot()
{
    if (!compass_->isShutDown())
    {
        logerr << "not shut down";
        compass_->shutdown();
    }

    destroyManagers();

    compass_.reset();
}

/**
 */
void CompassRoot::init()
{
    traced_assert(task_manager_);
    task_manager_->init();
}

/**
 */
COMPASS& CompassRoot::compass()
{
    traced_assert(compass_);
    return *compass_;
}

/**
 * Creates the main window on first use.
 */
MainWindow& CompassRoot::mainWindow()
{
    if (!main_window_)
    {
        main_window_ = new MainWindow(*compass_);

        QObject::connect(view_manager_.get(), &ViewManager::loadingStartedSignal,
                         main_window_, &MainWindow::loadingStarted);
        QObject::connect(view_manager_.get(), &ViewManager::loadingDoneSignal,
                         main_window_, &MainWindow::loadingDone);

        compass_->setMainWindow(main_window_);
    }

    traced_assert(main_window_);
    return *main_window_;
}

/**
 * Registers the managers as sub-configurables of COMPASS.
 * Notice: the order is the creation order of managers missing in the configuration.
 */
void CompassRoot::registerManagerCreators()
{
    auto createLicenseManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!license_manager_);
        license_manager_.reset(new LicenseManager(config, *compass_));
        compass_->managers().license_manager = license_manager_.get();
    };

    auto createDBInterface = [ this ] (nlohmann::json& config)
    {
        traced_assert(!db_interface_);
        db_interface_.reset(new DBInterface(config, *compass_));
        compass_->managers().db_interface = db_interface_.get();
    };

    auto createDBContentManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!dbcontent_manager_);
        dbcontent_manager_.reset(new DBContentManager(config, *compass_));
        compass_->managers().dbcontent_manager = dbcontent_manager_.get();
    };

    auto createFilterManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!filter_manager_);
        filter_manager_.reset(new FilterManager(config, *compass_));
        compass_->managers().filter_manager  = filter_manager_.get();
        compass_->managers().filter_provider = filter_manager_.get();
    };

    auto createTaskManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!task_manager_);
        task_manager_.reset(new TaskManager(config, *compass_));
        compass_->managers().task_manager  = task_manager_.get();
        compass_->managers().task_provider = task_manager_.get();
    };

    auto createViewManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!view_manager_);
        view_manager_.reset(new ViewManager(config, *compass_));
        compass_->managers().view_manager  = view_manager_.get();
        compass_->managers().view_provider = view_manager_.get();
    };

    auto createEvaluationManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!eval_manager_);
        eval_manager_.reset(new EvaluationManager(config, *compass_));
        compass_->managers().eval_manager  = eval_manager_.get();
        compass_->managers().eval_provider = eval_manager_.get();
    };

    auto createJobManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!job_manager_);
        job_manager_.reset(new JobManager(config, compass_.get()));
        compass_->managers().job_manager = job_manager_.get();
    };

    auto createUnitManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!unit_manager_);
        unit_manager_.reset(new UnitManager(config, compass_.get()));
        compass_->managers().unit_manager = unit_manager_.get();
    };

    auto createProjectionManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!projection_manager_);
        projection_manager_.reset(new ProjectionManager(config, compass_.get()));
        compass_->managers().projection_manager = projection_manager_.get();
    };

    auto createRTCommandManager = [ this ] (nlohmann::json& config)
    {
        traced_assert(!rt_cmd_manager_);
        rt_cmd_manager_.reset(new RTCommandManager(config, compass_.get()));
        compass_->managers().rt_cmd_manager = rt_cmd_manager_.get();
    };

    compass_->registerManagerCreator("LicenseManager"   , "LicenseManager0"   , createLicenseManager);
    compass_->registerManagerCreator("DBInterface"      , "DBInterface0"      , createDBInterface);
    compass_->registerManagerCreator("DBContentManager" , "DBContentManager0" , createDBContentManager);
    compass_->registerManagerCreator("FilterManager"    , "FilterManager0"    , createFilterManager);
    compass_->registerManagerCreator("TaskManager"      , "TaskManager0"      , createTaskManager);
    compass_->registerManagerCreator("ViewManager"      , "ViewManager0"      , createViewManager);
    compass_->registerManagerCreator("EvaluationManager", "EvaluationManager0", createEvaluationManager);
    compass_->registerManagerCreator("JobManager"       , "JobManager0"       , createJobManager);
    compass_->registerManagerCreator("UnitManager"      , "UnitManager0"      , createUnitManager);
    compass_->registerManagerCreator("ProjectionManager", "ProjectionManager0", createProjectionManager);
    compass_->registerManagerCreator("RTCommandManager" , "RTCommandManager0" , createRTCommandManager);
}

/**
 * Starts the job and command threads and creates the managers that are no sub-configurables.
 */
void CompassRoot::startManagers()
{
    COMPASS::Managers& managers = compass_->managers();

    // wire up variable resolver for projection manager (breaks core→db dep)
    var_resolver_.reset(new DBContentManagerVariableResolver(*dbcontent_manager_));
    projection_manager_->varResolver(*var_resolver_);

    job_manager_->start();
    rt_cmd_manager_->start();

    rt_cmd_runner_.reset(new rtcommand::RTCommandRunner(*compass_));
    managers.rt_cmd_runner = rt_cmd_runner_.get();

    // DBContextManager - not a Configurable, created directly
    context_manager_.reset(new context::DBContextManager(*compass_));
    managers.context_manager = context_manager_.get();
}

/**
 * Connects the managers.
 * Notice: the connection order is the order in which the slots run.
 */
void CompassRoot::connectManagers()
{
    COMPASS*           compass      = compass_.get();
    LogStore*          log_store    = &compass_->logStore();
    ReconstructorTask* reconst_task = &task_manager_->reconstructReferencesTask();
    ASTERIXImportTask* import_task  = &task_manager_->asterixImporterTask();

    // database opening & closing - DBContextManager connects FIRST

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     context_manager_.get(), &context::DBContextManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     context_manager_.get(), &context::DBContextManager::databaseClosedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     dbcontent_manager_.get(), &DBContentManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     dbcontent_manager_.get(), &DBContentManager::databaseClosedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     filter_manager_.get(), &FilterManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     filter_manager_.get(), &FilterManager::databaseClosedSlot);

    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     filter_manager_.get(), &FilterManager::dataSourcesChangedSlot);
    QObject::connect(context_manager_.get(), &context::DBContextManager::dataSourcesChangedSignal,
                     filter_manager_.get(), &FilterManager::dataSourcesChangedSlot);

    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     filter_manager_.get(), &FilterManager::sectorsChangedSlot);
    QObject::connect(context_manager_.get(), &context::DBContextManager::sectorsChangedSignal,
                     filter_manager_.get(), &FilterManager::sectorsChangedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     view_manager_.get(), &ViewManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     view_manager_.get(), &ViewManager::databaseClosedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     eval_manager_.get(), &EvaluationManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     eval_manager_.get(), &EvaluationManager::databaseClosedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     task_manager_.get(), &TaskManager::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     task_manager_.get(), &TaskManager::databaseClosedSlot);

    QObject::connect(compass, &COMPASS::databaseOpenedSignal,
                     log_store, &LogStore::databaseOpenedSlot);
    QObject::connect(compass, &COMPASS::databaseClosedSignal,
                     log_store, &LogStore::databaseClosedSlot);

    // data sources changed
    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     eval_manager_.get(), &EvaluationManager::dataSourcesChangedSlot);
    QObject::connect(context_manager_.get(), &context::DBContextManager::dataSourcesChangedSignal,
                     eval_manager_.get(), &EvaluationManager::dataSourcesChangedSlot);

    // sectors changed (direct edits within active context)
    QObject::connect(context_manager_.get(), &context::DBContextManager::sectorsChangedSignal,
                     reconst_task, &ReconstructorTask::sectorsChangedSlot);
    // context changed (DB open, context switch - sectors come with the new context)
    QObject::connect(context_manager_.get(), &context::DBContextManager::activeContextChangedSignal,
                     reconst_task, &ReconstructorTask::sectorsChangedSlot);

    // view data flows from the current source (ViewManager owns it); the load bookends
    // (loadingStarted/DoneSignal) are ViewManager's too - driven off the operation it owns.
    // The data-sources status widget connects directly to ViewManager::dataDistributedSignal.

    // appmode: ASTERIX ingestion switches before all other slots (importer first, so no
    // trailing inserts land after a pause)
    QObject::connect(compass, &COMPASS::appModeAboutToSwitchSignal,
                     import_task, &ASTERIXImportTask::appModeSwitchSlot);

    QObject::connect(compass, &COMPASS::appModeSwitchSignal,
                     filter_manager_.get(), &FilterManager::appModeSwitchSlot);
    // retained no-op hook (live bookends now synthesized in ViewManager::appModeSwitchSlot)
    QObject::connect(compass, &COMPASS::appModeSwitchSignal,
                     dbcontent_manager_.get(), &DBContentManager::appModeSwitchSlot);

    // results of the evaluation are locked when radar plot positions were recalculated
    QObject::connect(task_manager_.get(), &TaskManager::taskRadarPlotPositionsDoneSignal,
                     eval_manager_.get(), &EvaluationManager::lockResultsSlot);

    // features
    QObject::connect(license_manager_.get(), &LicenseManager::changed, task_manager_.get(), &TaskManager::updateFeatures);
    QObject::connect(license_manager_.get(), &LicenseManager::changed, view_manager_.get(), &ViewManager::updateFeatures);

    // view signals for code below the view layer
    QObject::connect(view_manager_.get(), &ViewManager::dataDistributedSignal,
                     compass, &COMPASS::viewDataDistributedSignal);
    QObject::connect(view_manager_.get(), &ViewManager::loadingDoneSignal,
                     compass, &COMPASS::viewLoadingDoneSignal);
}

/**
 * Shutdown sequence, run by COMPASS::shutdown().
 * Notice: the order matters, see the comments of the single steps.
 */
void CompassRoot::shutdown()
{
    COMPASS::Managers& managers = compass_->managers();

    // asynchronous view processing (geo geometry builds) may still read manager state -
    // wait it out before any manager is torn down
    if (view_manager_ && view_manager_->hasPendingViewProcessing())
    {
        loginf << "waiting for pending view processing";
        Utils::Async::pumpUntil([ this ] { return !view_manager_->hasPendingViewProcessing(); });
    }

    traced_assert(task_manager_);
    task_manager_->shutdown();
    managers.task_provider = nullptr;
    managers.task_manager  = nullptr;
    task_manager_ = nullptr;

    // A job still in flight keeps using the managers reset below. The live delete job is
    // the one that hits this: its database cleanup reconnects, and reopening asks for the
    // DBContent manager. So all jobs have to be done before anything is torn down.
    // The call waits until no job is left.
    if (job_manager_)
        job_manager_->shutdown();

    traced_assert(db_interface_);

    // Views still ask the managers below while their widgets are torn down, e.g. a Table View
    // header reads the display names from the DBContent manager. So close the views first.
    traced_assert(view_manager_);
    view_manager_->close();
    managers.view_provider = nullptr;
    managers.view_manager  = nullptr;
    view_manager_ = nullptr;

    managers.context_manager = nullptr;
    context_manager_ = nullptr;

    traced_assert(dbcontent_manager_);
    if (db_interface_->ready())
        dbcontent_manager_->saveTargets();
    managers.dbcontent_manager = nullptr;
    dbcontent_manager_ = nullptr;

    traced_assert(eval_manager_);
    eval_manager_->close();
    managers.eval_provider = nullptr;
    managers.eval_manager  = nullptr;
    eval_manager_ = nullptr;

#if USE_EXPERIMENTAL_SOURCE == true
    // Release cached textures, retired GL contexts, and drain osgEarth's
    // job pool - must happen while OSG globals are still alive.
    geo_view::shutdown();
#endif

    traced_assert(filter_manager_);
    managers.filter_provider = nullptr;
    managers.filter_manager  = nullptr;
    filter_manager_ = nullptr;

    if (db_interface_->ready())
        db_interface_->closeDB();

    managers.db_interface = nullptr;
    db_interface_ = nullptr;

    traced_assert(license_manager_);
    managers.license_manager = nullptr;
    license_manager_ = nullptr;

    //shut down command manager at the end
    if (rt_cmd_manager_)
        rt_cmd_manager_->shutdown();
}

/**
 * Destroys the managers that are still alive after the shutdown sequence.
 * Notice: same order as the former member destruction of COMPASS.
 */
void CompassRoot::destroyManagers()
{
    COMPASS::Managers& managers = compass_->managers();

    managers.rt_cmd_runner = nullptr;
    rt_cmd_runner_ = nullptr;

    var_resolver_ = nullptr;

    managers.rt_cmd_manager = nullptr;
    rt_cmd_manager_ = nullptr;

    managers.projection_manager = nullptr;
    projection_manager_ = nullptr;

    managers.unit_manager = nullptr;
    unit_manager_ = nullptr;

    managers.job_manager = nullptr;
    job_manager_ = nullptr;
}
