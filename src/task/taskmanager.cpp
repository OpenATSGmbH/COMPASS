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

#include "taskmanager.h"
#include "taskresultswidget.h"

#include "compass.h"
#include "createartasassociationstask.h"
#include "jsonimporttask.h"
#include "radarplotpositioncalculatortask.h"
#include "viewpointsimporttask.h"
#include "gpstrailimporttask.h"
//#include "gpsimportcsvtask.h"
#include "reconstructortask.h"
#include "analyzedatasourcetask.h"
#include "mainwindow.h"
#include "viewabledataconfig.h"
#include "viewmanager.h"
#include "dbinterface.h"
#include "asynctask.h"

#include "asteriximporttask.h"
#include "asteriximporttaskwidget.h"

#include "taskresult.h"
#include "taskresultswidget.h"

#include "reportexport.h"
#include "reportexportdialog.h"

#include "evaluationtaskresult.h"
#include "datasourceanalysistaskresult.h"

#include "traced_assert.h"

#include <QCoreApplication>
#include <QApplication>
#include <QMainWindow>
#include <QThread>
#include <QProgressDialog>
#include <QMessageBox>

//#include "boost/date_time/posix_time/posix_time.hpp"

using namespace Utils;

const bool TaskManager::CleanupDBIfNeeded = true;

/**
 */
// TaskManager::TaskManager(const std::string& class_name, const std::string& instance_name, COMPASS* compass)
//     : Configurable(class_name, instance_name, compass, "task.json") ...

TaskManager::TaskManager(nlohmann::json& config, COMPASS& compass)
    : Configurable(config, &compass), compass_(compass)
{
    setObjectName("TaskManager");

    createSubConfigurables();
}

/**
 */
TaskManager::~TaskManager()
{
    widget_ = nullptr;
}

/**
 */
void TaskManager::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    if (class_name == "ASTERIXImportTask")
    {
        traced_assert(!asterix_importer_task_);
        asterix_importer_task_.reset(new ASTERIXImportTask(child_json, this));
        traced_assert(asterix_importer_task_);
        addTask(class_name, asterix_importer_task_.get());
    }
    else if (class_name == "ViewPointsImportTask")
    {
        traced_assert(!view_points_import_task_);
        view_points_import_task_.reset(new ViewPointsImportTask(child_json, this));
        traced_assert(view_points_import_task_);
        addTask(class_name, view_points_import_task_.get());
    }
    else if (class_name == "JSONImportTask")
    {
        traced_assert(!json_import_task_);
        json_import_task_.reset(new JSONImportTask(child_json, this));
        traced_assert(json_import_task_);
        addTask(class_name, json_import_task_.get());
    }
    else if (class_name == "GPSTrailImportTask")
    {
        traced_assert(!gps_trail_import_task_);
        gps_trail_import_task_.reset(new GPSTrailImportTask(child_json, this));
        traced_assert(gps_trail_import_task_);
        addTask(class_name, gps_trail_import_task_.get());
    }
    // else if (class_name == "GPSImportCSVTask")
    // {
    //     traced_assert(!gps_import_csv_task_);
    //     gps_import_csv_task_.reset(new GPSImportCSVTask(child_json, *this, this));
    //     traced_assert(gps_import_csv_task_);
    //     addTask(class_name, gps_import_csv_task_.get());
    // }
    else if (class_name == "RadarPlotPositionCalculatorTask")
    {
        traced_assert(!radar_plot_position_calculator_task_);
        radar_plot_position_calculator_task_.reset(new RadarPlotPositionCalculatorTask(child_json, this, compass_));
        traced_assert(radar_plot_position_calculator_task_);
        addTask(class_name, radar_plot_position_calculator_task_.get());

        connect(radar_plot_position_calculator_task_.get(), &RadarPlotPositionCalculatorTask::doneSignal,
                this, &TaskManager::taskRadarPlotPositionsDoneSignal);
    }
    else if (class_name == "CreateARTASAssociationsTask")
    {
        traced_assert(!create_artas_associations_task_);
        create_artas_associations_task_.reset(
                    new CreateARTASAssociationsTask(child_json, this));
        traced_assert(create_artas_associations_task_);
        addTask(class_name, create_artas_associations_task_.get());
    }
    else if (class_name == "ReconstructorTask")
    {
        traced_assert(!reconstruct_references_task_);
        reconstruct_references_task_.reset(new ReconstructorTask(child_json, this));
        traced_assert(reconstruct_references_task_);
        addTask(class_name, reconstruct_references_task_.get());
    }
    else if (class_name == "AnalyzeDataSourceTask")
    {
        traced_assert(!analyze_data_source_task_);
        analyze_data_source_task_.reset(new AnalyzeDataSourceTask(child_json, this));
        traced_assert(analyze_data_source_task_);
        addTask(class_name, analyze_data_source_task_.get());
    }
    else if (class_name == "ReportExport")
    {
        traced_assert(!report_export_);
        report_export_.reset(new ResultReport::ReportExport(child_json, this));
        traced_assert(report_export_);
    }
    else
    {
        throw std::runtime_error("TaskManager: generateSubConfigurable: unknown class_name " +
                                 class_name);
    }
}

/**
 */
void TaskManager::addTask(const std::string& class_name, Task* task)
{
    traced_assert(task);
    traced_assert(!tasks_.count(class_name));
    tasks_[class_name] = task;
}

/**
 */
void TaskManager::checkSubConfigurables()
{
    if (!asterix_importer_task_)
    {
        generateSubConfigurableFromConfig("ASTERIXImportTask", "ASTERIXImportTask0");
        traced_assert(asterix_importer_task_);
    }

    if (!view_points_import_task_)
    {
        generateSubConfigurableFromConfig("ViewPointsImportTask", "ViewPointsImportTask0");
        traced_assert(view_points_import_task_);
    }

    if (!json_import_task_)
    {
        generateSubConfigurableFromConfig("JSONImportTask", "JSONImportTask0");
        traced_assert(json_import_task_);
    }

    if (!gps_trail_import_task_)
    {
        generateSubConfigurableFromConfig("GPSTrailImportTask", "GPSTrailImportTask0");
        traced_assert(gps_trail_import_task_);
    }

    // if (!gps_import_csv_task_)
    // {
    //     generateSubConfigurable("GPSImportCSVTask", "GPSImportCSVTask0");
    //     traced_assert(gps_import_csv_task_);
    // }

    if (!radar_plot_position_calculator_task_)
    {
        generateSubConfigurableFromConfig("RadarPlotPositionCalculatorTask",
                                        "RadarPlotPositionCalculatorTask0");
        traced_assert(radar_plot_position_calculator_task_);
    }

    if (!create_artas_associations_task_)
    {
        generateSubConfigurableFromConfig("CreateARTASAssociationsTask", "CreateARTASAssociationsTask0");
        traced_assert(create_artas_associations_task_);
    }

    if (!reconstruct_references_task_)
    {
        generateSubConfigurableFromConfig("ReconstructorTask", "ReconstructorTask0");
        traced_assert(reconstruct_references_task_);
    }

    if (!analyze_data_source_task_)
    {
        generateSubConfigurableFromConfig("AnalyzeDataSourceTask", "AnalyzeDataSourceTask0");
        traced_assert(analyze_data_source_task_);
    }

    if (!report_export_)
    {
        generateSubConfigurableFromConfig("ReportExport", "ReportExport0");
        traced_assert(report_export_);
    }
}

/**
 */
std::map<std::string, Task*> TaskManager::tasks() const 
{ 
    return tasks_; 
}

/**
 */
void TaskManager::init()
{
    //init all tasks
    for (const auto& t : tasks_)
        if (t.second)
            t.second->initTask();

    //update features
    updateFeatures();
}

/**
 */
void TaskManager::shutdown()
{
    logdbg;

    asterix_importer_task_->stop(); // stops if active
    asterix_importer_task_ = nullptr;

    view_points_import_task_ = nullptr;
    json_import_task_ = nullptr;
    gps_trail_import_task_ = nullptr;
    //gps_import_csv_task_ = nullptr;
    radar_plot_position_calculator_task_ = nullptr;
    create_artas_associations_task_ = nullptr;
    reconstruct_references_task_ = nullptr;
    analyze_data_source_task_ = nullptr;
}

/**
 */
void TaskManager::runTask(const std::string& task_name)
{
    loginf << "name " << task_name;

    traced_assert(tasks_.count(task_name));
    traced_assert(tasks_.at(task_name)->canRun());

    tasks_.at(task_name)->run();
}

/**
 */
ASTERIXImportTask& TaskManager::asterixImporterTask() const
{
    traced_assert(asterix_importer_task_);
    return *asterix_importer_task_;
}

/**
 */
ViewPointsImportTask& TaskManager::viewPointsImportTask() const
{
    traced_assert(view_points_import_task_);
    return *view_points_import_task_;
}

/**
 */
JSONImportTask& TaskManager::jsonImporterTask() const
{
    traced_assert(json_import_task_);
    return *json_import_task_;
}

/**
 */
GPSTrailImportTask& TaskManager::gpsTrailImportTask() const
{
    traced_assert(gps_trail_import_task_);
    return *gps_trail_import_task_;
}

/**
 */
// GPSImportCSVTask& TaskManager::gpsImportCSVTask() const
// {
//     traced_assert(gps_import_csv_task_);
//     return *gps_import_csv_task_;
// }

/**
 */
RadarPlotPositionCalculatorTask& TaskManager::radarPlotPositionCalculatorTask() const
{
    traced_assert(radar_plot_position_calculator_task_);
    return *radar_plot_position_calculator_task_;
}

/**
 */
CreateARTASAssociationsTask& TaskManager::createArtasAssociationsTask() const
{
    traced_assert(create_artas_associations_task_);
    return *create_artas_associations_task_;
}

/**
 */
ReconstructorTask& TaskManager::reconstructReferencesTask() const
{
    traced_assert(reconstruct_references_task_);
    return *reconstruct_references_task_;
}

/**
 */
AnalyzeDataSourceTask& TaskManager::analyzeDataSourceTask() const
{
    traced_assert(analyze_data_source_task_);
    return *analyze_data_source_task_;
}

/**
 */
TaskResultsWidget* TaskManager::widget()
{
    if (!widget_)
        widget_ =new TaskResultsWidget(*this);

    traced_assert(widget_);
    return widget_;
}

/**
 */
std::shared_ptr<TaskResult> TaskManager::createResult(unsigned int id, 
                                                      task::TaskResultType type)
{
    std::shared_ptr<TaskResult> result;

    //generate result depending on stored type (@TODO: factory?)
    if (type == task::TaskResultType::Generic)
    {
        result.reset(new TaskResult(id, *this));
    }
    else if (type == task::TaskResultType::Evaluation)
    {
        result.reset(new EvaluationTaskResult(id, *this, compass_));
    }
    else if (type == task::TaskResultType::DataSourceAnalysis)
    {
        result.reset(new DataSourceAnalysisTaskResult(id, *this));
    }

    return result;
}

/**
 */
void TaskManager::beginTaskResultWriting(const std::string& name,
                                         task::TaskResultType type,
                                         bool clear_existing)
{
    if (widget_)
        widget_->setDisabled(true);

    if (current_result_)
        logerr << "result id " << current_result_->id()
               << " name " << current_result_->name() << " already present";

    traced_assert(!current_result_);
    current_result_ = getOrCreateResult(name, type);

    //prepare result for new content (clears the report) - opt out for results
    //that accumulate across runs (e.g. ASTERIX Import)
    if (clear_existing)
    {
        auto res = current_result_->prepareResult();
        if (!res.ok())
            logerr << "result could not be initialized: " << res.error();

        traced_assert(res.ok());
    }

    loginf << "beginning result id " << current_result_->id()
           << " name " << current_result_->name()
           << " clear_existing " << clear_existing;
}

/**
 */
bool TaskManager::hasCurrentResult() const
{
    return current_result_ != nullptr;
}

std::shared_ptr<TaskResult>& TaskManager::currentResult()
{
    traced_assert(current_result_);
    return current_result_;
}

std::shared_ptr<ResultReport::Report>& TaskManager::currentReport()
{
    traced_assert(current_result_);
    return current_result_->report();
}

/**
 */
void TaskManager::endTaskResultWriting(bool store_result, bool show_dialog)
{
    loginf << "store_result " << store_result;

    if (widget_)
        widget_->setDisabled(false);

    traced_assert(current_result_);

    //finalize result after adding content
    auto res = current_result_->finalizeResult();
    if (!res.ok())
        logerr << "Result could not be finalized: " << res.error();

    traced_assert(res.ok());

    //store result?
    if (store_result)
    {
        loginf << "Storing result...";

        auto result_ptr = current_result_.get();
        bool cleanup_db = CleanupDBIfNeeded;

        auto cb = [ this, result_ptr, cleanup_db ] (const AsyncTaskState& s, AsyncTaskProgressWrapper& p)
        {
            return compass_.dbInterface().saveResult(*result_ptr, cleanup_db);
        };

        AsyncFuncTask task(cb, "Save Result", "Saving result", false);
        bool ok = show_dialog ? task.runAsyncDialog() : task.runAsync();

        //@TODO
        if (!ok)
            logerr << "Storing result failed: " << task.taskState().error;
        
        traced_assert(ok);
    }

    traced_assert(current_result_);

    loginf << "ending result id " << current_result_->id()
           << " name " << current_result_->name();

    current_result_ = nullptr;

    emit taskResultsChangedSignal();
}

/**
 */
void TaskManager::resultHeaderChanged(const TaskResult& result)
{
    //update result header upon change
    auto res = compass_.dbInterface().updateResultHeader(result);
    traced_assert(res.ok());

    emit taskResultHeaderChangedSignal(QString::fromStdString(result.name()));
}

/**
 */
void TaskManager::resultContentChanged(const TaskResult& result)
{
    //update result content upon change
    auto res = compass_.dbInterface().updateResultContent(result);
    traced_assert(res.ok());
}

/**
 */
MainWindow* TaskManager::getMainWindow()
{
    for(QWidget* pWidget : QApplication::topLevelWidgets())
    {
        QMainWindow* qt_main_window = qobject_cast<QMainWindow*>(pWidget);

        if (qt_main_window)
        {
            MainWindow* main_window = dynamic_cast<MainWindow*>(qt_main_window);
            traced_assert(main_window);
            return main_window;
        }
    }
    return nullptr;
}

/**
 */
void TaskManager::updateFeatures()
{
    for (auto& t : tasks_)
        if (t.second)
            t.second->updateFeatures();
}

/**
 */
const std::map<unsigned int, std::shared_ptr<TaskResult>>& TaskManager::results() const
{
    return results_;
}

/**
 */
std::shared_ptr<TaskResult> TaskManager::result(unsigned int id) const // get existing result
{
    traced_assert(results_.count(id));
    return results_.at(id);
}

/**
 */
std::shared_ptr<TaskResult> TaskManager::result(const std::string& name) const // get existing result
{
    auto id = findResult(name);
    return id.has_value() ? results_.at(id.value()) : std::shared_ptr<TaskResult>();
}

/**
 */
std::shared_ptr<TaskResult> TaskManager::getOrCreateResult(const std::string& name,
                                                           task::TaskResultType type)
{
    auto id = findResult(name);

    if (id.has_value())
    {
        return results_.at(id.value());
    }
    else // create
    {
        unsigned int new_id{0};

        if (results_.size())
            new_id = results_.rbegin()->first + 1;

        auto r = createResult(new_id, type);

        results_[new_id] = r;
        results_.at(new_id)->name(name);

        return results_.at(new_id);
    }
}

/**
 */
boost::optional<unsigned int> TaskManager::findResult(const std::string& name) const
{
    auto it = std::find_if(results_.begin(), results_.end(),
                           [&name](const std::pair<const unsigned int, std::shared_ptr<TaskResult>>& pair) {
                               return pair.second && pair.second->name() == name; });
    if (it == results_.end())
        return boost::optional<unsigned int>();

    return it->first;
}

/**
 */
bool TaskManager::hasResult (const std::string& name) const
{
    return findResult(name).has_value();
}

/**
 */
bool TaskManager::removeResult(const std::string& name,
                               bool inform_changes)
{
    auto id = findResult(name);
    if (!id.has_value())
        return true;

    const auto& result = results_.at(id.value());
    traced_assert(result);

    auto res = compass_.dbInterface().deleteResult(*result, CleanupDBIfNeeded);
    if (!res.ok())
        return false;

    results_.erase(id.value());

    if (inform_changes)
        emit taskResultsChangedSignal();

    return true;
}

/**
 */
ResultT<nlohmann::json> TaskManager::exportResult(const std::string& name, 
                                                  ResultReport::ReportExportMode mode,
                                                  bool no_interaction_mode,
                                                  const boost::optional<std::string>& export_dir,
                                                  const std::string& section)
{
    traced_assert(report_export_);
    traced_assert(hasResult(name));

    auto r = result(name);
    traced_assert(r);

    ResultReport::ReportExportDialog dlg(*r, 
                                         *report_export_, 
                                         mode,
                                         no_interaction_mode,
                                         export_dir,
                                         section);
    dlg.exec();

    return dlg.result();
}

/**
 */
void TaskManager::databaseOpenedSlot()
{
    loadResults();
}

/**
 */
void TaskManager::databaseClosedSlot()
{
    clearResults();
}

/**
 */
void TaskManager::setViewableDataConfig(const nlohmann::json::object_t& data,
                                        bool load_blocking)
{
    viewable_data_cfg_.reset(new ViewableDataConfig(data));

    compass_.viewManager().setCurrentViewPoint(viewable_data_cfg_.get(), load_blocking);
}

/**
 */
void TaskManager::unsetViewableDataConfig()
{
    compass_.viewManager().unsetCurrentViewPoint();
    viewable_data_cfg_.reset();
}

/**
 */
std::shared_ptr<ResultReport::SectionContent> TaskManager::loadContent(ResultReport::Section* section, 
                                                                       unsigned int content_id,
                                                                       bool show_dialog) const
{
    ResultT<TaskResult::ContentPtr> result;

    if (show_dialog)
    {
        //run as async task with dialog
        auto result_ptr = &result;

        auto cb = [ this, result_ptr, section, content_id ] (const AsyncTaskState&, AsyncTaskProgressWrapper&) 
        { 
            *result_ptr = compass_.dbInterface().loadContent(section, content_id);
            return true;
        };

        AsyncFuncTask task(cb, "Loading", "Loading section content", false);
        task.runAsyncDialog();
    }
    else
    {
        //directly run
        result = compass_.dbInterface().loadContent(section, content_id);
    }

    if (!result.ok())
    {
        logerr << "could not load stored content: " << result.error();
        return std::shared_ptr<ResultReport::SectionContent>();
    }

    return result.result();
}

/**
 */
void TaskManager::loadResults()
{
    traced_assert(!current_result_);

    results_.clear();
    
    auto res = compass_.dbInterface().loadResults();
    if (!res.ok())
    {
        logerr << "could not load stored results: " << res.error();
        return;
    }

    for (const auto& r : res.result())
        results_[ r->id() ] = r;

    loginf << "Loaded " << results_.size() << " result(s)";

    emit taskResultsChangedSignal();
}

/**
 */
void TaskManager::clearResults()
{
    traced_assert(!current_result_);

    results_.clear();
    
    emit taskResultsChangedSignal();
}

/**
 */
void TaskManager::storeBackupSection()
{
    if (widget_)
        widget_->storeBackupSection();
}

/**
 */
void TaskManager::restoreBackupSection()
{
    if (widget_)
        widget_->restoreBackupSection();
}

