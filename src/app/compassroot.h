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

#include <memory>

class COMPASS;
class ConfigurationManager;
class DBInterface;
class DBContentManager;
class FilterManager;
class TaskManager;
class ViewManager;
class EvaluationManager;
class LicenseManager;
class JobManager;
class UnitManager;
class ProjectionManager;
class RTCommandManager;
class IDBVariableResolver;
class SensorStatusTracker;
class MainWindow;

namespace context
{
    class DBContextManager;
}

namespace rtcommand
{
    class RTCommandRunner;
}

/**
 * Composition root of the client application.
 * - creates and owns COMPASS, all managers and the main window
 * - hands the managers to COMPASS and wires them to COMPASS and to each other
 * - runs the ordered shutdown sequence (triggered via COMPASS::shutdown())
 */
class CompassRoot
{
public:
    explicit CompassRoot(ConfigurationManager& config_manager);
    ~CompassRoot();

    void init();

    COMPASS& compass();
    MainWindow& mainWindow();

    // managers of the upper layers, for the app layer (COMPASS only offers the core managers)
    FilterManager& filterManager();
    ViewManager& viewManager();
    EvaluationManager& evaluationManager();
    TaskManager& taskManager();

    SensorStatusTracker& sensorStatusTracker();

private:
    void registerManagerCreators();
    void startManagers();
    void connectManagers();
    void shutdown();
    void destroyManagers();

    std::unique_ptr<COMPASS> compass_;

    // sub-configurable managers of COMPASS
    std::unique_ptr<LicenseManager>    license_manager_;
    std::unique_ptr<DBInterface>       db_interface_;
    std::unique_ptr<DBContentManager>  dbcontent_manager_;
    std::unique_ptr<FilterManager>     filter_manager_;
    std::unique_ptr<TaskManager>       task_manager_;
    std::unique_ptr<ViewManager>       view_manager_;
    std::unique_ptr<EvaluationManager> eval_manager_;
    std::unique_ptr<JobManager>        job_manager_;
    std::unique_ptr<UnitManager>       unit_manager_;
    std::unique_ptr<ProjectionManager> projection_manager_;
    std::unique_ptr<RTCommandManager>  rt_cmd_manager_;

    std::unique_ptr<IDBVariableResolver>        var_resolver_;    // DBContent variables for the projection manager
    std::unique_ptr<rtcommand::RTCommandRunner> rt_cmd_runner_;
    std::unique_ptr<context::DBContextManager>  context_manager_;
    std::unique_ptr<SensorStatusTracker>        sensor_status_tracker_; // sensor status of the trackers (live)

    MainWindow* main_window_ = nullptr;
};
