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

#include "mainwindow_commands.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/reportvariable.h"
#include "mainwindow.h"
#include "compassroot.h"
#include "compass.h"
#include "db_context_manager.h"
#include "taskmanager.h"
#include "taskresult.h"
#include "evaluationmanager.h"
#include "dbcontentmanager.h"
#include "radarplotpositioncalculatortask.h"
#include "createartasassociationstask.h"
#include "reconstructortask.h"
#include "analyzedatasourcetask.h"
#include "viewmanager.h"
#include "viewpointsreportgenerator.h"
#include "viewpointsreportgeneratordialog.h"
#include "evaluationmanager.h"
#include "sectorlayer.h"
#include "logger.h"
#include "files.h"
#include "rtcommand_registry.h"
#include "config.h"

#include "mainwindow_commands_file.h"
#include "mainwindow_commands_import.h"
#include "context_commands.h"
#include "datasource_commands.h"

#include "event_log.h"

#include <QTimer>
#include <QCoreApplication>
#include <QThread>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

using namespace std;
using namespace Utils;

//file

//import

namespace main_window
{

/**
 */
void init_commands(rtcommand::RTCommandRegistry& registry,
                   CompassRoot& root)
{
    // file
    registry.registerCommand<main_window::RTCommandOpenDB>(root);
    registry.registerCommand<main_window::RTCommandOpenRecentDB>(root);
    registry.registerCommand<main_window::RTCommandCreateDB>(root);
    registry.registerCommand<main_window::RTCommandImportDataSourcesFile>(root);
    registry.registerCommand<main_window::RTCommandDeleteAllDataSources>(root);
    registry.registerCommand<main_window::RTCommandCloseDB>(root);
    registry.registerCommand<main_window::RTCommandQuit>();

    // import
    registry.registerCommand<main_window::RTCommandImportViewPointsFile>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXFile>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXFiles>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXPCAPFile>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXPCAPFiles>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXNetworkStart>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXNetworkStop>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXNetworkPause>(root);
    registry.registerCommand<main_window::RTCommandImportASTERIXNetworkResume>(root);

    registry.registerCommand<main_window::RTCommandImportJSONFile>(root);
    registry.registerCommand<main_window::RTCommandImportGPSTrail>(root);
    registry.registerCommand<main_window::RTCommandImportSectorsJSON>(root);
    registry.registerCommand<main_window::RTCommandCalculateRadarPlotPositions>(root);
    registry.registerCommand<main_window::RTCommandCalculateARTASAssociations>(root);
    registry.registerCommand<main_window::RTCommandReconstructReferences>(root);
    registry.registerCommand<main_window::RTCommandAnalyzeDataSource>(root);
    registry.registerCommand<main_window::RTCommandLoadData>(root);
    registry.registerCommand<main_window::RTCommandExportViewPointsReport>(root);
    registry.registerCommand<main_window::RTCommandExportReport>(root);
    registry.registerCommand<main_window::RTCommandGetExistingReports>(root);
    registry.registerCommand<main_window::RTCommandGetReport>(root);

    registry.registerCommand<main_window::RTCommandGetEvents>();
    registry.registerCommand<main_window::RTCommandReconfigure>(root);
    registry.registerCommand<main_window::RTCommandClientInfo>(root);

    context_cmd::init_context_commands(registry, root.compass().dbContextManager());
    dbContent::init_data_source_commands(registry,
                                         root.compass().dbContextManager(),
                                         root.compass().dbContentManager(),
                                         root.sensorStatusTracker());
}

// import ds

/**
 */
RTCommandImportDataSourcesFile::RTCommandImportDataSourcesFile(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid  RTCommandImportDataSourcesFile::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandImportDataSourcesFile::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

//    if (!root_.compass().dbOpened())
//    {
//        setResultMessage("Database not opened");
//        return false;
//    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    root_.compass().dbContextManager().importSensors(filename_);

    return true;
}

void RTCommandImportDataSourcesFile::collectOptions_impl(OptionsDescription& options,
                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json'");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportDataSourcesFile::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
}

// delete_all_data_sources

/**
 */
RTCommandDeleteAllDataSources::RTCommandDeleteAllDataSources(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

bool RTCommandDeleteAllDataSources::run_impl()
{
    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    // the open database's data references its data sources
    if (root_.compass().dbOpened() && root_.compass().dbContextManager().hasInsertedData())
    {
        setResultMessage("Cannot delete data sources while a database with imported data is open");
        return false;
    }

    // clear all data sources from active context
    if (root_.compass().dbContextManager().hasActiveContext())
        root_.compass().dbContextManager().deleteAllDataSources();

    return true;
}

// import sectors json
/**
 */
RTCommandImportSectorsJSON::RTCommandImportSectorsJSON(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid  RTCommandImportSectorsJSON::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(!Files::fileExists(filename_), string("File '")+filename_+"' does not exist")

    return RTCommand::valid();
}

bool RTCommandImportSectorsJSON::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!Files::fileExists(filename_))
    {
        setResultMessage("File '"+filename_+"' does not exist");
        return false;
    }

    try
    {
        root_.compass().dbContextManager().importSectors(filename_);
    }
    catch(const std::exception& e)
    {
        setResultMessage(e.what());
        return false;
    }
    catch(...)
    {
        setResultMessage("Unknown error");
        return false;
    }

    size_t num_sectors = 0;

    traced_assert(root_.compass().dbContextManager().sectorsLoaded());
    const auto& sector_layers = root_.compass().dbContextManager().sectorLayers();

    for (const auto& sl : sector_layers)
        num_sectors += sl->size();
    
    //return some information on imported sectors
    nlohmann::json reply;
    reply[ "num_sector_layers" ] = sector_layers.size();
    reply[ "num_sectors"       ] = num_sectors;

    setJSONReply(reply);

    return true;
}

void RTCommandImportSectorsJSON::collectOptions_impl(OptionsDescription& options,
                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/file1.json’");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandImportSectorsJSON::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
}

// calc radar plot pos
RTCommandCalculateRadarPlotPositions::RTCommandCalculateRadarPlotPositions(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
    condition.setSignal("compass.taskmanager.radarplotpositioncalculatortask.doneSignal", -1); // think about max duration
}

bool RTCommandCalculateRadarPlotPositions::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    RadarPlotPositionCalculatorTask& task = root_.taskManager().radarPlotPositionCalculatorTask();

    if(!task.canRun())
    {
        setResultMessage("Calculate radar plot positions task can not be run");
        return false;
    }

    task.allowUserInteractions(false);
    task.run();

    // if ok
    return true;
}

// associate ARTAS target reports
RTCommandCalculateARTASAssociations::RTCommandCalculateARTASAssociations(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
    condition.setSignal("compass.taskmanager.createartasassociationstask.doneSignal", -1); // think about max duration
}

bool RTCommandCalculateARTASAssociations::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    CreateARTASAssociationsTask& task = root_.taskManager().createArtasAssociationsTask();

    if(!task.canRun())
    {
        setResultMessage("Calculate ARTAS associations task can not be run");
        return false;
    }

    task.allowUserInteractions(false);
    task.run();

    // if ok
    return true;
}

bool RTCommandCalculateARTASAssociations::checkResult_impl()
{
    const auto& stats = root_.taskManager().createArtasAssociationsTask().resultStats();

    nlohmann::json reply;

    reply["found_hashes"]               = stats.found_hashes_;
    reply["missing_hashes_at_beginning"] = stats.missing_hashes_at_beginning_;
    reply["missing_hashes"]             = stats.missing_hashes_;
    reply["found_hash_duplicates"]      = stats.found_hash_duplicates_;
    reply["dubious_associations"]       = stats.dubious_associations_;

    nlohmann::json counts = nlohmann::json::object();

    for (const auto& cnt_it : stats.association_counts_)
        counts[cnt_it.first] = { {"count", cnt_it.second.first},
                                 {"associated_count", cnt_it.second.second} };

    reply["association_counts"] = counts;

    setJSONReply(reply);

    return true;
}

// calc ref
RTCommandReconstructReferences::RTCommandReconstructReferences(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
    condition.setSignal("compass.taskmanager.reconstructortask.doneSignal", -1); // think about max duration
}

bool RTCommandReconstructReferences::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    ReconstructorTask& task = root_.taskManager().reconstructReferencesTask();

    if (!config_.empty())
    {
        auto res = task.applyJSONStringParameters(config_);
        if (!res.ok())
        {
            setResultMessage("Could not apply configuration: " + res.error());
            return false;
        }
    }

    if (!disabled_sensors_.empty())
    {
        auto& ctx_man = root_.compass().dbContextManager();

        std::vector<std::string> sensors_to_disable = Utils::String::split(disabled_sensors_, ';');
        for (const auto& sensor_name : sensors_to_disable)
        {
            if (sensor_name.empty())
            {
                setResultMessage("Invalid list of disabled sensors");
                return false;
            }

            if (!ctx_man.hasDataSource(sensor_name))
            {
                setResultMessage("Invalid sensor '" + sensor_name + "'");
                return false;
            }

            auto ds_id = ctx_man.getDataSourceId(sensor_name);

            //deactivate sensor
            task.useDataSource(ds_id, false);
        }
    }

    if(!task.canRun())
    {
        setResultMessage("Reconstruct references task can not be run");
        return false;
    }

    task.allowUserInteractions(false);
    task.run();

    // if ok
    return true;
}

void RTCommandReconstructReferences::collectOptions_impl(OptionsDescription& options,
                                                         PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("config,c", po::value<std::string>()->default_value(""), "reconstructor configuration as json string")
        ("disable_sensors,d", po::value<std::string>()->default_value(""), "optional list of sensor names to disables, e.g. ‘GPS Trail 1;GPS Trail 2’");
}

void RTCommandReconstructReferences::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "config", std::string, config_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "disable_sensors", std::string, disabled_sensors_)
}

// analyze data source
RTCommandAnalyzeDataSource::RTCommandAnalyzeDataSource(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
    condition.setSignal("compass.taskmanager.analyzedatasourcetask.doneSignal", -1);
}

bool RTCommandAnalyzeDataSource::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline)
    {
        setResultMessage("Wrong application mode " + root_.compass().appState().appModeStr());
        return false;
    }

    if (ds_type_.empty())
    {
        setResultMessage("Missing required option --ds_type");
        return false;
    }

    AnalyzeDataSourceTask* task_ptr = root_.taskManager().analyzeDataSourceTask(ds_type_);
    if (!task_ptr)
    {
        setResultMessage("Unknown DSType '" + ds_type_ + "' (expected MLAT, ADSB or SMR)");
        return false;
    }
    AnalyzeDataSourceTask& task = *task_ptr;

    if (task.dsType() != ds_type_)
    {
        setResultMessage("Configured DSType '" + task.dsType()
                         + "' does not match requested '" + ds_type_ + "'");
        return false;
    }

    if (!config_.empty())
    {
        auto res = task.applyJSONStringParameters(config_);
        if (!res.ok())
        {
            setResultMessage("Could not apply configuration: " + res.error());
            return false;
        }
    }

    std::string reason;
    if (!task.canRunConfiguration(&reason))
    {
        setResultMessage("Analyze data source task cannot be run: " + reason);
        return false;
    }

    //name under which the task stores the report
    nlohmann::json reply;
    reply["report"] = task.reportName();
    setJSONReply(reply);

    task.allowUserInteractions(false);
    task.run();

    return true;
}

void RTCommandAnalyzeDataSource::collectOptions_impl(OptionsDescription& options,
                                                     PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("ds_type,t", po::value<std::string>()->required(),
         "DSType to analyze (mandatory), e.g. 'MLAT'")
        ("config,c", po::value<std::string>()->default_value(""),
         "analyze data source configuration as json string");
}

void RTCommandAnalyzeDataSource::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "ds_type", std::string, ds_type_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "config", std::string, config_)

    // The completion signal is hardcoded to the MLAT task instance in the
    // constructor. Every DSType has its own task instance under its own
    // configurable class name, and the object path resolves by class name, so
    // the signal must be redirected to the instance of the requested DSType.
    // Without it the wait condition spies the MLAT instance, whose doneSignal
    // never fires, and the command never completes - which also blocks any
    // queued follow-up such as --quit.
    // Must happen here at configure time, before the runner registers the spy.
    if (ds_type_ == "ADSB")
        condition.setSignal("compass.taskmanager.analyzeadsbdatasourcetask.doneSignal", -1);
    else if (ds_type_ == "SMR")
        condition.setSignal("compass.taskmanager.analyzesmrdatasourcetask.doneSignal", -1);
}

// load data
RTCommandLoadData::RTCommandLoadData(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
    condition.setSignal("mainwindow.dataLoaded", -1); // think about max duration
}

bool RTCommandLoadData::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    main_window->loadButtonSlot();

    // if ok
    return true;
}

// export vp report
/**
 */
RTCommandExportViewPointsReport::RTCommandExportViewPointsReport(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid  RTCommandExportViewPointsReport::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!filename_.size(), "Filename empty")

    return RTCommand::valid();
}

bool RTCommandExportViewPointsReport::run_impl()
{
    if (!filename_.size())
    {
        setResultMessage("Filename empty");
        return false;
    }

    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    MainWindow* main_window = dynamic_cast<MainWindow*> (rtcommand::mainWindow());
    traced_assert(main_window);

    //main_window->showViewPointsTab();

    ViewPointsReportGenerator& gen = root_.viewManager().viewPointsGenerator();

    ViewPointsReportGeneratorDialog& dialog = gen.dialog();
    // batch export: pure progress display, do not steal os focus from other applications
    dialog.setAttribute(Qt::WA_ShowWithoutActivating, true);
    dialog.show();

    QCoreApplication::processEvents();

    gen.reportPathAndFilename(filename_);
    gen.showDone(false);

    gen.run();

    return true;
}

void RTCommandExportViewPointsReport::collectOptions_impl(OptionsDescription& options,
                                          PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("filename,f", po::value<std::string>()->required(), "given filename, e.g. ’/data/db2/report.tex'");

    ADD_RTCOMMAND_POS_OPTION(positional, "filename") // give position
}

void RTCommandExportViewPointsReport::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "filename", std::string, filename_)
}

// export evaluation report
/**
 */
RTCommandExportReport::RTCommandExportReport(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid  RTCommandExportReport::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(!result_name_.size(), "Report name empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(mode_ == ResultReport::ReportExportMode::JSONBlob, "Command not available for JSONBlob mode")
    
    return RTCommand::valid();
}

bool RTCommandExportReport::run_impl()
{
    if (!result_name_.size())
    {
        setResultMessage("Report name empty");
        return false;
    }

    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    auto& task_manager = root_.taskManager();

    if (!task_manager.hasResult(result_name_))
    {
        setResultMessage("Report '" + result_name_ + "' not available");
        return false;
    }

    boost::optional<std::string> export_dir;
    if (!export_dir_.empty())
        export_dir = export_dir_;

    auto res = task_manager.exportResult(result_name_, mode_, true, export_dir);

    if (!res.ok())
    {
        setResultMessage(res.error());
        return false;
    }

    return true;
}

void RTCommandExportReport::collectOptions_impl(OptionsDescription& options,
                                                PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("report,r", po::value<std::string>()->required(), "report name, e.g. 'EUROCAE ED-87E Evaluation'")
        ("dir,f", po::value<std::string>()->default_value(""), "export directory, e.g. '/data/report2/'")
        ("mode,m", po::value<std::string>()->required(), "export mode, i.e. 'DocX','JSON','Latex','PDF'");

    ADD_RTCOMMAND_POS_OPTION(positional, "report")
    ADD_RTCOMMAND_POS_OPTION(positional, "dir"   )
    ADD_RTCOMMAND_POS_OPTION(positional, "mode"  )
}

void RTCommandExportReport::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "report", std::string, result_name_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "dir"   , std::string, export_dir_)

    std::string mode_str;
    RTCOMMAND_GET_VAR_OR_THROW(variables, "mode"  , std::string, mode_str)

    mode_ = ResultReport::reportExportModeFromString(mode_str);
}

// get_existing_reports

/**
*/
RTCommandGetExistingReports::RTCommandGetExistingReports(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

/**
*/
bool RTCommandGetExistingReports::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    auto& task_manager = root_.taskManager();

    std::vector<std::string> results;
    nlohmann::json           details = nlohmann::json::array();

    auto& dbcont_man = root_.compass().dbContentManager();

    for (const auto& r : task_manager.results())
    {
        const auto& result = *r.second;

        results.push_back(result.name());

        // the Report Table catalog with the Report Variables offered in the Views
        nlohmann::json detail;
        detail[ "name"   ] = result.name();
        detail[ "id"     ] = result.id();
        detail[ "tables" ] = nlohmann::json::array();

        for (const auto& table : result.reportTables())
        {
            nlohmann::json t;
            t[ "key"             ] = table.key;
            t[ "display_name"    ] = table.display_name;
            t[ "kind"            ] = table.kind == ReportTableKind::Record ? "record" : "cell";
            t[ "num_rows"        ] = table.num_rows;
            t[ "host_dbcontents" ] = table.host_dbcontents;
            t[ "columns"         ] = nlohmann::json::array();

            for (const auto& column : table.columns)
                t[ "columns" ].push_back(column.name);

            t[ "variables" ] = nlohmann::json::array();

            if (dbcont_man.existsReportContent(result.name()))
                for (const auto& var_it : dbcont_man.reportContent(result.name()).variables())
                    if (var_it.second->tableKey() == table.key)
                        t[ "variables" ].push_back(var_it.first);

            detail[ "tables" ].push_back(t);
        }

        details.push_back(detail);
    }

    nlohmann::json j;
    j["reports"] = results;
    j["details"] = details;

    setJSONReply(j);

    return true;
}

// get_report

/**
*/
RTCommandGetReport::RTCommandGetReport(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid RTCommandGetReport::valid() const 
{
    CHECK_RTCOMMAND_INVALID_CONDITION(result_name.empty(), "Report name missing")
    
    return rtcommand::RTCommand::valid(); 
}

/**
*/
bool RTCommandGetReport::run_impl()
{
    if (!root_.compass().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (root_.compass().appState().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+root_.compass().appState().appModeStr());
        return false;
    }

    auto& task_manager = root_.taskManager();

    if (!task_manager.hasResult(result_name))
    {
        setResultMessage("Report '" + result_name + "' not available");
        return false;
    }

    if (!section.empty())
    {
        //export specified section
        auto res = task_manager.exportResult(result_name, ResultReport::ReportExportMode::JSONBlob, true, {}, section);

        if (!res.ok())
        {
            setResultMessage(res.error());
            return false;
        }

        setJSONReply(res.result());
    }
    else
    {
        //return list of existing sections
        auto mode = ResultReport::ReportExportMode::JSONBlob;
        auto ids = task_manager.result(result_name)->report()->getReportSectionIDs(&mode);

        nlohmann::json j;
        j[ "sections" ] = ids;

        setJSONReply(j);
    }

    return true;
}

/**
*/
bool RTCommandGetReport::checkResult_impl()
{
    return true;
}

/**
 */
void RTCommandGetReport::collectOptions_impl(OptionsDescription &options,
                                             PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("report", po::value<std::string>()->required(), "name of the report to retrieve, e.g. 'EUROCAE ED-87E Evaluation'")
        ("section", po::value<std::string>()->default_value(""), "optional path of the section to retrieve, e.g. 'Report:Results:Overview:Results'");

    ADD_RTCOMMAND_POS_OPTION(positional, "report" )
    ADD_RTCOMMAND_POS_OPTION(positional, "section")
}

/**
 */
void RTCommandGetReport::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "report" , std::string, result_name)
    RTCOMMAND_GET_VAR_OR_THROW(vars, "section", std::string, section    )
}

// get_events

void RTCommandGetEvents::collectOptions_impl(OptionsDescription& options,
                                             PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("fresh", "return only fresh (unseen) events")
        ("max_items", po::value<unsigned int>()->default_value(0), "maximum number of items to return");
}

void RTCommandGetEvents::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_CHECK_VAR(variables, "fresh", fresh_)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "max_items", unsigned int, max_items_)
}

bool RTCommandGetEvents::run_impl()
{
    auto query = max_items_ > 0 ? logger::EventQuery(fresh_, logger::EventQuery::EventType::All, logger::EventQuery::QueryType::Newest, max_items_) :
                                  logger::EventQuery(fresh_, logger::EventQuery::EventType::All, logger::EventQuery::QueryType::All);

    auto json_obj = Logger::getInstance().getEventLog()->getEventsAsJSON(query);

    setJSONReply(json_obj);

    return true;
}

// reconfigure

/**
 */
RTCommandReconfigure::RTCommandReconfigure(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

rtcommand::IsValid RTCommandReconfigure::valid() const
{
    CHECK_RTCOMMAND_INVALID_CONDITION(path.empty(), "Path empty")
    CHECK_RTCOMMAND_INVALID_CONDITION(json_config.empty(), "JSON config empty")

    return RTCommand::valid(); 
}

void RTCommandReconfigure::collectOptions_impl(OptionsDescription& options,
                                               PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("path", po::value<std::string>()->required(), "configurable to reconfigure")
        ("config", po::value<std::string>()->required(), "json configuration to apply");

    ADD_RTCOMMAND_POS_OPTION(positional, "path")
    ADD_RTCOMMAND_POS_OPTION(positional, "config")
}

void RTCommandReconfigure::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "path", std::string, path)
    RTCOMMAND_GET_VAR_OR_THROW(variables, "config", std::string, json_config)
}

bool RTCommandReconfigure::run_impl()
{
    //try to find configurable by path
    auto find_result = root_.compass().findSubConfigurablePath(path);
    if (find_result.first != rtcommand::FindObjectErrCode::NoError)
    {
        //not found by path => try to find configurable by name
        find_result = root_.compass().findSubConfigurableName(path);

        //not found
        if (find_result.first != rtcommand::FindObjectErrCode::NoError)
        {
            setResultMessage("Configurable '" + path + "' not found");
            return false;
        }
    }

    nlohmann::json config;

    loginf << "config: " << json_config;

    try
    {
        config = nlohmann::json::parse(json_config);
    }
    catch(const std::exception& e)
    {
        setResultMessage("Parsing json config failed: " + std::string(e.what()));
        return false;
    }
    catch(...)
    {
        setResultMessage("Parsing json config failed: Unknown error");
        return false;
    }

    //reconfigure using json config
    std::vector<Configurable::MissingKey> missing_subconfig_keys;
    std::vector<Configurable::MissingKey> missing_param_keys;

    auto result = find_result.second->reconfigure(config, &missing_subconfig_keys, &missing_param_keys, false);

    bool ok = false;

    //everything ok?
    if (result.first == Configurable::ReconfigureError::NoError)
        ok = true;
    else if (result.first == Configurable::ReconfigureError::GeneralError)
        setResultMessage(result.second);
    else if (result.first == Configurable::ReconfigureError::PreCheckFailed)
        setResultMessage("Configuration incompatible");
    else if (result.first == Configurable::ReconfigureError::ApplyFailed)
        setResultMessage("Configuration could not be applied");
    else // Configurable::ReconfigureError::UnknownError
        setResultMessage("Unknown error");

    auto missingKeyType2String = [ & ] (Configurable::MissingKeyType type)
    {
        if (type == Configurable::MissingKeyType::CreationFailed)
            return "creation_failed";
        else if (type == Configurable::MissingKeyType::Skipped)
            return "skipped";
        else if (type == Configurable::MissingKeyType::Missing)
            return "missing";
        return "";
    };

    auto createKeyInformation = [ & ] (const std::vector<Configurable::MissingKey>& keys)
    {
        auto missing_keys_vec = nlohmann::json::array();
    
        for (const auto& key : missing_subconfig_keys)
        {
            nlohmann::json entry;
            entry[ "id"   ] = key.first;
            entry[ "type" ] = missingKeyType2String(key.second);

            missing_keys_vec.push_back(entry);
        }

        return missing_keys_vec;
    };
    
    //store missing keys to reply
    nlohmann::json json_reply;

    json_reply[ "missing_subconfig_keys" ] = createKeyInformation(missing_subconfig_keys);
    json_reply[ "missing_param_keys"     ] = createKeyInformation(missing_param_keys    );

    setJSONReply(json_reply);

    return ok;
}

// client_info

/**
 */
RTCommandClientInfo::RTCommandClientInfo(CompassRoot& root)
    : rtcommand::RTCommand()
    , root_(root)
{
}

bool RTCommandClientInfo::run_impl()
{
    nlohmann::json info;

    info[ "appimage" ] = AppState::isAppImage();
    info[ "version"  ] = root_.compass().config().getString("version");
    info[ "appmode"  ] = root_.compass().appState().appModeStr();

    setJSONReply(info);

    return true;
}

}
