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

#include "analyze_commands.h"
#include "analyzedatasourcetask.h"
#include "datasourceinspectorbase.h"
#include "compass.h"
#include "taskmanager.h"
#include "rtcommand/rtcommand_macros.h"
#include "rtcommand_registry.h"

#include "json.hpp"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(RTCommandGetAnalyzeInspectors)

static COMPASS* s_compass = nullptr;

void init_analyze_commands(COMPASS& compass)
{
    s_compass = &compass;
    RTCommandGetAnalyzeInspectors::init();
}

RTCommandGetAnalyzeInspectors::RTCommandGetAnalyzeInspectors()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetAnalyzeInspectors::run_impl()
{
    if (!s_compass)
    {
        setResultMessage("COMPASS not initialized");
        return false;
    }

    if (ds_type_.empty())
    {
        setResultMessage("Missing required option --ds_type");
        return false;
    }

    AnalyzeDataSourceTask& task = (ds_type_ == "ADSB")
        ? s_compass->taskManager().analyzeADSBDataSourceTask()
        : s_compass->taskManager().analyzeMLATDataSourceTask();

    if (task.dsType() != ds_type_)
    {
        setResultMessage("Configured DSType '" + task.dsType()
                         + "' does not match requested '" + ds_type_ + "'");
        return false;
    }

    return true;
}

bool RTCommandGetAnalyzeInspectors::checkResult_impl()
{
    AnalyzeDataSourceTask& task = (ds_type_ == "ADSB")
        ? s_compass->taskManager().analyzeADSBDataSourceTask()
        : s_compass->taskManager().analyzeMLATDataSourceTask();

    nlohmann::json inspectors_array = nlohmann::json::array();

    for (const auto& ins_ptr : task.inspectors())
    {
        const DataSourceInspectorBase& ins = *ins_ptr;

        nlohmann::json j;
        j["class_name"]  = ins.className();
        j["name"]        = ins.name();
        j["description"] = ins.description();
        j["tier"]        = ins.requiresProfessionalLicense() ? "pro" : "free";

        inspectors_array.push_back(std::move(j));
    }

    nlohmann::json task_defaults;
    task_defaults["cell_size_m"]        = task.cellSizeMeters();
    task_defaults["cell_size_ft"]       = task.cellSizeFeet();
    task_defaults["max_cells_per_axis"] = task.maxCellsPerAxis();

    nlohmann::json reply;
    reply["ds_type"]       = task.dsType();
    reply["inspectors"]    = std::move(inspectors_array);
    reply["task_defaults"] = std::move(task_defaults);
    setJSONReply(reply);

    return true;
}

void RTCommandGetAnalyzeInspectors::collectOptions_impl(OptionsDescription& options,
                                                       PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("ds_type,t", po::value<std::string>()->required(),
         "DSType whose inspectors should be listed (e.g. 'MLAT')");
}

void RTCommandGetAnalyzeInspectors::assignVariables_impl(const VariablesMap& variables)
{
    RTCOMMAND_GET_VAR_OR_THROW(variables, "ds_type", std::string, ds_type_)
}
