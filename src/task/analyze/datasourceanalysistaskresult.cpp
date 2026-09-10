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

#include "datasourceanalysistaskresult.h"
#include "analyzedatasourcetask.h"
#include "taskmanager.h"
#include "compass.h"
#include "evaluationmanager.h"
#include "logger.h"

#include "json.hpp"

/**
 */
DataSourceAnalysisTaskResult::DataSourceAnalysisTaskResult(unsigned int id, TaskManager& task_man)
    : TaskResult(id, task_man)
{
    //the data change signals of the evaluation manager apply to the analysis as well
    connect(&task_man.compass().evaluationManager(), &EvaluationManager::resultsNeedUpdate,
            this, &DataSourceAnalysisTaskResult::informUpdateAnalysisResult);
}

/**
 */
AnalyzeDataSourceTask* DataSourceAnalysisTaskResult::analysisTask() const
{
    if (!hasJSONConfiguration())
        return nullptr;

    const auto& config = jsonConfiguration();

    if (!config.is_object() || !config.contains("parameters") || !config[ "parameters" ].is_object())
        return nullptr;

    const auto& params = config[ "parameters" ];

    if (!params.contains("ds_type") || !params[ "ds_type" ].is_string())
        return nullptr;

    return taskManager().analyzeDataSourceTask(params[ "ds_type" ].get<std::string>());
}

/**
 */
Result DataSourceAnalysisTaskResult::canUpdate_impl(UpdateState state) const
{
    if (!analysisTask())
        return Result::failed("No analysis task for the stored configuration");

    return Result::succeeded();
}

/**
 * Re-runs the analysis with the stored configuration, which replaces this report.
 */
Result DataSourceAnalysisTaskResult::update_impl(UpdateState state)
{
    auto* task = analysisTask();
    if (!task)
        return Result::failed("No analysis task for the stored configuration");

    auto res = task->applyStoredConfiguration(jsonConfiguration());
    if (!res.ok())
        return res;

    if (!task->canRun())
        return Result::failed("Analysis task cannot run with the stored configuration");

    loginf << "running full update of analysis report '" << name() << "'";

    task->run();

    return Result::succeeded();
}

/**
 * Data changes and locks apply, the evaluation specific partial and content updates do not.
 */
void DataSourceAnalysisTaskResult::informUpdateAnalysisResult(int state)
{
    auto update_state = (UpdateState)state;

    if (update_state == UpdateState::FullUpdateNeeded || update_state == UpdateState::Locked)
        informUpdate(update_state);
}
