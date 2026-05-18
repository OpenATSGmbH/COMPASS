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

#include "taskresult.h"

/**
 * TaskResult subclass produced by AnalyzeDataSourceTask. The only override
 * is `type()` so this result is tagged DataSourceAnalysis (=2) in the
 * persisted `task_results.type` column, distinct from Generic (=0) which
 * is what ad-hoc / reconstruction / import results carry. Lets downstream
 * consumers (web KPI extraction, report listings, future per-type
 * filtering) discriminate analysis reports without name-pattern matching.
 */
class DataSourceAnalysisTaskResult : public TaskResult
{
public:
    DataSourceAnalysisTaskResult(unsigned int id, TaskManager& task_man);
    ~DataSourceAnalysisTaskResult() override = default;

    task::TaskResultType type() const override final
    {
        return task::TaskResultType::DataSourceAnalysis;
    }
};
