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

#include <QObject>

class AnalyzeDataSourceTask;

/**
 * TaskResult subclass produced by AnalyzeDataSourceTask, tagged DataSourceAnalysis (=2) in the
 * persisted `task_results.type` column, distinct from Generic (=0) which is what ad-hoc /
 * reconstruction / import results carry.
 *
 * The result stores the task configuration of the run, so it follows the update and lock state of
 * the data like an evaluation report does, and an update re-runs the analysis with the stored
 * configuration.
 */
class DataSourceAnalysisTaskResult : public QObject, public TaskResult
{
    Q_OBJECT

public:
    DataSourceAnalysisTaskResult(unsigned int id, TaskManager& task_man);
    ~DataSourceAnalysisTaskResult() override = default;

    task::TaskResultType type() const override final
    {
        return task::TaskResultType::DataSourceAnalysis;
    }

    /// task of the stored configuration's DSType, null if there is none
    AnalyzeDataSourceTask* analysisTask() const;

protected:
    Result update_impl(UpdateState state) override final;
    Result canUpdate_impl(UpdateState state) const override final;

protected slots:
    void informUpdateAnalysisResult(int state);
};
