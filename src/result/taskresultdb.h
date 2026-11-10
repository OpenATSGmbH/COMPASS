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

#include <cstdint>

#include "result.h"

#include "json.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <vector>

class DBInterface;
class ITaskProvider;
class TaskResult;

namespace ResultReport
{
    class Section;
    class SectionContent;
}

/**
 * Persistence of the task results in the database: the results, report contents and per-target
 * report sections tables.
 * - built on the generic table operations of DBInterface
 * - owned by TaskManager, which creates the loaded results (ITaskProvider::createResult())
 */
class TaskResultDB
{
public:
    TaskResultDB(DBInterface& db_interface, ITaskProvider& task_provider);

    bool existsTaskResultsTable() const;
    bool existsReportContentsTable() const;
    bool existsReportSectionsTable() const;
    void createTaskResultsTable();
    void createReportContentsTable();
    void createReportSectionsTable();
    Result saveResult(const TaskResult& result,
                      bool cleanup_db_if_needed);
    Result deleteResult(const TaskResult& result,
                        bool cleanup_db_if_needed,
                        bool* deleted = nullptr,
                        bool keep_own_tables = false);

    Result updateResultHeader(const TaskResult& result);
    Result updateResultContent(const TaskResult& result);
    ResultT<std::vector<std::shared_ptr<TaskResult>>> loadResults();
    ResultT<nlohmann::json> loadResultContent(unsigned int result_id);
    ResultT<std::shared_ptr<ResultReport::SectionContent>> loadContent(ResultReport::Section* section, unsigned int content_id);

    boost::optional<std::uint64_t> getMaxReportContentID();

private:
    nlohmann::json resultContentWithoutSections(const TaskResult& result,
                                                std::vector<std::string>& sections) const;
    void deleteReportSections(unsigned int result_id);
    void writeReportSections(unsigned int result_id,
                             std::vector<std::string>& sections);

    DBInterface&   db_interface_;
    ITaskProvider& task_provider_;
};
