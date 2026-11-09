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

#include "taskdefs.h"
#include "result.h"

#include "json.hpp"

#include <memory>
#include <string>

class AppState;
class DBInterface;
class DBContentManager;
class LicenseManager;
class ViewManager;
class TaskResult;

namespace ResultReport
{
    class Report;
    class Section;
    class SectionContent;
}

/**
 * The task manager functions that code below the task layer uses (results, reports, evaluation),
 * via COMPASS::taskProvider() or the task results. Implemented by TaskManager.
 */
class ITaskProvider
{
public:
    virtual ~ITaskProvider() = default;

    // managers the results and reports use
    virtual AppState& appState() = 0;
    virtual DBInterface& dbInterface() = 0;
    virtual DBContentManager& dbContentManager() = 0;
    virtual LicenseManager& licenseManager() = 0;
    virtual ViewManager& viewManager() = 0; // views for figure rendering and export

    // result writing
    virtual void beginTaskResultWriting(const std::string& name,
                                        task::TaskResultType type,
                                        bool clear_existing = true) = 0;
    virtual bool hasCurrentResult() const = 0;
    virtual std::shared_ptr<TaskResult>& currentResult() = 0;
    virtual std::shared_ptr<ResultReport::Report>& currentReport() = 0;
    virtual void endTaskResultWriting(bool store_result,
                                      bool show_dialog = false) = 0;

    virtual std::shared_ptr<TaskResult> result(const std::string& name) const = 0; // get existing result
    virtual bool hasResult(const std::string& name) const = 0;

    virtual std::shared_ptr<TaskResult> createResult(unsigned int id,
                                                     task::TaskResultType type) = 0; // new result of the given type

    // callbacks of results and reports
    virtual void resultHeaderChanged(const TaskResult& result) = 0;
    virtual void resultContentChanged(const TaskResult& result) = 0;

    virtual void setViewableDataConfig(const nlohmann::json::object_t& data,
                                       bool load_blocking = false) = 0;
    virtual void unsetViewableDataConfig() = 0;

    virtual std::shared_ptr<ResultReport::SectionContent> loadContent(ResultReport::Section* section,
                                                                      unsigned int content_id,
                                                                      bool show_dialog = false) const = 0;
    virtual ResultT<nlohmann::json> loadResultContent(unsigned int result_id) const = 0;

    virtual void storeBackupSection() = 0;
    virtual void restoreBackupSection() = 0;

    virtual void selectResultID(const std::string& id, bool show_figure) = 0; // select in the results widget
};
