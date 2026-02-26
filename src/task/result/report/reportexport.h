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

#include "reportdefs.h"
#include "configurable.h"

#include "result.h"

#include <map>
#include <string>
#include <memory>

#include <QObject>

class COMPASS;
class TaskResult;
class TaskManager;

namespace ResultReport
{

class Section;
class SectionContent;

class ReportExporter;

/**
 */
class ReportExport : public QObject, public Configurable
{
    Q_OBJECT
public:
    // ReportExport(const std::string& class_id,
    //              const std::string& instance_id,
    //              TaskManager* task_manager);
    /// @brief Constructor backed by a json reference
    ReportExport(nlohmann::json& config, TaskManager* parent);
    virtual ~ReportExport();

    COMPASS& compass() const;

    ResultT<nlohmann::json> exportReport(TaskResult& result,
                                         ReportExportMode mode,
                                         const std::string& fn,
                                         const std::string& resource_dir,
                                         const std::string& section = std::string(),
                                         bool interaction_mode = true);

    ReportExportSettings& settings() { return settings_; }
    const ReportExportSettings& settings() const { return settings_; }

    const std::string& status() const { return status_; }
    double progress() const { return progress_; }

signals:
    void progressChanged();

private:
    TaskManager& task_manager_;

    std::unique_ptr<ReportExporter> createExporter(ReportExportMode mode,
                                                   const std::string& fn,
                                                   const std::string& resource_dir,
                                                   bool interaction_mode) const;
    void updateProgress(ReportExporter* exporter);

    ReportExportSettings settings_;

    std::string status_;
    double      progress_ = 0.0;
};

} // namespace ResultReport
