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

#include "configurable.h"
#include "task.h"
#include "global.h"
#include "json_fwd.hpp"

#include <QObject>

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

class TaskManager;
class COMPASS;
class DataSourceInspectorBase;
class InspectorSettingsBase;

class MLATDataItemInspectorSettings;
class MLATCoverageInspectorSettings;

#if USE_EXPERIMENTAL_SOURCE == true
class MLATAccuracyInspectorSettings;
#endif

namespace ResultReport { class Section; }

class AnalyseDataSourceTask : public Task, public Configurable
{
    Q_OBJECT

public:
    AnalyseDataSourceTask(nlohmann::json& config, TaskManager* parent);
    ~AnalyseDataSourceTask() override;

    void generateSubConfigurable(nlohmann::json& child_json) override;

    void initTask() override final;
    bool canRun() override;
    void run() override;

    void showDialog();

    /// DSType this task analyses (currently always "MLAT").
    const std::string& dsType() const { return ds_type_; }

    /// Per-data-source enable flags.
    bool useDataSource(unsigned int ds_id) const;
    void useDataSource(unsigned int ds_id, bool value);
    bool useDataSourceLine(unsigned int ds_id, unsigned int line_id) const;
    void useDataSourceLine(unsigned int ds_id, unsigned int line_id, bool value);

    /// IDs of selected data sources of `ds_type_` (filters to existing DS only).
    std::set<unsigned int> selectedDataSourceIDs() const;

    /// True if any selected data source has CAT020 reports (asterixInfo lookup).
    bool selectionContainsCAT020() const;

    /// Inspector enabled / ticked state, keyed by inspector className().
    bool inspectorEnabled(const std::string& inspector_class) const;
    void inspectorEnabled(const std::string& inspector_class, bool value);

    /// Inspectors registered with this task. Pointers are owned by the task.
    const std::vector<std::unique_ptr<DataSourceInspectorBase>>& inspectors() const { return inspectors_; }

    /// Returns the inspector with the given class name, or nullptr.
    DataSourceInspectorBase* inspector(const std::string& class_name) const;

    MLATDataItemInspectorSettings& dataItemSettings() const;
    MLATCoverageInspectorSettings& coverageSettings()  const;

#if USE_EXPERIMENTAL_SOURCE == true
    MLATAccuracyInspectorSettings& accuracySettings() const;
#endif

    /// True if the active license enables Professional features.
    bool professionalLicenseEnabled() const;

    COMPASS& compass() const;

protected:
    void checkSubConfigurables() override;

private:
    void registerInspectors();

    std::string ds_type_;

    nlohmann::json use_data_sources_;        // ds_id -> bool
    nlohmann::json use_data_sources_lines_;  // ds_id -> line_id -> bool
    nlohmann::json inspector_enabled_;       // inspector_class_name -> bool

    std::unique_ptr<MLATDataItemInspectorSettings> data_item_settings_;
    std::unique_ptr<MLATCoverageInspectorSettings> coverage_settings_;

#if USE_EXPERIMENTAL_SOURCE == true
    std::unique_ptr<MLATAccuracyInspectorSettings> accuracy_settings_;
#endif

    std::vector<std::unique_ptr<DataSourceInspectorBase>> inspectors_;
};
