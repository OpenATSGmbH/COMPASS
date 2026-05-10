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

class AnalysisDataset;

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

    /// Per-data-source enable flags (test side).
    bool useDataSource(unsigned int ds_id) const;
    void useDataSource(unsigned int ds_id, bool value);
    bool useDataSourceLine(unsigned int ds_id, unsigned int line_id) const;
    void useDataSourceLine(unsigned int ds_id, unsigned int line_id, bool value);

    /// IDs of selected test data sources of `ds_type_` (filters to existing DS only).
    std::set<unsigned int> selectedDataSourceIDs() const;

    /// Common test-side line ID (0..3, displayed as L1..L4).
    unsigned int lineIDTst() const { return line_id_tst_; }
    void setLineIDTst(unsigned int line_id);

    /// Per-data-source enable flags for the reference (RefTraj) side.
    bool useReferenceDataSource(unsigned int ds_id) const;
    void useReferenceDataSource(unsigned int ds_id, bool value);

    /// IDs of selected reference (RefTraj) data sources (filters to existing DS only).
    std::set<unsigned int> selectedReferenceDataSourceIDs() const;

    /// Reference-side line ID (0..3).
    unsigned int lineIDRef() const { return line_id_ref_; }
    void setLineIDRef(unsigned int line_id);

    /// All existing RefTraj data source IDs in the active context.
    std::set<unsigned int> referenceDataSourceCandidateIDs() const;

    /// Editable report name. `reportName()` returns the custom name if set,
    /// otherwise `suggestReportName()`.
    std::string reportName() const;
    std::string suggestReportName() const;
    bool hasCustomReportName() const { return !custom_report_name_.empty(); }
    void setCustomReportName(const std::string& name);
    void resetCustomReportName();

    /// Shared grid resolution (consumed by all 3D-grid inspectors).
    float cellSizeMeters() const { return cell_size_m_; }
    float cellSizeFeet()   const { return cell_size_ft_; }
    unsigned int maxCellsPerAxis() const { return max_cells_per_axis_; }
    void setCellSizeMeters(float v);
    void setCellSizeFeet(float v);
    void setMaxCellsPerAxis(unsigned int v);

    /// Effective cell sizes for an inspector: the configured horizontal and
    /// vertical cell sizes, multiplied by an integer factor on the 1-2-5
    /// ladder (1, 2, 5, 10, 20, 50, ...) only as much as is needed to keep
    /// each axis under `maxCellsPerAxis()` cells across the loaded data
    /// extent. The factor is the smallest ladder value that fits, so the
    /// resulting cells are always integer multiples of the configured cell
    /// size and stay close to the limit.
    ///
    /// Inputs come from the dataset extents (lat/lon/alt) - the task does not
    /// scan the data itself. Altitude defaults to 50000 ft when the dataset
    /// did not record an altitude extent.
    struct CellSizing
    {
        float        cell_size_m;
        float        cell_size_ft;
        unsigned int horizontal_multiplier; // applied to configured cell_size_m
        unsigned int vertical_multiplier;   // applied to configured cell_size_ft
        bool         horizontal_clamped;
        bool         vertical_clamped;
    };
    CellSizing clampedCellSizes(const AnalysisDataset& dataset) const;

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

    nlohmann::json use_data_sources_;            // ds_id -> bool (test side)
    nlohmann::json use_data_sources_lines_;      // ds_id -> line_id -> bool
    nlohmann::json use_reference_data_sources_;  // ds_id -> bool (reference side)
    nlohmann::json inspector_enabled_;           // inspector_class_name -> bool

    unsigned int line_id_tst_ = 0;
    unsigned int line_id_ref_ = 0;
    std::string  custom_report_name_;

    float        cell_size_m_        = 20.0f;
    float        cell_size_ft_       = 100.0f;
    unsigned int max_cells_per_axis_ = 1000;

    std::unique_ptr<MLATDataItemInspectorSettings> data_item_settings_;
    std::unique_ptr<MLATCoverageInspectorSettings> coverage_settings_;

#if USE_EXPERIMENTAL_SOURCE == true
    std::unique_ptr<MLATAccuracyInspectorSettings> accuracy_settings_;
#endif

    std::vector<std::unique_ptr<DataSourceInspectorBase>> inspectors_;
};
