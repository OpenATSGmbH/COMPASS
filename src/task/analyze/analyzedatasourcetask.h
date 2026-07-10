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
class SectorLayer;

class AnalysisDataset;

class MLATDataItemInspectorSettings;
class MLATCoverageInspectorSettings;

class ADSBDataItemInspectorSettings;
class ADSBCoverageInspectorSettings;

#if USE_EXPERIMENTAL_SOURCE == true
class MLATAccuracyInspectorSettings;
class MLATRUCoverageInspectorSettings;
class MLATRUEffectInspectorSettings;
class ADSBAccuracyInspectorSettings;
#endif

namespace ResultReport { class Section; }

class AnalyzeDataSourceTask : public Task, public Configurable
{
    Q_OBJECT

public:
    /// `ds_type_default` binds this task instance to a DSType ("MLAT" or "ADSB"):
    /// it sets the default for the `ds_type` parameter and selects which
    /// inspector set is registered. `object_name` distinguishes the two
    /// instances (one per DSType menu entry). The defaults preserve the
    /// original MLAT-only behavior.
    AnalyzeDataSourceTask(nlohmann::json& config, TaskManager* parent,
                          const std::string& ds_type_default = "MLAT",
                          const std::string& object_name = "AnalyzeDataSourceTask");
    ~AnalyzeDataSourceTask() override;

    void generateSubConfigurable(nlohmann::json& child_json) override;

    /// Accept JSON-injected configuration. Extends the base behavior with a
    /// top-level `inspector_settings` key shaped as `{<InspectorClassName>:
    /// {...params...}}`; each entry is forwarded to the matching inspector
    /// settings via its own `applyJSONParameters()`. The remaining keys are
    /// applied to the task's own registered parameters.
    Result applyJSONParameters(const nlohmann::json& params_json) override;

    void initTask() override final;
    bool canRun() override;
    void run() override;

    void showDialog();

    /// DSType this task instance analyzes ("MLAT" or "ADSB").
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

    /// Shared per-report scope filter (ground-only / flight-level band),
    /// applied when the combined dataset is built (all grid inspectors).
    bool  useGroundOnly() const { return use_ground_only_; }
    bool  useMinFL()      const { return use_min_fl_; }
    float minFL()         const { return min_fl_; }
    bool  useMaxFL()      const { return use_max_fl_; }
    float maxFL()         const { return max_fl_; }
    void  setUseGroundOnly(bool v) { use_ground_only_ = v; }
    void  setUseMinFL(bool v)      { use_min_fl_ = v; }
    void  setMinFL(float v)        { min_fl_ = v; }
    void  setUseMaxFL(bool v)      { use_max_fl_ = v; }
    void  setMaxFL(float v)        { max_fl_ = v; }

    /// Limit the analysis to reports inside the selected sector layers (precise
    /// per-report inside test, as in the evaluation). When off, all data is used.
    bool  limitBySectors() const { return limit_by_sectors_; }
    void  setLimitBySectors(bool v) { limit_by_sectors_ = v; }
    /// Per sector-layer "used" flag (defaults to true for an unlisted layer).
    bool  useSector(const std::string& layer_name) const;
    void  setUseSector(const std::string& layer_name, bool value);
    /// Names of the currently selected (used) sector layers that still exist.
    std::vector<std::string> selectedSectorLayers() const;
    /// The selected sector layers resolved to objects, for the per-sector
    /// result breakdown (independent of `limit_by_sectors_`).
    std::vector<std::shared_ptr<SectorLayer>> scopeSectorLayers() const;

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

    ADSBDataItemInspectorSettings& adsbDataItemSettings() const;
    ADSBCoverageInspectorSettings& adsbCoverageSettings() const;

#if USE_EXPERIMENTAL_SOURCE == true
    MLATAccuracyInspectorSettings&   accuracySettings()   const;
    MLATRUCoverageInspectorSettings& ruCoverageSettings() const;
    MLATRUEffectInspectorSettings&   ruEffectSettings()   const;
    ADSBAccuracyInspectorSettings&   adsbAccuracySettings() const;
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

    bool         use_ground_only_    = false;
    bool         use_min_fl_         = false;
    float        min_fl_             = 0.0f;
    bool         use_max_fl_         = false;
    float        max_fl_             = 600.0f;

    bool           limit_by_sectors_ = false;
    nlohmann::json used_sectors_;     // sector-layer name -> bool (unlisted = true)

    std::unique_ptr<MLATDataItemInspectorSettings> data_item_settings_;
    std::unique_ptr<MLATCoverageInspectorSettings> coverage_settings_;

    std::unique_ptr<ADSBDataItemInspectorSettings> adsb_data_item_settings_;
    std::unique_ptr<ADSBCoverageInspectorSettings> adsb_coverage_settings_;

#if USE_EXPERIMENTAL_SOURCE == true
    std::unique_ptr<MLATAccuracyInspectorSettings>   accuracy_settings_;
    std::unique_ptr<MLATRUCoverageInspectorSettings> ru_coverage_settings_;
    std::unique_ptr<MLATRUEffectInspectorSettings>   ru_effect_settings_;
    std::unique_ptr<ADSBAccuracyInspectorSettings>   adsb_accuracy_settings_;
#endif

    std::vector<std::unique_ptr<DataSourceInspectorBase>> inspectors_;
};
