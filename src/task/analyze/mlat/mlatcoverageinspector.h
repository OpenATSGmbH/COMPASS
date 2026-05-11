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

#include "datasourceinspectorbase.h"
#include "inspectorsettingsbase.h"

#include <cstdint>
#include <memory>

class TargetReport3DGrid;

class MLATCoverageInspectorSettings : public InspectorSettingsBase
{
public:
    MLATCoverageInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~MLATCoverageInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "MLATCoverageInspector"; }

    // PD calculation method
    enum class PDMethod : int { TimeDifference = 0, StatusPeriodMessage = 1 };
    int  pd_method_int_ = static_cast<int>(PDMethod::TimeDifference);

    PDMethod pdMethod() const
    { return static_cast<PDMethod>(pd_method_int_); }

    float update_interval_s_ = 1.0f;   // UI; Time Difference only.

    // Miss test (mirrors the detection requirement, simplified to miss tolerance only).
    bool  use_miss_tolerance_ = true;
    float miss_tolerance_s_   = 0.1f;

    // Reference-period construction: a gap larger than this in the loaded
    // reference chain starts a new period. Mirrors the per-standard
    // `referenceMaxTimeDiff()` used by the detection requirement.
    float ref_max_time_diff_s_ = 4.0f;

    // PD color thresholds: green at/above acceptable, red at/below unacceptable,
    // orange in between. Values are PD ratios (0..1).
    float pd_acceptable_above_    = 0.95f;
    float pd_unacceptable_below_  = 0.70f;
};

/**
 * Feature 2: PD per cell across a 3D grid using RefTraj as ground truth.
 *
 * The full algorithm (per-target slot-walk, gap construction, miss test) is
 * specified in src/task/analysis/mlat_ru/readme_analysis_mlat_ru.md. The data
 * walk is deferred to a subsequent iteration; this implementation produces the
 * settings recap section and an explicit placeholder for the report contents
 * that depend on the per-target loop.
 */
class MLATCoverageInspector : public DataSourceInspectorBase
{
public:
    MLATCoverageInspector(AnalyseDataSourceTask& task, MLATCoverageInspectorSettings& settings);

    std::string className() const override { return "MLATCoverageInspector"; }
    std::string name()      const override { return "Sensor Coverage"; }
    std::string dsType()    const override { return "MLAT"; }
    std::string description() const override
    {
        return "Per-cell Probability of Detection on a 3D grid (lat/lon/baro_alt). "
               "Reference Trajectory is the only ground truth.";
    }

    bool requiresReferenceTrajectory() const override { return true; }
    bool requiresLoadedDataset()       const override { return true; }

    std::set<std::string> testDBContentNames() const override;

    bool prerequisitesMet(std::string& reason_out) const override;

    void compute(AnalysisDataset* dataset) override;
    void writeReport(ResultReport::Section& root) override;

    ~MLATCoverageInspector() override;

private:
    /// Aggregated result of the per-target slot-walk. Populated by `compute()`
    /// (worker-thread safe) and consumed by `writeReport()` (main thread).
    struct ComputeResult
    {
        bool   valid              = false;
        unsigned int targets_walked = 0;
        unsigned int targets_no_ref = 0;
        unsigned int targets_no_tst = 0;
        std::uint64_t total_eui    = 0;
        std::uint64_t total_mui    = 0;
        double sector_pd           = 0.0;
        std::size_t cells_with_eui = 0;
        double median_per_cell_pd  = 0.0;
        double p5_per_cell_pd      = 0.0;
        bool   has_worst_cell      = false;
        double worst_cell_pd       = 1.0;
    };

    ComputeResult                      result_;
    std::unique_ptr<TargetReport3DGrid> grid_;
};
