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
#include <string>
#include <vector>

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

    float update_interval_s_ = 1.0f;   // moving UI; Time Difference only.

    // Motion-adaptive cadence (Time Difference only): a standing target (MLAT
    // reports it less often) is expected at the slower standing UI instead of
    // accruing false misses. A slot is standing when the reported ground speed
    // (RefTraj speed as fallback) is below `standing_speed_max_mps_`; unknown
    // speed counts as moving. MLAT surface cadence is fast, so the standing UI
    // is shorter than the ADS-B equivalent.
    float update_interval_standing_s_ = 2.0f;   // standing UI
    float standing_speed_max_mps_     = 0.5f;   // ground speed below this = standing

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

    // Update-interval histogram. `ui_hist_max_s_` is the upper edge; intervals
    // at or above it fold into a single overflow bin (so long detection breaks
    // do not stretch the axis). 0 = auto-derive a rounded-up max from the data.
    int   ui_hist_num_bins_ = 20;
    float ui_hist_max_s_    = 15.0f;
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
    MLATCoverageInspector(AnalyzeDataSourceTask& task, MLATCoverageInspectorSettings& settings);

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
    /// One sector row in the "PD by Sector" overview table. The "All" reference
    /// row reuses the aggregate ComputeResult.
    struct SectorRow
    {
        std::string   label;
        std::size_t   num_targets = 0;
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        double        pd  = 0.0;
    };

    /// Aggregated result of the per-target slot-walk. Populated by `compute()`
    /// (worker-thread safe) and consumed by `writeReport()` (main thread).
    struct ComputeResult
    {
        bool   valid              = false;
        std::string error;
        unsigned int targets_walked = 0;
        unsigned int targets_no_ref = 0;
        unsigned int targets_no_tst = 0;
        std::uint64_t total_eui    = 0;
        std::uint64_t total_mui    = 0;
        double overall_pd          = 0.0;

        // Measured per-target inter-report interval (the actual update cadence),
        // to help the operator pick a representative nominal Update Interval.
        std::size_t ui_num_intervals = 0;
        double ui_median_s = 0.0;
        double ui_p10_s    = 0.0;
        double ui_p90_s    = 0.0;
        double ui_mean_s   = 0.0;

        // Update-interval histogram: per-bin counts over [0, ui_hist_max_s] with
        // width ui_hist_bin_width; ui_hist_overflow holds the count of intervals
        // at or above ui_hist_max_s (the single overflow bin).
        double ui_hist_max_s      = 0.0;
        double ui_hist_bin_width  = 0.0;
        std::vector<std::uint32_t> ui_hist_bins;
        std::uint32_t ui_hist_overflow = 0;

        std::size_t cells_with_eui = 0;
        double median_per_cell_pd  = 0.0;
        double p5_per_cell_pd      = 0.0;
        bool   has_worst_cell      = false;
        double worst_cell_pd       = 1.0;

        std::vector<SectorRow> sectors;  // per selected sector layer
    };

    ComputeResult                      result_;
    std::unique_ptr<TargetReport3DGrid> grid_;
};
