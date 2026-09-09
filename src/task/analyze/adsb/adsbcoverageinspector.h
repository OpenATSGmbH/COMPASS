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

class ADSBCoverageInspectorSettings : public InspectorSettingsBase
{
public:
    ADSBCoverageInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~ADSBCoverageInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "ADSBCoverageInspector"; }

    // ADS-B has no Remote Units and no CAT019-style cycle messages, so the
    // cadence is always time-difference against a nominal Update Interval.
    // Surface squitter is motion-adaptive: aircraft transmit at the moving rate
    // while taxiing and drop to a slower rate when stopped, so a standing target
    // is "expected" less often. A slot is standing when the ADS-B-reported ground
    // speed (RefTraj speed as fallback) is below `standing_speed_max_mps_`; such
    // slots use `update_interval_standing_s_` instead of the moving `update_interval_s_`.
    float update_interval_s_          = 1.0f;   // moving nominal UI
    float update_interval_standing_s_ = 5.0f;   // standing nominal UI
    float standing_speed_max_mps_     = 0.5f;   // ground speed below this = standing

    // Miss test (mirrors the detection requirement, simplified to miss tolerance only).
    bool  use_miss_tolerance_ = true;
    float miss_tolerance_s_   = 0.1f;

    // Reference-period construction: a gap larger than this in the loaded
    // reference chain starts a new period.
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
 * Feature 2 (ADS-B): operational Probability of Detection per cell of the
 * shared 3D grid (lat/lon/baro_alt) using RefTraj as ground truth, broken
 * down per transponder (24-bit ICAO aircraft address).
 *
 * The aggregate per-cell algorithm is identical to the MLAT coverage
 * inspector (time-difference slot-walk, gap construction, miss test). In
 * addition, the per-target #EUI / #MUI totals are accumulated per aircraft
 * address so the report can surface individual transponders with coverage
 * gaps that an aggregate map averages away. See
 * experimental_src/analysis/readme_analysis_adsb.md (Feature 2).
 */
class ADSBCoverageInspector : public DataSourceInspectorBase
{
public:
    ADSBCoverageInspector(AnalyzeDataSourceTask& task, ADSBCoverageInspectorSettings& settings);
    ~ADSBCoverageInspector() override;

    std::string className() const override { return "ADSBCoverageInspector"; }
    std::string name()      const override { return "Sensor Coverage"; }
    std::string dsType()    const override { return "ADSB"; }
    std::string description() const override
    {
        return "Per-cell Probability of Detection on a 3D grid (lat/lon/baro_alt), "
               "broken down per transponder. Reference Trajectory is the only ground truth.";
    }

    bool requiresReferenceTrajectory() const override { return true; }
    bool requiresLoadedDataset()       const override { return true; }

    std::set<std::string> testDBContentNames() const override;

    bool prerequisitesMet(std::string& reason_out) const override;

    void compute(AnalysisDataset* dataset) override;
    void writeReport(ResultReport::Section& root) override;

private:
    /// One transponder (aircraft address) row in the per-transponder table.
    struct TransponderRow
    {
        unsigned int  acad     = 0;
        bool          has_acad = false;
        std::string   callsign;
        std::uint64_t eui      = 0;
        std::uint64_t mui      = 0;
    };

    /// One sector row in the "PD by Sector" overview table. The "All" reference
    /// row reuses the aggregate ComputeResult.
    struct SectorRow
    {
        std::string   label;
        std::size_t   num_transponders = 0;
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

        // Update-interval histogram: `ui_hist_bins` are the per-bin counts over
        // [0, ui_hist_max_s] with width ui_hist_bin_width; ui_hist_overflow holds
        // the count of intervals at or above ui_hist_max_s (the single overflow bin).
        double ui_hist_max_s      = 0.0;
        double ui_hist_bin_width  = 0.0;
        std::vector<std::uint32_t> ui_hist_bins;
        std::uint32_t ui_hist_overflow = 0;
        std::size_t cells_with_eui = 0;
        double median_per_cell_pd  = 0.0;
        double p5_per_cell_pd      = 0.0;
        bool   has_worst_cell      = false;
        double worst_cell_pd       = 1.0;

        std::vector<TransponderRow> transponders;  // sorted by PD ascending
        std::size_t type_groups_dropped = 0;

        std::vector<SectorRow> sectors;  // per selected sector layer
    };

    /// One MOPS-version or target-type group: aggregate PD plus a grid holding
    /// only that group's slots, rendered as a summary heat-map.
    struct Group
    {
        std::string   label;
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        std::unique_ptr<TargetReport3DGrid> grid;
    };

    ComputeResult                      result_;
    std::unique_ptr<TargetReport3DGrid> grid_;
    std::vector<Group>                 mops_groups_;
    std::vector<Group>                 type_groups_;
};
