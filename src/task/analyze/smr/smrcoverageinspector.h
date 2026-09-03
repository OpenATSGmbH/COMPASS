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

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class TargetReport3DGrid;

/**
 * Settings of the SMR coverage inspector.
 */
class SMRCoverageInspectorSettings : public InspectorSettingsBase
{
public:
    SMRCoverageInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~SMRCoverageInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "SMRCoverageInspector"; }

    // Cadence source: scan cycles from the source's CAT010 Start of Update
    // Cycle messages (per data source), or a fixed nominal scan period.
    enum class CadenceSource : int { ScanCycles = 0, NominalPeriod = 1 };

    int cadence_source_int_ = static_cast<int>(CadenceSource::ScanCycles);
    CadenceSource cadenceSource() const
    { return static_cast<CadenceSource>(cadence_source_int_); }

    float scan_period_s_ = 1.0f;   // nominal scan period, also the fallback without cycles

    // Small target rule: a small target must produce exactly one report per
    // scan, extra reports are counted as splits. Large targets may produce
    // several (0 = unlimited). Size class from the reconstructed target
    // category, else from the reported target length (I010/270).
    float        small_target_max_length_m_        = 20.0f;
    unsigned int large_target_max_reports_per_scan_ = 0;

    // Slots with a reference position farther from the antenna than this are
    // skipped (0 = no limit, also when the data source has no position).
    float max_range_m_ = 5000.0f;

    // Estimated coverage. Per azimuth bin the radial band [r_min, r_max] of
    // the source's own target reports is taken as the area the sensor covers
    // in that direction. A slot whose reference position falls outside its
    // source's band is not expected, so PD is measured only where the sensor
    // has shown coverage. An SMR does not see the whole airport: buildings
    // shadow whole azimuth wedges and stands, and the range is limited.
    // `coverage_min_reports_` rejects bins carrying too few reports to define
    // a band, they count as no coverage. `coverage_smooth_bins_` closes single
    // empty bins between two covered ones. 0 as the bin width disables the
    // estimate and every slot inside `max_range_m_` is expected again.
    float        coverage_azimuth_bin_deg_ = 0.5f;
    unsigned int coverage_min_reports_     = 20;
    unsigned int coverage_smooth_bins_     = 1;

    // Reference-period construction: a gap larger than this in the loaded
    // reference chain starts a new period.
    float ref_max_time_diff_s_ = 4.0f;

    // Diagnostic standing / moving breakdown only (the expectation is one
    // report per scan regardless of movement).
    float standing_speed_max_mps_ = 0.5f;

    // PD color thresholds (ratios 0..1).
    float pd_acceptable_above_    = 0.95f;
    float pd_unacceptable_below_  = 0.70f;

    // Extra reports per expected slot (split map) color thresholds.
    float extra_ratio_acceptable_below_   = 0.02f;
    float extra_ratio_unacceptable_above_ = 0.20f;

    // Measured inter-report interval histogram.
    int   ui_hist_num_bins_ = 20;
    float ui_hist_max_s_    = 5.0f;
};

/**
 * SMR Feature 2: scan-based Probability of Detection per cell of the
 * horizontal grid, with the number of reports per target and scan. See
 * experimental_src/analysis/readme_analysis_smr.md (Feature 2).
 */
class SMRCoverageInspector : public DataSourceInspectorBase
{
public:
    SMRCoverageInspector(AnalyzeDataSourceTask& task, SMRCoverageInspectorSettings& settings);
    ~SMRCoverageInspector() override;

    std::string className() const override { return "SMRCoverageInspector"; }
    std::string name()      const override { return "Sensor Coverage"; }
    std::string dsType()    const override { return "SMR"; }
    std::string description() const override
    {
        return "Scan-based Probability of Detection per cell of the horizontal grid, "
               "one report per target and antenna scan expected. Small targets must "
               "produce exactly one report per scan, large targets may produce several.";
    }

    bool requiresReferenceTrajectory() const override { return true; }
    bool requiresLoadedDataset()       const override { return true; }
    std::set<std::string> testDBContentNames() const override;

    bool prerequisitesMet(std::string& reason_out) const override;

    void compute(AnalysisDataset* dataset) override;
    void writeReport(ResultReport::Section& root) override;

private:
    /// One selected SMR data source.
    struct SourceRow
    {
        unsigned int  ds_id = 0;
        std::string   label;
        std::string   cadence;          // "scan cycles" or "nominal period"
        bool          has_position = false;
        std::size_t   num_cycles = 0;
        double        period_median_s = 0.0;
        double        period_p5_s     = 0.0;
        double        period_p95_s    = 0.0;
        unsigned int  missing_scans   = 0;
        // Estimated coverage of this source.
        bool          coverage_valid        = false;
        std::size_t   coverage_bins_total   = 0;
        std::size_t   coverage_bins_covered = 0;
        std::uint64_t coverage_reports      = 0;
        double        coverage_r_min_median = 0.0;
        double        coverage_r_max_median = 0.0;
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        double        pd  = 0.0;
    };

    /// One size class (small / large targets).
    struct ClassRow
    {
        std::string   label;
        std::size_t   num_targets = 0;
        std::uint64_t scans_0     = 0;  // slots with no report (misses)
        std::uint64_t scans_1     = 0;
        std::uint64_t scans_2     = 0;
        std::uint64_t scans_3plus = 0;
        std::uint64_t extra       = 0;  // reports beyond the allowed count
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        double        pd  = 0.0;
    };

    /// Aggregate PD of a group (movement state).
    struct GroupRow
    {
        std::string   label;
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        double        pd  = 0.0;
    };

    /// One sector row in the "PD by Sector" overview table.
    struct SectorRow
    {
        std::string   label;
        std::size_t   num_targets = 0;
        std::uint64_t eui = 0;
        std::uint64_t mui = 0;
        double        pd  = 0.0;
    };

    struct ComputeResult
    {
        bool        valid = false;
        std::string error;

        unsigned int  targets_walked = 0;
        unsigned int  targets_no_ref = 0;
        unsigned int  targets_no_tst = 0;
        std::uint64_t slots_out_of_range    = 0;
        std::uint64_t slots_out_of_coverage = 0;
        bool          coverage_estimated    = false;

        std::uint64_t total_eui   = 0;
        std::uint64_t total_mui   = 0;
        std::uint64_t total_extra = 0;
        double        overall_pd  = 0.0;

        std::size_t cells_with_eui    = 0;
        double      median_per_cell_pd = 0.0;
        double      p5_per_cell_pd     = 0.0;
        bool        has_worst_cell     = false;
        double      worst_cell_pd      = 1.0;

        // Measured inter-report interval (all sources).
        std::size_t ui_num_intervals = 0;
        double ui_median_s = 0.0;
        double ui_p10_s    = 0.0;
        double ui_p90_s    = 0.0;
        double ui_mean_s   = 0.0;
        double ui_hist_max_s      = 0.0;
        double ui_hist_bin_width  = 0.0;
        std::vector<std::uint32_t> ui_hist_bins;
        std::uint32_t ui_hist_overflow = 0;

        // Estimated coverage per horizontal cell, for the coverage figure:
        // share of the slots of that cell that were inside a source's band.
        std::unordered_map<std::uint64_t, double>        coverage_by_key;
        std::unordered_map<std::uint64_t, std::uint64_t> coverage_samples;

        std::vector<SourceRow>   sources;
        std::array<ClassRow, 2>  classes;   // 0 small, 1 large
        std::array<GroupRow, 2>  movement;  // 0 moving, 1 standing
        std::vector<SectorRow>   sectors;
    };

    ComputeResult                       result_;
    std::unique_ptr<TargetReport3DGrid> grid_;
};
