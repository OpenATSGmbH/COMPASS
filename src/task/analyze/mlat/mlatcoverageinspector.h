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

class MLATCoverageInspectorSettings : public InspectorSettingsBase
{
public:
    MLATCoverageInspectorSettings(nlohmann::json& config_json, Configurable* parent);
    ~MLATCoverageInspectorSettings() override = default;

    std::string inspectorClassName() const override { return "MLATCoverageInspector"; }

    // Cadence
    enum class CadenceSource : int { TimeDifference = 0, CAT019Cycle = 1 };
    int  cadence_source_int_ = static_cast<int>(CadenceSource::TimeDifference);

    CadenceSource cadenceSource() const
    { return static_cast<CadenceSource>(cadence_source_int_); }

    float update_interval_s_ = 1.0f;   // UI

    // Miss test (mirrors the detection requirement)
    bool  use_miss_tolerance_ = true;
    float miss_tolerance_s_   = 0.1f;
    bool  use_min_gap_length_ = false;
    float min_gap_length_s_   = 5.0f;
    bool  use_max_gap_length_ = false;
    float max_gap_length_s_   = 600.0f;

    // 3D grid resolution (shared with other 3D-grid inspectors).
    float cell_size_m_  = 1000.0f;
    float cell_size_ft_ = 1000.0f;

    // PD color thresholds (ED-117 / ESASSP guidance).
    float pd_red_below_     = 0.97f;
    float pd_yellow_below_  = 0.99f;
    // green at and above pd_yellow_below_.
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
    ~MLATCoverageInspector() override = default;

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

    ComputeResult result_;
};
