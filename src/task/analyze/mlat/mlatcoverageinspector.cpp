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

#include "mlatcoverageinspector.h"
#include "mlatcoveragehelpers.h"
#include "analysedatasourcetask.h"
#include "analysisdataset.h"
#include "targetreport3dgrid.h"

#include "compass.h"
#include "logger.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "system.h"
#include "targetposition.h"
#include "dbcontent/target/targetreportchain.h"

#include "eval/requirement/detection/detection_pd_helpers.h"

#include "grid2dlayer.h"
#include "grid2dlayerrenderer.h"
#include "grid2drendersettings.h"
#include "colormap.h"
#include "colorlegend.h"

#include "viewpointgenerator.h"
#include "plotmetadata.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <utility>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;

MLATCoverageInspectorSettings::MLATCoverageInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("pd_method_int",   &pd_method_int_,   pd_method_int_);
    registerParameter("update_interval_s", &update_interval_s_, update_interval_s_);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, use_miss_tolerance_);
    registerParameter("miss_tolerance_s",   &miss_tolerance_s_,   miss_tolerance_s_);

    registerParameter("ref_max_time_diff_s", &ref_max_time_diff_s_, ref_max_time_diff_s_);

    registerParameter("pd_acceptable_above",   &pd_acceptable_above_,   pd_acceptable_above_);
    registerParameter("pd_unacceptable_below", &pd_unacceptable_below_, pd_unacceptable_below_);
}

MLATCoverageInspector::MLATCoverageInspector(AnalyseDataSourceTask& task,
                                             MLATCoverageInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

MLATCoverageInspector::~MLATCoverageInspector() = default;

std::set<std::string> MLATCoverageInspector::testDBContentNames() const
{
    return {"CAT020", "CAT010"};
}

bool MLATCoverageInspector::prerequisitesMet(std::string& reason_out) const
{
    reason_out.clear();

    if (task_.selectedDataSourceIDs().empty())
    {
        reason_out = "No data source selected.";
        return false;
    }
    return true;
}

namespace
{
using Settings = MLATCoverageInspectorSettings;
using EvaluationRequirement::PDHelpers::MissTestParams;
using EvaluationRequirement::PDHelpers::RefPeriod;

MissTestParams missParamsFromSettings(const Settings& s)
{
    MissTestParams p;
    p.update_interval_s  = s.update_interval_s_;
    p.use_miss_tolerance = s.use_miss_tolerance_;
    p.miss_tolerance_s   = s.miss_tolerance_s_;
    // min/max gap filters not exposed on the inspector: the dataset
    // already restricts the load to selected sources, so out-of-coverage
    // gating from the detection requirement is not needed here.
    return p;
}

double partialSeconds(const time_duration& d)
{
    return static_cast<double>(d.total_microseconds()) / 1.0e6;
}

ptime addSeconds(ptime t, double s)
{
    long long us = static_cast<long long>(std::llround(s * 1.0e6));
    return t + boost::posix_time::microseconds(us);
}

time_duration durationFromSeconds(double s)
{
    long long us = static_cast<long long>(std::llround(s * 1.0e6));
    return boost::posix_time::microseconds(us);
}

double percentile(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    if (p <= 0.0) return v.front();
    if (p >= 1.0) return v.back();
    double idx = p * (v.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(idx));
    size_t hi = static_cast<size_t>(std::ceil(idx));
    double w = idx - lo;
    return v[lo] * (1.0 - w) + v[hi] * w;
}

std::string formatNumber(double v, int prec = 4)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

// Coordinates of the grid cell to attribute a counter to. `valid == false`
// when `mappedRefPos()` could not interpolate the reference for the query
// timestamp (no usable bracket within `d_max`).
struct CellAttribution
{
    bool   valid  = false;
    double lat    = 0.0;
    double lon    = 0.0;
    double alt_ft = 0.0;
};

// Look up the reference position for `(utn, t)` and unpack it into the
// counter-attribution coordinates used by `TargetReport3DGrid`.
CellAttribution refCellAt(AnalysisDataset& dataset,
                          unsigned int utn,
                          ptime t,
                          time_duration d_max)
{
    CellAttribution out;
    auto pos = dataset.mappedRefPos(utn, t, d_max);
    if (!pos.has_value())
        return out;
    out.valid  = true;
    out.lat    = pos->latitude_;
    out.lon    = pos->longitude_;
    out.alt_ft = pos->has_altitude_ ? static_cast<double>(pos->altitude_) : 0.0;
    return out;
}

// Gather and sort all test timestamps for `utn` across the test dbcontents
// present in the dataset.
std::vector<ptime> gatherTestTimestamps(unsigned int utn,
                                        AnalysisDataset& dataset)
{
    std::vector<ptime> out;
    for (const auto& dbc : dataset.testDbContentsPresent())
    {
        if (!dataset.hasTestChain(utn, dbc))
            continue;
        auto& chain = dataset.testChain(utn, dbc);
        for (const auto& kv : chain.timestampIndexes())
            out.push_back(kv.first);
    }
    std::sort(out.begin(), out.end());
    return out;
}

// Time-difference walk per target.
//
// For each reference period:
//   - attribute one #EUI per UI slot, at the cell of the reference position
//     at slot timestamp `period.begin + k * UI`;
//   - walk the test timestamps that fall inside `[period.begin, period.end]`
//     and form gaps (period.begin -> first test, between consecutive tests,
//     last test -> period.end; or `period` itself when no tests are inside);
//   - for each gap that passes the miss test, attribute one #MUI per missed
//     UI slot at the cell of the reference position at the slot timestamp.
//
// Both per-cell counters live on `grid`; the caller aggregates them.
void walkTargetTimeDifference(unsigned int utn,
                              const std::vector<RefPeriod>& periods,
                              const std::vector<ptime>& tst_ts_sorted,
                              AnalysisDataset& dataset,
                              TargetReport3DGrid& grid,
                              const Settings& settings)
{
    const auto miss_params = missParamsFromSettings(settings);

    const time_duration d_max = boost::posix_time::seconds(60);

    for (const auto& period : periods)
    {
        const double period_s = partialSeconds(period.end - period.begin);
        if (period_s <= 0.0 || settings.update_interval_s_ <= 0.0f)
            continue;

        const unsigned int n_slots = static_cast<unsigned int>(
            std::floor(period_s / settings.update_interval_s_));

        for (unsigned int k = 0; k < n_slots; ++k)
        {
            ptime t_slot = addSeconds(period.begin,
                                      k * settings.update_interval_s_);
            auto ca = refCellAt(dataset, utn, t_slot, d_max);
            if (ca.valid)
                grid.addEUI(ca.lat, ca.lon, ca.alt_ft);
        }

        // Test timestamps inside [period.begin, period.end].
        auto first = std::lower_bound(tst_ts_sorted.begin(),
                                      tst_ts_sorted.end(), period.begin);
        auto last  = std::upper_bound(tst_ts_sorted.begin(),
                                      tst_ts_sorted.end(), period.end);

        std::vector<ptime> walk;
        walk.reserve(static_cast<std::size_t>(std::distance(first, last)) + 2);
        walk.push_back(period.begin);
        for (auto it = first; it != last; ++it)
            walk.push_back(*it);
        walk.push_back(period.end);

        for (std::size_t i = 0; i + 1 < walk.size(); ++i)
        {
            ptime gap_start = walk[i];
            ptime gap_end   = walk[i + 1];
            if (gap_end <= gap_start)
                continue;
            const float gap_s =
                static_cast<float>(partialSeconds(gap_end - gap_start));

            if (!EvaluationRequirement::PDHelpers::isMiss(gap_s, miss_params))
                continue;

            const unsigned int n_misses =
                EvaluationRequirement::PDHelpers::numMisses(gap_s, miss_params);

            for (unsigned int m = 0; m < n_misses; ++m)
            {
                ptime t_miss = addSeconds(gap_start,
                                          (m + 1) * settings.update_interval_s_);
                if (t_miss >= gap_end)
                    break;
                auto ca = refCellAt(dataset, utn, t_miss, d_max);
                if (ca.valid)
                    grid.addMUI(ca.lat, ca.lon, ca.alt_ft);
            }
        }
    }
}
}  // anonymous namespace

namespace mlatcoverage_internal
{
std::vector<CycleEvent> evaluateCyclesInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<ptime>& cycles_sorted,
    const std::vector<ptime>& tst_ts_sorted)
{
    std::vector<CycleEvent> out;

    auto cyc_begin = std::lower_bound(cycles_sorted.begin(),
                                      cycles_sorted.end(),
                                      period.begin);
    auto cyc_end   = std::upper_bound(cycles_sorted.begin(),
                                      cycles_sorted.end(),
                                      period.end);

    out.reserve(static_cast<std::size_t>(std::distance(cyc_begin, cyc_end)));

    for (auto it = cyc_begin; it != cyc_end; ++it)
    {
        ptime t_cycle = *it;
        auto next_it = std::next(it);
        ptime t_window_end =
            (next_it != cyc_end) ? *next_it : period.end;

        if (t_window_end <= t_cycle)
            continue;

        auto tst_lo = std::lower_bound(tst_ts_sorted.begin(),
                                       tst_ts_sorted.end(),
                                       t_cycle);
        const bool has_test =
            (tst_lo != tst_ts_sorted.end() && *tst_lo < t_window_end);

        CycleEvent ev;
        ev.t_cycle = t_cycle;
        ev.is_miss = !has_test;
        out.push_back(ev);
    }

    return out;
}
}  // namespace mlatcoverage_internal

namespace
{

// Status-message walk per target.
//
// For each reference period and each cycle timestamp inside it, attribute
// one #EUI to the cell of the reference position at the cycle timestamp;
// if no test report falls in the cycle window `[t_cycle, t_next_cycle)`
// for this target, also attribute one #MUI to the same cell.
//
// No miss test, no gap math -- the cycle stream defines the cadence
// directly (readme_analysis_mlat_ru.md, "CAT019 period-based variant").
void walkTargetStatusMessage(unsigned int utn,
                             const std::vector<RefPeriod>& periods,
                             const std::vector<ptime>& tst_ts_sorted,
                             const std::vector<ptime>& cycles_sorted,
                             AnalysisDataset& dataset,
                             TargetReport3DGrid& grid,
                             const Settings& /*settings*/)
{
    const time_duration d_max = boost::posix_time::seconds(60);

    for (const auto& period : periods)
    {
        auto events = mlatcoverage_internal::evaluateCyclesInPeriod(
            period, cycles_sorted, tst_ts_sorted);

        for (const auto& ev : events)
        {
            auto ca = refCellAt(dataset, utn, ev.t_cycle, d_max);
            if (!ca.valid)
                continue;
            grid.addEUI(ca.lat, ca.lon, ca.alt_ft);
            if (ev.is_miss)
                grid.addMUI(ca.lat, ca.lon, ca.alt_ft);
        }
    }
}
}  // anonymous namespace

void MLATCoverageInspector::compute(AnalysisDataset* dataset)
{
    result_ = ComputeResult{};
    grid_.reset();

    if (!dataset)
        return;

    auto& settings = static_cast<MLATCoverageInspectorSettings&>(settings_);

    if (!dataset->hasPositionExtent())
    {
        logwrn << "MLATCoverageInspector: dataset has no reference positions, skipping";
        return;
    }

    double ref_lat = dataset->centreLatitudeDeg();

    auto sizing = task_.clampedCellSizes(*dataset);
    if (sizing.horizontal_clamped || sizing.vertical_clamped)
    {
        loginf << "MLATCoverageInspector: cell sizes clamped to fit max "
               << task_.maxCellsPerAxis() << " cells/axis -- "
               << "horizontal " << task_.cellSizeMeters() << " m x"
               << sizing.horizontal_multiplier << " = " << sizing.cell_size_m << " m, "
               << "vertical " << task_.cellSizeFeet() << " ft x"
               << sizing.vertical_multiplier   << " = " << sizing.cell_size_ft << " ft";
    }

    grid_.reset(new TargetReport3DGrid(sizing.cell_size_m, sizing.cell_size_ft, ref_lat));
    auto& grid = *grid_;

    const bool use_status_method =
        settings.pdMethod() ==
        MLATCoverageInspectorSettings::PDMethod::StatusPeriodMessage;

    const auto& status_cycles = dataset->statusCycles();

    if (use_status_method && status_cycles.empty())
    {
        result_.error =
            "Status-Period-Message PD method selected, but no CAT019 "
            "start-of-cycle messages were loaded. The DB has no usable "
            "status content; switch to Time-Difference or import CAT019.";
        logwrn << result_.error;
        return;
    }

    const time_duration ref_max_gap =
        durationFromSeconds(settings.ref_max_time_diff_s_);
    const time_duration min_period_duration = boost::posix_time::seconds(1);

    unsigned int targets_walked = 0;
    unsigned int targets_no_ref = 0;
    unsigned int targets_no_tst = 0;

    const auto utns = dataset->utns();

    for (auto utn : utns)
    {
        if (!dataset->hasReferenceChain(utn))
        {
            ++targets_no_ref;
            continue;
        }
        auto& ref_chain = dataset->referenceChain(utn);
        if (!ref_chain.hasData())
        {
            ++targets_no_ref;
            continue;
        }

        std::set<ptime> ref_ts;
        for (const auto& kv : ref_chain.timestampIndexes())
            ref_ts.insert(kv.first);

        auto periods = EvaluationRequirement::PDHelpers::buildReferencePeriods(
            ref_ts, ref_max_gap, min_period_duration);
        if (periods.empty())
        {
            ++targets_no_ref;
            continue;
        }

        auto tst_ts_sorted = gatherTestTimestamps(utn, *dataset);
        if (tst_ts_sorted.empty())
            ++targets_no_tst;

        ++targets_walked;

        if (use_status_method)
            walkTargetStatusMessage(utn, periods, tst_ts_sorted,
                                    status_cycles, *dataset, grid, settings);
        else
            walkTargetTimeDifference(utn, periods, tst_ts_sorted,
                                     *dataset, grid, settings);
    }

    auto horizontal = grid.projectHorizontal();
    std::uint64_t total_eui = 0, total_mui = 0;
    double worst_pd = 1.0;
    bool   worst_set = false;
    std::vector<double> per_cell_pd;
    per_cell_pd.reserve(horizontal.size());
    for (const auto& kv : horizontal)
    {
        total_eui += kv.second.num_eui;
        total_mui += kv.second.num_mui;
        if (kv.second.num_eui == 0) continue;
        double pd = (static_cast<double>(kv.second.num_eui)
                     - static_cast<double>(kv.second.num_mui))
                    / static_cast<double>(kv.second.num_eui);
        per_cell_pd.push_back(pd);
        if (kv.second.num_eui >= 5 && (!worst_set || pd < worst_pd))
        {
            worst_pd  = pd;
            worst_set = true;
        }
    }
    double sector_pd = total_eui > 0
                           ? (static_cast<double>(total_eui) - static_cast<double>(total_mui))
                                 / static_cast<double>(total_eui)
                           : 0.0;

    result_.valid              = true;
    result_.targets_walked     = targets_walked;
    result_.targets_no_ref     = targets_no_ref;
    result_.targets_no_tst     = targets_no_tst;
    result_.total_eui          = total_eui;
    result_.total_mui          = total_mui;
    result_.sector_pd          = sector_pd;
    result_.cells_with_eui     = per_cell_pd.size();
    result_.median_per_cell_pd = percentile(per_cell_pd, 0.5);
    result_.p5_per_cell_pd     = percentile(per_cell_pd, 0.05);
    result_.has_worst_cell     = worst_set;
    result_.worst_cell_pd      = worst_pd;
}

namespace
{
// Per-cell PD scalar: only cells with #EUI > 0 contribute.
std::optional<double> pdScalar(const TargetReport3DGrid::Cell& c)
{
    if (c.num_eui == 0)
        return std::nullopt;
    return (static_cast<double>(c.num_eui) - static_cast<double>(c.num_mui))
           / static_cast<double>(c.num_eui);
}

std::uint64_t pdSampleCount(const TargetReport3DGrid::Cell& c)
{
    return c.num_eui;
}

void attachPDFigure(ResultReport::Section& section,
                    const std::string& fig_name,
                    TargetReport3DGrid::ProjectionResult& proj,
                    const MLATCoverageInspectorSettings& settings)
{
    if (!proj.valid || !proj.layer)
        return;

    // Three discrete colour bands so unacceptable / orange-between /
    // acceptable map directly to the three render colours. The explicit value
    // range on the colour map is required for colorLegend() to emit entries.
    const std::pair<double, double> range(settings.pd_unacceptable_below_,
                                          settings.pd_acceptable_above_);

    Grid2DRenderSettings rs;
    rs.color_map.create(ColorMap::ColorScale::Red2Green, 3,
                        ColorMap::Type::LinearSamples, range);
    rs.min_value = settings.pd_unacceptable_below_;
    rs.max_value = settings.pd_acceptable_above_;

    PlotMetadata meta("Sensor Coverage", fig_name,
                      proj.x_axis_label, proj.y_axis_label);

    auto vp = std::make_unique<ViewPointGenVP>(fig_name, 0, "Grid");
    vp->noDataLoaded(true);

    auto* anno = vp->annotations().getOrCreateAnnotation("PD");

    // Note: ViewPointGenFeatureGeoImage / ViewPointGenFeatureGrid take
    // ownership of the layer payload via copy; the unique_ptr in `proj`
    // keeps the Grid2DLayer alive until end of scope.
    if (proj.x_axis_label == "Longitude (deg)" && proj.y_axis_label == "Latitude (deg)")
    {
        auto rendered = Grid2DLayerRenderer::render(*proj.layer, rs);

        // Reverse so the colour-map's "good" end (green for PD) ends up at the
        // top of the legend tree, matching operator expectation.
        auto raw = rs.color_map.colorLegend(
            /*add_sel_color=*/false,
            /*add_null_color=*/false,
            [](double v) { return Utils::String::doubleToStringPrecision(v, 2); });
        ColorLegend legend;
        const auto& entries = raw.entries();
        for (auto it = entries.rbegin(); it != entries.rend(); ++it)
            legend.addEntry(it->first, it->second);

        anno->addFeature(new ViewPointGenFeatureGeoImage(rendered.first, rendered.second, legend));
    }
    else
    {
        anno->addFeature(new ViewPointGenFeatureGrid(*proj.layer, rs, meta));
    }

    nlohmann::json vp_json;
    vp->toJSON(vp_json);
    section.addFigure(fig_name, ResultReport::SectionContentViewable(vp_json));
}
}

void MLATCoverageInspector::writeReport(ResultReport::Section& root)
{
    auto& settings = static_cast<MLATCoverageInspectorSettings&>(settings_);
    auto& section  = root.addSubSection(name());

    {
        auto& intro = section.addText("About");
        intro.addText(
            "Probability of Detection (PD) of the selected MLAT sensors per "
            "cell of a three-dimensional grid in latitude, longitude and "
            "barometric flight level, using the Reference Trajectory as "
            "ground truth. PD is the fraction of expected updates the sensor "
            "actually delivered: each target's reference track contributes "
            "expected updates at the configured cadence, and each gap in the "
            "MLAT report stream long enough to swallow a cadence slot "
            "contributes a missed update at the location where the report "
            "was expected.\n"
            "Cadence source is operator-selected. Time Difference uses a "
            "fixed nominal Update Interval together with the EUROCAE ED-117 "
            "miss tolerance. Status Period Message follows the MLAT system's "
            "own start-of-cycle messages from CAT019 directly, with no "
            "tolerance.\n"
            "Three projections of the per-cell PD are rendered: a top-down "
            "horizontal map and two vertical profiles (altitude over "
            "longitude, altitude over latitude). The summary tabulates the "
            "sector-aggregate PD and the per-cell distribution (median, 5th "
            "percentile, worst cell). Cells with no expected updates are "
            "blank.\n"
            "Use the report to locate coverage holes against the published "
            "service volume, to assess PD compliance with ED-117 / ED-116 "
            "thresholds, and to compare the actual horizontal and vertical "
            "coverage geometry against the system design.");
    }

    auto& recap = section.addTable("Settings", 2, {"Setting", "Value"}, false);
    recap.addRow({"PD calculation method",
                  settings.pdMethod() ==
                          MLATCoverageInspectorSettings::PDMethod::TimeDifference
                      ? std::string("Time Difference")
                      : std::string("Status Period Message Based\n"
                                    "(CAT019 cycles)")});
    {
        std::ostringstream os;
        os << settings.update_interval_s_ << " s";
        recap.addRow({"Nominal update interval (UI)", os.str()});
    }
    recap.addRow({"Use miss tolerance", settings.use_miss_tolerance_ ? "yes" : "no"});
    {
        std::ostringstream os;
        os << settings.miss_tolerance_s_ << " s";
        recap.addRow({"Miss tolerance", os.str()});
    }
    {
        std::ostringstream os;
        os << settings.ref_max_time_diff_s_ << " s";
        recap.addRow({"Reference period split threshold", os.str()});
    }
    {
        std::ostringstream os;
        os << task_.cellSizeMeters() << " m horizontal\n"
           << task_.cellSizeFeet()   << " ft vertical\n"
           << "max " << task_.maxCellsPerAxis() << " cells/axis";
        recap.addRow({"Grid resolution", os.str()});
    }
    {
        std::ostringstream os;
        os << "green >= " << settings.pd_acceptable_above_ << "\n"
           << "red <= "   << settings.pd_unacceptable_below_ << "\n"
           << "orange in between";
        recap.addRow({"PD color thresholds", os.str()});
    }

    if (!result_.valid)
    {
        auto& note = section.addText("Note");
        if (!result_.error.empty())
            note.addText(result_.error);
        else
            note.addText("No data loaded; coverage analysis skipped.");
        return;
    }

    auto& summary = section.addTable("Summary", 2, {"Property", "Value"}, false);
    summary.addRow({"Targets walked",        std::to_string(result_.targets_walked)});
    summary.addRow({"Targets w/o RefTraj",   std::to_string(result_.targets_no_ref)});
    summary.addRow({"Targets w/o test data", std::to_string(result_.targets_no_tst)});
    summary.addRow({"Total expected slots (#EUI)", std::to_string(result_.total_eui)});
    summary.addRow({"Total missed slots (#MUI)",   std::to_string(result_.total_mui)});
    summary.addRow({"Sector-aggregate PD",         formatNumber(result_.sector_pd)});
    summary.addRow({"Cells with EUI",              std::to_string(result_.cells_with_eui)});
    summary.addRow({"Median per-cell PD",          formatNumber(result_.median_per_cell_pd)});
    summary.addRow({"P5 per-cell PD",              formatNumber(result_.p5_per_cell_pd)});
    if (result_.has_worst_cell)
        summary.addRow({"Worst cell PD (>=5 EUI)", formatNumber(result_.worst_cell_pd)});

    if (grid_)
    {
        auto horiz  = grid_->projectionLayer(TargetReport3DGrid::Projection::Horizontal,
                                             pdScalar, pdSampleCount, "pd");

        auto altlon = grid_->projectionLayer(TargetReport3DGrid::Projection::AltLon,
                                             pdScalar, pdSampleCount, "pd");

        auto altlat = grid_->projectionLayer(TargetReport3DGrid::Projection::AltLat,
                                             pdScalar, pdSampleCount, "pd");

        attachPDFigure(section, "PD - Horizontal",         horiz,  settings);
        attachPDFigure(section, "PD - Altitude/Longitude", altlon, settings);
        attachPDFigure(section, "PD - Altitude/Latitude",  altlat, settings);
    }

    loginf << "MLATCoverageInspector: walked " << result_.targets_walked << " target(s), "
           << result_.total_eui << " EUI, " << result_.total_mui
           << " MUI, sector PD " << result_.sector_pd;
}
