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
#include "analyzedatasourcetask.h"
#include "analysisdataset.h"
#include "targetreport3dgrid.h"
#include "movementui.h"

#include "compass.h"
#include "logger.h"
#include "number.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "system.h"
#include "targetposition.h"
#include "sectorlayer.h"
#include "dbcontent/target/targetreportchain.h"

#include "eval/requirement/detection/detection_pd_helpers.h"
#include "coveragepdwalk.h"

#include "grid2dlayer.h"
#include "grid2dlayerrenderer.h"
#include "grid2drendersettings.h"
#include "colormap.h"
#include "colorlegend.h"

#include "viewpointgenerator.h"
#include "histogram_raw.h"
#include "plotmetadata.h"

#include "json.hpp"

#include <QColor>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <utility>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using Utils::Number::mean;
using Utils::Number::percentile;
using analysis::MovementUI;
using analysis::SpeedSamples;
using analysis::gatherTestSpeeds;
using analysis::gatherRefSpeeds;

MLATCoverageInspectorSettings::MLATCoverageInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("pd_method_int",   &pd_method_int_,   pd_method_int_);
    registerParameter("update_interval_s", &update_interval_s_, update_interval_s_);
    registerParameter("update_interval_standing_s", &update_interval_standing_s_,
                      update_interval_standing_s_);
    registerParameter("standing_speed_max_mps", &standing_speed_max_mps_,
                      standing_speed_max_mps_);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, use_miss_tolerance_);
    registerParameter("miss_tolerance_s",   &miss_tolerance_s_,   miss_tolerance_s_);

    registerParameter("ref_max_time_diff_s", &ref_max_time_diff_s_, ref_max_time_diff_s_);

    registerParameter("pd_acceptable_above",   &pd_acceptable_above_,   pd_acceptable_above_);
    registerParameter("pd_unacceptable_below", &pd_unacceptable_below_, pd_unacceptable_below_);

    registerParameter("ui_hist_num_bins", &ui_hist_num_bins_, ui_hist_num_bins_);
    registerParameter("ui_hist_max_s",    &ui_hist_max_s_,    ui_hist_max_s_);
}

MLATCoverageInspector::MLATCoverageInspector(AnalyzeDataSourceTask& task,
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
using EvaluationRequirement::PDHelpers::RefPeriod;
using analysis::PDWalkParams;
using analysis::walkReferencePeriodsTimeDifference;

double partialSeconds(const time_duration& d)
{
    return static_cast<double>(d.total_microseconds()) / 1.0e6;
}

time_duration durationFromSeconds(double s)
{
    long long us = static_cast<long long>(std::llround(s * 1.0e6));
    return boost::posix_time::microseconds(us);
}

// Inter-report intervals above this are treated as coverage gaps (target not
// continuously tracked) and excluded from the measured-cadence statistics.
constexpr double kMaxUpdateIntervalS = 300.0;

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

// Per-sector slot accumulation: a slot is attributed to every selected sector
// layer it lies in (evaluation's exact SectorLayer::isInside). `eui`/`mui`
// accumulate across targets; `touched` is reset per target so the caller can
// count each target once per sector.
struct SectorWalkAccum
{
    const std::vector<std::shared_ptr<SectorLayer>>* layers = nullptr;
    std::vector<std::uint64_t> eui;
    std::vector<std::uint64_t> mui;
    std::vector<char>          touched;

    void accum(const CellAttribution& ca, bool is_miss)
    {
        if (!layers)
            return;
        dbContent::TargetPosition pos;
        pos.latitude_     = ca.lat;
        pos.longitude_    = ca.lon;
        pos.has_altitude_ = true;
        pos.altitude_     = ca.alt_ft;
        for (std::size_t i = 0; i < layers->size(); ++i)
        {
            const auto& L = (*layers)[i];
            if (L && L->isInside(pos, false, false))
            {
                if (is_miss) ++mui[i]; else ++eui[i];
                touched[i] = 1;
            }
        }
    }
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
//   - for each gap, attribute one #MUI per missed UI slot at the cell of the
//     reference position at the slot timestamp.
//
// The walk itself is shared with the ADS-B coverage inspector, see
// `analysis::walkReferencePeriodTimeDifference()`.
// Both per-cell counters live on `grid`; the caller aggregates them.
void walkTargetTimeDifference(unsigned int utn,
                              const std::vector<RefPeriod>& periods,
                              const std::vector<ptime>& tst_ts_sorted,
                              AnalysisDataset& dataset,
                              TargetReport3DGrid& grid,
                              const Settings& settings,
                              const analysis::MovementUI& mv,
                              SectorWalkAccum* sec)
{
    const time_duration d_max = boost::posix_time::seconds(60);

    PDWalkParams walk_params;
    walk_params.mv                 = &mv;
    walk_params.use_miss_tolerance = settings.use_miss_tolerance_;
    walk_params.miss_tolerance_s   = settings.miss_tolerance_s_;

    auto slotFunc = [ & ] (const ptime& t, bool is_miss)
    {
        auto ca = refCellAt(dataset, utn, t, d_max);
        if (!ca.valid)
            return;

        is_miss ? grid.addMUI(ca.lat, ca.lon, ca.alt_ft) : grid.addEUI(ca.lat, ca.lon, ca.alt_ft);

        if (sec) sec->accum(ca, is_miss);
    };

    walkReferencePeriodsTimeDifference(
        periods, tst_ts_sorted, walk_params,
        [ & ] (const ptime& t) { slotFunc(t, false); },
        [ & ] (const ptime& t) { slotFunc(t, true ); });
}
}  // anonymous namespace

namespace mlatcoverage_internal
{
std::vector<CycleEvent> evaluateCyclesInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<ptime>& cycles_sorted,
    const std::vector<ptime>& tst_ts_sorted,
    const analysis::MovementUI* mv)
{
    std::vector<CycleEvent> out;

    auto cyc_begin = std::lower_bound(cycles_sorted.begin(),
                                      cycles_sorted.end(),
                                      period.begin);
    auto cyc_end   = std::upper_bound(cycles_sorted.begin(),
                                      cycles_sorted.end(),
                                      period.end);

    out.reserve(static_cast<std::size_t>(std::distance(cyc_begin, cyc_end)));

    const double standing_ui = mv ? mv->ui_standing : 0.0;

    for (auto it = cyc_begin; it != cyc_end; )
    {
        ptime t_cycle = *it;

        // Standing targets are expected only once per standing UI, not every
        // cycle; the window then spans that interval and the cycles inside it
        // are skipped (a standing target legitimately reports less often than
        // the cycle rate). Moving targets keep the per-cycle window.
        const bool standing = mv && standing_ui > 0.0 && mv->standingAt(t_cycle);

        ptime t_window_end;
        if (standing)
        {
            t_window_end = t_cycle
                + boost::posix_time::microseconds(
                      static_cast<long long>(std::llround(standing_ui * 1.0e6)));
            if (t_window_end > period.end)
                t_window_end = period.end;
        }
        else
        {
            auto next_it = std::next(it);
            t_window_end = (next_it != cyc_end) ? *next_it : period.end;
        }

        if (t_window_end <= t_cycle)
        {
            ++it;
            continue;
        }

        auto tst_lo = std::lower_bound(tst_ts_sorted.begin(),
                                       tst_ts_sorted.end(),
                                       t_cycle);
        const bool has_test =
            (tst_lo != tst_ts_sorted.end() && *tst_lo < t_window_end);

        CycleEvent ev;
        ev.t_cycle = t_cycle;
        ev.is_miss = !has_test;
        out.push_back(ev);

        if (standing)
            it = std::lower_bound(it, cyc_end, t_window_end);  // skip covered cycles
        else
            ++it;
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
                             const Settings& /*settings*/,
                             const analysis::MovementUI& mv,
                             SectorWalkAccum* sec)
{
    const time_duration d_max = boost::posix_time::seconds(60);

    for (const auto& period : periods)
    {
        auto events = mlatcoverage_internal::evaluateCyclesInPeriod(
            period, cycles_sorted, tst_ts_sorted, &mv);

        for (const auto& ev : events)
        {
            auto ca = refCellAt(dataset, utn, ev.t_cycle, d_max);
            if (!ca.valid)
                continue;
            grid.addEUI(ca.lat, ca.lon, ca.alt_ft);
            if (sec) sec->accum(ca, false);
            if (ev.is_miss)
            {
                grid.addMUI(ca.lat, ca.lon, ca.alt_ft);
                if (sec) sec->accum(ca, true);
            }
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

    double ref_lat = dataset->centerLatitudeDeg();

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

    // Per-sector breakdown (over the selected sector layers, independent of the
    // limit-by-sectors toggle). A slot is attributed to every sector it is in.
    auto sector_layers = task_.scopeSectorLayers();
    SectorWalkAccum sec;
    sec.layers = &sector_layers;
    sec.eui.assign(sector_layers.size(), 0);
    sec.mui.assign(sector_layers.size(), 0);
    sec.touched.assign(sector_layers.size(), 0);
    std::vector<std::size_t> sector_target_count(sector_layers.size(), 0);

    unsigned int targets_walked = 0;
    unsigned int targets_no_ref = 0;
    unsigned int targets_no_tst = 0;

    // Measured update cadence: consecutive inter-report intervals per target.
    std::vector<double> intervals;

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

        for (std::size_t i = 1; i < tst_ts_sorted.size(); ++i)
        {
            double dt = partialSeconds(tst_ts_sorted[i] - tst_ts_sorted[i - 1]);
            if (dt > 0.0 && dt <= kMaxUpdateIntervalS)
                intervals.push_back(dt);
        }

        ++targets_walked;

        std::fill(sec.touched.begin(), sec.touched.end(), 0);

        // Movement classification for the time-difference walk: MLAT-reported
        // ground speed where available, RefTraj speed as fallback.
        SpeedSamples test_spd = gatherTestSpeeds(utn, *dataset);
        SpeedSamples ref_spd  = gatherRefSpeeds(ref_chain);
        MovementUI mv;
        mv.test         = &test_spd;
        mv.ref          = &ref_spd;
        mv.standing_max = settings.standing_speed_max_mps_;
        mv.ui_moving    = settings.update_interval_s_;
        mv.ui_standing  = settings.update_interval_standing_s_;
        mv.window_s     = std::max(6.0, 2.0 * settings.update_interval_standing_s_);

        if (use_status_method)
            walkTargetStatusMessage(utn, periods, tst_ts_sorted,
                                    status_cycles, *dataset, grid, settings, mv, &sec);
        else
            walkTargetTimeDifference(utn, periods, tst_ts_sorted,
                                     *dataset, grid, settings, mv, &sec);

        for (std::size_t si = 0; si < sector_layers.size(); ++si)
            if (sec.touched[si])
                ++sector_target_count[si];
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
    double overall_pd = total_eui > 0
                           ? (static_cast<double>(total_eui) - static_cast<double>(total_mui))
                                 / static_cast<double>(total_eui)
                           : 0.0;

    result_.valid              = true;
    result_.targets_walked     = targets_walked;
    result_.targets_no_ref     = targets_no_ref;
    result_.targets_no_tst     = targets_no_tst;
    result_.total_eui          = total_eui;
    result_.total_mui          = total_mui;
    result_.overall_pd         = overall_pd;
    result_.cells_with_eui     = per_cell_pd.size();
    result_.median_per_cell_pd = percentile(per_cell_pd, 0.5);
    result_.p5_per_cell_pd     = percentile(per_cell_pd, 0.05);
    result_.has_worst_cell     = worst_set;
    result_.worst_cell_pd      = worst_pd;

    result_.ui_num_intervals   = intervals.size();
    if (!intervals.empty())
    {
        result_.ui_median_s = percentile(intervals, 0.5);
        result_.ui_p10_s    = percentile(intervals, 0.1);
        result_.ui_p90_s    = percentile(intervals, 0.9);
        result_.ui_mean_s   = mean(intervals);

        // Histogram: bins over [0, max], plus one overflow bin for >= max so
        // long detection breaks do not stretch the axis. max = configured value
        // or, when <= 0, a rounded-up value derived from the data (P99).
        double hist_max = settings.ui_hist_max_s_ > 0.0f
                              ? static_cast<double>(settings.ui_hist_max_s_)
                              : std::ceil(percentile(intervals, 0.99));
        if (hist_max <= 0.0)
            hist_max = 15.0;
        const unsigned int nbins =
            static_cast<unsigned int>(std::max(1, settings.ui_hist_num_bins_));
        const double w = hist_max / static_cast<double>(nbins);

        std::vector<std::uint32_t> bins(nbins, 0);
        std::uint32_t overflow = 0;
        for (double dt : intervals)
        {
            if (dt >= hist_max)
            {
                ++overflow;
                continue;
            }
            unsigned int b = static_cast<unsigned int>(std::floor(dt / w));
            if (b >= nbins)
                b = nbins - 1;
            ++bins[b];
        }
        result_.ui_hist_max_s     = hist_max;
        result_.ui_hist_bin_width = w;
        result_.ui_hist_bins      = std::move(bins);
        result_.ui_hist_overflow  = overflow;
    }

    // Per-sector rows (over the selected sector layers).
    for (std::size_t si = 0; si < sector_layers.size(); ++si)
    {
        if (!sector_layers[si])
            continue;
        SectorRow r;
        r.label       = sector_layers[si]->name();
        r.num_targets = sector_target_count[si];
        r.eui         = sec.eui[si];
        r.mui         = sec.mui[si];
        r.pd          = sec.eui[si] > 0
            ? (static_cast<double>(sec.eui[si]) - static_cast<double>(sec.mui[si]))
                  / static_cast<double>(sec.eui[si])
            : 0.0;
        result_.sectors.push_back(std::move(r));
    }
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

    // Three discrete color bands so unacceptable / orange-between /
    // acceptable map directly to the three render colors. The explicit value
    // range on the color map is required for colorLegend() to emit entries.
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

        // Reverse so the color-map's "good" end (green for PD) ends up at the
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

        // Frame the Geographic View on the populated region of the rendered
        // raster instead of falling back to the full loaded-data extent.
        QRectF vp_roi = Grid2DLayerRenderer::geoROIOfOpaquePixels(
            rendered.first, rendered.second);
        if (!vp_roi.isEmpty())
            vp->setROI(vp_roi);
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
            "overall PD and the per-cell distribution (median, 5th "
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
        recap.addRow({"Update interval (moving)", os.str()});
    }
    {
        std::ostringstream os;
        os << settings.update_interval_standing_s_ << " s";
        recap.addRow({"Update interval (standing)", os.str()});
    }
    {
        std::ostringstream os;
        os << settings.standing_speed_max_mps_ << " m/s";
        recap.addRow({"Standing speed threshold", os.str()});
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
    summary.addRow({"Overall PD",                  formatNumber(result_.overall_pd)});
    summary.addRow({"Cells with EUI",              std::to_string(result_.cells_with_eui)});
    summary.addRow({"Median per-cell PD",          formatNumber(result_.median_per_cell_pd)});
    summary.addRow({"P5 per-cell PD",              formatNumber(result_.p5_per_cell_pd)});
    if (result_.has_worst_cell)
        summary.addRow({"Worst cell PD (>=5 EUI)", formatNumber(result_.worst_cell_pd)});

    // Measured update cadence: the actual per-target inter-report interval,
    // independent of the configured nominal UI. Use it to pick an Update
    // Interval that matches the sensor (set UI at or above the median, near the
    // P90, to avoid counting the sensor's own cadence as missed updates).
    {
        auto& ui_tbl = section.addTable("Calculated Update Interval", 2,
                                        {"Property", "Value"}, false);
        if (result_.ui_num_intervals > 0)
        {
            ui_tbl.addRow({"Calculated UI Median (s)", formatNumber(result_.ui_median_s, 3)});
            ui_tbl.addRow({"Calculated UI P10 (s)",    formatNumber(result_.ui_p10_s, 3)});
            ui_tbl.addRow({"Calculated UI P90 (s)",    formatNumber(result_.ui_p90_s, 3)});
            ui_tbl.addRow({"Calculated UI Mean (s)",   formatNumber(result_.ui_mean_s, 3)});
            ui_tbl.addRow({"Update interval samples",  std::to_string(result_.ui_num_intervals)});
        }
        else
        {
            ui_tbl.addRow({"Calculated UI Median (s)", "-"});
        }
    }

    // Update-interval histogram figure.
    if (!result_.ui_hist_bins.empty())
    {
        RawHistogram h;
        const double w = result_.ui_hist_bin_width;
        for (unsigned int i = 0; i < result_.ui_hist_bins.size(); ++i)
        {
            std::string lmin = formatNumber(i * w, 2);
            std::string lmax = formatNumber((i + 1) * w, 2);
            h.addBin(RawHistogramBin(result_.ui_hist_bins[i], lmin + "-" + lmax,
                                     RawHistogramBin::Tag::Standard, lmin, lmax));
        }
        std::string ov_lbl = ">=" + formatNumber(result_.ui_hist_max_s, 2);
        h.addBin(RawHistogramBin(result_.ui_hist_overflow, ov_lbl,
                                 RawHistogramBin::Tag::OutOfRange, ov_lbl, ""));

        const std::string fig = "Update Interval Histogram";
        auto vp = std::make_unique<ViewPointGenVP>(fig, 0, "Histogram");
        vp->noDataLoaded(true);
        auto* anno = vp->annotations().getOrCreateAnnotation("Update Interval");
        anno->addFeature(new ViewPointGenFeatureHistogram(
            h, "Update Interval", QColor(0, 128, 192), boost::optional<bool>(),
            PlotMetadata("Sensor Coverage", fig, "Update Interval (s)", "Count")));

        nlohmann::json vp_json;
        vp->toJSON(vp_json);
        section.addFigure(fig, ResultReport::SectionContentViewable(vp_json));
    }

    // Per-sector overview (selected sector layers). "All" is the aggregate
    // reference; sectors can overlap, so the rows need not sum to "All".
    if (!result_.sectors.empty())
    {
        auto& st = section.addTable("PD by Sector", 5,
                                    {"Sector", "Targets", "#EUI", "#MUI", "PD"}, false);
        st.addRow({"All (in scope)",
                   std::to_string(result_.targets_walked),
                   std::to_string(result_.total_eui),
                   std::to_string(result_.total_mui),
                   formatNumber(result_.overall_pd)});
        for (const auto& r : result_.sectors)
            st.addRow({r.label,
                       std::to_string(r.num_targets),
                       std::to_string(r.eui),
                       std::to_string(r.mui),
                       formatNumber(r.pd)});
    }

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
           << " MUI, overall PD " << result_.overall_pd;
}
