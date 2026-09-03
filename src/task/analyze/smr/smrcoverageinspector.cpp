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

#include "smrcoverageinspector.h"
#include "analyzedatasourcetask.h"
#include "analysisdataset.h"
#include "targetreport3dgrid.h"
#include "movementui.h"
#include "scancyclewalk.h"

#include "compass.h"
#include "db_context_manager.h"
#include "data_source.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/target/targetbase.h"
#include "dbcontent/dbcontentaccessor.h"
#include "dbcontent/target/targetreportchain.h"
#include "idbvariableresolver.h"
#include "logger.h"
#include "number.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "targetposition.h"
#include "sectorlayer.h"
#include "transformation.h"

#include "eval/requirement/detection/detection_pd_helpers.h"

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
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using Utils::Number::mean;
using Utils::Number::percentile;
using analysis::CycleEvent;
using analysis::MovementUI;
using analysis::SpeedSamples;
using analysis::gatherRefSpeeds;
using dbContent::TargetReport::Chain;

SMRCoverageInspectorSettings::SMRCoverageInspectorSettings(nlohmann::json& config_json,
                                                           Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("cadence_source_int", &cadence_source_int_, cadence_source_int_);
    registerParameter("scan_period_s",      &scan_period_s_,      scan_period_s_);

    registerParameter("small_target_max_length_m",
                      &small_target_max_length_m_, small_target_max_length_m_);
    registerParameter("large_target_max_reports_per_scan",
                      &large_target_max_reports_per_scan_, large_target_max_reports_per_scan_);

    registerParameter("max_range_m",         &max_range_m_,         max_range_m_);

    registerParameter("coverage_azimuth_bin_deg", &coverage_azimuth_bin_deg_,
                      coverage_azimuth_bin_deg_);
    registerParameter("coverage_min_reports",     &coverage_min_reports_,
                      coverage_min_reports_);
    registerParameter("coverage_smooth_bins",     &coverage_smooth_bins_,
                      coverage_smooth_bins_);
    registerParameter("ref_max_time_diff_s", &ref_max_time_diff_s_, ref_max_time_diff_s_);
    registerParameter("standing_speed_max_mps",
                      &standing_speed_max_mps_, standing_speed_max_mps_);

    registerParameter("pd_acceptable_above",   &pd_acceptable_above_,   pd_acceptable_above_);
    registerParameter("pd_unacceptable_below", &pd_unacceptable_below_, pd_unacceptable_below_);

    registerParameter("extra_ratio_acceptable_below",
                      &extra_ratio_acceptable_below_, extra_ratio_acceptable_below_);
    registerParameter("extra_ratio_unacceptable_above",
                      &extra_ratio_unacceptable_above_, extra_ratio_unacceptable_above_);

    registerParameter("ui_hist_num_bins", &ui_hist_num_bins_, ui_hist_num_bins_);
    registerParameter("ui_hist_max_s",    &ui_hist_max_s_,    ui_hist_max_s_);
}

SMRCoverageInspector::SMRCoverageInspector(AnalyzeDataSourceTask& task,
                                           SMRCoverageInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

SMRCoverageInspector::~SMRCoverageInspector() = default;

std::set<std::string> SMRCoverageInspector::testDBContentNames() const
{
    return {"CAT010"};
}

bool SMRCoverageInspector::prerequisitesMet(std::string& reason_out) const
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
using EvaluationRequirement::PDHelpers::RefPeriod;

const std::string kTestDBContent = "CAT010";

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

double pdOf(std::uint64_t eui, std::uint64_t mui)
{
    if (eui == 0)
        return 0.0;
    return (static_cast<double>(eui) - static_cast<double>(mui)) / static_cast<double>(eui);
}

std::uint32_t clampCount(std::uint64_t v)
{
    return v > std::numeric_limits<std::uint32_t>::max()
               ? std::numeric_limits<std::uint32_t>::max()
               : static_cast<std::uint32_t>(v);
}

// Coordinates of the grid cell to attribute a counter to. `valid == false`
// when `mappedRefPos()` could not interpolate the reference for the query
// timestamp (no usable bracket within `d_max`).
struct CellAttribution
{
    bool   valid = false;
    double lat   = 0.0;
    double lon   = 0.0;
};

CellAttribution refCellAt(AnalysisDataset& dataset,
                          unsigned int utn,
                          ptime t,
                          time_duration d_max)
{
    CellAttribution out;
    auto pos = dataset.mappedRefPos(utn, t, d_max);
    if (!pos.has_value())
        return out;
    out.valid = true;
    out.lat   = pos->latitude_;
    out.lon   = pos->longitude_;
    return out;
}

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
        pos.has_altitude_ = false;
        for (std::size_t i = 0; i < layers->size(); ++i)
        {
            const auto& L = (*layers)[i];
            // SMR reports are on ground
            if (L && L->isInside(pos, true, true))
            {
                if (is_miss) ++mui[i]; else ++eui[i];
                touched[i] = 1;
            }
        }
    }
};

// Size class of a target: small targets must give exactly one report per
// scan. From the reconstructed target category where it decides, else from
// the median reported target length, else large (lenient).
bool isSmallTarget(DBContentManager& dbcont_man,
                   unsigned int utn,
                   std::vector<double>& lengths_m,
                   float small_max_length_m)
{
    using Cat = TargetBase::Category;

    if (dbcont_man.existsTarget(utn))
    {
        switch (dbcont_man.emitterCategory(utn))
        {
            case Cat::Vehicle:
            case Cat::LightAircraft:
            case Cat::SmallAircraft:
            case Cat::Rotocraft:
            case Cat::OtherAirborne:
            case Cat::Obstacle:
            case Cat::FFT:
                return true;
            case Cat::MediumAircraft:
            case Cat::HighVortexLargeAircraft:
            case Cat::HeavyAircraft:
            case Cat::HighSpeedManoeuvrable:
                return false;
            default:
                break;
        }
    }

    if (!lengths_m.empty())
        return percentile(lengths_m, 0.5) < static_cast<double>(small_max_length_m);

    return false;
}

// Per-cell PD scalar: only cells with #EUI > 0 contribute.
std::optional<double> pdScalar(const TargetReport3DGrid::Cell& c)
{
    if (c.num_eui == 0)
        return std::nullopt;
    return pdOf(c.num_eui, c.num_mui);
}

// Per-cell extra reports per expected slot (split map).
std::optional<double> extraRatioScalar(const TargetReport3DGrid::Cell& c)
{
    if (c.num_eui == 0)
        return std::nullopt;
    return static_cast<double>(c.num_extra) / static_cast<double>(c.num_eui);
}

std::uint64_t euiSampleCount(const TargetReport3DGrid::Cell& c)
{
    return c.num_eui;
}

void attachGridFigure(ResultReport::Section& section,
                      const std::string& fig_name,
                      const std::string& annotation_name,
                      TargetReport3DGrid::ProjectionResult& proj,
                      const Grid2DRenderSettings& rs)
{
    if (!proj.valid || !proj.layer)
        return;

    PlotMetadata meta("Sensor Coverage", fig_name,
                      proj.x_axis_label, proj.y_axis_label);

    auto vp = std::make_unique<ViewPointGenVP>(fig_name, 0, "Grid");
    vp->noDataLoaded(true);

    auto* anno = vp->annotations().getOrCreateAnnotation(annotation_name);

    auto rendered = Grid2DLayerRenderer::render(*proj.layer, rs);

    // Reverse so the color-map's "good" end ends up at the top of the legend
    // tree, matching operator expectation.
    auto raw = rs.color_map.colorLegend(
        /*add_sel_color=*/false,
        /*add_null_color=*/false,
        [](double v) { return Utils::String::doubleToStringPrecision(v, 2); });
    ColorLegend legend;
    const auto& entries = raw.entries();
    for (auto it = entries.rbegin(); it != entries.rend(); ++it)
        legend.addEntry(it->first, it->second);

    anno->addFeature(new ViewPointGenFeatureGeoImage(rendered.first, rendered.second, legend));

    QRectF vp_roi = Grid2DLayerRenderer::geoROIOfOpaquePixels(rendered.first, rendered.second);
    if (!vp_roi.isEmpty())
        vp->setROI(vp_roi);

    nlohmann::json vp_json;
    vp->toJSON(vp_json);
    section.addFigure(fig_name, ResultReport::SectionContentViewable(vp_json));
}
}  // anonymous namespace

void SMRCoverageInspector::compute(AnalysisDataset* dataset)
{
    result_ = ComputeResult{};
    result_.classes[0].label  = "Small targets";
    result_.classes[1].label  = "Large targets";
    result_.movement[0].label = "Moving";
    result_.movement[1].label = "Standing";
    grid_.reset();

    if (!dataset)
        return;

    auto& settings = static_cast<SMRCoverageInspectorSettings&>(settings_);

    if (!dataset->hasPositionExtent())
    {
        logwrn << "SMRCoverageInspector: dataset has no reference positions, skipping";
        return;
    }

    const double ref_lat = dataset->centerLatitudeDeg();

    auto sizing = task_.clampedCellSizes(*dataset);
    if (sizing.horizontal_clamped)
    {
        loginf << "SMRCoverageInspector: cell size clamped to fit max "
               << task_.maxCellsPerAxis() << " cells/axis -- horizontal "
               << task_.cellSizeMeters() << " m x" << sizing.horizontal_multiplier
               << " = " << sizing.cell_size_m << " m";
    }

    grid_.reset(new TargetReport3DGrid(sizing.cell_size_m, sizing.cell_size_ft, ref_lat));
    auto& grid = *grid_;

    auto& ctx        = task_.compass().dbContextManager();
    auto& dbcont_man = task_.compass().dbContentManager();

    const bool   use_cycles       = settings.cadenceSource()
                                    == SMRCoverageInspectorSettings::CadenceSource::ScanCycles;
    const double nominal_period_s = settings.scan_period_s_;

    // Selected SMR sources: antenna position, scan cycles, cadence statistics.
    // One azimuth bin of a source's estimated radial coverage band.
    struct CoverageBin
    {
        std::uint64_t count   = 0;
        double        r_min   = 0.0;
        double        r_max   = 0.0;
        bool          covered = false;
    };

    struct SourceCtx
    {
        unsigned int              ds_id   = 0;
        bool                      has_pos = false;
        double                    lat     = 0.0;
        double                    lon     = 0.0;
        const std::vector<ptime>* cycles  = nullptr;

        // Estimated coverage, one entry per azimuth bin. Empty when the
        // estimate is disabled or the source has no position.
        std::vector<CoverageBin> coverage;
    };
    std::vector<SourceCtx> sources;

    for (auto ds_id : task_.selectedDataSourceIDs())
    {
        SourceCtx sc;
        sc.ds_id = ds_id;

        SourceRow row;
        row.ds_id = ds_id;

        const auto* ds = ctx.dataSource(ds_id);
        if (ds)
        {
            row.label = ds->name() + " (" + std::to_string(ds->sac())
                        + "/" + std::to_string(ds->sic()) + ")";
            if (ds->hasPosition())
            {
                sc.has_pos       = true;
                sc.lat           = ds->latitude();
                sc.lon           = ds->longitude();
                row.has_position = true;
            }
        }
        else
        {
            row.label = std::to_string(ds_id);
        }

        if (use_cycles && dataset->hasStatusCycles(ds_id))
        {
            sc.cycles      = &dataset->statusCycles(ds_id);
            row.cadence    = "scan cycles";
            row.num_cycles = sc.cycles->size();

            // Scan period statistics and missing scans (cycle gaps longer
            // than 1.5 median periods, counted in periods).
            std::vector<double> periods;
            periods.reserve(sc.cycles->size());
            for (std::size_t i = 1; i < sc.cycles->size(); ++i)
            {
                double dt = partialSeconds((*sc.cycles)[i] - (*sc.cycles)[i - 1]);
                if (dt > 0.0)
                    periods.push_back(dt);
            }
            if (!periods.empty())
            {
                row.period_median_s = percentile(periods, 0.5);
                row.period_p5_s     = percentile(periods, 0.05);
                row.period_p95_s    = percentile(periods, 0.95);
                if (row.period_median_s > 0.0)
                {
                    for (double dt : periods)
                        if (dt > 1.5 * row.period_median_s)
                            row.missing_scans += static_cast<unsigned int>(
                                std::llround(dt / row.period_median_s) - 1);
                }
            }
        }
        else
        {
            row.cadence = "nominal period";
        }

        sources.push_back(sc);
        result_.sources.push_back(std::move(row));
    }

    if (sources.empty())
    {
        result_.error = "No data source selected.";
        return;
    }

    // Estimated coverage per source, from the source's own target reports.
    // An SMR does not cover the whole airport: buildings shadow azimuth
    // wedges and the range is limited. Expecting a report where the sensor
    // never delivered one makes PD meaningless, so the radial band of each
    // azimuth bin is taken as that source's coverage.
    {
        using namespace dbcontent_vars;

        const double bin_deg = settings.coverage_azimuth_bin_deg_;
        auto         acc     = dataset->accessor();

        if (bin_deg > 0.0 && acc && acc->has(kTestDBContent)
            && acc->hasMetaVar<unsigned int>(kTestDBContent, meta_var_ds_id_)
            && acc->hasMetaVar<double>(kTestDBContent, meta_var_latitude_)
            && acc->hasMetaVar<double>(kTestDBContent, meta_var_longitude_))
        {
            const std::size_t num_bins =
                std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(360.0 / bin_deg)));

            std::map<unsigned int, std::size_t> src_index;
            for (std::size_t i = 0; i < sources.size(); ++i)
                if (sources[i].has_pos)
                    src_index[sources[i].ds_id] = i;

            // Report ranges per source and azimuth bin. Kept as flat sample
            // lists because the band uses percentiles, not sums.
            std::vector<std::vector<std::vector<double>>> ranges(
                sources.size(), std::vector<std::vector<double>>(num_bins));

            auto& ds_vec  = acc->getMetaVar<unsigned int>(kTestDBContent, meta_var_ds_id_);
            auto& lat_vec = acc->getMetaVar<double>(kTestDBContent, meta_var_latitude_);
            auto& lon_vec = acc->getMetaVar<double>(kTestDBContent, meta_var_longitude_);

            const unsigned int num_rows = acc->get(kTestDBContent)->size();
            Transformation cov_trafo;

            for (unsigned int idx = 0; idx < num_rows; ++idx)
            {
                if (ds_vec.isNull(idx) || lat_vec.isNull(idx) || lon_vec.isNull(idx))
                    continue;

                auto it = src_index.find(ds_vec.get(idx));
                if (it == src_index.end())
                    continue;

                const SourceCtx& sc = sources[it->second];

                bool   ok = false;
                double dx = 0.0, dy = 0.0;
                std::tie(ok, dx, dy) = cov_trafo.distanceCart(
                    sc.lat, sc.lon, lat_vec.get(idx), lon_vec.get(idx));
                if (!ok)
                    continue;

                const double rng = std::sqrt(dx * dx + dy * dy);
                if (rng <= 0.0)
                    continue;

                double az = std::atan2(dx, dy) * (180.0 / M_PI);
                if (az < 0.0)
                    az += 360.0;

                std::size_t bin = static_cast<std::size_t>(az / bin_deg);
                if (bin >= num_bins)
                    bin = num_bins - 1;

                ranges[it->second][bin].push_back(rng);
            }

            // Band per bin: P1 and P99 of the report ranges, so a single
            // spurious far return does not inflate it. A bin with too few
            // reports carries no band at all.
            for (std::size_t si = 0; si < sources.size(); ++si)
            {
                if (!sources[si].has_pos)
                    continue;

                std::vector<CoverageBin> bins(num_bins);
                for (std::size_t b = 0; b < num_bins; ++b)
                {
                    auto& v = ranges[si][b];
                    bins[b].count = v.size();
                    if (v.size() < settings.coverage_min_reports_)
                        continue;
                    bins[b].r_min   = percentile(v, 0.01);
                    bins[b].r_max   = percentile(v, 0.99);
                    bins[b].covered = bins[b].r_max > bins[b].r_min;
                }

                // Close single empty bins between two covered ones, so a
                // direction nothing happened to cross is not cut out.
                for (unsigned int pass = 0; pass < settings.coverage_smooth_bins_; ++pass)
                {
                    std::vector<CoverageBin> next = bins;
                    for (std::size_t b = 0; b < num_bins; ++b)
                    {
                        if (bins[b].covered)
                            continue;
                        const CoverageBin& prev = bins[(b + num_bins - 1) % num_bins];
                        const CoverageBin& succ = bins[(b + 1) % num_bins];
                        if (!prev.covered || !succ.covered)
                            continue;
                        next[b].covered = true;
                        next[b].r_min   = std::min(prev.r_min, succ.r_min);
                        next[b].r_max   = std::max(prev.r_max, succ.r_max);
                    }
                    bins.swap(next);
                }

                sources[si].coverage = std::move(bins);
            }
        }
    }
    for (std::size_t i = 0; i < sources.size(); ++i)
    {
        SourceRow& row = result_.sources[i];
        const auto& cov = sources[i].coverage;
        if (cov.empty())
            continue;

        std::vector<double> r_mins, r_maxs;
        for (const auto& b : cov)
        {
            row.coverage_reports += b.count;
            if (!b.covered)
                continue;
            ++row.coverage_bins_covered;
            r_mins.push_back(b.r_min);
            r_maxs.push_back(b.r_max);
        }
        row.coverage_valid      = row.coverage_bins_covered > 0;
        row.coverage_bins_total = cov.size();
        if (!r_mins.empty())
        {
            row.coverage_r_min_median = percentile(r_mins, 0.5);
            row.coverage_r_max_median = percentile(r_maxs, 0.5);
        }
        if (row.coverage_valid)
            result_.coverage_estimated = true;
    }

    const time_duration ref_max_gap         = durationFromSeconds(settings.ref_max_time_diff_s_);
    const time_duration min_period_duration = boost::posix_time::seconds(1);
    const time_duration d_max               = boost::posix_time::seconds(60);
    const double        max_range_m         = settings.max_range_m_;
    const unsigned int  large_cap           = settings.large_target_max_reports_per_scan_;

    Transformation trafo;

    // Per-sector breakdown (over the selected sector layers, independent of the
    // limit-by-sectors toggle). A slot is attributed to every sector it is in.
    auto sector_layers = task_.scopeSectorLayers();
    SectorWalkAccum sec;
    sec.layers = &sector_layers;
    sec.eui.assign(sector_layers.size(), 0);
    sec.mui.assign(sector_layers.size(), 0);
    sec.touched.assign(sector_layers.size(), 0);
    std::vector<std::size_t> sector_target_count(sector_layers.size(), 0);

    // Measured inter-report intervals per target and source.
    std::vector<double> intervals;

    static const std::vector<ptime> kNoTimestamps;

    const auto utns = dataset->utns();

    for (auto utn : utns)
    {
        if (!dataset->hasReferenceChain(utn))
        {
            ++result_.targets_no_ref;
            continue;
        }
        auto& ref_chain = dataset->referenceChain(utn);
        if (!ref_chain.hasData())
        {
            ++result_.targets_no_ref;
            continue;
        }

        std::set<ptime> ref_ts;
        for (const auto& kv : ref_chain.timestampIndexes())
            ref_ts.insert(kv.first);

        auto periods = EvaluationRequirement::PDHelpers::buildReferencePeriods(
            ref_ts, ref_max_gap, min_period_duration);
        if (periods.empty())
        {
            ++result_.targets_no_ref;
            continue;
        }

        // Test reports of this target per source, plus reported target lengths.
        std::map<unsigned int, std::vector<ptime>> tst_by_ds;
        std::vector<double> lengths_m;

        if (dataset->hasTestChain(utn, kTestDBContent))
        {
            auto& chain = dataset->testChain(utn, kTestDBContent);
            for (const auto& kv : chain.timestampIndexes())
            {
                Chain::DataID id(kv.first, kv.second);
                tst_by_ds[chain.dsID(id)].push_back(kv.first);

                auto len = chain.targetLength(id);
                if (len.has_value() && std::isfinite(*len) && *len > 0.0f)
                    lengths_m.push_back(static_cast<double>(*len));
            }
        }

        for (auto& kv : tst_by_ds)
        {
            auto& ts = kv.second;
            std::sort(ts.begin(), ts.end());
            for (std::size_t i = 1; i < ts.size(); ++i)
            {
                double dt = partialSeconds(ts[i] - ts[i - 1]);
                if (dt > 0.0 && dt <= kMaxUpdateIntervalS)
                    intervals.push_back(dt);
            }
        }

        if (tst_by_ds.empty())
            ++result_.targets_no_tst;

        const bool small = isSmallTarget(dbcont_man, utn, lengths_m,
                                         settings.small_target_max_length_m_);
        ClassRow& cls = result_.classes[small ? 0 : 1];
        ++cls.num_targets;

        // Movement classification for the diagnostic breakdown only.
        SpeedSamples ref_spd = gatherRefSpeeds(ref_chain);
        MovementUI mv;
        mv.ref          = &ref_spd;
        mv.standing_max = settings.standing_speed_max_mps_;
        mv.window_s     = 10.0;

        ++result_.targets_walked;

        std::fill(sec.touched.begin(), sec.touched.end(), 0);

        for (std::size_t src_idx = 0; src_idx < sources.size(); ++src_idx)
        {
            const SourceCtx& src  = sources[src_idx];
            SourceRow&       srow = result_.sources[src_idx];

            auto tst_it = tst_by_ds.find(src.ds_id);
            const std::vector<ptime>& tst =
                (tst_it != tst_by_ds.end()) ? tst_it->second : kNoTimestamps;

            for (const auto& period : periods)
            {
                std::vector<CycleEvent> events =
                    src.cycles
                        ? analysis::evaluateCyclesInPeriod(period, *src.cycles, tst, nullptr)
                        : analysis::evaluateNominalSlotsInPeriod(period, nominal_period_s, tst);

                for (const auto& ev : events)
                {
                    auto ca = refCellAt(*dataset, utn, ev.t_cycle, d_max);
                    if (!ca.valid)
                        continue;

                    if (src.has_pos && max_range_m > 0.0)
                    {
                        bool   ok   = false;
                        double dist = 0.0;
                        std::tie(ok, dist) = trafo.distanceL2Cart(src.lat, src.lon, ca.lat, ca.lon);
                        if (ok && dist > max_range_m)
                        {
                            ++result_.slots_out_of_range;
                            continue;
                        }
                    }

                    // Estimated coverage gate. A slot outside the source's
                    // radial band in its azimuth is not expected: the sensor
                    // has never delivered a report there, so counting it as a
                    // miss would measure the airport layout, not the sensor.
                    // Both outcomes feed the coverage figure, so the map shows
                    // what was kept and what was cut.
                    if (!src.coverage.empty())
                    {
                        bool   ok = false;
                        double dx = 0.0, dy = 0.0;
                        std::tie(ok, dx, dy) = trafo.distanceCart(src.lat, src.lon,
                                                                  ca.lat, ca.lon);
                        bool inside = false;
                        if (ok)
                        {
                            const double rng = std::sqrt(dx * dx + dy * dy);
                            double az = std::atan2(dx, dy) * (180.0 / M_PI);
                            if (az < 0.0)
                                az += 360.0;

                            std::size_t bin = static_cast<std::size_t>(
                                az / settings.coverage_azimuth_bin_deg_);
                            if (bin >= src.coverage.size())
                                bin = src.coverage.size() - 1;

                            const auto& cb = src.coverage[bin];
                            inside = cb.covered && rng >= cb.r_min && rng <= cb.r_max;
                        }

                        const std::uint64_t ckey = grid.horizontalKey(ca.lat, ca.lon);
                        ++result_.coverage_samples[ckey];
                        if (inside)
                            result_.coverage_by_key[ckey] += 1.0;

                        if (!inside)
                        {
                            ++result_.slots_out_of_coverage;
                            continue;
                        }
                    }

                    grid.addEUI(ca.lat, ca.lon, 0.0);
                    ++srow.eui;
                    ++cls.eui;
                    GroupRow& mrow = result_.movement[mv.standingAt(ev.t_cycle) ? 1 : 0];
                    ++mrow.eui;
                    sec.accum(ca, false);

                    const unsigned int n = ev.num_reports;

                    if (n == 0)
                    {
                        grid.addMUI(ca.lat, ca.lon, 0.0);
                        ++srow.mui;
                        ++cls.mui;
                        ++mrow.mui;
                        ++cls.scans_0;
                        sec.accum(ca, true);
                        continue;
                    }

                    if (n == 1)      ++cls.scans_1;
                    else if (n == 2) ++cls.scans_2;
                    else             ++cls.scans_3plus;

                    std::uint64_t extra = 0;
                    if (small)
                        extra = n - 1;
                    else if (large_cap > 0 && n > large_cap)
                        extra = n - large_cap;

                    if (extra > 0)
                    {
                        grid.addExtra(ca.lat, ca.lon, 0.0, extra);
                        cls.extra           += extra;
                        result_.total_extra += extra;
                    }
                }
            }
        }

        for (std::size_t si = 0; si < sector_layers.size(); ++si)
            if (sec.touched[si])
                ++sector_target_count[si];
    }

    // Coverage figure values: share of the cell's slots that were inside a
    // source's band. Accumulated as a count during the walk.
    for (auto& kv : result_.coverage_by_key)
    {
        auto it = result_.coverage_samples.find(kv.first);
        if (it != result_.coverage_samples.end() && it->second > 0)
            kv.second /= static_cast<double>(it->second);
    }
    for (const auto& kv : result_.coverage_samples)
        result_.coverage_by_key.emplace(kv.first, 0.0);

    // Aggregate per-cell PD over the horizontal projection.
    auto horizontal = grid.projectHorizontal();
    std::uint64_t total_eui = 0, total_mui = 0;
    double worst_pd  = 1.0;
    bool   worst_set = false;
    std::vector<double> per_cell_pd;
    per_cell_pd.reserve(horizontal.size());
    for (const auto& kv : horizontal)
    {
        total_eui += kv.second.num_eui;
        total_mui += kv.second.num_mui;
        if (kv.second.num_eui == 0)
            continue;
        double pd = pdOf(kv.second.num_eui, kv.second.num_mui);
        per_cell_pd.push_back(pd);
        if (kv.second.num_eui >= 5 && (!worst_set || pd < worst_pd))
        {
            worst_pd  = pd;
            worst_set = true;
        }
    }

    result_.valid              = true;
    result_.total_eui          = total_eui;
    result_.total_mui          = total_mui;
    result_.overall_pd         = pdOf(total_eui, total_mui);
    result_.cells_with_eui     = per_cell_pd.size();
    result_.median_per_cell_pd = percentile(per_cell_pd, 0.5);
    result_.p5_per_cell_pd     = percentile(per_cell_pd, 0.05);
    result_.has_worst_cell     = worst_set;
    result_.worst_cell_pd      = worst_pd;

    for (auto& row : result_.sources)
        row.pd = pdOf(row.eui, row.mui);
    for (auto& row : result_.classes)
        row.pd = pdOf(row.eui, row.mui);
    for (auto& row : result_.movement)
        row.pd = pdOf(row.eui, row.mui);

    result_.ui_num_intervals = intervals.size();
    if (!intervals.empty())
    {
        result_.ui_median_s = percentile(intervals, 0.5);
        result_.ui_p10_s    = percentile(intervals, 0.1);
        result_.ui_p90_s    = percentile(intervals, 0.9);
        result_.ui_mean_s   = mean(intervals);

        double hist_max = settings.ui_hist_max_s_ > 0.0f
                              ? static_cast<double>(settings.ui_hist_max_s_)
                              : std::ceil(percentile(intervals, 0.99));
        if (hist_max <= 0.0)
            hist_max = 5.0;
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

    for (std::size_t si = 0; si < sector_layers.size(); ++si)
    {
        if (!sector_layers[si])
            continue;
        SectorRow r;
        r.label       = sector_layers[si]->name();
        r.num_targets = sector_target_count[si];
        r.eui         = sec.eui[si];
        r.mui         = sec.mui[si];
        r.pd          = pdOf(r.eui, r.mui);
        result_.sectors.push_back(std::move(r));
    }

    loginf << "SMRCoverageInspector: walked " << result_.targets_walked << " target(s), "
           << result_.total_eui << " EUI, " << result_.total_mui << " MUI, "
           << result_.total_extra << " extra, overall PD " << result_.overall_pd;
}

void SMRCoverageInspector::writeReport(ResultReport::Section& root)
{
    auto& settings = static_cast<SMRCoverageInspectorSettings&>(settings_);
    auto& section  = root.addSubSection(name());

    {
        auto& intro = section.addText("About");
        intro.addText(
            "Scan-based Probability of Detection (PD) of the selected SMR sources per "
            "cell of the horizontal grid, using the Reference Trajectory as ground "
            "truth. Every target inside coverage is expected once per antenna scan, "
            "standing or moving, per source: the scan windows come from the source's "
            "CAT010 Start of Update Cycle messages, or from a nominal scan period when "
            "a source sends no cycle messages. A scan window without a report of the "
            "target is a missed update at the reference position of the scan.\n"
            "Small targets (vehicles, light aircraft, or a reported length below the "
            "configured limit) must produce exactly one report per scan. Additional "
            "reports are counted as extra reports (splits) and drawn on the Extra "
            "Reports map, they do not change PD. Large targets may produce several "
            "reports per scan.\n"
            "The Scan Period table lists the measured scan period and the missing "
            "scans per source. The Reports per Scan table shows how often a target "
            "produced 0, 1, 2 or more reports in a scan. The PD breakdowns by "
            "movement, size class, source and sector locate where and for whom "
            "detection fails.\n"
            "PD is measured only inside the estimated coverage of each source. An "
            "SMR does not see the whole airport: buildings shadow whole azimuth "
            "wedges and stands, and the range is limited. The coverage is estimated "
            "from the source's own target reports, per azimuth bin the radial band "
            "between the 1st and the 99th percentile of the report ranges. A bin "
            "with too few reports counts as not covered. Slots outside the band are "
            "not expected and are reported separately. The Estimated Coverage map "
            "shows which part of the walked area was kept.");
    }

    auto& recap = section.addTable("Settings", 2, {"Setting", "Value"}, false);
    recap.addRow({"Cadence source",
                  settings.cadenceSource() == SMRCoverageInspectorSettings::CadenceSource::ScanCycles
                      ? std::string("Scan cycles (CAT010 Start of Update Cycle), "
                                    "nominal period as fallback")
                      : std::string("Nominal scan period")});
    {
        std::ostringstream os;
        os << settings.scan_period_s_ << " s";
        recap.addRow({"Nominal scan period", os.str()});
    }
    {
        std::ostringstream os;
        os << "small: exactly 1 report per scan\n"
           << "large: " << (settings.large_target_max_reports_per_scan_ == 0
                                ? std::string("unlimited")
                                : "max " + std::to_string(settings.large_target_max_reports_per_scan_))
           << " reports per scan\n"
           << "small if reported length < " << settings.small_target_max_length_m_
           << " m (when the target category does not decide)";
        recap.addRow({"Small target rule", os.str()});
    }
    {
        std::ostringstream os;
        if (settings.max_range_m_ > 0.0f)
            os << settings.max_range_m_ << " m from the antenna (sources with position)";
        else
            os << "none";
        recap.addRow({"Maximum range", os.str()});
    }
    {
        std::ostringstream os;
        if (settings.coverage_azimuth_bin_deg_ > 0.0f)
            os << "radial band per " << settings.coverage_azimuth_bin_deg_
               << " deg azimuth bin, P1 to P99 of the report ranges\n"
               << "at least " << settings.coverage_min_reports_
               << " reports per bin, " << settings.coverage_smooth_bins_
               << " smoothing pass(es)";
        else
            os << "off (every slot inside the maximum range is expected)";
        recap.addRow({"Estimated coverage", os.str()});
    }
    {
        std::ostringstream os;
        os << settings.ref_max_time_diff_s_ << " s";
        recap.addRow({"Reference period split threshold", os.str()});
    }
    {
        std::ostringstream os;
        os << settings.standing_speed_max_mps_ << " m/s (breakdown only)";
        recap.addRow({"Standing speed threshold", os.str()});
    }
    {
        std::ostringstream os;
        os << task_.cellSizeMeters() << " m horizontal\n"
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
    {
        std::ostringstream os;
        os << "green <= " << settings.extra_ratio_acceptable_below_ << "\n"
           << "red >= "   << settings.extra_ratio_unacceptable_above_ << "\n"
           << "orange in between (extra reports per expected slot)";
        recap.addRow({"Extra reports color thresholds", os.str()});
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

    // Scan period per source.
    {
        auto& st = section.addTable("Scan Period", 8,
                                    {"Data Source", "Cadence", "Position", "Cycles",
                                     "Median (s)", "P5 (s)", "P95 (s)", "Missing Scans"},
                                    false);
        for (const auto& r : result_.sources)
        {
            const bool has_stats = r.num_cycles >= 2;
            st.addRow({r.label,
                       r.cadence,
                       r.has_position ? "yes" : "no",
                       std::to_string(r.num_cycles),
                       has_stats ? formatNumber(r.period_median_s, 3) : std::string("-"),
                       has_stats ? formatNumber(r.period_p5_s, 3) : std::string("-"),
                       has_stats ? formatNumber(r.period_p95_s, 3) : std::string("-"),
                       has_stats ? std::to_string(r.missing_scans) : std::string("-")});
        }
    }

    // Estimated coverage per source.
    if (result_.coverage_estimated)
    {
        auto& ct = section.addTable("Estimated Coverage", 6,
                                    {"Data Source", "Reports", "Azimuth Bins",
                                     "Bins with Coverage", "Median Inner Range (m)",
                                     "Median Outer Range (m)"},
                                    false);
        for (const auto& r : result_.sources)
        {
            if (!r.coverage_valid)
            {
                ct.addRow({r.label, std::to_string(r.coverage_reports), "-", "-", "-", "-"});
                continue;
            }
            std::ostringstream cov;
            cov << r.coverage_bins_covered << " ("
                << formatNumber(100.0 * static_cast<double>(r.coverage_bins_covered)
                                / static_cast<double>(r.coverage_bins_total), 1)
                << " %)";
            ct.addRow({r.label,
                       std::to_string(r.coverage_reports),
                       std::to_string(r.coverage_bins_total),
                       cov.str(),
                       formatNumber(r.coverage_r_min_median, 1),
                       formatNumber(r.coverage_r_max_median, 1)});
        }
    }

    auto& summary = section.addTable("Summary", 2, {"Property", "Value"}, false);
    summary.addRow({"Targets walked",         std::to_string(result_.targets_walked)});
    summary.addRow({"Targets w/o RefTraj",    std::to_string(result_.targets_no_ref)});
    summary.addRow({"Targets w/o test data",  std::to_string(result_.targets_no_tst)});
    summary.addRow({"Small targets",          std::to_string(result_.classes[0].num_targets)});
    summary.addRow({"Large targets",          std::to_string(result_.classes[1].num_targets)});
    summary.addRow({"Slots beyond maximum range", std::to_string(result_.slots_out_of_range)});
    if (result_.coverage_estimated)
        summary.addRow({"Slots outside estimated coverage",
                        std::to_string(result_.slots_out_of_coverage)});
    summary.addRow({"Total expected slots (#EUI)", std::to_string(result_.total_eui)});
    summary.addRow({"Total missed slots (#MUI)",   std::to_string(result_.total_mui)});
    summary.addRow({"Overall PD",                  formatNumber(result_.overall_pd)});
    summary.addRow({"Total extra reports",         std::to_string(result_.total_extra)});
    summary.addRow({"Cells with EUI",              std::to_string(result_.cells_with_eui)});
    summary.addRow({"Median per-cell PD",          formatNumber(result_.median_per_cell_pd)});
    summary.addRow({"P5 per-cell PD",              formatNumber(result_.p5_per_cell_pd)});
    if (result_.has_worst_cell)
        summary.addRow({"Worst cell PD (>=5 EUI)", formatNumber(result_.worst_cell_pd)});

    // Reports per scan, per size class.
    {
        auto& rt = section.addTable("Reports per Scan", 9,
                                    {"Size Class", "Targets", "0", "1", "2", "3+",
                                     "Split Scans (%)", "Extra Reports", "PD"},
                                    false);
        for (const auto& c : result_.classes)
        {
            const std::uint64_t hits = c.scans_1 + c.scans_2 + c.scans_3plus;
            const std::uint64_t split_scans = c.scans_2 + c.scans_3plus;
            const double split_pct = hits > 0
                ? 100.0 * static_cast<double>(split_scans) / static_cast<double>(hits)
                : 0.0;
            rt.addRow({c.label,
                       std::to_string(c.num_targets),
                       std::to_string(c.scans_0),
                       std::to_string(c.scans_1),
                       std::to_string(c.scans_2),
                       std::to_string(c.scans_3plus),
                       formatNumber(split_pct, 2),
                       std::to_string(c.extra),
                       formatNumber(c.pd)});
        }
    }

    // Reports per scan histograms.
    for (const auto& c : result_.classes)
    {
        if (c.eui == 0)
            continue;

        RawHistogram h;
        h.addBin(RawHistogramBin(clampCount(c.scans_0), "0", RawHistogramBin::Tag::Standard, "0", "0"));
        h.addBin(RawHistogramBin(clampCount(c.scans_1), "1", RawHistogramBin::Tag::Standard, "1", "1"));
        h.addBin(RawHistogramBin(clampCount(c.scans_2), "2", RawHistogramBin::Tag::Standard, "2", "2"));
        h.addBin(RawHistogramBin(clampCount(c.scans_3plus), "3+", RawHistogramBin::Tag::Standard, "3", ""));

        const std::string fig = "Reports per Scan - " + c.label;
        auto vp = std::make_unique<ViewPointGenVP>(fig, 0, "Histogram");
        vp->noDataLoaded(true);
        auto* anno = vp->annotations().getOrCreateAnnotation("Reports per Scan");
        anno->addFeature(new ViewPointGenFeatureHistogram(
            h, "Reports per Scan", QColor(0, 128, 192), boost::optional<bool>(),
            PlotMetadata("Sensor Coverage", fig, "Reports per Scan", "Count")));

        nlohmann::json vp_json;
        vp->toJSON(vp_json);
        section.addFigure(fig, ResultReport::SectionContentViewable(vp_json));
    }

    // Measured inter-report interval.
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

    // PD breakdowns.
    {
        auto& mt = section.addTable("PD by Movement", 4,
                                    {"Movement", "#EUI", "#MUI", "PD"}, false);
        for (const auto& r : result_.movement)
            mt.addRow({r.label, std::to_string(r.eui), std::to_string(r.mui), formatNumber(r.pd)});
    }
    {
        auto& ct = section.addTable("PD by Size Class", 5,
                                    {"Size Class", "Targets", "#EUI", "#MUI", "PD"}, false);
        for (const auto& r : result_.classes)
            ct.addRow({r.label, std::to_string(r.num_targets), std::to_string(r.eui),
                       std::to_string(r.mui), formatNumber(r.pd)});
    }
    {
        auto& dt = section.addTable("PD by Data Source", 4,
                                    {"Data Source", "#EUI", "#MUI", "PD"}, false);
        for (const auto& r : result_.sources)
            dt.addRow({r.label, std::to_string(r.eui), std::to_string(r.mui), formatNumber(r.pd)});
    }

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
        {
            auto proj = grid_->projectionLayer(TargetReport3DGrid::Projection::Horizontal,
                                               pdScalar, euiSampleCount, "pd");

            const std::pair<double, double> range(settings.pd_unacceptable_below_,
                                                  settings.pd_acceptable_above_);
            Grid2DRenderSettings rs;
            rs.color_map.create(ColorMap::ColorScale::Red2Green, 3,
                                ColorMap::Type::LinearSamples, range);
            rs.min_value = range.first;
            rs.max_value = range.second;

            attachGridFigure(section, "PD - Horizontal", "PD", proj, rs);
        }

        if (!result_.coverage_by_key.empty())
        {
            auto proj = grid_->horizontalLayer(result_.coverage_by_key,
                                               result_.coverage_samples, "coverage");
            Grid2DRenderSettings rs;
            rs.color_map.create(ColorMap::ColorScale::Red2Green, 5,
                                ColorMap::Type::LinearSamples, std::make_pair(0.0, 1.0));
            rs.min_value = 0.0;
            rs.max_value = 1.0;

            attachGridFigure(section, "Estimated Coverage - Horizontal",
                             "In Coverage", proj, rs);
        }

        if (result_.total_extra > 0)
        {
            auto proj = grid_->projectionLayer(TargetReport3DGrid::Projection::Horizontal,
                                               extraRatioScalar, euiSampleCount, "extra");

            const std::pair<double, double> range(settings.extra_ratio_acceptable_below_,
                                                  settings.extra_ratio_unacceptable_above_);
            Grid2DRenderSettings rs;
            rs.color_map.create(ColorMap::ColorScale::Green2Red, 3,
                                ColorMap::Type::LinearSamples, range);
            rs.min_value = range.first;
            rs.max_value = range.second;

            attachGridFigure(section, "Extra Reports - Horizontal", "Extra Reports", proj, rs);
        }
    }

    loginf << "SMRCoverageInspector: report written, " << result_.targets_walked
           << " target(s), overall PD " << result_.overall_pd;
}
