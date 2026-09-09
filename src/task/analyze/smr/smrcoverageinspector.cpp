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
#include "sector.h"
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
    registerParameter("coverage_gap_m",           &coverage_gap_m_,
                      coverage_gap_m_);
    registerParameter("coverage_min_segment_reports", &coverage_min_segment_reports_,
                      coverage_min_segment_reports_);
    registerParameter("coverage_sector_link_m",   &coverage_sector_link_m_,
                      coverage_sector_link_m_);
    registerParameter("coverage_sector_bridge_bins", &coverage_sector_bridge_bins_,
                      coverage_sector_bridge_bins_);
    registerParameter("coverage_sector_smooth_bins", &coverage_sector_smooth_bins_,
                      coverage_sector_smooth_bins_);
    registerParameter("coverage_sector_simplify_m", &coverage_sector_simplify_m_,
                      coverage_sector_simplify_m_);
    registerParameter("coverage_sector_min_area_m2", &coverage_sector_min_area_m2_,
                      coverage_sector_min_area_m2_);
    registerParameter("coverage_min_run_reports_per_100m", &coverage_min_run_reports_per_100m_,
                      coverage_min_run_reports_per_100m_);
    registerParameter("create_coverage_sectors",  &create_coverage_sectors_,
                      create_coverage_sectors_);
    registerParameter("coverage_sector_layer_suffix", &coverage_sector_layer_suffix_,
                      coverage_sector_layer_suffix_);
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
    // One covered range segment of an azimuth bin. A direction can hold several:
    // a taxiway close in and a runway far out, with a dead gap between them the
    // sensor never reports in. A single interval per bin would claim that gap as
    // coverage and, where only some bins catch the far returns, draw a needle
    // across the map.
    struct CoverageSegment
    {
        std::uint64_t count = 0;
        double        r_min = 0.0;
        double        r_max = 0.0;

        bool overlaps(const CoverageSegment& other) const
        { return r_min <= other.r_max && other.r_min <= r_max; }
    };

    struct CoverageBin
    {
        std::uint64_t count = 0;   // all reports of the bin, covered or not

        // Ordered by range, disjoint. Empty means the bin is not covered.
        std::vector<CoverageSegment> segments;

        bool covered() const { return !segments.empty(); }

        double rMin() const { return segments.front().r_min; }
        double rMax() const { return segments.back().r_max; }

        bool contains(double range_m) const
        {
            for (const auto& seg : segments)
                if (range_m >= seg.r_min && range_m <= seg.r_max)
                    return true;
            return false;
        }

        std::uint64_t coveredCount() const
        {
            std::uint64_t sum = 0;
            for (const auto& seg : segments)
                sum += seg.count;
            return sum;
        }
    };

    // One chain of overlapping segments across adjacent azimuth bins: what
    // becomes one sector, and what the density check judges. `band` holds one
    // range interval per bin, starting at walk index `begin_j` of the walk that
    // started at bin `cut`, so the azimuth of entry i is (cut + begin_j + i)
    // times the bin width and increases across the 360 degree seam.
    struct CoverageRun
    {
        std::size_t   cut     = 0;
        std::size_t   begin_j = 0;
        std::uint64_t reports = 0;

        std::vector<std::pair<double, double>> band;
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

        // The same coverage as chains of overlapping segments, one per sector.
        std::vector<CoverageRun> coverage_runs;
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

                    // Split the bin's reports where a gap in range carries none,
                    // each cluster is one covered segment from its first to its
                    // last report. No percentile trimming: a lone far return does
                    // not survive the report minimum of its own segment, so the
                    // segment can keep the full extent of what was seen.
                    std::sort(v.begin(), v.end());

                    const double gap_m = settings.coverage_gap_m_;

                    std::size_t seg_begin = 0;
                    for (std::size_t i = 1; i <= v.size(); ++i)
                    {
                        if (i < v.size() && (gap_m <= 0.0 || v[i] - v[i - 1] <= gap_m))
                            continue;

                        const std::size_t n = i - seg_begin;
                        if (n >= settings.coverage_min_segment_reports_)
                        {
                            CoverageSegment seg;
                            seg.count = n;
                            seg.r_min = v[seg_begin];
                            seg.r_max = v[i - 1];

                            if (seg.r_max > seg.r_min)
                                bins[b].segments.push_back(seg);
                        }

                        seg_begin = i;
                    }
                }

                // Close single empty bins between two covered ones, so a
                // direction nothing happened to cross is not cut out. The closed
                // bin takes the segments of both neighbors.
                for (unsigned int pass = 0; pass < settings.coverage_smooth_bins_; ++pass)
                {
                    std::vector<CoverageBin> next = bins;
                    for (std::size_t b = 0; b < num_bins; ++b)
                    {
                        if (bins[b].covered())
                            continue;

                        const CoverageBin& prev = bins[(b + num_bins - 1) % num_bins];
                        const CoverageBin& succ = bins[(b + 1) % num_bins];

                        if (!prev.covered() || !succ.covered())
                            continue;

                        std::vector<CoverageSegment> merged = prev.segments;
                        merged.insert(merged.end(), succ.segments.begin(), succ.segments.end());
                        std::sort(merged.begin(), merged.end(),
                                  [](const CoverageSegment& a, const CoverageSegment& c)
                                  { return a.r_min < c.r_min; });

                        // union of the overlapping ones, the closed bin must not
                        // claim more segments than its neighbors show
                        std::vector<CoverageSegment> unioned;
                        for (const auto& seg : merged)
                        {
                            if (!unioned.empty() && unioned.back().overlaps(seg))
                            {
                                unioned.back().r_max = std::max(unioned.back().r_max, seg.r_max);
                                unioned.back().count += seg.count;
                            }
                            else
                            {
                                unioned.push_back(seg);
                            }
                        }

                        next[b].segments = std::move(unioned);
                    }
                    bins.swap(next);
                }

                // Chain the segments into runs: a segment continues the run of a
                // segment in the previous bin when their ranges overlap, or lie
                // within `link_m` of each other. A run is what becomes one
                // sector, and what the density check judges. The walk starts at
                // an uncovered bin so a run across the 360 degree seam stays
                // whole, and the walk index keeps the azimuths increasing.
                auto chain_runs = [&](const std::vector<CoverageBin>& src, double link_m,
                                      std::size_t bridge_bins)
                {
                    std::size_t cut = 0;
                    while (cut < src.size() && src[cut].covered())
                        ++cut;
                    if (cut == src.size())   // fully covered, the seam falls somewhere
                        cut = 0;

                    std::vector<CoverageRun> runs;
                    std::vector<std::size_t> open;   // active in the last bin

                    for (std::size_t j = 0; j < src.size(); ++j)
                    {
                        const CoverageBin& bin = src[(cut + j) % src.size()];

                        std::vector<std::size_t> next_open;

                        for (const auto& seg : bin.segments)
                        {
                            std::vector<std::size_t> matches;
                            for (std::size_t r : open)
                            {
                                const auto& last = runs[r].band.back();
                                if (seg.r_min - link_m <= last.second
                                    && last.first - link_m <= seg.r_max)
                                    matches.push_back(r);
                            }

                            // a run whose last entry lies further back takes the
                            // bridged bins, filled with the union of both ends
                            for (std::size_t r : matches)
                            {
                                CoverageRun& run = runs[r];
                                while (run.begin_j + run.band.size() < j)
                                {
                                    const auto& last = run.band.back();
                                    run.band.emplace_back(std::min(last.first,  seg.r_min),
                                                          std::max(last.second, seg.r_max));
                                }
                            }

                            if (matches.empty())
                            {
                                CoverageRun run;
                                run.cut     = cut;
                                run.begin_j = j;
                                run.band.emplace_back(seg.r_min, seg.r_max);
                                run.reports = seg.count;
                                runs.push_back(std::move(run));
                                next_open.push_back(runs.size() - 1);
                                continue;
                            }

                            // the first match takes the segment, the others merge
                            // into it: their bands are unioned bin by bin
                            CoverageRun& target = runs[matches[0]];

                            for (std::size_t m = 1; m < matches.size(); ++m)
                            {
                                CoverageRun& other = runs[matches[m]];

                                const std::size_t begin_j = std::min(target.begin_j, other.begin_j);
                                const std::size_t end_j   = std::max(
                                    target.begin_j + target.band.size(),
                                    other.begin_j + other.band.size());

                                std::vector<std::pair<double, double>> band(
                                    end_j - begin_j,
                                    std::make_pair(std::numeric_limits<double>::max(), 0.0));

                                auto merge_into = [&](const CoverageRun& from)
                                {
                                    for (std::size_t i = 0; i < from.band.size(); ++i)
                                    {
                                        auto& dst = band[from.begin_j - begin_j + i];
                                        dst.first  = std::min(dst.first,  from.band[i].first);
                                        dst.second = std::max(dst.second, from.band[i].second);
                                    }
                                };
                                merge_into(target);
                                merge_into(other);

                                target.band     = std::move(band);
                                target.begin_j  = begin_j;
                                target.reports += other.reports;

                                other.band.clear();
                                other.reports = 0;

                                next_open.erase(std::remove(next_open.begin(), next_open.end(),
                                                            matches[m]),
                                                next_open.end());
                            }

                            if (target.begin_j + target.band.size() == j)
                            {
                                target.band.emplace_back(seg.r_min, seg.r_max);
                            }
                            else
                            {
                                auto& dst = target.band.back();
                                dst.first  = std::min(dst.first,  seg.r_min);
                                dst.second = std::max(dst.second, seg.r_max);
                            }

                            target.reports += seg.count;

                            if (std::find(next_open.begin(), next_open.end(), matches[0])
                                == next_open.end())
                                next_open.push_back(matches[0]);
                        }

                        // runs without a segment in this bin stay open while the
                        // gap is short enough to bridge
                        for (std::size_t r : open)
                            if (std::find(next_open.begin(), next_open.end(), r) == next_open.end()
                                && !runs[r].band.empty()
                                && j + 1 - (runs[r].begin_j + runs[r].band.size()) <= bridge_bins)
                                next_open.push_back(r);

                        open.swap(next_open);
                    }

                    runs.erase(std::remove_if(runs.begin(), runs.end(),
                                              [](const CoverageRun& r) { return r.band.empty(); }),
                               runs.end());
                    return runs;
                };

                std::vector<CoverageRun> runs = chain_runs(bins, 0.0, 0);

                // Cut thin runs: reports spread thinly along a long band are a
                // single pass through that direction, not coverage.
                for (auto& run : runs)
                {
                    if (run.band.empty())
                        continue;

                    double r_min = std::numeric_limits<double>::max();
                    double r_max = 0.0;
                    for (const auto& e : run.band)
                    {
                        r_min = std::min(r_min, e.first);
                        r_max = std::max(r_max, e.second);
                    }

                    // No floor worth speaking of: with segments a short band is
                    // dense by nature, the check is about long and thin runs.
                    const double band_m  = std::max(1.0, r_max - r_min);
                    const double density = run.reports * 100.0 / band_m;
                    const bool   cut_run = settings.coverage_min_run_reports_per_100m_ > 0.0f
                                           && density < settings.coverage_min_run_reports_per_100m_;

                    loginf << sources[si].ds_id << ": coverage run "
                           << Utils::String::doubleToStringPrecision(
                                  (run.cut + run.begin_j) * bin_deg, 2)
                           << " to "
                           << Utils::String::doubleToStringPrecision(
                                  (run.cut + run.begin_j + run.band.size()) * bin_deg, 2)
                           << " deg, " << run.band.size() << " bin(s), " << run.reports
                           << " report(s), band "
                           << Utils::String::doubleToStringPrecision(r_max - r_min, 0)
                           << " m, density "
                           << Utils::String::doubleToStringPrecision(density, 1)
                           << " per 100 m" << (cut_run ? " - cut" : "");

                    if (!cut_run)
                        continue;

                    // drop the run's segments, so the gate and the sectors agree
                    for (std::size_t i = 0; i < run.band.size(); ++i)
                    {
                        const std::size_t b = (run.cut + run.begin_j + i) % num_bins;
                        auto& segs = bins[b].segments;

                        segs.erase(std::remove_if(segs.begin(), segs.end(),
                                                  [&](const CoverageSegment& seg)
                                                  {
                                                      return seg.r_min <= run.band[i].second
                                                             && run.band[i].first <= seg.r_max;
                                                  }),
                                   segs.end());
                    }

                    run.band.clear();
                }

                runs.erase(std::remove_if(runs.begin(), runs.end(),
                                          [](const CoverageRun& r) { return r.band.empty(); }),
                           runs.end());

                // The sector runs are glued and smoothed: segments of neighboring
                // bins within the link distance become one sector, and its edges
                // are widened by a rolling maximum outward and a rolling minimum
                // inward. Both only ever grow the area, so no report the sensor
                // delivered falls out of its sector. The gate above keeps the
                // exact segments, only the drawn sectors are simplified.
                auto sector_runs = chain_runs(bins, settings.coverage_sector_link_m_,
                                              settings.coverage_sector_bridge_bins_);

                const int smooth_w = static_cast<int>(settings.coverage_sector_smooth_bins_);

                if (smooth_w > 0)
                {
                    for (auto& run : sector_runs)
                    {
                        const int n = static_cast<int>(run.band.size());
                        std::vector<std::pair<double, double>> widened(run.band.size());

                        for (int i = 0; i < n; ++i)
                        {
                            double lo = run.band[i].first;
                            double hi = run.band[i].second;

                            for (int d = -smooth_w; d <= smooth_w; ++d)
                            {
                                const int k = i + d;
                                if (k < 0 || k >= n)
                                    continue;
                                lo = std::min(lo, run.band[k].first);
                                hi = std::max(hi, run.band[k].second);
                            }

                            widened[i] = std::make_pair(lo, hi);
                        }

                        run.band.swap(widened);
                    }
                }

                // Merge sector runs that would intersect: two runs sharing bins
                // where their bands overlap, or come within the simplification
                // tolerance of each other, become one sector. Two sectors of a
                // source can then no longer cross, and the ones that stay apart
                // keep a real gap between them. Repeated until nothing changes,
                // a merge can create a new overlap.
                {
                    const double tol = settings.coverage_sector_simplify_m_;

                    bool merged_any = true;
                    while (merged_any)
                    {
                        merged_any = false;

                        for (std::size_t i = 0; i < sector_runs.size() && !merged_any; ++i)
                        {
                            if (sector_runs[i].band.empty())
                                continue;

                            for (std::size_t j = i + 1; j < sector_runs.size() && !merged_any; ++j)
                            {
                                CoverageRun& a = sector_runs[i];
                                CoverageRun& b = sector_runs[j];

                                if (b.band.empty())
                                    continue;

                                const std::size_t a_end = a.begin_j + a.band.size();
                                const std::size_t b_end = b.begin_j + b.band.size();

                                const std::size_t from = std::max(a.begin_j, b.begin_j);
                                const std::size_t to   = std::min(a_end, b_end);

                                if (from >= to)
                                    continue;   // no shared bin

                                bool overlaps = false;
                                for (std::size_t k = from; k < to && !overlaps; ++k)
                                {
                                    const auto& ea = a.band[k - a.begin_j];
                                    const auto& eb = b.band[k - b.begin_j];

                                    overlaps = ea.first - tol <= eb.second
                                               && eb.first - tol <= ea.second;
                                }

                                if (!overlaps)
                                    continue;

                                const std::size_t begin_j = std::min(a.begin_j, b.begin_j);
                                const std::size_t end_j   = std::max(a_end, b_end);

                                std::vector<std::pair<double, double>> band(
                                    end_j - begin_j,
                                    std::make_pair(std::numeric_limits<double>::max(), 0.0));

                                auto union_into = [&](const CoverageRun& from_run)
                                {
                                    for (std::size_t k = 0; k < from_run.band.size(); ++k)
                                    {
                                        auto& dst = band[from_run.begin_j - begin_j + k];
                                        dst.first  = std::min(dst.first,  from_run.band[k].first);
                                        dst.second = std::max(dst.second, from_run.band[k].second);
                                    }
                                };
                                union_into(a);
                                union_into(b);

                                a.band     = std::move(band);
                                a.begin_j  = begin_j;
                                a.reports += b.reports;

                                b.band.clear();
                                b.reports = 0;

                                merged_any = true;
                            }
                        }
                    }

                    sector_runs.erase(std::remove_if(sector_runs.begin(), sector_runs.end(),
                                                     [](const CoverageRun& r)
                                                     { return r.band.empty(); }),
                                      sector_runs.end());
                }

                // Drop slivers: what survived the segment and run checks can
                // still be a patch of a few hundred square meters, not worth a
                // sector of its own.
                if (settings.coverage_sector_min_area_m2_ > 0.0f)
                {
                    const double bin_share = bin_deg / 360.0;

                    sector_runs.erase(
                        std::remove_if(
                            sector_runs.begin(), sector_runs.end(),
                            [&](const CoverageRun& run)
                            {
                                double area = 0.0;
                                for (const auto& e : run.band)
                                    area += bin_share * M_PI
                                            * (e.second * e.second - e.first * e.first);

                                if (area >= settings.coverage_sector_min_area_m2_)
                                    return false;

                                loginf << sources[si].ds_id << ": coverage sector at "
                                       << Utils::String::doubleToStringPrecision(
                                              (run.cut + run.begin_j) * bin_deg, 2)
                                       << " deg dropped, area "
                                       << Utils::String::doubleToStringPrecision(area, 0)
                                       << " m2 below "
                                       << settings.coverage_sector_min_area_m2_ << " m2";
                                return true;
                            }),
                        sector_runs.end());
                }

                sources[si].coverage_runs = std::move(sector_runs);
                sources[si].coverage      = std::move(bins);
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
            if (!b.covered())
                continue;
            ++row.coverage_bins_covered;
            row.coverage_segments += b.segments.size();
            r_mins.push_back(b.rMin());
            r_maxs.push_back(b.rMax());
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

    // Estimated coverage as polygons: one per run of chained segments, traced
    // outward along the run's upper edge and back along its lower edge, so a
    // shadowed wedge, a dead gap in range and the unreported area at the antenna
    // all stay outside. Only built here, writeReport() writes the sectors on the
    // main thread.
    if (settings.create_coverage_sectors_ && settings.coverage_azimuth_bin_deg_ > 0.0f)
    {
        const double bin_deg = settings.coverage_azimuth_bin_deg_;

        // The boundary follows range circle segments: consecutive bins whose
        // edge differs by less than this share one arc at a constant range, the
        // step between two arcs is radial. A straight line between two azimuths
        // would cut the corner inward and leave own target reports outside.
        const double range_tolerance_m = settings.coverage_sector_simplify_m_;
        const double max_arc_step_deg  = 1.0;

        for (const auto& sc : sources)
        {
            if (!sc.has_pos || sc.coverage_runs.empty())
                continue;

            const auto* ds = ctx.dataSource(sc.ds_id);
            const std::string ds_name = ds ? ds->name() : std::to_string(sc.ds_id);

            static const std::string kSectorName = "Coverage";

            FixedTransformation sector_trafo(sc.lat, sc.lon);

            auto add_point = [&sector_trafo](double az_deg, double range_m,
                                             std::vector<std::pair<double, double>>& points)
            {
                const double rad = az_deg * M_PI / 180.0;

                bool   ok  = false;
                double lat = 0.0, lon = 0.0;
                std::tie(ok, lat, lon) = sector_trafo.wgsAddCartOffset(std::sin(rad) * range_m,
                                                                       std::cos(rad) * range_m);
                if (ok)
                    points.emplace_back(lat, lon);
            };

            // Sampled along the arc so the chord stays within centimeters of the
            // circle, azimuths may run backwards for the lower edge.
            auto add_arc = [&](double az_from_deg, double az_to_deg, double range_m,
                               std::vector<std::pair<double, double>>& points)
            {
                const double delta = az_to_deg - az_from_deg;
                const int    steps = std::max(
                    1, static_cast<int>(std::ceil(std::fabs(delta) / max_arc_step_deg)));

                for (int i = 0; i <= steps; ++i)
                    add_point(az_from_deg + delta * i / steps, range_m, points);
            };

            unsigned int run_cnt = 0;

            for (const auto& run : sc.coverage_runs)
            {
                const std::size_t len   = run.band.size();
                const double      az0   = (run.cut + run.begin_j) * bin_deg;

                // Arcs of one edge as (azimuth from, azimuth to, range). The group
                // keeps the widest edge of its bins, outward for the upper and
                // inward for the lower one, so grouping never cuts a report out.
                auto build_arcs = [&](bool upper)
                {
                    std::vector<std::tuple<double, double, double>> arcs;

                    auto edge_at = [&](std::size_t k)
                    { return upper ? run.band[k].second : run.band[k].first; };

                    std::size_t seg_begin = 0;
                    double      seg_start = edge_at(0);   // bounds the tolerance
                    double      seg_range = seg_start;    // the arc is drawn here

                    for (std::size_t k = 1; k <= len; ++k)
                    {
                        // measured against the start of the group, so a slowly
                        // rising edge can not collapse into one arc at its top:
                        // an arc never leaves the tolerance around its own start
                        if (k < len && std::fabs(edge_at(k) - seg_start) <= range_tolerance_m)
                        {
                            seg_range = upper ? std::max(seg_range, edge_at(k))
                                              : std::min(seg_range, edge_at(k));
                            continue;
                        }

                        arcs.emplace_back(az0 + seg_begin * bin_deg, az0 + k * bin_deg, seg_range);

                        if (k < len)
                        {
                            seg_begin = k;
                            seg_start = edge_at(k);
                            seg_range = seg_start;
                        }
                    }

                    return arcs;
                };

                ComputeResult::CoverageSector sector;
                sector.ds_id = sc.ds_id;

                for (const auto& arc : build_arcs(true))
                    add_arc(std::get<0>(arc), std::get<1>(arc), std::get<2>(arc), sector.points);

                const auto lower_arcs = build_arcs(false);
                for (auto it = lower_arcs.rbegin(); it != lower_arcs.rend(); ++it)
                    add_arc(std::get<1>(*it), std::get<0>(*it), std::get<2>(*it), sector.points);

                if (sector.points.size() < 3)
                    continue;

                ++run_cnt;
                sector.name = kSectorName;   // numbered below when there are several
                result_.coverage_sectors.push_back(std::move(sector));
            }

            // the layer already carries the source name, the sectors are numbered
            if (run_cnt > 1)
            {
                unsigned int idx = 0;
                for (auto& sector : result_.coverage_sectors)
                    if (sector.ds_id == sc.ds_id && sector.name == kSectorName)
                        sector.name = kSectorName + " " + std::to_string(++idx);
            }

            loginf << ds_name << ": " << run_cnt << " coverage sector(s) from "
                   << sc.coverage_runs.size() << " coverage run(s)";
        }
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

                            inside = src.coverage[bin].contains(rng);
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

    // The estimated coverage as sectors of a Data Context sector layer. Done
    // here and not in compute(): creating a sector rebuilds the sector layers
    // and the views reading them, which only the main thread may do.
    if (settings.create_coverage_sectors_ && !result_.coverage_sectors.empty())
    {
        auto& ctx = task_.compass().dbContextManager();

        // One layer per sensor, named after it, e.g. "RETS SMR Nord Coverage".
        auto layer_of = [&](unsigned int ds_id)
        {
            const auto* ds = ctx.dataSource(ds_id);
            return (ds ? ds->name() : std::to_string(ds_id)) + " "
                   + settings.coverage_sector_layer_suffix_;
        };

        // Clear what a previous run left in those layers, a new estimate can
        // consist of fewer sectors than the one it replaces.
        std::set<std::string> layer_names;
        for (const auto& sector : result_.coverage_sectors)
            layer_names.insert(layer_of(sector.ds_id));

        std::vector<std::shared_ptr<Sector>> stale;
        for (const auto& layer : ctx.sectorLayers())
        {
            if (!layer_names.count(layer->name()))
                continue;
            for (const auto& sector : layer->sectors())
                stale.push_back(sector);
        }
        if (!stale.empty())
            ctx.deleteSectors(stale);

        for (const auto& sector : result_.coverage_sectors)
        {
            // Yellow, the sector layer is read on top of the target report
            // colors of the source it was estimated from.
            ctx.createOrReplaceSector(sector.name, layer_of(sector.ds_id), false,
                                      QColor(Qt::yellow), sector.points);
        }

        loginf << "created " << result_.coverage_sectors.size() << " coverage sector(s) in "
               << layer_names.size() << " layer(s)";
    }

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
        {
            os << "radial band per " << settings.coverage_azimuth_bin_deg_
               << " deg azimuth bin, P1 to P99 of the report ranges\n"
               << "at least " << settings.coverage_min_reports_
               << " reports per bin, " << settings.coverage_smooth_bins_
               << " smoothing pass(es)";

            if (settings.coverage_min_run_reports_per_100m_ > 0.0f)
                os << "\nat least " << settings.coverage_min_run_reports_per_100m_
                   << " reports per 100 m of band per covered run";
        }
        else
            os << "off (every slot inside the maximum range is expected)";
        recap.addRow({"Estimated coverage", os.str()});
    }
    {
        std::ostringstream os;
        if (settings.create_coverage_sectors_ && settings.coverage_azimuth_bin_deg_ > 0.0f)
            os << "one layer per source, named '<data source> "
               << settings.coverage_sector_layer_suffix_ << "'";
        else
            os << "off";
        recap.addRow({"Coverage sectors", os.str()});
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
        auto& ct = section.addTable("Estimated Coverage", 7,
                                    {"Data Source", "Reports", "Azimuth Bins",
                                     "Bins with Coverage", "Segments",
                                     "Median Inner Range (m)", "Median Outer Range (m)"},
                                    false);
        for (const auto& r : result_.sources)
        {
            if (!r.coverage_valid)
            {
                ct.addRow({r.label, std::to_string(r.coverage_reports), "-", "-", "-", "-", "-"});
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
                       std::to_string(r.coverage_segments),
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
