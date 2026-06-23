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

#include "adsbcoverageinspector.h"
#include "analyzedatasourcetask.h"
#include "analysisdataset.h"
#include "targetreport3dgrid.h"

#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "logger.h"
#include "number.h"
#include "section.h"
#include "sectioncontenttable.h"
#include "sectioncontenttext.h"
#include "stringconv.h"
#include "targetposition.h"
#include "dbcontent/target/targetreportchain.h"
#include "dbcontent/target/targetbase.h"

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

#include <boost/optional.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <utility>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using dbContent::TargetReport::Chain;
using Utils::Number::mean;
using Utils::Number::percentile;

ADSBCoverageInspectorSettings::ADSBCoverageInspectorSettings(nlohmann::json& config_json,
                                                             Configurable* parent)
    : InspectorSettingsBase(config_json, parent)
{
    registerParameter("update_interval_s", &update_interval_s_, update_interval_s_);

    registerParameter("use_miss_tolerance", &use_miss_tolerance_, use_miss_tolerance_);
    registerParameter("miss_tolerance_s",   &miss_tolerance_s_,   miss_tolerance_s_);

    registerParameter("ref_max_time_diff_s", &ref_max_time_diff_s_, ref_max_time_diff_s_);

    registerParameter("pd_acceptable_above",   &pd_acceptable_above_,   pd_acceptable_above_);
    registerParameter("pd_unacceptable_below", &pd_unacceptable_below_, pd_unacceptable_below_);

    registerParameter("ui_hist_num_bins", &ui_hist_num_bins_, ui_hist_num_bins_);
    registerParameter("ui_hist_max_s",    &ui_hist_max_s_,    ui_hist_max_s_);
}

ADSBCoverageInspector::ADSBCoverageInspector(AnalyzeDataSourceTask& task,
                                             ADSBCoverageInspectorSettings& settings)
    : DataSourceInspectorBase(task, settings)
{
}

ADSBCoverageInspector::~ADSBCoverageInspector() = default;

std::set<std::string> ADSBCoverageInspector::testDBContentNames() const
{
    return {"CAT021"};
}

bool ADSBCoverageInspector::prerequisitesMet(std::string& reason_out) const
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
using Settings = ADSBCoverageInspectorSettings;
using EvaluationRequirement::PDHelpers::MissTestParams;
using EvaluationRequirement::PDHelpers::RefPeriod;

MissTestParams missParamsFromSettings(const Settings& s)
{
    MissTestParams p;
    p.update_interval_s  = s.update_interval_s_;
    p.use_miss_tolerance = s.use_miss_tolerance_;
    p.miss_tolerance_s   = s.miss_tolerance_s_;
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

// Inter-report intervals above this are treated as coverage gaps (target not
// continuously tracked) and excluded from the measured-cadence statistics.
constexpr double kMaxUpdateIntervalS = 300.0;

// Cap on the number of target-type groups rendered as figures (the rest are
// dropped, with a note in the report). MOPS versions are at most three.
constexpr std::size_t kMaxTypeGroups = 12;

std::string formatNumber(double v, int prec = 4)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(prec) << v;
    return os.str();
}

struct CellAttribution
{
    bool   valid  = false;
    double lat    = 0.0;
    double lon    = 0.0;
    double alt_ft = 0.0;
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
    out.valid  = true;
    out.lat    = pos->latitude_;
    out.lon    = pos->longitude_;
    out.alt_ft = pos->has_altitude_ ? static_cast<double>(pos->altitude_) : 0.0;
    return out;
}

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

// Transponder identity for a target: the most common acad across the target's
// CAT021 test reports, the first non-empty callsign, and the dominant MOPS
// version (used to route the target's slots into the per-MOPS summary grid). A
// CAT021 target almost always carries a single acad; the vote guards against
// the rare mixed-association case. The emitter category is taken from the
// reconstructed target, not voted here.
struct TransponderId
{
    bool         has_acad = false;
    unsigned int acad     = 0;
    std::string  callsign;
    bool         has_mops = false;
    unsigned int mops     = 0;
};

template <typename K>
K dominantVote(const std::map<K, unsigned int>& votes, bool& has, K fallback)
{
    unsigned int best = 0;
    K out = fallback;
    has = false;
    for (const auto& kv : votes)
    {
        if (kv.second > best)
        {
            best = kv.second;
            out  = kv.first;
            has  = true;
        }
    }
    return out;
}

TransponderId transponderIdOf(unsigned int utn, AnalysisDataset& dataset)
{
    TransponderId out;
    std::map<unsigned int, unsigned int> acad_votes;
    std::map<unsigned int, unsigned int> mops_votes;

    for (const auto& dbc : dataset.testDbContentsPresent())
    {
        if (!dataset.hasTestChain(utn, dbc))
            continue;
        auto& chain = dataset.testChain(utn, dbc);
        for (const auto& kv : chain.timestampIndexes())
        {
            Chain::DataID id(kv.first);
            auto a = chain.acad(id);
            if (a.has_value())
                ++acad_votes[*a];
            auto m = chain.mopsVersion(id);
            if (m.has_value())
                ++mops_votes[*m];

            if (out.callsign.empty())
            {
                auto c = chain.acid(id);
                if (c.has_value())
                {
                    std::string s = Utils::String::trim(*c);
                    if (!s.empty())
                        out.callsign = s;
                }
            }
        }
    }

    out.acad = dominantVote<unsigned int>(acad_votes, out.has_acad, 0);
    out.mops = dominantVote<unsigned int>(mops_votes, out.has_mops, 0);
    return out;
}

// Time-difference walk per target, counting the #EUI / #MUI attributed to this
// target while also attributing them to the shared grid cells. Mirrors the MLAT
// coverage walk; the returned pair feeds the per-transponder breakdown.
std::pair<std::uint64_t, std::uint64_t>
walkTargetTimeDifferenceCounted(unsigned int utn,
                                const std::vector<RefPeriod>& periods,
                                const std::vector<ptime>& tst_ts_sorted,
                                AnalysisDataset& dataset,
                                const std::vector<TargetReport3DGrid*>& grids,
                                const Settings& settings)
{
    const auto miss_params = missParamsFromSettings(settings);
    const time_duration d_max = boost::posix_time::seconds(60);

    std::uint64_t eui = 0;
    std::uint64_t mui = 0;

    for (const auto& period : periods)
    {
        const double period_s = partialSeconds(period.end - period.begin);
        if (period_s <= 0.0 || settings.update_interval_s_ <= 0.0f)
            continue;

        const unsigned int n_slots = static_cast<unsigned int>(
            std::floor(period_s / settings.update_interval_s_));

        for (unsigned int k = 0; k < n_slots; ++k)
        {
            ptime t_slot = addSeconds(period.begin, k * settings.update_interval_s_);
            auto ca = refCellAt(dataset, utn, t_slot, d_max);
            if (ca.valid)
            {
                for (auto* g : grids)
                    g->addEUI(ca.lat, ca.lon, ca.alt_ft);
                ++eui;
            }
        }

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
            const float gap_s = static_cast<float>(partialSeconds(gap_end - gap_start));

            if (!EvaluationRequirement::PDHelpers::isMiss(gap_s, miss_params))
                continue;

            const unsigned int n_misses =
                EvaluationRequirement::PDHelpers::numMisses(gap_s, miss_params);

            for (unsigned int m = 0; m < n_misses; ++m)
            {
                ptime t_miss = addSeconds(gap_start, (m + 1) * settings.update_interval_s_);
                if (t_miss >= gap_end)
                    break;
                auto ca = refCellAt(dataset, utn, t_miss, d_max);
                if (ca.valid)
                {
                    for (auto* g : grids)
                        g->addMUI(ca.lat, ca.lon, ca.alt_ft);
                    ++mui;
                }
            }
        }
    }

    return {eui, mui};
}
}  // anonymous namespace

void ADSBCoverageInspector::compute(AnalysisDataset* dataset)
{
    result_ = ComputeResult{};
    grid_.reset();

    if (!dataset)
        return;

    auto& settings = static_cast<ADSBCoverageInspectorSettings&>(settings_);

    if (!dataset->hasPositionExtent())
    {
        logwrn << "ADSBCoverageInspector: dataset has no reference positions, skipping";
        return;
    }

    double ref_lat = dataset->centerLatitudeDeg();

    auto sizing = task_.clampedCellSizes(*dataset);
    if (sizing.horizontal_clamped || sizing.vertical_clamped)
    {
        loginf << "ADSBCoverageInspector: cell sizes clamped to fit max "
               << task_.maxCellsPerAxis() << " cells/axis";
    }

    grid_.reset(new TargetReport3DGrid(sizing.cell_size_m, sizing.cell_size_ft, ref_lat));
    auto& grid = *grid_;

    auto makeGrid = [&]() {
        return std::unique_ptr<TargetReport3DGrid>(
            new TargetReport3DGrid(sizing.cell_size_m, sizing.cell_size_ft, ref_lat));
    };

    // Per-MOPS-version and per-target-type summary grids + PD counters. The
    // target type is the reconstructed target category, not the raw report ECAT.
    struct GroupCounts { std::uint64_t eui = 0; std::uint64_t mui = 0; };
    std::map<unsigned int, std::unique_ptr<TargetReport3DGrid>> mops_grids;
    std::map<unsigned int, GroupCounts>                         mops_counts;
    std::map<int, std::unique_ptr<TargetReport3DGrid>>          type_grids;  // Category int
    std::map<int, GroupCounts>                                  type_counts;
    std::map<int, std::string>                                  type_label;

    auto& dbcont_man = task_.compass().dbContentManager();

    const time_duration ref_max_gap = durationFromSeconds(settings.ref_max_time_diff_s_);
    const time_duration min_period_duration = boost::posix_time::seconds(1);

    unsigned int targets_walked = 0;
    unsigned int targets_no_ref = 0;
    unsigned int targets_no_tst = 0;

    // Per-transponder accumulation, keyed by acad (and an "unknown" bucket).
    std::map<unsigned int, TransponderRow> by_acad;
    TransponderRow unknown_row;
    unknown_row.has_acad = false;

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

        auto tid = transponderIdOf(utn, *dataset);

        // Resolved target type (reconstructed target category), used to route
        // the target's slots into the per-target-type summary grid.
        TargetBase::Category type_cat = TargetBase::Category::Unknown;
        if (dbcont_man.existsTarget(utn))
            type_cat = dbcont_man.emitterCategory(utn);
        int type_key = static_cast<int>(type_cat);

        // Route this target's slots into the aggregate grid plus its MOPS and
        // target-type summary grids (one dominant value per transponder).
        std::vector<TargetReport3DGrid*> grids{&grid};
        if (tid.has_mops)
        {
            auto& g = mops_grids[tid.mops];
            if (!g)
                g = makeGrid();
            grids.push_back(g.get());
        }
        auto& tg = type_grids[type_key];
        if (!tg)
            tg = makeGrid();
        grids.push_back(tg.get());
        type_label[type_key] = TargetBase::toString(type_cat);

        auto counts = walkTargetTimeDifferenceCounted(
            utn, periods, tst_ts_sorted, *dataset, grids, settings);

        if (tid.has_mops)
        {
            mops_counts[tid.mops].eui += counts.first;
            mops_counts[tid.mops].mui += counts.second;
        }
        type_counts[type_key].eui += counts.first;
        type_counts[type_key].mui += counts.second;

        TransponderRow* row = nullptr;
        if (tid.has_acad)
        {
            row = &by_acad[tid.acad];
            row->acad     = tid.acad;
            row->has_acad = true;
        }
        else
        {
            row = &unknown_row;
        }
        if (row->callsign.empty() && !tid.callsign.empty())
            row->callsign = tid.callsign;
        row->eui += counts.first;
        row->mui += counts.second;
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

    // Materialize the per-transponder rows, sorted by ascending PD so the
    // weakest transponders sort to the top.
    for (auto& kv : by_acad)
        result_.transponders.push_back(kv.second);
    if (unknown_row.eui > 0 || unknown_row.mui > 0)
        result_.transponders.push_back(unknown_row);

    auto pdOf = [](const TransponderRow& r) -> double {
        if (r.eui == 0) return 1.0;
        return (static_cast<double>(r.eui) - static_cast<double>(r.mui))
               / static_cast<double>(r.eui);
    };
    std::sort(result_.transponders.begin(), result_.transponders.end(),
              [&](const TransponderRow& a, const TransponderRow& b) {
                  return pdOf(a) < pdOf(b);
              });

    // Materialize summary groups (move grids out, attach aggregate counters).
    for (auto& kv : mops_grids)
    {
        Group grp;
        grp.label = "MOPS v" + std::to_string(kv.first);
        grp.eui   = mops_counts[kv.first].eui;
        grp.mui   = mops_counts[kv.first].mui;
        grp.grid  = std::move(kv.second);
        mops_groups_.push_back(std::move(grp));
    }

    // Target-type groups: keep the busiest kMaxTypeGroups by #EUI.
    std::vector<int> type_keys;
    type_keys.reserve(type_grids.size());
    for (auto& kv : type_grids)
        type_keys.push_back(kv.first);
    std::sort(type_keys.begin(), type_keys.end(), [&](int a, int b) {
        return type_counts[a].eui > type_counts[b].eui;
    });
    for (std::size_t i = 0; i < type_keys.size(); ++i)
    {
        int catk = type_keys[i];
        if (i >= kMaxTypeGroups)
        {
            ++result_.type_groups_dropped;
            continue;
        }
        Group grp;
        grp.label = type_label[catk];
        grp.eui   = type_counts[catk].eui;
        grp.mui   = type_counts[catk].mui;
        grp.grid  = std::move(type_grids[catk]);
        type_groups_.push_back(std::move(grp));
    }
}

namespace
{
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
                    const ADSBCoverageInspectorSettings& settings)
{
    if (!proj.valid || !proj.layer)
        return;

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

    if (proj.x_axis_label == "Longitude (deg)" && proj.y_axis_label == "Latitude (deg)")
    {
        auto rendered = Grid2DLayerRenderer::render(*proj.layer, rs);

        auto raw = rs.color_map.colorLegend(
            /*add_sel_color=*/false,
            /*add_null_color=*/false,
            [](double v) { return Utils::String::doubleToStringPrecision(v, 2); });
        ColorLegend legend;
        const auto& entries = raw.entries();
        for (auto it = entries.rbegin(); it != entries.rend(); ++it)
            legend.addEntry(it->first, it->second);

        anno->addFeature(new ViewPointGenFeatureGeoImage(rendered.first, rendered.second, legend));

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

void ADSBCoverageInspector::writeReport(ResultReport::Section& root)
{
    auto& settings = static_cast<ADSBCoverageInspectorSettings&>(settings_);
    auto& section  = root.addSubSection(name());

    {
        auto& intro = section.addText("About");
        intro.addText(
            "Probability of Detection (PD) of the selected ADS-B sources per "
            "cell of a three-dimensional grid in latitude, longitude and "
            "barometric flight level, using the Reference Trajectory as "
            "ground truth, with an additional breakdown per transponder. PD "
            "is the fraction of expected updates the system actually "
            "delivered: each target's reference track contributes expected "
            "updates at the nominal Update Interval, and each gap in the "
            "ADS-B report stream long enough to swallow a cadence slot "
            "contributes a missed update at the location where the report "
            "was expected.\n"
            "ADS-B has no Remote Units and no system-status cycle messages, "
            "so the cadence is always the configured nominal Update Interval "
            "together with a miss tolerance.\n"
            "Three projections of the per-cell PD are rendered: a top-down "
            "horizontal map and two vertical profiles (altitude over "
            "longitude, altitude over latitude). The summary tabulates the "
            "overall PD and the per-cell distribution. The "
            "per-transponder table lists each aircraft address with its "
            "expected and missed updates and resulting PD, so individual "
            "transponders with coverage gaps (intermittent or low-power "
            "transmitters) can be spotted where an aggregate map averages "
            "them away.\n"
            "Use the report to locate coverage holes against the published "
            "service volume and to identify individual aircraft whose ADS-B "
            "performance is degraded.");
    }

    auto& recap = section.addTable("Settings", 2, {"Setting", "Value"}, false);
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
    summary.addRow({"Transponders",          std::to_string(result_.transponders.size())});
    summary.addRow({"Total expected slots (#EUI)", std::to_string(result_.total_eui)});
    summary.addRow({"Total missed slots (#MUI)",   std::to_string(result_.total_mui)});
    summary.addRow({"Overall PD",                  formatNumber(result_.overall_pd)});
    summary.addRow({"Cells with EUI",              std::to_string(result_.cells_with_eui)});
    summary.addRow({"Median per-cell PD",          formatNumber(result_.median_per_cell_pd)});
    summary.addRow({"P5 per-cell PD",              formatNumber(result_.p5_per_cell_pd)});
    if (result_.has_worst_cell)
        summary.addRow({"Worst cell PD (>=5 EUI)", formatNumber(result_.worst_cell_pd)});

    // Measured update cadence: the actual per-transponder inter-report interval,
    // independent of the configured nominal UI. Use it to pick an Update
    // Interval that matches the feed (set UI at or above the median, near the
    // P90, to avoid counting the feed's own cadence as missed updates).
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

    // Per-transponder table.
    auto& tt = section.addTable("Per-Transponder PD", 5,
                                {"Aircraft Address", "Callsign", "#EUI", "#MUI", "PD"}, false);
    for (const auto& r : result_.transponders)
    {
        std::string acad_str = r.has_acad
            ? Utils::String::hexStringFromInt(static_cast<int>(r.acad), 6, '0')
            : std::string("(unknown)");
        double pd = r.eui == 0 ? 1.0
                              : (static_cast<double>(r.eui) - static_cast<double>(r.mui))
                                    / static_cast<double>(r.eui);
        tt.addRow({acad_str,
                   r.callsign.empty() ? std::string("-") : r.callsign,
                   std::to_string(r.eui),
                   std::to_string(r.mui),
                   formatNumber(pd)});
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

    // Summary breakdowns by MOPS version and by target type: per group, a PD
    // counter row plus a horizontal PD map. These replace infeasible
    // per-transponder figures (one per aircraft address).
    auto pdRatio = [](std::uint64_t eui, std::uint64_t mui) -> double {
        if (eui == 0) return 1.0;
        return (static_cast<double>(eui) - static_cast<double>(mui))
               / static_cast<double>(eui);
    };

    auto writeGroupBreakdown = [&](const std::string& title,
                                   const std::string& key_header,
                                   std::vector<Group>& groups,
                                   std::size_t dropped) {
        if (groups.empty())
            return;

        auto& gsec = section.addSubSection(title);

        auto& gtbl = gsec.addTable("Overview", 4,
                                   {key_header, "#EUI", "#MUI", "PD"}, false);
        for (const auto& g : groups)
            gtbl.addRow({g.label, std::to_string(g.eui), std::to_string(g.mui),
                         formatNumber(pdRatio(g.eui, g.mui))});
        if (dropped > 0)
            gtbl.addRow({"(" + std::to_string(dropped) + " more, not shown)",
                         "-", "-", "-"});

        for (auto& g : groups)
        {
            if (!g.grid)
                continue;
            auto& gsub = gsec.addSubSection(g.label);
            auto proj = g.grid->projectionLayer(
                TargetReport3DGrid::Projection::Horizontal,
                pdScalar, pdSampleCount, "pd");
            attachPDFigure(gsub, g.label + " - PD - Horizontal", proj, settings);
        }
    };

    writeGroupBreakdown("PD by MOPS Version", "MOPS Version", mops_groups_, 0);
    writeGroupBreakdown("PD by Target Type", "Target Type",
                        type_groups_, result_.type_groups_dropped);

    loginf << "ADSBCoverageInspector: walked " << result_.targets_walked << " target(s), "
           << result_.transponders.size() << " transponder(s), "
           << result_.total_eui << " EUI, " << result_.total_mui
           << " MUI, overall PD " << result_.overall_pd;
}
