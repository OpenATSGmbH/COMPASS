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


#include "smr_coverage_helpers.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace analysis
{
namespace SMRCoverageHelpers
{

/**
 */
bool CoverageBin::contains(double range_m) const
{
    for (const auto& seg : segments)
        if (range_m >= seg.r_min && range_m <= seg.r_max)
            return true;
    return false;
}

/**
 */
std::uint64_t CoverageBin::coveredCount() const
{
    std::uint64_t sum = 0;
    for (const auto& seg : segments)
        sum += seg.count;
    return sum;
}

/**
 */
std::optional<Polar> polarOf(double dx_east_m, double dy_north_m)
{
    const double range_m = std::sqrt(dx_east_m * dx_east_m + dy_north_m * dy_north_m);

    if (range_m <= 0.0)
        return std::nullopt;

    double az = std::atan2(dx_east_m, dy_north_m) * (180.0 / M_PI);
    if (az < 0.0)
        az += 360.0;

    return Polar{ range_m, az };
}

/**
 */
std::size_t numAzimuthBins(double bin_deg)
{
    return std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(360.0 / bin_deg)));
}

/**
 */
std::size_t azimuthBin(double azimuth_deg, double bin_deg, std::size_t num_bins)
{
    std::size_t bin = static_cast<std::size_t>(azimuth_deg / bin_deg);
    if (bin >= num_bins)
        bin = num_bins - 1;
    return bin;
}

/**
 */
std::vector<CoverageSegment> splitSegments(std::vector<double> ranges_m, double gap_m,
                                           std::size_t min_segment_reports)
{
    // No percentile trimming: a lone far return does not survive the report minimum of its own
    // segment, so the segment can keep the full extent of what was seen.
    std::sort(ranges_m.begin(), ranges_m.end());

    std::vector<CoverageSegment> segments;

    std::size_t seg_begin = 0;
    for (std::size_t i = 1; i <= ranges_m.size(); ++i)
    {
        if (i < ranges_m.size() && (gap_m <= 0.0 || ranges_m[i] - ranges_m[i - 1] <= gap_m))
            continue;

        const std::size_t n = i - seg_begin;
        if (n >= min_segment_reports)
        {
            CoverageSegment seg;
            seg.count = n;
            seg.r_min = ranges_m[seg_begin];
            seg.r_max = ranges_m[i - 1];

            if (seg.r_max > seg.r_min)
                segments.push_back(seg);
        }

        seg_begin = i;
    }

    return segments;
}

/**
 */
void closeSingleGaps(std::vector<CoverageBin>& bins, unsigned int passes)
{
    const std::size_t num_bins = bins.size();

    for (unsigned int pass = 0; pass < passes; ++pass)
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
                      [](const CoverageSegment& a, const CoverageSegment& c) { return a.r_min < c.r_min; });

            // union of the overlapping ones, the closed bin must not claim more segments than its
            // neighbors show
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
}

namespace
{

/// union of two runs bin by bin, into a
void unionRuns(CoverageRun& a, CoverageRun& b)
{
    const std::size_t begin_j = std::min(a.begin_j, b.begin_j);
    const std::size_t end_j   = std::max(a.begin_j + a.band.size(), b.begin_j + b.band.size());

    std::vector<std::pair<double, double>> band(
        end_j - begin_j, std::make_pair(std::numeric_limits<double>::max(), 0.0));

    auto union_into = [&](const CoverageRun& from)
    {
        for (std::size_t i = 0; i < from.band.size(); ++i)
        {
            auto& dst = band[from.begin_j - begin_j + i];
            dst.first  = std::min(dst.first,  from.band[i].first);
            dst.second = std::max(dst.second, from.band[i].second);
        }
    };
    union_into(a);
    union_into(b);

    a.band     = std::move(band);
    a.begin_j  = begin_j;
    a.reports += b.reports;

    b.band.clear();
    b.reports = 0;
}

void removeEmptyRuns(std::vector<CoverageRun>& runs)
{
    runs.erase(std::remove_if(runs.begin(), runs.end(), [](const CoverageRun& r) { return r.band.empty(); }),
               runs.end());
}

} // namespace

/**
 */
std::vector<CoverageRun> chainRuns(const std::vector<CoverageBin>& bins, double link_m, std::size_t bridge_bins)
{
    // the walk starts at an uncovered bin so a run across the 360 degree seam stays whole, and
    // the walk index keeps the azimuths increasing
    std::size_t cut = 0;
    while (cut < bins.size() && bins[cut].covered())
        ++cut;
    if (cut == bins.size())   // fully covered, the seam falls somewhere
        cut = 0;

    std::vector<CoverageRun> runs;
    std::vector<std::size_t> open;   // active in the last bin

    for (std::size_t j = 0; j < bins.size(); ++j)
    {
        const CoverageBin& bin = bins[(cut + j) % bins.size()];

        std::vector<std::size_t> next_open;

        for (const auto& seg : bin.segments)
        {
            std::vector<std::size_t> matches;
            for (std::size_t r : open)
            {
                // a run merged into another one by an earlier segment of this bin is empty
                if (runs[r].band.empty())
                    continue;

                const auto& last = runs[r].band.back();
                if (seg.r_min - link_m <= last.second && last.first - link_m <= seg.r_max)
                    matches.push_back(r);
            }

            // a run whose last entry lies further back takes the bridged bins, filled with the
            // union of both ends
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

            // the first match takes the segment, the others merge into it
            CoverageRun& target = runs[matches[0]];

            for (std::size_t m = 1; m < matches.size(); ++m)
            {
                unionRuns(target, runs[matches[m]]);

                next_open.erase(std::remove(next_open.begin(), next_open.end(), matches[m]),
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

            if (std::find(next_open.begin(), next_open.end(), matches[0]) == next_open.end())
                next_open.push_back(matches[0]);
        }

        // runs without a segment in this bin stay open while the gap is short enough to bridge
        for (std::size_t r : open)
            if (std::find(next_open.begin(), next_open.end(), r) == next_open.end()
                && !runs[r].band.empty()
                && j + 1 - (runs[r].begin_j + runs[r].band.size()) <= bridge_bins)
                next_open.push_back(r);

        open.swap(next_open);
    }

    removeEmptyRuns(runs);

    return runs;
}

/**
 */
RunExtent runExtent(const CoverageRun& run)
{
    RunExtent extent;
    extent.r_min = std::numeric_limits<double>::max();

    for (const auto& e : run.band)
    {
        extent.r_min = std::min(extent.r_min, e.first);
        extent.r_max = std::max(extent.r_max, e.second);
    }

    // no floor worth speaking of: with segments a short band is dense by nature, the check is
    // about long and thin runs
    const double band_m = std::max(1.0, extent.r_max - extent.r_min);
    extent.density = run.reports * 100.0 / band_m;

    return extent;
}

/**
 */
void removeRunSegments(const CoverageRun& run, std::vector<CoverageBin>& bins)
{
    for (std::size_t i = 0; i < run.band.size(); ++i)
    {
        const std::size_t b = (run.cut + run.begin_j + i) % bins.size();
        auto& segs = bins[b].segments;

        segs.erase(std::remove_if(segs.begin(), segs.end(),
                                  [&](const CoverageSegment& seg)
                                  {
                                      return seg.r_min <= run.band[i].second && run.band[i].first <= seg.r_max;
                                  }),
                   segs.end());
    }
}

/**
 */
void widenRuns(std::vector<CoverageRun>& runs, int smooth_bins)
{
    if (smooth_bins <= 0)
        return;

    for (auto& run : runs)
    {
        const int n = static_cast<int>(run.band.size());
        std::vector<std::pair<double, double>> widened(run.band.size());

        for (int i = 0; i < n; ++i)
        {
            double lo = run.band[i].first;
            double hi = run.band[i].second;

            for (int d = -smooth_bins; d <= smooth_bins; ++d)
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

/**
 */
void mergeOverlappingRuns(std::vector<CoverageRun>& runs, double tol_m)
{
    // repeated until nothing changes, a merge can create a new overlap
    bool merged_any = true;
    while (merged_any)
    {
        merged_any = false;

        for (std::size_t i = 0; i < runs.size() && !merged_any; ++i)
        {
            if (runs[i].band.empty())
                continue;

            for (std::size_t j = i + 1; j < runs.size() && !merged_any; ++j)
            {
                CoverageRun& a = runs[i];
                CoverageRun& b = runs[j];

                if (b.band.empty())
                    continue;

                const std::size_t from = std::max(a.begin_j, b.begin_j);
                const std::size_t to   = std::min(a.begin_j + a.band.size(), b.begin_j + b.band.size());

                if (from >= to)
                    continue;   // no shared bin

                bool overlaps = false;
                for (std::size_t k = from; k < to && !overlaps; ++k)
                {
                    const auto& ea = a.band[k - a.begin_j];
                    const auto& eb = b.band[k - b.begin_j];

                    overlaps = ea.first - tol_m <= eb.second && eb.first - tol_m <= ea.second;
                }

                if (!overlaps)
                    continue;

                unionRuns(a, b);
                merged_any = true;
            }
        }
    }

    removeEmptyRuns(runs);
}

/**
 */
double runArea(const CoverageRun& run, double bin_deg)
{
    const double bin_share = bin_deg / 360.0;

    double area = 0.0;
    for (const auto& e : run.band)
        area += bin_share * M_PI * (e.second * e.second - e.first * e.first);

    return area;
}

/**
 */
CoverageEstimate estimateCoverage(std::vector<std::vector<double>> ranges_per_bin, const CoverageParams& params)
{
    CoverageEstimate est;

    const std::size_t num_bins = ranges_per_bin.size();
    est.bins.resize(num_bins);

    for (std::size_t b = 0; b < num_bins; ++b)
    {
        est.bins[b].count = ranges_per_bin[b].size();

        if (ranges_per_bin[b].size() < params.min_reports)
            continue;

        est.bins[b].segments = splitSegments(std::move(ranges_per_bin[b]), params.gap_m,
                                             params.min_segment_reports);
    }

    // close single empty bins between two covered ones, so a direction nothing happened to cross
    // is not cut out
    closeSingleGaps(est.bins, params.smooth_bins);

    // cut thin runs: reports spread thinly along a long band are a single pass through that
    // direction, not coverage; their segments are dropped, so the gate and the sectors agree
    for (const auto& run : chainRuns(est.bins, 0.0, 0))
    {
        const RunExtent extent = runExtent(run);

        RunCheck check;
        check.azimuth_from_deg = (run.cut + run.begin_j) * params.bin_deg;
        check.azimuth_to_deg   = (run.cut + run.begin_j + run.band.size()) * params.bin_deg;
        check.num_bins         = run.band.size();
        check.reports          = run.reports;
        check.band_m           = extent.r_max - extent.r_min;
        check.density          = extent.density;
        check.cut              = params.min_run_reports_per_100m > 0.0
                                 && extent.density < params.min_run_reports_per_100m;

        est.run_checks.push_back(check);

        if (check.cut)
            removeRunSegments(run, est.bins);
    }

    // the sector runs are glued and widened by a rolling maximum outward and a rolling minimum
    // inward; both only grow the area, so no report falls out of its sector, and runs that would
    // intersect become one sector; the bins keep the exact segments for the gate
    est.sector_runs = chainRuns(est.bins, params.sector_link_m, params.sector_bridge_bins);

    widenRuns(est.sector_runs, params.sector_smooth_bins);
    mergeOverlappingRuns(est.sector_runs, params.sector_simplify_m);

    // drop slivers: what survived the segment and run checks can still be a patch of a few
    // hundred square meters, not worth a sector of its own
    if (params.sector_min_area_m2 > 0.0)
    {
        est.sector_runs.erase(
            std::remove_if(est.sector_runs.begin(), est.sector_runs.end(),
                           [&](const CoverageRun& run)
                           {
                               const double area = runArea(run, params.bin_deg);

                               if (area >= params.sector_min_area_m2)
                                   return false;

                               est.slivers.push_back({ (run.cut + run.begin_j) * params.bin_deg, area });
                               return true;
                           }),
            est.sector_runs.end());
    }

    return est;
}

} // namespace SMRCoverageHelpers
} // namespace analysis
