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

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace analysis
{

/**
 * Estimated coverage of a surface movement radar from the ranges of its own target reports per
 * azimuth bin: covered range segments per bin, runs of segments across bins, and the runs drawn
 * as coverage sectors. Free of the dataset, so it can be unit tested on plain values.
 */
namespace SMRCoverageHelpers
{

/// One covered range segment of an azimuth bin. A direction can hold several: a taxiway close in
/// and a runway far out, with a dead gap between them the sensor never reports in.
struct CoverageSegment
{
    std::uint64_t count = 0;
    double        r_min = 0.0;
    double        r_max = 0.0;

    bool overlaps(const CoverageSegment& other) const
    { return r_min <= other.r_max && other.r_min <= r_max; }
};

/// One azimuth bin of the estimated coverage.
struct CoverageBin
{
    std::uint64_t count = 0;   // all reports of the bin, covered or not

    // Ordered by range, disjoint. Empty means the bin is not covered.
    std::vector<CoverageSegment> segments;

    bool covered() const { return !segments.empty(); }

    double rMin() const { return segments.front().r_min; }
    double rMax() const { return segments.back().r_max; }

    bool contains(double range_m) const;
    std::uint64_t coveredCount() const;
};

/// One chain of overlapping segments across adjacent azimuth bins: what becomes one sector, and
/// what the density check judges. `band` holds one range interval per bin, starting at walk index
/// `begin_j` of the walk that started at bin `cut`, so the azimuth of entry i is
/// (cut + begin_j + i) times the bin width and increases across the 360 degree seam.
struct CoverageRun
{
    std::size_t   cut     = 0;
    std::size_t   begin_j = 0;
    std::uint64_t reports = 0;

    std::vector<std::pair<double, double>> band;
};

struct Polar
{
    double range_m     = 0.0;
    double azimuth_deg = 0.0;   // from north, clockwise, [0, 360)
};

/// range and azimuth of an offset east / north of the antenna, none at range 0
std::optional<Polar> polarOf(double dx_east_m, double dy_north_m);

/// number of azimuth bins of the given width over 360 degrees, at least 1
std::size_t numAzimuthBins(double bin_deg);

/// azimuth bin of an azimuth, the last bin for azimuths beyond it
std::size_t azimuthBin(double azimuth_deg, double bin_deg, std::size_t num_bins);

/// splits the sorted ranges where neighbors are more than gap_m apart (gap_m <= 0: no split), a
/// cluster becomes a segment from its first to its last range if it has min_segment_reports and a
/// length > 0
std::vector<CoverageSegment> splitSegments(std::vector<double> ranges_m, double gap_m,
                                           std::size_t min_segment_reports);

/// closes uncovered bins between two covered ones, once per pass; the closed bin takes the union
/// of the overlapping segments of both neighbors
void closeSingleGaps(std::vector<CoverageBin>& bins, unsigned int passes);

/// chains the segments into runs: a segment continues a run of the previous bin when the ranges
/// overlap or lie within link_m; a run without a segment stays open for bridge_bins bins
std::vector<CoverageRun> chainRuns(const std::vector<CoverageBin>& bins, double link_m,
                                   std::size_t bridge_bins);

struct RunExtent
{
    double r_min      = 0.0;
    double r_max      = 0.0;
    double density    = 0.0;   // reports per 100 m of the run's range extent (at least 1 m)
};

/// range extent and report density of a run
RunExtent runExtent(const CoverageRun& run);

/// removes the segments of a run from the bins
void removeRunSegments(const CoverageRun& run, std::vector<CoverageBin>& bins);

/// widens each band entry to the minimum / maximum of its neighbors within smooth_bins
void widenRuns(std::vector<CoverageRun>& runs, int smooth_bins);

/// merges runs that share a bin where their bands overlap or come within tol_m, until none do
void mergeOverlappingRuns(std::vector<CoverageRun>& runs, double tol_m);

/// area of a run's band in square meters
double runArea(const CoverageRun& run, double bin_deg);

struct CoverageParams
{
    double       bin_deg                   = 0.25;
    std::size_t  min_reports               = 20;   // a bin with fewer reports is not covered
    double       gap_m                     = 150.0;
    std::size_t  min_segment_reports       = 5;
    unsigned int smooth_bins               = 1;
    double       min_run_reports_per_100m  = 25.0; // <= 0: no density cut
    double       sector_link_m             = 200.0;
    std::size_t  sector_bridge_bins        = 8;
    int          sector_smooth_bins        = 2;
    double       sector_simplify_m         = 10.0;
    double       sector_min_area_m2        = 5000.0; // <= 0: no sliver drop
};

/// density check of one run, for the log
struct RunCheck
{
    double        azimuth_from_deg = 0.0;
    double        azimuth_to_deg   = 0.0;
    std::size_t   num_bins         = 0;
    std::uint64_t reports          = 0;
    double        band_m           = 0.0;
    double        density          = 0.0;
    bool          cut              = false;
};

/// dropped sliver sector, for the log
struct SliverDrop
{
    double azimuth_deg = 0.0;
    double area_m2     = 0.0;
};

struct CoverageEstimate
{
    std::vector<CoverageBin> bins;          // exact segments, used as the coverage gate
    std::vector<CoverageRun> sector_runs;   // glued and widened, drawn as sectors
    std::vector<RunCheck>    run_checks;
    std::vector<SliverDrop>  slivers;
};

/// the whole estimate from the report ranges per azimuth bin
CoverageEstimate estimateCoverage(std::vector<std::vector<double>> ranges_per_bin, const CoverageParams& params);

} // namespace SMRCoverageHelpers

} // namespace analysis
