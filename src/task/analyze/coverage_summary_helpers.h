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

#include <boost/date_time/posix_time/ptime.hpp>

#include <cstdint>
#include <optional>
#include <vector>

namespace analysis
{

/**
 * Summaries shared by the coverage inspectors (MLAT, ADS-B, SMR): probability of detection from
 * expected (#EUI) and missed (#MUI) update slots, and update interval statistics. Free of the
 * dataset, so they can be unit tested on plain values.
 */
namespace CoverageSummaryHelpers
{

constexpr double MaxUpdateIntervalS = 300.0; // longer intervals are breaks, not update intervals
constexpr double WorstCellMinEUI    = 5.0;   // a cell needs this many expected slots to be the worst cell

/// probability of detection (eui - mui) / eui, 0 when nothing is expected
double pd(double eui, double mui);

/// probability of detection of a grid cell, none when nothing is expected; a negative value
/// (more missed than expected slots, an error of the walk) is logged and set to 0
std::optional<double> cellPD(double eui, double mui);

struct CellCounts
{
    double eui = 0.0;
    double mui = 0.0;
};

struct PDSummary
{
    double      total_eui      = 0.0;
    double      total_mui      = 0.0;
    double      overall_pd     = 0.0;
    std::size_t cells_with_eui = 0;
    double      median_cell_pd = 0.0;
    double      p5_cell_pd     = 0.0;
    bool        has_worst_cell = false;
    double      worst_cell_pd  = 1.0;
};

/// totals and per-cell PD statistics over the cells of a projection
PDSummary summarizePD(const std::vector<CellCounts>& cells);

/// appends the intervals between consecutive sorted time stamps that are > 0 and <= max_interval_s
void appendUpdateIntervals(const std::vector<boost::posix_time::ptime>& sorted_times,
                           std::vector<double>& intervals_s,
                           double max_interval_s = MaxUpdateIntervalS);

struct IntervalStats
{
    std::size_t num       = 0;
    double      median_s  = 0.0;
    double      p10_s     = 0.0;
    double      p90_s     = 0.0;
    double      mean_s    = 0.0;

    // histogram over [0, hist_max_s) in equal bins, intervals >= hist_max_s in the overflow bin
    double                     hist_max_s       = 0.0;
    double                     hist_bin_width_s = 0.0;
    std::vector<std::uint32_t> hist_bins;
    std::uint32_t              hist_overflow    = 0;
};

/// statistics and histogram of update intervals; hist_max_s <= 0 uses the rounded-up P99, and
/// fallback_max_s if that is 0 too; empty input gives num 0 and no histogram
IntervalStats intervalStats(const std::vector<double>& intervals_s, double hist_max_s, int num_bins,
                            double fallback_max_s);

} // namespace CoverageSummaryHelpers

} // namespace analysis
