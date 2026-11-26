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


#include "coverage_summary_helpers.h"

#include "logger.h"
#include "number.h"

#include <algorithm>
#include <cmath>

namespace analysis
{
namespace CoverageSummaryHelpers
{

/**
 */
double pd(double eui, double mui)
{
    return eui > 0.0 ? (eui - mui) / eui : 0.0;
}

/**
 */
std::optional<double> cellPD(double eui, double mui)
{
    if (eui <= 0.0)
        return std::nullopt;

    double value = (eui - mui) / eui;

    if (value < -1e-9)
        logerr << "cell with more missed than expected slots, #EUI " << eui << " #MUI " << mui;

    return std::max(0.0, value);
}

/**
 */
PDSummary summarizePD(const std::vector<CellCounts>& cells)
{
    PDSummary summary;

    std::vector<double> cell_pds;
    cell_pds.reserve(cells.size());

    for (const auto& cell : cells)
    {
        summary.total_eui += cell.eui;
        summary.total_mui += cell.mui;

        auto cell_pd = cellPD(cell.eui, cell.mui);

        if (!cell_pd)
            continue;

        cell_pds.push_back(*cell_pd);

        if (cell.eui >= WorstCellMinEUI && (!summary.has_worst_cell || *cell_pd < summary.worst_cell_pd))
        {
            summary.worst_cell_pd  = *cell_pd;
            summary.has_worst_cell = true;
        }
    }

    summary.overall_pd     = pd(summary.total_eui, summary.total_mui);
    summary.cells_with_eui = cell_pds.size();
    summary.median_cell_pd = Utils::Number::percentile(cell_pds, 0.5);
    summary.p5_cell_pd     = Utils::Number::percentile(cell_pds, 0.05);

    return summary;
}

/**
 */
void appendUpdateIntervals(const std::vector<boost::posix_time::ptime>& sorted_times,
                           std::vector<double>& intervals_s,
                           double max_interval_s)
{
    for (std::size_t cnt = 1; cnt < sorted_times.size(); ++cnt)
    {
        double dt = (sorted_times[cnt] - sorted_times[cnt - 1]).total_microseconds() / 1e6;

        if (dt > 0.0 && dt <= max_interval_s)
            intervals_s.push_back(dt);
    }
}

/**
 */
IntervalStats intervalStats(const std::vector<double>& intervals_s, double hist_max_s, int num_bins,
                            double fallback_max_s)
{
    IntervalStats stats;
    stats.num = intervals_s.size();

    if (intervals_s.empty())
        return stats;

    stats.median_s = Utils::Number::percentile(intervals_s, 0.5);
    stats.p10_s    = Utils::Number::percentile(intervals_s, 0.1);
    stats.p90_s    = Utils::Number::percentile(intervals_s, 0.9);
    stats.mean_s   = Utils::Number::mean(intervals_s);

    // the overflow bin keeps long detection breaks from stretching the axis
    double max_s = hist_max_s > 0.0 ? hist_max_s : std::ceil(Utils::Number::percentile(intervals_s, 0.99));

    if (max_s <= 0.0)
        max_s = fallback_max_s;

    const unsigned int nbins = static_cast<unsigned int>(std::max(1, num_bins));
    const double width = max_s / static_cast<double>(nbins);

    stats.hist_bins.assign(nbins, 0);

    for (double dt : intervals_s)
    {
        if (dt >= max_s)
        {
            ++stats.hist_overflow;
            continue;
        }

        unsigned int bin = static_cast<unsigned int>(std::floor(dt / width));

        if (bin >= nbins)
            bin = nbins - 1;

        ++stats.hist_bins[bin];
    }

    stats.hist_max_s       = max_s;
    stats.hist_bin_width_s = width;

    return stats;
}

} // namespace CoverageSummaryHelpers
} // namespace analysis
