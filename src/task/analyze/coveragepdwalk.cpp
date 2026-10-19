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

#include "coveragepdwalk.h"

#include "boost/date_time/posix_time/posix_time_duration.hpp"

#include <algorithm>
#include <cmath>

using namespace boost::posix_time;

namespace analysis
{

namespace
{
    double partialSeconds(const time_duration& d)
    {
        return static_cast<double>(d.total_microseconds()) / 1.0e6;
    }

    ptime addSeconds(ptime t, double s)
    {
        long long us = static_cast<long long>(std::llround(s * 1.0e6));
        return t + microseconds(us);
    }
}

/**
 */
void walkReferencePeriodTimeDifference(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<ptime>&                          tst_ts_sorted,
    const PDWalkParams&                                params,
    const PDWalkPieceFunc&                             on_piece,
    const PDWalkGapFunc&                               on_gap)
{
    if (!params.mv)
        return;

    if (partialSeconds(period.end - period.begin) <= 0.0)
        return;

    const MovementUI& mv = *params.mv;

    // time line: period start, test reports inside the period, period end
    auto first = std::lower_bound(tst_ts_sorted.begin(), tst_ts_sorted.end(), period.begin);
    auto last  = std::upper_bound(tst_ts_sorted.begin(), tst_ts_sorted.end(), period.end);

    std::vector<ptime> walk;
    walk.reserve(static_cast<std::size_t>(std::distance(first, last)) + 2);

    walk.push_back(period.begin);
    for (auto it = first; it != last; ++it)
        walk.push_back(*it);
    walk.push_back(period.end);

    const double tol_s = params.use_miss_tolerance ? params.miss_tolerance_s : 0.0;

    std::vector<ptime>  miss_t;
    std::vector<double> miss_ui;

    for (std::size_t i = 0; i + 1 < walk.size(); ++i)
    {
        const ptime gap_start = walk[ i     ];
        const ptime gap_end   = walk[ i + 1 ];

        if (gap_end <= gap_start)
            continue;

        const double ui_start = mv.uiAt(gap_start);
        if (ui_start <= 0.0)
            continue;

        // missed slots by the gap rule
        miss_t.clear();
        miss_ui.clear();

        std::size_t guard = 0;
        for (ptime t_miss = gap_start;;)
        {
            const double ui = mv.uiAt(t_miss);
            if (ui <= 0.0 || ++guard > params.max_iterations)
                break;

            t_miss = addSeconds(t_miss, ui);

            if (t_miss >= gap_end || partialSeconds(gap_end - t_miss) < tol_s)
                break;

            miss_t.push_back(t_miss);
            miss_ui.push_back(mv.uiAt(t_miss));
        }

        const std::size_t num_missed = miss_t.size();

        if (num_missed == 0)
        {
            // detected gap: the whole gap as a fraction of the update interval
            if (on_piece)
                on_piece(gap_start, gap_end, partialSeconds(gap_end - gap_start) / ui_start, false);
            continue;
        }

        // detected piece up to the first miss, carrying the rest of the last slot
        const double ui_last  = miss_ui.back() > 0.0 ? miss_ui.back() : ui_start;
        const double rest     = partialSeconds(gap_end - miss_t.back()) / ui_last;

        if (on_piece)
        {
            on_piece(gap_start, miss_t.front(), rest, false);

            for (std::size_t k = 0; k < num_missed; ++k)
            {
                const ptime piece_end = k + 1 < num_missed ? miss_t[ k + 1 ] : gap_end;
                on_piece(miss_t[ k ], piece_end, 1.0, true);
            }
        }

        if (on_gap)
            on_gap(gap_start, gap_end, static_cast<unsigned int>(num_missed));
    }
}

/**
 */
void walkReferencePeriodsTimeDifference(
    const std::vector<EvaluationRequirement::PDHelpers::RefPeriod>& periods,
    const std::vector<ptime>&                                       tst_ts_sorted,
    const PDWalkParams&                                             params,
    const PDWalkPieceFunc&                                          on_piece,
    const PDWalkGapFunc&                                            on_gap)
{
    for (const auto& period : periods)
        walkReferencePeriodTimeDifference(period, tst_ts_sorted, params, on_piece, on_gap);
}

/**
 */
void samplePiece(const ptime&            begin,
                 const ptime&            end,
                 double                  max_step_s,
                 const PDWalkSampleFunc& on_sample)
{
    if (!on_sample)
        return;

    const double len_s = partialSeconds(end - begin);
    if (len_s <= 0.0)
    {
        // zero-length piece: one sample at its time carries all of it
        on_sample(begin, 1.0);
        return;
    }

    const double step = max_step_s > 0.0 ? max_step_s : len_s;
    const std::size_t n = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(len_s / step)));
    const double sub = len_s / static_cast<double>(n);
    const double frac = 1.0 / static_cast<double>(n);

    for (std::size_t k = 0; k < n; ++k)
        on_sample(addSeconds(begin, (static_cast<double>(k) + 0.5) * sub), frac);
}

} // namespace analysis
