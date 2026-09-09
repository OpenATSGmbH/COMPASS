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
    const PDWalkSlotFunc&                              on_expected,
    const PDWalkSlotFunc&                              on_miss)
{
    if (!params.mv)
        return;

    if (partialSeconds(period.end - period.begin) <= 0.0)
        return;

    const MovementUI& mv = *params.mv;

    // expected slots, stepped with the update interval valid at each slot
    {
        std::size_t guard = 0;

        for (ptime t_slot = period.begin; t_slot < period.end; )
        {
            if (on_expected)
                on_expected(t_slot);

            const double ui = mv.uiAt(t_slot);
            if (ui <= 0.0 || ++guard > params.max_iterations)
                break;

            t_slot = addSeconds(t_slot, ui);
        }
    }

    // gaps: period start, test reports inside the period, period end
    auto first = std::lower_bound(tst_ts_sorted.begin(), tst_ts_sorted.end(), period.begin);
    auto last  = std::upper_bound(tst_ts_sorted.begin(), tst_ts_sorted.end(), period.end);

    std::vector<ptime> walk;
    walk.reserve(static_cast<std::size_t>(std::distance(first, last)) + 2);

    walk.push_back(period.begin);
    for (auto it = first; it != last; ++it)
        walk.push_back(*it);
    walk.push_back(period.end);

    const double tol_s = params.use_miss_tolerance ? params.miss_tolerance_s : 0.0;

    for (std::size_t i = 0; i + 1 < walk.size(); ++i)
    {
        const ptime gap_start = walk[ i     ];
        const ptime gap_end   = walk[ i + 1 ];

        if (gap_end <= gap_start)
            continue;

        std::size_t guard = 0;

        for (ptime t_miss = gap_start;;)
        {
            const double ui = mv.uiAt(t_miss);
            if (ui <= 0.0 || ++guard > params.max_iterations)
                break;

            t_miss = addSeconds(t_miss, ui);

            if (t_miss >= gap_end || partialSeconds(gap_end - t_miss) < tol_s)
                break;

            if (on_miss)
                on_miss(t_miss);
        }
    }
}

/**
 */
void walkReferencePeriodsTimeDifference(
    const std::vector<EvaluationRequirement::PDHelpers::RefPeriod>& periods,
    const std::vector<ptime>&                                       tst_ts_sorted,
    const PDWalkParams&                                             params,
    const PDWalkSlotFunc&                                           on_expected,
    const PDWalkSlotFunc&                                           on_miss)
{
    for (const auto& period : periods)
        walkReferencePeriodTimeDifference(period, tst_ts_sorted, params, on_expected, on_miss);
}

} // namespace analysis
