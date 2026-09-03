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

#include "scancyclewalk.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;

namespace analysis
{

std::vector<CycleEvent> evaluateCyclesInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<ptime>& cycles_sorted,
    const std::vector<ptime>& tst_ts_sorted,
    const MovementUI* mv)
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
        auto tst_hi = std::lower_bound(tst_lo,
                                       tst_ts_sorted.end(),
                                       t_window_end);

        CycleEvent ev;
        ev.t_cycle     = t_cycle;
        ev.num_reports = static_cast<unsigned int>(std::distance(tst_lo, tst_hi));
        ev.is_miss     = (ev.num_reports == 0);
        out.push_back(ev);

        if (standing)
            it = std::lower_bound(it, cyc_end, t_window_end);  // skip covered cycles
        else
            ++it;
    }

    return out;
}

std::vector<CycleEvent> evaluateNominalSlotsInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    double period_s,
    const std::vector<ptime>& tst_ts_sorted,
    std::size_t max_slots)
{
    std::vector<CycleEvent> out;

    if (period_s <= 0.0 || period.empty())
        return out;

    const time_duration step = boost::posix_time::microseconds(
        static_cast<long long>(std::llround(period_s * 1.0e6)));
    if (step.total_microseconds() <= 0)
        return out;

    ptime t = period.begin;
    std::size_t n = 0;

    // only slots that lie completely inside the period (floor semantics)
    while (t + step <= period.end && n < max_slots)
    {
        const ptime t_end = t + step;

        auto tst_lo = std::lower_bound(tst_ts_sorted.begin(), tst_ts_sorted.end(), t);
        auto tst_hi = std::lower_bound(tst_lo, tst_ts_sorted.end(), t_end);

        CycleEvent ev;
        ev.t_cycle     = t;
        ev.num_reports = static_cast<unsigned int>(std::distance(tst_lo, tst_hi));
        ev.is_miss     = (ev.num_reports == 0);
        out.push_back(ev);

        t += step;
        ++n;
    }

    return out;
}

} // namespace analysis
