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

#include "requirement/detection/detection_walk.h"
#include "timeperiod.h"

#include "timeconv.h"
#include "traced_assert.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

using boost::posix_time::ptime;

namespace EvaluationRequirement
{
namespace DetectionWalk
{

/**
 * A new period starts at the first inside reference time after an outside one, or when the
 * time to the end of the last period is larger than the maximum reference time difference.
 */
TimePeriodCollection referencePeriods(const std::vector<std::pair<ptime, bool>>& ref_inside,
                                      const boost::posix_time::time_duration& max_ref_time_diff)
{
    TimePeriodCollection ref_periods;

    bool first      = true;
    bool is_inside  = false;
    bool was_inside = false;

    for (const auto& ref : ref_inside)
    {
        const ptime& timestamp = ref.first;

        was_inside = is_inside;
        is_inside  = ref.second;

        if (first)
        {
            if (is_inside) // create time period
                ref_periods.add({timestamp, timestamp});

            first = false;

            continue;
        }

        if (was_inside)
        {
            if (is_inside)
            {
                // extend last time period, if possible, or finish last and create new one
                if (ref_periods.lastPeriod().isCloseToEnd(timestamp, max_ref_time_diff))
                    ref_periods.lastPeriod().extend(timestamp);
                else
                    ref_periods.add({timestamp, timestamp});
            }
        }
        else if (is_inside) // was not inside and is now inside
            ref_periods.add({timestamp, timestamp}); // create new time period
    }

    ref_periods.removeSmallPeriods(Utils::Time::partialSeconds(1));

    return ref_periods;
}

/**
 */
double missedAmount(float t_diff, const PDHelpers::MissTestParams& p, const Params& params)
{
    if (params.use_gap_count)
        return 1.0;

    return params.use_time_ratio ? (double)PDHelpers::missDuration(t_diff, p)
                                 : (double)PDHelpers::numMisses(t_diff, p);
}

/**
 */
Result walk(const TimePeriodCollection& ref_periods,
            const std::vector<ptime>& tst_times,
            const TstCheck& has_ref_pos,
            const TstCheck& inside_sector,
            const GapUIFunc& gap_ui,
            const Params& params)
{
    Result result;

    // expected total: number of test reports accepted by the walk (gap count mode),
    // reference duration in seconds (time-ratio mode) or number of update intervals
    // (counting mode). In gap count mode the accepted reports are counted below,
    // wherever a report updates period_last_tst_times.
    result.sum_expected = params.use_gap_count  ? 0.0
                        : params.use_time_ratio ? ref_periods.getDurationSeconds()
                                                : (double)ref_periods.getUIs(params.miss_params.update_interval_s);

    // fills the gap of a step and adds it
    auto addGapStep = [ & ] (Step step, const ptime& t0, const ptime& t1)
    {
        traced_assert(t1 >= t0);

        step.t0      = t0;
        step.t1      = t1;
        step.t_diff  = Utils::Time::partialSeconds(t1 - t0);
        step.has_gap = true;

        auto p = params.miss_params;
        p.update_interval_s = gap_ui(t0, t1);

        step.miss = PDHelpers::isMiss(step.t_diff, p);

        if (step.miss)
        {
            step.missed = missedAmount(step.t_diff, p, params);
            result.sum_missed += step.missed;
        }

        step.sum_missed = result.sum_missed;

        result.steps.push_back(step);
    };

    auto addStep = [ & ] (Step step)
    {
        step.sum_missed = result.sum_missed;
        result.steps.push_back(step);
    };

    if (tst_times.empty()) // no test data, every period is a gap
    {
        for (unsigned int period_cnt = 0; period_cnt < ref_periods.size(); ++period_cnt)
        {
            Step step;
            step.type   = Step::Type::PeriodWithoutTestData;
            step.period = period_cnt;

            addGapStep(step, ref_periods.period(period_cnt).begin(), ref_periods.period(period_cnt).end());
        }

        return result;
    }

    std::map<unsigned int, ptime> period_last_tst_times; // period number -> last test time
    std::set<unsigned int>        finished_periods;

    // finishes a period: gap from the last test time (or the period begin) to the period end
    auto finishPeriod = [ & ] (unsigned int period_cnt, int tst_index)
    {
        Step step;
        step.type      = Step::Type::PeriodEnd;
        step.tst_index = tst_index;
        step.period    = period_cnt;

        ptime t0 = ref_periods.period(period_cnt).begin();

        if (period_last_tst_times.count(period_cnt))
        {
            t0 = period_last_tst_times.at(period_cnt);
            step.t0_is_tst = true;
        }

        addGapStep(step, t0, ref_periods.period(period_cnt).end());

        finished_periods.insert(period_cnt);
    };

    ptime timestamp, last_ts;
    bool  was_outside = false;

    for (size_t i = 0; i < tst_times.size(); ++i)
    {
        last_ts   = timestamp;
        timestamp = tst_times[ i ];

        bool is_inside_ref_time_period = ref_periods.isInside(timestamp);

        // finish previous periods
        int period_max_index_before_ts = ref_periods.getPeriodMaxIndexBefore(timestamp);

        for (int period_cnt = 0; period_cnt <= period_max_index_before_ts; ++period_cnt)
            if (!finished_periods.count(period_cnt))
                finishPeriod(period_cnt, (int)i);

        Step step;
        step.tst_index = (int)i;

        if (!is_inside_ref_time_period)
        {
            step.type = Step::Type::OutsideRefPeriods;
            addStep(step);
            continue;
        }

        unsigned int period_index = ref_periods.getPeriodIndex(timestamp);
        step.period = period_index;

        if (!has_ref_pos(i)) // only happens if test time is exact beginning of reference interval
        {
            step.type = Step::Type::NoRefPos;
            addStep(step);
            continue;
        }

        if (!inside_sector(i))
        {
            step.type = Step::Type::OutsideSector;
            addStep(step);

            was_outside = true;
            continue;
        }

        if (!period_last_tst_times.count(period_index) || was_outside) // first in period or was outside
        {
            if (was_outside)
            {
                step.type = Step::Type::FirstAfterOutside;
                addStep(step);
            }
            else // first in period, gap from the period begin
            {
                step.type = Step::Type::FirstInPeriod;
                addGapStep(step, ref_periods.period(period_index).begin(), timestamp);
            }

            was_outside = false;
            period_last_tst_times[period_index] = timestamp;

            if (params.use_gap_count)
                result.sum_expected += 1.0;

            continue;
        }

        step.type      = Step::Type::Gap;
        step.t0_is_tst = true;
        addGapStep(step, last_ts, timestamp);

        period_last_tst_times[period_index] = timestamp;

        if (params.use_gap_count)
            result.sum_expected += 1.0;
    }

    // finish the remaining periods
    for (unsigned int period_cnt = 0; period_cnt < ref_periods.size(); ++period_cnt)
        if (!finished_periods.count(period_cnt))
            finishPeriod(period_cnt, -1);

    return result;
}

/**
 */
unsigned int cyclesPerPeriod(const std::vector<boost::posix_time::ptime>& cycles, float update_interval_s)
{
    traced_assert(cycles.size() >= 2);

    // median reported cycle length, to map the configured update interval onto a
    // whole number of reported cycles
    std::vector<double> cycle_lengths;
    cycle_lengths.reserve(cycles.size());

    for (size_t i = 1; i < cycles.size(); ++i)
        cycle_lengths.push_back(Utils::Time::partialSeconds(cycles[i] - cycles[i - 1]));

    std::sort(cycle_lengths.begin(), cycle_lengths.end());

    const double median_cycle_s = cycle_lengths.at(cycle_lengths.size() / 2);

    if (median_cycle_s <= 0)
        return 1;

    return (unsigned int) std::max(1L, std::lround(update_interval_s / median_cycle_s));
}

/**
 */
CycleResult statusCycleWalk(const TimePeriodCollection& ref_periods,
                            const std::vector<boost::posix_time::ptime>& cycles,
                            const std::vector<boost::posix_time::ptime>& tst_times,
                            const TstCheck& detects,
                            unsigned int cycles_per_period)
{
    CycleResult result;

    for (unsigned int period_cnt = 0; period_cnt < ref_periods.size(); ++period_cnt)
    {
        const TimePeriod& period_it = ref_periods.period(period_cnt);

        auto cycle_it = std::lower_bound(cycles.begin(), cycles.end(), period_it.begin());

        while (cycle_it != cycles.end())
        {
            auto next_it = cycle_it;

            for (unsigned int cnt = 0; cnt < cycles_per_period && next_it != cycles.end(); ++cnt)
                ++next_it;

            if (next_it == cycles.end())
                break; // incomplete period at the end of the cycle stream

            const boost::posix_time::ptime period_begin = *cycle_it;
            const boost::posix_time::ptime period_end   = *next_it;

            if (period_end > period_it.end())
                break; // period not fully inside the reference period

            result.sum_expected += 1.0;

            bool detected = false;

            for (auto tst_it = std::lower_bound(tst_times.begin(), tst_times.end(), period_begin);
                 tst_it != tst_times.end() && *tst_it < period_end; ++tst_it)
            {
                if (detects(tst_it - tst_times.begin()))
                {
                    detected = true;
                    break;
                }
            }

            if (!detected)
            {
                result.sum_missed += 1.0;
                result.misses.push_back({ period_begin, period_end, result.sum_missed });
            }

            cycle_it = next_it;
        }
    }

    return result;
}

} // namespace DetectionWalk
} // namespace EvaluationRequirement
