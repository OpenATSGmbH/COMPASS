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

#include "intervalhelpers.h"
#include "timeperiod.h"

#include "timeconv.h"
#include "traced_assert.h"

#include <algorithm>
#include <cmath>

namespace EvaluationRequirement
{

/**
*/
boost::posix_time::time_duration Event::dt() const
{
    return interval_time1 - interval_time0;
}

/**
*/
double Event::dtSeconds() const
{
    return Utils::Time::partialSeconds(dt());
}

namespace IntervalHelpers
{

/**
*/
uint32_t numMisses(double dt, const MissParams& p)
{
    if (p.miss_tolerance_s.has_value())
        dt -= p.miss_tolerance_s.value();

    if (p.min_gap_length_s.has_value() && dt < p.min_gap_length_s.value()) // supress gaps smaller as min gap length
        return 0;

    if (p.max_gap_length_s.has_value() && dt > p.max_gap_length_s.value()) // supress gaps larger as max gap length
        return 0;

    if (dt <= p.update_interval_s)
        return 0;

    return (uint32_t)std::floor(dt / p.update_interval_s);
}

/**
 * Adds the outside periods up to the test data times and sorts the test updates into the periods.
 * Every test update must land in a period.
 */
void addTestUpdates(TimePeriodCollection& periods,
                    const std::vector<dbContent::TargetReport::DataID>& tst_updates)
{
    boost::optional<boost::posix_time::ptime> tmin_tst;
    boost::optional<boost::posix_time::ptime> tmax_tst;
    if (!tst_updates.empty())
    {
        tmin_tst = tst_updates.front().timestamp();
        tmax_tst = tst_updates.back().timestamp();
    }

    periods.fillInOutsidePeriods(tmin_tst, tmax_tst);

    for (const auto& data_id : tst_updates)
    {
        bool inside_period = periods.isInside(data_id.timestamp());
        traced_assert(inside_period); //!should _always_ land in a period!

        TimePeriodUpdate update;
        update.data_id = data_id;

        periods.addUpdate(update);
    }
}

/**
*/
std::vector<Event> periodEvents(const TimePeriod& period,
                                const ValidityFunc& validity,
                                const MissParams& p,
                                bool skip_no_data_details)
{
    std::vector<Event> events;

    auto logEvent = [ & ] (std::vector<Event>& events,
                           Event::Type type, 
                           const TimePeriodUpdate* update,
                           const boost::posix_time::ptime& time0,
                           const boost::posix_time::ptime& time1,
                           bool compute_misses)
    {
        traced_assert(time1 >= time0);

        Event e;
        e.type           = type;
        e.interval_time0 = time0;
        e.interval_time1 = time1;

        if (update)
            e.data_id = update->data_id;

        if (compute_misses)
        {
            double dt = Utils::Time::partialSeconds(time1 - time0);

            e.misses = numMisses(dt, p);
        }

        events.push_back(e);
    };

    //outside sector period => add all updates as OutsideSector
    if (period.type() == TimePeriod::Type::OutsideSector)
    {
        if (!skip_no_data_details)
        {
            for (const auto& update : period.getUpdates())
            {
                logEvent(events, Event::TypeNoReference, &update, update.data_id.timestamp(), update.data_id.timestamp(), false);
            }
        }

        return events;
    }

    //log period entered
    if (!skip_no_data_details)
        logEvent(events, Event::TypeEnterPeriod, nullptr, period.begin(), period.end(), false);

    //inside sector period
    const auto& updates = period.getUpdates();

    if (updates.empty())
    {
        //no updates in period => add single event for this case
        if (!skip_no_data_details)
            logEvent(events, Event::TypeEmptyPeriod, nullptr, period.begin(), period.end(), false);
    }
    else
    {
        //period obtains updates
        size_t n = updates.size();

        const TimePeriodUpdate* valid_last     = nullptr;
        int                     valid_last_idx = -1;

        for (size_t i = 0; i < n; ++i)
        {
            const auto& update = updates[ i ];

            //check if update is valid
            auto update_validity = validity(update.data_id);

            //invalid update => log and skip
            if (update_validity.value == IntervalValidity::Value::Invalid)
            {
                logEvent(events, Event::TypeInvalid, &update, update.data_id.timestamp(), update.data_id.timestamp(), false);
                continue;
            }

            //first valid update? => log
            if (!valid_last && !skip_no_data_details)
                logEvent(events, Event::TypeFirstValidUpdateInPeriod, &update, update.data_id.timestamp(), update.data_id.timestamp(), false);

            //valid update => check for misses
            logEvent(events, 
                     valid_last ? Event::TypeValid : Event::Type::TypeValidFirst,
                     &update,
                     valid_last ? valid_last->data_id.timestamp() : period.begin(),      // time0 is either the last valid update or period begin
                     update.data_id.timestamp(),                                         // time1 always update time
                     true);                                                              // check misses in both cases

            valid_last     = &updates[ i ];
            valid_last_idx = (int)events.size();
        }

        //add event for last valid update
        if (valid_last)
        {
            std::vector<Event> last_events;

            if (!skip_no_data_details)
                logEvent(last_events, Event::TypeLastValidUpdateInPeriod, valid_last, valid_last->data_id.timestamp(), valid_last->data_id.timestamp(), false);
            logEvent(last_events, Event::TypeValidLast, valid_last, valid_last->data_id.timestamp(), period.end(), true);

            events.insert(events.begin() + valid_last_idx, last_events.begin(), last_events.end());
        }

        //no valid updates in period => add single event for this case, the whole period is missed
        //(test target reports without a correct code, as between valid updates)
        if (!valid_last)
            logEvent(events, Event::TypeEmptyPeriod, nullptr, period.begin(), period.end(), true);
    }

    //log period left
    if (!skip_no_data_details)
        logEvent(events, Event::TypeLeavePeriod, nullptr, period.begin(), period.end(), false);

    return events;
}

/**
*/
std::vector<Event> periodEvents(const TimePeriodCollection& periods,
                                const ValidityFunc& validity,
                                const MissParams& p,
                                bool skip_no_data_details)
{
    std::vector<Event> events;

    uint32_t inside_period_cnt = 0;

    for (size_t i = 0; i < periods.size(); ++i)
    {
        const auto& period = periods.period(i);

        //create events for period
        auto events_period = periodEvents(period, validity, p, skip_no_data_details);

        //period is inside period?
        if (period.type() == TimePeriod::Type::InsideSector)
        {
            //add inside period index to events
            for (auto& e : events_period)
                e.period = inside_period_cnt;

            ++inside_period_cnt;
        }

        //collect
        if (!events_period.empty())
            events.insert(events.end(), events_period.begin(), events_period.end());
    }

    return events;
}

/**
 * Expected update intervals of the inside periods and the misses of the events, optionally
 * without the detection misses.
 */
Counts counts(const TimePeriodCollection& periods,
              const std::vector<Event>& events,
              const MissParams& p,
              bool exclude_detection_misses)
{
    Counts c;
    c.sum_uis = periods.getUIs(p.update_interval_s);

    for (const auto& e : events)
        c.misses += e.misses;

    if (exclude_detection_misses)
        excludeDetectionMisses(periods, p, c);

    return c;
}

/**
 * Removes the update intervals without any test target report from the expected and the
 * missed update intervals, e.g. EUROCAE ED-142 6.4.3: the probability of code detection
 * only counts update intervals in which a target report was output.
 *
 * The detection misses are counted as the code misses, but over the gaps between all test
 * target reports of an inside period (including period begin and end). An inside period
 * without any test target report has no code misses, its update intervals are only removed
 * from the expected ones.
 */
void excludeDetectionMisses(const TimePeriodCollection& periods,
                            const MissParams& p,
                            Counts& counts)
{
    uint32_t detection_misses       = 0; // in periods with test target reports
    uint32_t empty_period_misses    = 0; // in periods without test target reports

    for (size_t i = 0; i < periods.size(); ++i)
    {
        const auto& period = periods.period(i);

        if (period.type() != TimePeriod::Type::InsideSector)
            continue;

        if (period.getUpdates().empty())
        {
            empty_period_misses += numMisses(Utils::Time::partialSeconds(period.duration()), p);
            continue;
        }

        auto t_last = period.begin();

        for (const auto& update : period.getUpdates())
        {
            auto t = update.data_id.timestamp();

            detection_misses += numMisses(Utils::Time::partialSeconds(t - t_last), p);
            t_last = t;
        }

        detection_misses += numMisses(Utils::Time::partialSeconds(period.end() - t_last), p);
    }

    uint32_t uis_removed = detection_misses + empty_period_misses;

    counts.sum_uis = counts.sum_uis > uis_removed ? counts.sum_uis - uis_removed : 0;
    counts.misses  = counts.misses > detection_misses ? counts.misses - detection_misses : 0;
    counts.misses  = std::min(counts.misses, counts.sum_uis);
}

} // namespace IntervalHelpers

} // namespace EvaluationRequirement
