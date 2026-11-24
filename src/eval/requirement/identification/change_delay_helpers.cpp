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

#include "requirement/identification/change_delay_helpers.h"

#include "timeconv.h"

#include <algorithm>

using boost::posix_time::ptime;

namespace EvaluationRequirement
{
namespace ChangeDelayHelpers
{

/**
 */
boost::optional<std::string> normalizedACID(const std::string& acid)
{
    std::string value = acid;
    value.erase(value.find_last_not_of(' ') + 1);

    if (value.empty())
        return {};

    return value;
}

/**
 */
std::vector<Change> changeEvents(const std::vector<TimedValue>& ref_values,
                                 float min_stable_time_s)
{
    std::vector<Change> changes;

    boost::optional<std::string> stable_value;
    boost::optional<std::string> candidate_value;
    size_t                       candidate_start = 0;

    for (size_t i = 0; i < ref_values.size(); ++i)
    {
        const auto& value = ref_values[ i ].second;

        if (!value.has_value())
            continue;

        if (stable_value.has_value() && value.value() == stable_value.value())
        {
            // back at the stable value: candidate was transitional
            candidate_value.reset();
            continue;
        }

        if (candidate_value.has_value() && value.value() == candidate_value.value())
        {
            if (Utils::Time::partialSeconds(ref_values[ i ].first - ref_values[ candidate_start ].first) >= min_stable_time_s)
            {
                // candidate held long enough: new stable value
                if (stable_value.has_value())
                    changes.push_back({ candidate_start, stable_value.value(), candidate_value.value() });

                stable_value = candidate_value;
                candidate_value.reset();
            }

            continue;
        }

        // new candidate value
        candidate_value = value;
        candidate_start = i;
    }

    return changes;
}

/**
 */
Assessment assessEvent(const ptime& event_ts,
                       const std::string& new_value,
                       const std::vector<TimedValue>& tst_values,
                       float max_delay_s)
{
    Assessment a;

    auto it = std::lower_bound(tst_values.begin(), tst_values.end(), event_ts,
                               [] (const TimedValue& v, const ptime& t) { return v.first < t; });

    for (; it != tst_values.end(); ++it)
    {
        if (it->second.has_value() && it->second.value() == new_value)
        {
            a.delay_s = Utils::Time::partialSeconds(it->first - event_ts);
            break;
        }
    }

    if (!a.delay_s.has_value())
    {
        // no test report with the new value: only a failure if enough
        // test data time remained after the event to expect one
        if (tst_values.empty() || Utils::Time::partialSeconds(tst_values.back().first - event_ts) < max_delay_s)
            a.result = Assessment::Result::NotAssessable;
        else
            a.result = Assessment::Result::NeverReported;

        return a;
    }

    a.result = a.delay_s.value() <= max_delay_s ? Assessment::Result::Passed : Assessment::Result::Failed;

    return a;
}

} // namespace ChangeDelayHelpers
} // namespace EvaluationRequirement
