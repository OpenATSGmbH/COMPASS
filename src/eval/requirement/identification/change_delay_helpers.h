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

#include <string>
#include <vector>

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

namespace EvaluationRequirement
{

/**
 * Algorithms of the Identification Change Delay requirement, free of the evaluation data
 * access, so they can be unit tested on plain inputs.
 */
namespace ChangeDelayHelpers
{

/// identity value of a report (time, value), no value if not set
typedef std::pair<boost::posix_time::ptime, boost::optional<std::string>> TimedValue;

/// change of the stable reference identity value
struct Change
{
    size_t      ref_index; // index of the first reference report with the new value (event time)
    std::string old_value;
    std::string new_value;
};

/// assessment of one change event
struct Assessment
{
    enum class Result
    {
        Passed,         // new value reported within the maximum delay
        Failed,         // new value reported later than the maximum delay
        NeverReported,  // new value never reported, enough test data after the event
        NotAssessable   // new value never reported, test data ends too early after the event
    };

    Result                  result = Result::NotAssessable;
    boost::optional<double> delay_s; // only if the new value was reported
};

/// ACID without trailing spaces, no value if empty
boost::optional<std::string> normalizedACID(const std::string& acid);

/**
 * Changes of the stable reference value. A new value becomes stable when a later reference
 * report still carries it at least min_stable_time_s after its first report (transitional
 * values are ignored). Each stable value after the first one is a change, with the event at
 * its first report. Reports without value are skipped.
 */
std::vector<Change> changeEvents(const std::vector<TimedValue>& ref_values,
                                 float min_stable_time_s);

/**
 * Searches the first test report (sorted by time) at or after the event time carrying the
 * new value.
 */
Assessment assessEvent(const boost::posix_time::ptime& event_ts,
                       const std::string& new_value,
                       const std::vector<TimedValue>& tst_values,
                       float max_delay_s);

} // namespace ChangeDelayHelpers

} // namespace EvaluationRequirement
