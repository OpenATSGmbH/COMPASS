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

#include <boost/optional.hpp>

namespace EvaluationRequirement
{

/**
 * Check of one test report of the ADS-B Latency requirement (ED-129C Section 3.3.4), free of the
 * evaluation data access, so it can be unit tested on plain values.
 */
namespace ADSBLatencyHelpers
{

enum class Outcome
{
    NoTimeData, // no Time of Message Reception for Position or no Time of Report Transmission: not counted
    Passed,
    Negative,   // report transmitted before the message reception
    TooHigh     // latency above the maximum
};

struct Check
{
    Outcome outcome = Outcome::NoTimeData;
    double  latency = 0.0; // [s]
};

/// report transmission time minus message reception time [s], corrected for the day wrap at midnight
double latency(double tomr_s, double tort_s);

/// a latency passes if it is between 0 and the maximum, both included
Check check(const boost::optional<float>& tomr_s, const boost::optional<float>& tort_s, float max_value_s);

} // namespace ADSBLatencyHelpers

} // namespace EvaluationRequirement
