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

#include "requirement/latency/adsblatency_helpers.h"

namespace EvaluationRequirement
{
namespace ADSBLatencyHelpers
{

/**
 */
double latency(double tomr_s, double tort_s)
{
    double latency = tort_s - tomr_s;

    // both times are seconds of day
    if (latency < -43200.0)
        latency += 86400.0;
    else if (latency > 43200.0)
        latency -= 86400.0;

    return latency;
}

/**
 */
Check check(const boost::optional<float>& tomr_s, const boost::optional<float>& tort_s, float max_value_s)
{
    Check c;

    if (!tomr_s.has_value() || !tort_s.has_value())
        return c;

    c.latency = latency(tomr_s.value(), tort_s.value());
    c.code    = 0;

    if (c.latency < 0.0)
    {
        c.outcome = Outcome::Negative;
        c.code    = NegativeLatency;
    }
    else if (c.latency > max_value_s)
        c.outcome = Outcome::TooHigh;
    else
        c.outcome = Outcome::Passed;

    return c;
}

} // namespace ADSBLatencyHelpers
} // namespace EvaluationRequirement
