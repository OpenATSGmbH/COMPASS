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

#include "requirement/trackangle/trackangle_helpers.h"
#include "number.h"

#include <cmath>

namespace EvaluationRequirement
{
namespace TrackAngleHelpers
{

/**
 */
Check check(double ref_speed_ms, double ref_angle_deg, const boost::optional<double>& tst_angle_deg,
            const Params& p)
{
    Check c;

    if (p.use_minimum_speed && ref_speed_ms < p.minimum_speed)
    {
        c.outcome = Outcome::RefSpeedTooLow;
        return c;
    }

    if (!tst_angle_deg.has_value())
        return c;

    c.diff    = Utils::Number::calculateMinAngleDifference(ref_angle_deg, *tst_angle_deg);
    c.outcome = compareValue(std::fabs(c.diff), p.threshold, p.check_type) ? Outcome::Passed : Outcome::Failed;

    return c;
}

} // namespace TrackAngleHelpers
} // namespace EvaluationRequirement
