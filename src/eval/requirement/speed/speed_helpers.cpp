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

#include "requirement/speed/speed_helpers.h"

#include <cmath>

namespace EvaluationRequirement
{
namespace SpeedHelpers
{

/**
 */
float threshold(double ref_speed_ms, const Params& p)
{
    // percent-based threshold on the reference speed, the "actual speed" of the standards
    float percent_threshold_value = static_cast<float>(ref_speed_ms) * p.threshold_percent / 100.0f;

    if (p.use_percent_if_higher && percent_threshold_value > p.threshold_value)
        return percent_threshold_value;

    return p.threshold_value;
}

/**
 */
Check check(double ref_speed_ms, const boost::optional<float>& tst_speed_ms, const Params& p)
{
    Check c;

    if (!tst_speed_ms.has_value())
        return c;

    c.code      = 0;
    c.diff      = std::fabs(ref_speed_ms - *tst_speed_ms);
    c.threshold = threshold(ref_speed_ms, p);
    c.outcome   = compareValue(c.diff, c.threshold, p.check_type) ? Outcome::Passed : Outcome::Failed;

    return c;
}

} // namespace SpeedHelpers
} // namespace EvaluationRequirement
