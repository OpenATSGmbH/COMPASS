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

#include "requirement/dubious/dubious_helpers.h"

#include "global.h"
#include "number.h"
#include "traced_assert.h"

#include <cmath>

namespace EvaluationRequirement
{
namespace DubiousHelpers
{

/**
 */
bool UpdateFlags::dubious() const
{
    return reported_speed_ms || calculated_speed_kts || acceleration || turnrate || rocd;
}

/**
 */
std::vector<UpdateFlags> checkUpdates(const std::vector<Update>& updates, const Params& p)
{
    std::vector<UpdateFlags> flags(updates.size());

    const double max_groundspeed_ms = p.max_groundspeed_kts * KNOTS2M_S;

    for (size_t cnt = 0; cnt < updates.size(); ++cnt)
    {
        const Update& u = updates[cnt];
        UpdateFlags&  f = flags[cnt];

        if (p.use_max_groundspeed && u.speed_ms.has_value() && *u.speed_ms > max_groundspeed_ms)
            f.reported_speed_ms = *u.speed_ms;

        if (cnt == 0)
            continue;

        const Update& last = updates[cnt - 1];

        const double time_diff = u.time_s - last.time_s;
        traced_assert(time_diff >= 0);

        if (p.use_max_groundspeed && p.check_calculated_speed && !f.reported_speed_ms.has_value() && time_diff > 0)
        {
            double spd_kts = M_S2KNOTS * u.distance_to_previous_m / time_diff;

            if (spd_kts > p.max_groundspeed_kts)
                f.calculated_speed_kts = spd_kts;
        }

        if (time_diff < p.min_comparison_time || time_diff > p.max_comparison_time)
            continue;

        if (p.use_max_acceleration && u.speed_ms.has_value() && last.speed_ms.has_value())
        {
            double acceleration = std::fabs(*u.speed_ms - *last.speed_ms) / time_diff;

            if (acceleration > p.max_acceleration)
                f.acceleration = acceleration;
        }

        if (p.use_max_turnrate && u.track_angle_deg.has_value() && last.track_angle_deg.has_value())
        {
            double turnrate = Utils::Number::calculateMinAngleDifference(*last.track_angle_deg, *u.track_angle_deg)
                              / time_diff;

            if (std::fabs(turnrate) > p.max_turnrate)
                f.turnrate = turnrate;
        }

        if (p.use_max_rocd && u.mode_c_ft.has_value() && last.mode_c_ft.has_value())
        {
            double rocd = std::fabs(*u.mode_c_ft - *last.mode_c_ft) / time_diff;

            if (rocd > p.max_rocd)
                f.rocd = rocd;
        }
    }

    return flags;
}

/**
 */
bool isDubious(unsigned int num_dubious, unsigned int num_updates, float dubious_prob)
{
    return num_updates && (float)num_dubious / (float)num_updates > dubious_prob;
}

} // namespace DubiousHelpers
} // namespace EvaluationRequirement
