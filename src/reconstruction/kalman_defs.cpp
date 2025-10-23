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

#include "kalman_defs.h"
#include "timeconv.h"

namespace kalman
{

std::string KalmanUpdateMinimal::print() const
{
    std::stringstream ss;
    ss << "x: \n" << x << "\n"
        << "P: \n" << P << "\n"
        << "t: " << Utils::Time::toString(t) << "\n"
        << "projection center: " << projection_center.x() << " " << projection_center.y() << "\n"
        << "lat: " << lat << "\n"
        << "lon: " << lon << "\n"
        << "Q_var: " << Q_var << "\n"
        << "has wgs84: " << has_wgs84_pos << "\n"
        << "valid: " << valid;

    return ss.str();
}

} // namespace kalman
