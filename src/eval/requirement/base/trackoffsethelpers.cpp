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

#include "requirement/base/trackoffsethelpers.h"

#include <cmath>

namespace EvaluationRequirement
{
namespace TrackOffsetHelpers
{

/**
 */
TrackOffset trackOffset(double distance_m, double offset_angle_rad, double track_angle_deg)
{
    const double east  = distance_m * std::cos(offset_angle_rad);
    const double north = distance_m * std::sin(offset_angle_rad);

    const double track_rad = track_angle_deg * M_PI / 180.0;

    TrackOffset offset;
    offset.along  = east * std::sin(track_rad) + north * std::cos(track_rad);
    offset.across = east * std::cos(track_rad) - north * std::sin(track_rad);

    return offset;
}

} // namespace TrackOffsetHelpers
} // namespace EvaluationRequirement
