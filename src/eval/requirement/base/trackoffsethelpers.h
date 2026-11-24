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

namespace EvaluationRequirement
{

/**
 * Along-track and across-track components of the offset of a test position from the reference
 * position (Position Along, Position Across, Position Latency), free of the evaluation data
 * access, so they can be unit tested on plain values.
 */
namespace TrackOffsetHelpers
{

struct TrackOffset
{
    double along  = 0.0; // [m], positive if the test position is ahead of the reference position
    double across = 0.0; // [m], positive if the test position is right of the reference track
};

/**
 * offset_angle_rad is the mathematical angle of the offset (atan2(north, east), as returned by
 * Transformation::distanceAngleCart()), track_angle_deg the reference track angle (true north,
 * clockwise, degrees).
 */
TrackOffset trackOffset(double distance_m, double offset_angle_rad, double track_angle_deg);

} // namespace TrackOffsetHelpers

} // namespace EvaluationRequirement
