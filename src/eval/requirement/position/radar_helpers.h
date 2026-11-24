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

#include <vector>

namespace EvaluationRequirement
{

/**
 * Checks of the radar requirements (Radar Azimuth, Radar Range), free of the evaluation data
 * access, so they can be unit tested on plain values.
 */
namespace RadarHelpers
{

struct LinearFit
{
    double gain = 1.0;
    double bias = 0.0;
};

/// test minus reference azimuth [deg] in [-180, 180), from azimuths in radians
double azimuthDifferenceDeg(double ref_azimuth_rad, double tst_azimuth_rad);

/// a difference passes if its absolute value is at most the threshold
bool passed(double difference, double threshold);

/// least squares fit ref = gain * tst + bias, needs at least one value pair
LinearFit linearFit(const std::vector<double>& tst_values, const std::vector<double>& ref_values);

} // namespace RadarHelpers

} // namespace EvaluationRequirement
