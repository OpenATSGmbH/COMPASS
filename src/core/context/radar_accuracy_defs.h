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

namespace context
{

/// Detection type for radar target reports.
/// Maps to ASTERIX CAT048 detection type values:
///   0 = Undefined, 1 = PSR only, 2 = SSR only, 3 = SSR+PSR,
///   4/5 = Mode S, 6/7 = Mode S+PSR
enum class DetectionType
{
    Undefined = 0,
    PrimaryOnly,     // single PSR
    ModeAC,          // single SSR
    ModeACCombined,  // SSR + PSR
    ModeS,           // Mode S (All-Call or Roll-Call)
    ModeSCombined    // Mode S + PSR
};

/// Default radar accuracy values per channel.
/// Used as fallback when a data source has no per-source accuracy configured.
struct RadarAccuracyDefaults
{
    double primary_azimuth_stddev{0.05};          // deg
    double primary_range_stddev{120.0};           // m
    double primary_azimuth_stddev_ground{0.05};   // deg (SMR)
    double primary_range_stddev_ground{7.5};      // m   (SMR)
    double secondary_azimuth_stddev{0.025};       // deg
    double secondary_range_stddev{70.0};          // m
    double mode_s_azimuth_stddev{0.02};           // deg
    double mode_s_range_stddev{50.0};             // m
};

/// Convert ASTERIX CAT048 detection type integer (0-7) to DetectionType.
inline DetectionType detectionTypeFromASTERIX(int asterix_type)
{
    switch (asterix_type)
    {
        case 1:          return DetectionType::PrimaryOnly;
        case 2:          return DetectionType::ModeAC;
        case 3:          return DetectionType::ModeACCombined;
        case 4: case 5:  return DetectionType::ModeS;
        case 6: case 7:  return DetectionType::ModeSCombined;
        default:         return DetectionType::Undefined;
    }
}

/// Cartesian position accuracy result from polar radar measurement.
struct RadarPositionAccuracy
{
    double x_stddev{0};  // m
    double y_stddev{0};  // m
    double xy_cov{0};    // m²
};

} // namespace context
