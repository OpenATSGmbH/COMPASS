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

#include <string>
#include <vector>

namespace EvaluationRequirement
{

/**
 * Checks of the radar requirements (Radar Azimuth, Radar Range), free of the evaluation data
 * access, so they can be unit tested on plain values.
 */
namespace RadarHelpers
{

/// statistic of the test vs. reference differences that a radar requirement checks
enum class Statistic : unsigned int
{
    StdDev = 0, // standard deviation of the differences
    Bias,       // azimuth: mean difference, range: bias of the range error model
    Gain        // range only: gain of the range error model
};

/// texts of a checked statistic, per requirement type
struct StatisticInfo
{
    Statistic   statistic;
    std::string label;            // GUI and combo box
    std::string short_name;       // condition result short name
    std::string name;             // condition result name
    std::string units;
    std::string threshold_label;  // requirement table and config widget
    std::string threshold_description;
};

/// statistics of the Radar Azimuth requirement (standard deviation, bias)
const std::vector<StatisticInfo>& azimuthStatistics();
/// statistics of the Radar Range requirement (standard deviation, bias, gain)
const std::vector<StatisticInfo>& rangeStatistics();
/// info of a statistic in one of the lists above, the first entry if not found
const StatisticInfo& statisticInfo(const std::vector<StatisticInfo>& infos, Statistic statistic);

/// range error model of the EUROCONTROL radar standard: test = (1 + gain) * reference + bias
struct RangeErrorModel
{
    double bias_m = 0.0;
    double gain_m_per_nm = 0.0;
};

/// test minus reference azimuth [deg] in [-180, 180), from azimuths in radians
double azimuthDifferenceDeg(double ref_azimuth_rad, double tst_azimuth_rad);

/// a difference passes if its absolute value is at most the threshold
bool passed(double difference, double threshold);

/// value checked against the threshold: the standard deviation, or the absolute bias or gain
double statisticValue(Statistic statistic, double stddev, double bias, double gain);

/// least squares fit of the range error model, needs at least two different reference values
RangeErrorModel rangeErrorModel(const std::vector<double>& ref_values_m, const std::vector<double>& tst_values_m);

} // namespace RadarHelpers

} // namespace EvaluationRequirement
