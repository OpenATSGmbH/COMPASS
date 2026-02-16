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
#include <tuple>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace Utils
{
namespace Number
{

/// @brief Returns random number between min and max
extern float randomNumber(float min, float max);
/// @brief Returns rounded number to nearest integer
extern float roundToNearest(float num);
extern double round(float num, unsigned int precision);
extern double round(double num, unsigned int precision);
extern double roundToClosestPowerOf10(double value);


extern void calculateWeightedAverageAndStdDev(
    const std::vector<double>& values, const std::vector<double>& std_devs, double& avg, double& std_dev);
void addWithWeightedAverage(double value1, double std_dev1, unsigned int value1_cnt,
                               double value2, double std_dev2, unsigned int value2_cnt,
                               double& weighted_avg, double& weighted_std_dev, unsigned int& weighted_cnt);

extern unsigned int numDecimals(double v, unsigned int dec_max = 9);

/// @brief Returns angle (degrees) calculated from given values
extern double calculateAngle(double degrees, double minutes, double seconds);

extern double calculateMinAngleDifference(double a_deg, double b_deg);

extern double deg2rad(double angle);
extern double rad2deg(double angle);
extern double rebaseAngle(double angle_deg, double new_base_deg);

extern double knots2MPS(double knots);
extern double mps2Knots(double mps);

extern std::pair<double, double> bearing2Vec(double bearing_deg);
extern double vec2Bearing(double dx, double dy);
extern std::pair<double, double> speedAngle2SpeedVec(double speed_mps, double bearing_deg);
extern std::pair<double, double> speedVec2SpeedAngle(double vx_mps, double vy_mps);

extern double interpolateBearing(double x0, double y0, 
                                 double x1, double y1, 
                                 double bearing0_deg, double bearing1_deg, 
                                 double factor);

//extern void convert(const std::string& conversion_type, NullableVector<unsigned int>& array_list);

extern unsigned int dsIdFrom (unsigned int sac, unsigned int sic);
extern unsigned int sacFromDsId (unsigned int ds_id);
extern unsigned int sicFromDsId (unsigned int ds_id);

extern unsigned long recNumAddDBContId (unsigned long rec_num_wo_dbcont_id, unsigned int dbcont_id);
extern unsigned long recNumGetWithoutDBContId (unsigned long rec_num);
extern unsigned int recNumGetDBContId (unsigned long rec_num);

template <typename T>
 std::tuple<double,double,double,double> getStatistics (const T& values);

extern std::tuple<double,double,double,double> getStatistics (const std::vector<double>& values);

// template <typename T>
// std::pair<double,double> calculateMeanStdDev (T values, float remove_top=0.0);

std::pair<double,double> calculateMeanStdDev (std::vector<double> values, float remove_top=0.0);

extern double calculateMedian(std::vector<double> data); // worked-on copy
extern double calculateIQR(std::vector<double> data);
extern double calculateMAD(std::vector<double> data);
extern std::tuple<double,double,double> getMedianStatistics (const std::vector<double>& values);

extern std::pair<unsigned int, unsigned int> computeGeoWindowResolution(
    double lat_min, double lat_max, double lon_min, double lon_max,
    double grid_max_resolution  = 0.001, unsigned int max_num_cells = 500);

extern double convertLatitude(const std::string& latStr, bool& ok);
extern double convertLongitude(const std::string& lonStr, bool& ok);

struct AzimuthBiasResult
{
    bool valid {false};
    double azimuth_bias_deg {0};
};

extern AzimuthBiasResult estimateAzimuthBias(
    const std::vector<double>& ref_azimuths_deg,
    const std::vector<double>& tst_azimuths_deg,
    double max_diff_deg = 30.0,
    double max_bias_deg = 30.0);

struct RangeBiasGainResult
{
    bool valid {false};
    double range_bias_m {0};
    double range_gain {0};
};

extern RangeBiasGainResult estimateRangeBiasGain(
    const std::vector<double>& tst_ranges_m,
    const std::vector<double>& ref_ranges_m,
    double max_range_ratio_diff = 0.1,
    double max_gain = 0.05,
    double max_bias_m = 5000.0);

//template <typename T>
//double getStatistics(const std::vector<>);

}  // namespace Number

}  // namespace Utils

