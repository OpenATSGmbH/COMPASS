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

#include "number.h"
#include "logger.h"
#include "traced_assert.h"

#include "tbbhack.h"

#include <QString>

#include <Eigen/Core>

#include <cmath>
#include <stdlib.h>
#include <algorithm>
#include <numeric>

using namespace std;

namespace Utils
{
namespace Number
{
float randomNumber(float min, float max)
{
    return (float)min + rand() / (RAND_MAX / (max - min) + 1);
}

float roundToNearest(float num) { return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5); }

double round(float num, unsigned int precision)
{
    return std::round(num * std::pow(10, precision)) / std::pow(10, precision);
}

double round(double num, unsigned int precision)
{
    return std::round(num * std::pow(10, precision)) / std::pow(10, precision);
}

double roundToClosestPowerOf10(double value) {
    if (value == 0.0f) {
        return 0.0f; // Edge case: return 0 if input is 0
    }

            // Calculate the logarithm base 10 of the absolute value
    double log10_value = std::log10(std::fabs(value));

            // Round the log10_value to the lower integer
    double rounded_log10 = std::round(log10_value);

            // Calculate the power of 10 corresponding to the rounded logarithm
    double power_of_10 = std::pow(10.0f, rounded_log10);

            // Preserve the sign of the original value
    if (value < 0) {
        power_of_10 = -power_of_10;
    }

    return power_of_10;
}

// Function to calculate weighted average and standard deviation of a data series
void calculateWeightedAverageAndStdDev(const std::vector<double>& values, const std::vector<double>& std_devs,
                                            double& avg, double& std_dev) {
    double weighted_sum = 0.0;
    double weight_sum = 0.0;

    for (size_t i = 0; i < values.size(); ++i) {
        double weight = 1.0 / (std_devs[i] * std_devs[i]);
        weighted_sum += values[i] * weight;
        weight_sum += weight;
    }

    avg = weighted_sum / weight_sum;
    std_dev = sqrt(1.0 / weight_sum);
}

const double min_std_dev = 1E-6;
const unsigned int good_sample_size = 5;

void addWithWeightedAverage(double value1, double std_dev1, unsigned int value1_cnt,
                               double value2, double std_dev2, unsigned int value2_cnt,
                               double& weighted_avg, double& weighted_std_dev, unsigned int& weighted_cnt)
{
    if (value1_cnt < good_sample_size && value2_cnt < good_sample_size) // both small
    {
        weighted_avg = (value1 + value2) / 2.0;
        weighted_std_dev = (std_dev1 + std_dev2) / 2.0;
    }
    else if ((value1_cnt >= good_sample_size && value2_cnt < good_sample_size)
               || (value1_cnt < good_sample_size && value2_cnt >= good_sample_size)) // one good, one bad
    {
        double weight1 = (double) value1_cnt;
        double weight2 = (double) value2_cnt;

        double new_weighted_avg = (value1 * weight1 + value2 * weight2) / (weight1 + weight2);
        traced_assert(std::isfinite(new_weighted_avg));

        double new_weighted_stddev = (std_dev1 * weight1 + std_dev2 * weight2) / (weight1 + weight2);
        traced_assert(std::isfinite(new_weighted_stddev));

        weighted_avg = new_weighted_avg;
        weighted_std_dev = new_weighted_stddev; // combined standard deviation
    }
    else
    {
        if (std_dev1 < min_std_dev)
            std_dev1 = min_std_dev;

        if (std_dev2 < min_std_dev)
            std_dev2 = min_std_dev;

        // Calculate weights based on standard deviations
        double weight1 = (double) value1_cnt / (std_dev1 * std_dev1);
        double weight2 = (double) value2_cnt / (std_dev2 * std_dev2);

                // Calculate the weighted average
        double new_weighted_avg = (value1 * weight1 + value2 * weight2) / (weight1 + weight2);

        if (!std::isfinite(new_weighted_avg))
        {
            logerr << "new_weighted_avg " << new_weighted_avg
                   << " stddevsum " << (std_dev1 * std_dev1)
                   << " weightvalsum " << (value1 * weight1 + value2 * weight2)
                   << " weightsum " << (weight1 + weight2);

            return;
        }

        weighted_avg = new_weighted_avg;
        weighted_std_dev = sqrt(1.0 / (weight1 + weight2)); // combined standard deviation
    }

    weighted_cnt = value1_cnt + value2_cnt;
}

unsigned int numDecimals(double v, unsigned int dec_max)
{
    auto str = QString::number(v, 'f', dec_max);

    //strange cases => return max dec for safety
    if (str.isEmpty() || str.count('.') > 1)
        return dec_max;

    int n   = str.count();
    int idx = str.lastIndexOf('.');

    //full number
    if (idx < 0)
        return 0;

    //strange case => return max dec for safety
    if (idx == n - 1)
        return dec_max;

    int idx_end = -1;
    for (int i = n - 1; i > idx; --i)
    {
        if (str[ i ] != '0')
        {
            idx_end = i;
            break;
        }
    }

    //only zeros after dec
    if (idx_end == -1)
        return 0;

    traced_assert(idx_end >= idx);

    return idx_end - idx;
}

double calculateAngle(double degrees, double minutes, double seconds)
{
    return degrees + minutes / 60.0 + seconds / 3600.0;
}

double calculateMinAngleDifference(double a_deg, double b_deg)
{
    //double distance = fmod(fmod(a_deg - b_deg, 360) + 540, 360) - 180;
    // return distance;

    double diff = b_deg - a_deg;
    // Normalize the difference to the range [-180, 180)
    while (diff < -180.0) diff += 360.0;
    while (diff >= 180.0) diff -= 360.0;

    return diff;
}

double deg2rad(double angle)
{
    return angle * M_PI / 180.0;
}

double rad2deg(double angle)
{
    return angle * 180.0 / M_PI;
}

namespace
{
    double normalizeAngle(double angle_deg)
    {
        if (angle_deg > 180.0)
            return angle_deg - 360.0;
        if (angle_deg < -180.0)
            return 360.0 + angle_deg;
        return angle_deg;
    };
}

/**
 * Rotates the given angle to a new 0-basis angle.
 * @param angle_deg Angle in [-180,180]
 * @param new_base_deg New 0-basis in [-180,180] 
 */
double rebaseAngle(double angle_deg, double new_base_deg)
{
    return normalizeAngle(angle_deg - new_base_deg);
}

/**
 * Interpolates the bearings given at the two positions at the given factor.
 * @param x0 First point x
 * @param y0 First point y
 * @param x1 Second point x
 * @param y1 Second point y
 * @param bearing0_deg Bearing at the first point in [-180,180]
 * @param bearing1_deg Bearing at the second point in [-180,180]
 * @param factor Interpolation factor in [0,1]
 */
double interpolateBearing(double x0, double y0, 
                          double x1, double y1, 
                          double bearing0_deg, double bearing1_deg, 
                          double factor)
{
    Eigen::Vector2d t(x1 - x0, y1 - y0);
    double tn = t.norm();

    //numerically instable?
    if (tn < 1e-06)
        return bearing0_deg;

    //normalized tangent estimate
    t /= tn;

    //bearing of tangent
    double angle_t_deg = rad2deg(atan2(t.x(), t.y()));

    //rotate to tangent angle as origin
    double da0 = rebaseAngle(bearing0_deg, angle_t_deg);
    double da1 = rebaseAngle(bearing1_deg, angle_t_deg);

    //which side of tangent are we on?
    bool sign0 = (da0 >= 0);
    bool sign1 = (da1 >= 0);

    if (sign0 != sign1)
    {
        //speed vecs on different sides of tangent => directly interpolate between angles
        double dinterp = da0 + (da1 - da0) * factor;

        //rotate back interpolated angle to bearing 0 as origin and convert back to deg
        return rebaseAngle(dinterp, -angle_t_deg);
    }
    
    //speed vecs approximately on same side of tangent => interpolate with minimum angle
    double diff = Number::calculateMinAngleDifference(bearing1_deg, bearing0_deg);
    return bearing0_deg + diff * factor;
}

unsigned int dsIdFrom (unsigned int sac, unsigned int sic)
{
    return sac * 255 + sic;
}

unsigned int sacFromDsId (unsigned int ds_id)
{
    return ds_id / 255;
}
unsigned int sicFromDsId (unsigned int ds_id)
{
    return ds_id % 255;
}

double knots2MPS(double knots)
{
    return knots * 0.514444;
}

double mps2Knots(double mps)
{
    return mps * 1.94384;
}

std::pair<double, double> bearing2Vec(double bearing_deg)
{
    double bearing_rad = deg2rad(bearing_deg);
    return std::make_pair(std::sin(bearing_rad), std::cos(bearing_rad));
}

double vec2Bearing(double dx, double dy)
{
    return rad2deg(std::atan2(dx, dy));
}

std::pair<double, double> speedAngle2SpeedVec(double speed_mps, 
                                              double bearing_deg)
{
    auto vec = bearing2Vec(bearing_deg);
    vec.first  *= speed_mps;
    vec.second *= speed_mps;

    return vec;
}

std::pair<double, double> speedVec2SpeedAngle(double vx_mps, 
                                              double vy_mps)
{
    double speed = Eigen::Vector2d(vx_mps, vy_mps).norm();
    double angle = vec2Bearing(vx_mps / speed, vy_mps / speed);

    return std::make_pair(speed, angle);
}

unsigned long recNumAddDBContId (unsigned long rec_num_wo_dbcont_id, unsigned int dbcont_id)
{
    traced_assert(dbcont_id < 256); // 8bit max
    traced_assert(rec_num_wo_dbcont_id < (1ul << 56)); // 56bit max
    return rec_num_wo_dbcont_id << 8 | dbcont_id;
}

unsigned long recNumGetWithoutDBContId (unsigned long rec_num)
{
    return rec_num >> 8;
}

unsigned int recNumGetDBContId (unsigned long rec_num)
{
    return rec_num & 0xFF; // first byte
}

template <typename T>
std::tuple<double,double,double,double> getStatistics (const T& values)
{
    double mean=0, stddev=0, min=0, max=0;

    if (values.empty()) {
        logerr << "empty vector";

        return {std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN()};
    }

    double sum = std::accumulate(values.begin(), values.end(), 0.0);

    mean = sum / values.size();

    std::vector<double> diff(values.size());
    std::transform(values.begin(), values.end(), diff.begin(),
                   [mean](const double val) { return val - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stddev = std::sqrt(sq_sum / values.size());

    min = *std::min_element(values.begin(), values.end());
    max = *std::max_element(values.begin(), values.end());

    return std::tuple<double,double,double,double>(mean, stddev, min, max);
}


template std::tuple<double,double,double,double> getStatistics (const tbb::concurrent_vector<double>& values);

std::tuple<double,double,double,double> getStatistics (const std::vector<double>& values)
{
    if (values.empty()) {
        logerr << "empty vector";

        return {std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN()};
    }

    double mean=0, stddev=0, min=0, max=0;

    double sum = std::accumulate(values.begin(), values.end(), 0.0);

    mean = sum / values.size();

    std::vector<double> diff(values.size());
    std::transform(values.begin(), values.end(), diff.begin(),
                   [mean](const double val) { return val - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stddev = std::sqrt(sq_sum / values.size());

    min = *std::min_element(values.begin(), values.end());
    max = *std::max_element(values.begin(), values.end());

    return std::tuple<double,double,double,double>(mean, stddev, min, max);
}

//template <typename T>
std::pair<double,double> calculateMeanStdDev (std::vector<double> values, float remove_top)
{
    double mean=0, stddev=0;

    if (values.empty())
    {
        logerr << "empty vector";

        return {std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN()};
    }

    unsigned int num_to_remove = static_cast<size_t>(std::floor(values.size() * remove_top));

    unsigned int num_to_check = values.size() - num_to_remove;

    if (num_to_check == 0)
    {
        logerr << "remove all values in vector";
        return {std::numeric_limits<double>::signaling_NaN(), std::numeric_limits<double>::signaling_NaN()};
    }

    std::sort(values.begin(), values.end());

    auto last_val_to_check_it = values.begin() + num_to_check;

    //assert (values.begin() != last_val_to_check_it);

    double sum = std::accumulate(values.begin(), last_val_to_check_it, 0.0);

    mean = sum / num_to_check;

    if (num_to_check == 1)
        return {mean, 0};

    std::vector<double> diff(num_to_check);
    std::transform(values.begin(), last_val_to_check_it, diff.begin(),
                   [mean](const double val) { return val - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    stddev = std::sqrt(sq_sum / num_to_check);

    return std::pair<double,double>(mean, stddev);
}

// Function to calculate the median of a vector
double calculateMedian(std::vector<double> data) {

    size_t size = data.size();

    if (size == 0) {
        logerr << "empty vector";

        return std::numeric_limits<double>::signaling_NaN();
    }
    std::sort(data.begin(), data.end());
    if (size % 2 == 0) {
        // Even number of elements
        return (data[size / 2 - 1] + data[size / 2]) / 2.0;
    } else {
        // Odd number of elements
        return data[size / 2];
    }
}

// Function to calculate the interquartile range (IQR)
double calculateIQR(std::vector<double> data) {
    size_t size = data.size();
    if (size < 4) {
        logerr << "too few data opints to compute IQR";
        return std::numeric_limits<double>::signaling_NaN();
    }
    std::sort(data.begin(), data.end());

    size_t mid = size / 2;
    std::vector<double> lower_half(data.begin(), data.begin() + mid);
    std::vector<double> upper_half;

    if (size % 2 == 0) {
        upper_half = std::vector<double>(data.begin() + mid, data.end());
    } else {
        upper_half = std::vector<double>(data.begin() + mid + 1, data.end());
    }

    double q1 = calculateMedian(lower_half);
    double q3 = calculateMedian(upper_half);

    return q3 - q1;
}

// Function to calculate the median absolute deviation (MAD)
double calculateMAD(std::vector<double> data) {
    if (data.empty()) {
        logerr << "empty vector";

        return std::numeric_limits<double>::signaling_NaN();
    }
    double median = calculateMedian(data);

    // Calculate absolute deviations from the median
    std::vector<double> abs_deviations;
    abs_deviations.reserve(data.size());
    for (double value : data) {
        abs_deviations.push_back(std::abs(value - median));
    }

    // Compute the median of absolute deviations
    return calculateMedian(abs_deviations);
}

std::tuple<double,double,double> getMedianStatistics (const std::vector<double>& values)
{
    try {
        double median = calculateMedian(values);
        double iqr = calculateIQR(values);
        double mad = calculateMAD(values);

        // std::cout << "Median: " << median << std::endl;
        // std::cout << "Interquartile Range (IQR): " << iqr << std::endl;
        // std::cout << "Median Absolute Deviation (MAD): " << mad << std::endl;

        return std::tuple<double,double,double>{median, iqr, mad};
    } catch (const std::domain_error& e) {
        logerr << "start" << e.what();
        return {std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN(),
                std::numeric_limits<double>::signaling_NaN()};
    }
}

/// Computes the optimal number of grid cells in latitude and longitude,
/// subject to a maximum resolution and a cap on the total number of cells.
/// @param lat_min    Minimum latitude (degrees)
/// @param lat_max    Maximum latitude (degrees)
/// @param lon_min    Minimum longitude (degrees)
/// @param lon_max    Maximum longitude (degrees)
/// @returns           A pair {num_lat_cells, num_lon_cells}
std::pair<unsigned int, unsigned int> computeGeoWindowResolution(
    double lat_min, double lat_max, double lon_min, double lon_max,
    double grid_max_resolution, unsigned int max_num_cells)
{
    // Compute extents
    double lat_extent = std::abs(lat_max - lat_min);
    double lon_extent = std::abs(lon_max - lon_min);

    // 2) Guard against degenerate spans
    if (lat_extent == 0.0)
        lat_extent = grid_max_resolution;
    if (lon_extent == 0.0)
        lon_extent = grid_max_resolution;

    // 3) Figure out the smallest resolution that won't exceed max_num_cells
    //    along *either* axis
    //
    //    To limit latitude cells to max_num_cells:
    //       resolution >= lat_extent / max_num_cells
    //    To limit longitude cells to max_num_cells:
    //       resolution >= lon_extent / max_num_cells
    //
    double min_res_lat = lat_extent / static_cast<double>(max_num_cells);
    double min_res_lon = lon_extent / static_cast<double>(max_num_cells);

    // 4) Final resolution is the worst of:
    //       - your finest allowable (grid_max_resolution)
    //       - the per-axis minima above
    double resolution = std::max({
        grid_max_resolution,
        min_res_lat,
        min_res_lon
    });

    // 5) Compute integer cell counts (rounding up)
    unsigned int num_lat_cells =
        static_cast<unsigned int>(std::ceil(lat_extent / resolution));
    unsigned int num_lon_cells =
        static_cast<unsigned int>(std::ceil(lon_extent / resolution));

    // 6) Ensure at least one cell in each dimension
    num_lat_cells = std::max<unsigned int>(1, num_lat_cells);
    num_lon_cells = std::max<unsigned int>(1, num_lon_cells);

    return { num_lat_cells, num_lon_cells };
}


/**
 * @brief Converts a latitude string "DD:MM:SS.SSS[NS]" (or without hemisphere suffix) to decimal degrees.
 *
 * @param lat_str Latitude string, e.g. "47:26:58.9526N" or "47:26:58.9526"
 * @param ok Reference to a boolean flag set to true on success, false on format error
 * @return double Signed decimal degrees (positive for N, negative for S), undefined if !ok
 */
double convertLatitude(const std::string& lat_str, bool& ok)
{
    ok = false;
    if (lat_str.empty())
        return 0.0;

    // Determine hemisphere: default to North if not specified
    char hemisphere = 'N';
    std::string core = lat_str;
    char last_char = lat_str.back();
    if (last_char == 'N' || last_char == 'S') {
        hemisphere = last_char;
        core = lat_str.substr(0, lat_str.size() - 1);
    }

    std::istringstream ss(core);
    double degrees = 0.0, minutes = 0.0, seconds = 0.0;
    char sep1 = '\0', sep2 = '\0';
    if (!(ss >> degrees >> sep1 >> minutes >> sep2 >> seconds) || sep1 != ':' || sep2 != ':') {
        return 0.0;
    }

    double decimal_degrees = degrees + (minutes / 60.0) + (seconds / 3600.0);
    ok = true;
    return (hemisphere == 'S') ? -decimal_degrees : decimal_degrees;
}

/**
 * @brief Converts a longitude string "DDD:MM:SS.SSS[EW]" (or without hemisphere suffix) to decimal degrees.
 *
 * @param lon_str Longitude string, e.g. "008:34:25.5397E" or "008:34:25.5397"
 * @param ok Reference to a boolean flag set to true on success, false on format error
 * @return double Signed decimal degrees (positive for E, negative for W), undefined if !ok
 */
double convertLongitude(const std::string& lon_str, bool& ok) {
    ok = false;
    if (lon_str.empty())
        return 0.0;

    // Determine hemisphere: default to East if not specified
    char hemisphere = 'E';
    std::string core = lon_str;
    char last_char = lon_str.back();
    if (last_char == 'E' || last_char == 'W') {
        hemisphere = last_char;
        core = lon_str.substr(0, lon_str.size() - 1);
    }

    std::istringstream ss(core);
    double degrees = 0.0, minutes = 0.0, seconds = 0.0;
    char sep1 = '\0', sep2 = '\0';
    if (!(ss >> degrees >> sep1 >> minutes >> sep2 >> seconds) || sep1 != ':' || sep2 != ':') {
        return 0.0;
    }

    double decimal_degrees = degrees + (minutes / 60.0) + (seconds / 3600.0);
    ok = true;
    return (hemisphere == 'W') ? -decimal_degrees : decimal_degrees;
}
}  // namespace Number

//void convert(const std::string& conversion_type, NullableVector<unsigned int>& array_list) {}

}  // namespace Utils
