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

#include "requirement/position/radar_helpers.h"

#include "global.h"
#include "number.h"
#include "traced_assert.h"

#include <Eigen/Dense>

#include <cmath>

namespace EvaluationRequirement
{
namespace RadarHelpers
{

/**
 */
double azimuthDifferenceDeg(double ref_azimuth_rad, double tst_azimuth_rad)
{
    return Utils::Number::calculateMinAngleDifference(ref_azimuth_rad * 180.0 / M_PI,
                                                      tst_azimuth_rad * 180.0 / M_PI);
}

/**
 */
bool passed(double difference, double threshold)
{
    return std::fabs(difference) <= threshold;
}

/**
 */
const std::vector<StatisticInfo>& azimuthStatistics()
{
    static const std::vector<StatisticInfo> infos {
        { Statistic::StdDev, "Standard Deviation", "AzmSD", "Standard deviation of the azimuth difference", "deg",
          "Maximum Standard Deviation [deg]",
          "The standard deviation of the azimuth difference between the test target reports and the reference must be smaller" },
        { Statistic::Bias, "Bias", "AzmBias", "Absolute mean of the azimuth difference", "deg",
          "Maximum Absolute Bias [deg]",
          "The absolute mean of the azimuth difference between the test target reports and the reference must be smaller" } };

    return infos;
}

/**
 */
const std::vector<StatisticInfo>& rangeStatistics()
{
    static const std::vector<StatisticInfo> infos {
        { Statistic::StdDev, "Standard Deviation", "RngSD", "Standard deviation of the slant range difference", "m",
          "Maximum Standard Deviation [m]",
          "The standard deviation of the slant range difference between the test target reports and the reference must be smaller" },
        { Statistic::Bias, "Bias", "RngBias", "Absolute slant range bias", "m",
          "Maximum Absolute Bias [m]",
          "The absolute bias of the slant range error model (test = (1 + gain) * reference + bias) must be smaller" },
        { Statistic::Gain, "Gain", "RngGain", "Absolute slant range gain", "m/NM",
          "Maximum Absolute Gain [m/NM]",
          "The absolute gain of the slant range error model (test = (1 + gain) * reference + bias) must be smaller" } };

    return infos;
}

/**
 */
const StatisticInfo& statisticInfo(const std::vector<StatisticInfo>& infos, Statistic statistic)
{
    traced_assert(!infos.empty());

    for (const auto& info : infos)
        if (info.statistic == statistic)
            return info;

    return infos.front();
}

/**
 */
double statisticValue(Statistic statistic, double stddev, double bias, double gain)
{
    switch (statistic)
    {
        case Statistic::Bias:
            return std::fabs(bias);
        case Statistic::Gain:
            return std::fabs(gain);
        default:
            return stddev;
    }
}

/**
 */
RangeErrorModel rangeErrorModel(const std::vector<double>& ref_values_m, const std::vector<double>& tst_values_m)
{
    traced_assert(!ref_values_m.empty());
    traced_assert(ref_values_m.size() == tst_values_m.size());

    const size_t n = ref_values_m.size();

    Eigen::MatrixXd x_mat = Eigen::MatrixXd::Ones(n, 2);
    Eigen::MatrixXd y_mat = Eigen::MatrixXd::Ones(n, 1);

    for (size_t cnt = 0; cnt < n; ++cnt)
    {
        x_mat(cnt, 0) = ref_values_m[cnt];
        y_mat(cnt, 0) = tst_values_m[cnt];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;

    svd.compute(x_mat, Eigen::ComputeThinV | Eigen::ComputeThinU);
    Eigen::MatrixXd x = svd.solve(y_mat);

    RangeErrorModel model;
    model.gain_m_per_nm = (x(0, 0) - 1.0) * NM2M;
    model.bias_m        = x(1, 0);

    return model;
}

} // namespace RadarHelpers
} // namespace EvaluationRequirement
