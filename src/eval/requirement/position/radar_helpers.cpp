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
LinearFit linearFit(const std::vector<double>& tst_values, const std::vector<double>& ref_values)
{
    traced_assert(!tst_values.empty());
    traced_assert(tst_values.size() == ref_values.size());

    const size_t n = tst_values.size();

    Eigen::MatrixXd x_mat = Eigen::MatrixXd::Ones(n, 2);
    Eigen::MatrixXd y_mat = Eigen::MatrixXd::Ones(n, 1);

    for (size_t cnt = 0; cnt < n; ++cnt)
    {
        x_mat(cnt, 0) = tst_values[cnt];
        y_mat(cnt, 0) = ref_values[cnt];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;

    svd.compute(x_mat, Eigen::ComputeThinV | Eigen::ComputeThinU);
    Eigen::MatrixXd x = svd.solve(y_mat);

    LinearFit fit;
    fit.gain = x(0, 0);
    fit.bias = x(1, 0);

    return fit;
}

} // namespace RadarHelpers
} // namespace EvaluationRequirement
