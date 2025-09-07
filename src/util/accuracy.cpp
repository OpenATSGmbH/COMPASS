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

#include "accuracy.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace Utils
{
namespace Accuracy
{

/**
 */
double GeodeticDistanceInfo::mahalanobisDistance(double eps) const
{
    const double sum_std_dev = std::max(eps, stddev0 + stddev1);
    return distance / sum_std_dev;
}

/**
 */
double GeodeticDistanceInfo::mahalanobisDistanceSqr(double eps) const
{
    const double d_m = mahalanobisDistance(eps);
    return std::pow(d_m, 2);
}

/**
 */
void estimateEllipse(EllipseDef& def,
                     double x_stddev,
                     double y_stddev,
                     double xy_cov)
{
    Eigen::Matrix2f cov_mat;
    cov_mat << std::pow(x_stddev, 2), xy_cov, xy_cov, std::pow(y_stddev, 2);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_mat, Eigen::ComputeThinU); //  | ComputeThinV

    auto singular_values = svd.singularValues();
    auto U = svd.matrixU();

    def.theta_rad = std::atan2(U(1,0), U(0, 0));
    def.rad1      = std::sqrt(singular_values(0));
    def.rad2      = std::sqrt(singular_values(1));
}

/**
 */
double estimateAccuracyAt(const EllipseDef& def, 
                          double bearing_rad)
{
    double x_e, y_e;

    x_e = def.rad1 * std::cos(def.theta_rad) * std::cos(bearing_rad) - def.rad2 * std::sin(def.theta_rad) * std::sin(bearing_rad);
    y_e = def.rad1 * std::sin(def.theta_rad) * std::cos(bearing_rad) + def.rad2 * std::cos(def.theta_rad) * std::sin(bearing_rad);

    return std::sqrt(std::pow(x_e, 2) + std::pow(y_e, 2));
}

}  // namespace Accuracy
}  // namespace Utils
