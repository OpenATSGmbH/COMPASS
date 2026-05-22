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
#include "global.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <osgEarth/GeoMath>

namespace Utils
{
namespace Accuracy
{

/********************************************************************************
 * GeodeticDistanceInfo
 ********************************************************************************/

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
double GeodeticDistanceInfo::likelihood(double eps) const
{
    const double sum_std_dev         = std::max(eps, stddev0 + stddev1);
    const double sum_std_dev_sqr     = std::pow(sum_std_dev, 2);
    const double distance_sqr        = std::pow(distance, 2);
    const double sum_std_dev_sqr_inv = 1.0 / sum_std_dev_sqr;
    const double factor              = 0.5 * M_1_PI * sum_std_dev_sqr_inv;
    const double exponent            = -0.5 * distance_sqr * sum_std_dev_sqr_inv;
    
    return factor * std::exp(exponent);
}

/********************************************************************************
 * GeoInfo
 ********************************************************************************/

/**
*/
double GeoInfo::geodeticDistance(const GeoInfo& other) const
{
    return osgEarth::GeoMath::distance(lat       * DEG2RAD,
                                       lon       * DEG2RAD,
                                       other.lat * DEG2RAD, 
                                       other.lon * DEG2RAD);
}

/**
*/
double GeoInfo::bearing(const GeoInfo& other) const
{
    return osgEarth::GeoMath::bearing(lat       * DEG2RAD,
                                      lon       * DEG2RAD,
                                      other.lat * DEG2RAD, 
                                      other.lon * DEG2RAD);
}

/********************************************************************************
 * GeoAccInfo
 ********************************************************************************/

/**
 */
GeodeticDistanceInfo GeoAccInfo::distance(const GeoAccInfo& other) const
{
    double d = geodeticDistance(other);
    double b = bearing(other);

    Utils::Accuracy::EllipseDef acc_ell;

    double stddev0 = x_stddev;
    if (x_stddev != y_stddev)
    {
        Utils::Accuracy::estimateEllipse(acc_ell, x_stddev, y_stddev, xy_cov);
        stddev0 = Utils::Accuracy::estimateAccuracyAt(acc_ell, b);
    }

    double stddev1 = other.x_stddev;
    if (other.x_stddev != other.y_stddev)
    {
        Utils::Accuracy::estimateEllipse(acc_ell, other.x_stddev, other.y_stddev, other.xy_cov);
        stddev1 = Utils::Accuracy::estimateAccuracyAt(acc_ell, b);
    }

    Utils::Accuracy::GeodeticDistanceInfo gdi;
    gdi.distance = d;
    gdi.bearing  = b;
    gdi.stddev0  = stddev0;
    gdi.stddev1  = stddev1;

    return gdi;
}

/********************************************************************************
 * Various
 ********************************************************************************/

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

void checkMaxCovariance(double& x_stddev, double& y_stddev, double &xy_cov)
{
    double max_valid_cov = x_stddev * y_stddev;
    if (std::abs(xy_cov) > max_valid_cov)
    {
        // if (verbose) {
        //     logwrn << "invalid covariance " << xy_cov << " exceeds max valid " 
        //            << max_valid_cov << ", clamping";
        // }
        //xy_cov = (xy_cov > 0) ? max_valid_cov : -max_valid_cov;

        double mean_stddev = (x_stddev + y_stddev) / 2.0;
        x_stddev = mean_stddev;
        y_stddev = mean_stddev;
        xy_cov = 0;
    }
}

}  // namespace Accuracy
}  // namespace Utils
