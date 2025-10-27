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

namespace Utils
{
namespace Accuracy
{

/**
 */
struct EllipseDef
{
    double rad1;
    double rad2;
    double theta_rad;
};

/**
 */
struct GeodeticDistanceInfo
{
    double mahalanobisDistance(double eps = 1e-09) const;
    double mahalanobisDistanceSqr(double eps = 1e-09) const;
    double likelihood(double eps = 1e-09) const;

    double distance = 0.0;
    double bearing  = 0.0;
    double stddev0  = 0.0;
    double stddev1  = 0.0;
};

/**
 */
struct GeoInfo
{
    GeoInfo() = default;
    GeoInfo(double latitude, double longitude) : lat(latitude), lon(longitude) {}

    double geodeticDistance(const GeoInfo& other) const;
    double bearing(const GeoInfo& other) const;

    double lat      = 0.0;
    double lon      = 0.0;
};

/**
 */
struct GeoAccInfo : public GeoInfo
{
    GeoAccInfo() = default;
    GeoAccInfo(double latitude, 
               double longitude,
               double xstddev,
               double ystddev,
               double xycov) 
    :   GeoInfo (latitude, longitude)
    ,   x_stddev(xstddev)
    ,   y_stddev(ystddev)
    ,   xy_cov  (xycov) {}

    GeodeticDistanceInfo distance(const GeoAccInfo& other) const;

    double x_stddev = 0.0;
    double y_stddev = 0.0;
    double xy_cov   = 0.0;
};

void estimateEllipse(EllipseDef& def,
                     double x_stddev,
                     double y_stddev,
                     double xy_cov);

double estimateAccuracyAt(const EllipseDef& def, 
                          double bearing_rad);

extern void checkMaxCovariance(double& x_stddev, double& y_stddev, double &xy_cov);

}  // namespace Accuracy
}  // namespace Utils
