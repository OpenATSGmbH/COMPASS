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

#include "projectioncoordinatesystembase.h"
#include "stringconv.h"
#include "logger.h"

// SSS D.2
const double ProjectionCoordinateSystemBase::EE_A = 6378137; //  WGS−84 geoid in m
const double ProjectionCoordinateSystemBase::EE_F = 1.0 / 298.257223563;
const double ProjectionCoordinateSystemBase::EE_E2 = EE_F * (2 - EE_F);


const double ProjectionCoordinateSystemBase::ALMOST_ZERO = 1e-10;
const double ProjectionCoordinateSystemBase::PRECISION_GEODESIC = 1e-8;

using namespace Utils;

ProjectionCoordinateSystemBase::ProjectionCoordinateSystemBase(unsigned int id,
                               double latitude_deg, double longitude_deg,
                               double altitude_m)
    : id_(id), latitude_deg_(latitude_deg), longitude_deg_(longitude_deg), altitude_m_(altitude_m)
{
    // best earth radius
    R_T_ = EE_A * (1 - EE_E2) / sqrt(pow(1 - EE_E2 * pow(sin(latitude_deg_ * DEG2RAD), 2), 3));

    h_r_ = altitude_m_;
}

// from SSS D.3
double ProjectionCoordinateSystemBase::rs2gElevation(double H, double rho)
{
    double El_rad = 0.0; // elevation angle

    if (rho >= ALMOST_ZERO)
    {
        double x = (2 * R_T_ * (H - h_r_) + pow(H, 2) - pow(h_r_, 2) - pow(rho, 2)) /
                   (2 * rho * (R_T_ + h_r_));

        if (fabs(x) <= 1.0)
            El_rad = asin(x);
    }

    //    if (rho >= ALMOST_ZERO)
    //    {
    //        //        elevation = asin((2 * rs2g_Rti_ * (z - rs2g_hi_) + pow(z, 2) - pow(rs2g_hi_,
    //        2) - pow(rho, 2)) /
    //        //                         (2 * rho * (rs2g_Rti_ + rs2g_hi_)));
    //        elevation = asin((z - rs2g_hi_)/rho);

    //        //        if (rho < 50000)
    //        //            loginf << "z " << z << " rho " <<
    //        rho << " elev " << elevation;
    //    }

    return El_rad;
}

void ProjectionCoordinateSystemBase::getGroundRange(
    double slant_range_m, bool has_altitude, double altitude_m,
    double& ground_range_m, double& adjusted_altitude_m, bool debug)
{
    if (debug)
        loginf << "slant_range_m "
               << String::doubleToStringPrecision(slant_range_m, 2)
               << " has_altitude " << has_altitude
               << " altitude_m " << String::doubleToStringPrecision(altitude_m, 2);

    double elevation_m {0};

    if (has_altitude)
        elevation_m = altitude_m;
    else
        elevation_m = h_r_;  // the Z value has not been filled so use at least the radar height

    double elev_angle_rad = rs2gElevation(elevation_m, slant_range_m);

    ground_range_m = slant_range_m * cos(elev_angle_rad);

    adjusted_altitude_m = h_r_ + slant_range_m * sin(elev_angle_rad);

    if (debug)
        loginf << "has_altitude " << has_altitude
               << " elevation_m " << String::doubleToStringPrecision(elevation_m, 2)
               << " elev_angle_rad " << String::doubleToStringPrecision(elev_angle_rad, 6)
               << " ground_range_m " << String::doubleToStringPrecision(ground_range_m, 2)
               << " adjusted_altitude_m " << String::doubleToStringPrecision(adjusted_altitude_m, 2);

    return;
}
