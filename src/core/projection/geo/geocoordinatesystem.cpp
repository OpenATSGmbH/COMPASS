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

#include "geocoordinatesystem.h"

#include "global.h"
#include "logger.h"


using namespace std;

GeoCoordinateSystem::GeoCoordinateSystem(unsigned int id, double latitude_deg, double longitude_deg,
                                         double altitude_m)
    : ProjectionCoordinateSystemBase(id, latitude_deg, longitude_deg, altitude_m)
{
    proj_.reset(new GeographicLib::LocalCartesian (latitude_deg, longitude_deg, altitude_m_));

    proj_wo_alt_.reset(new GeographicLib::LocalCartesian (latitude_deg, longitude_deg, 0));
}

GeoCoordinateSystem::~GeoCoordinateSystem() {}

bool GeoCoordinateSystem::polarSlantToCartesian(double azimuth_rad, double slant_range_m,
                                                bool has_altitude, double altitude_baro_m,
                                                double& x_pos_m, double& y_pos_m)
{
    if (has_altitude)
    {
        // Compute the difference in altitude

        //double delta_h = target_alt - radar_alt;
        double delta_h = altitude_baro_m - altitude_m_;

        // Check if delta_h is within valid range
        if (std::abs(delta_h) > slant_range_m)
        {
            logerr << "absolute value of delta_h "
                   << std::abs(delta_h) << " is greater than slant range " << slant_range_m;

            return polarHorizontalToCartesian(azimuth_rad, slant_range_m, x_pos_m, y_pos_m);
        }

        // Calculate horizontal range
        double horizontal_range_m = std::sqrt(slant_range_m * slant_range_m - delta_h * delta_h);

        return polarHorizontalToCartesian(azimuth_rad, horizontal_range_m, x_pos_m, y_pos_m);
    }
    else
        return polarHorizontalToCartesian(azimuth_rad, slant_range_m, x_pos_m, y_pos_m);
}

bool GeoCoordinateSystem::polarHorizontalToCartesian(double azimuth_rad, double horizontal_range_m,
                                                     double& x_pos_m, double& y_pos_m)
{
    x_pos_m = horizontal_range_m * sin(azimuth_rad);
    y_pos_m = horizontal_range_m * cos(azimuth_rad);

    return true;
}

bool GeoCoordinateSystem::wgs842Cartesian(double latitude_deg, double longitude_deg, double& x_pos,
                                          double& y_pos)
{
    logdbg << "lat " << latitude_deg << " long "
           << longitude_deg;

    double z;


    traced_assert(proj_);
    proj_->Forward(latitude_deg, longitude_deg, 0.0, x_pos, y_pos, z);

    traced_assert(proj_wo_alt_);
    proj_wo_alt_->Forward(latitude_deg, longitude_deg, 0.0, x_pos, y_pos, z);

    return true;
}

bool GeoCoordinateSystem::cartesian2WGS84(double x_pos, double y_pos, double& latitude,
                                          double& longitude)
{
    logdbg << "x_pos " << x_pos << " y_pos " << y_pos;

    double h_back;

    traced_assert(proj_wo_alt_);

    proj_wo_alt_->Reverse(x_pos, y_pos, 0.0, latitude, longitude, h_back);

    return true;
}

bool GeoCoordinateSystem::wgs842PolarHorizontal(
    double latitude_deg, double longitude_deg, double& azimuth_deg, double& ground_range_m)
{
    traced_assert(proj_wo_alt_);

    // Compute ENU coordinates of the target relative to the radar
    double north, east, up;
    proj_wo_alt_->Forward(latitude_deg, longitude_deg, 0.0, north, east, up);

    // Calculate slant range
    ground_range_m = sqrt(north * north + east * east); // + up * up

    // Calculate azimuth angle in radians
    double azimuth_rad = atan2(east, north);

    // Convert azimuth to degrees
    azimuth_deg = azimuth_rad * RAD2DEG;

    // Normalize azimuth to [0, 360) degrees
    if (azimuth_deg < 0) {
        azimuth_deg += 360.0;
    }

    return true;
}

