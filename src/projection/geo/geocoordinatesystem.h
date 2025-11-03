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

#include "projectioncoordinatesystembase.h"

#include <GeographicLib/LocalCartesian.hpp>

#include <memory>

class GeoCoordinateSystem : public ProjectionCoordinateSystemBase
{
public:
    GeoCoordinateSystem(unsigned int id, double latitude_deg, double longitude_deg,
                        double altitude_m);
    virtual ~GeoCoordinateSystem();

    bool polarSlantToCartesian(double azimuth_rad, double slant_range_m, bool has_altitude,
                               double altitude_baro_m, double& x_pos_m, double& y_pos_m);

    bool polarHorizontalToCartesian(double azimuth_rad, double horizontal_range_m, double& x_pos_m,
                                    double& y_pos_m);

    /// @brief Projects geo-coordinate in WGS-84 to cartesian coordinate, returns false on error
    bool wgs842Cartesian(double latitude_deg, double longitude_deg, double& x_pos_m,
                         double& y_pos_m);
    /// @brief Projects cartesian coordinate to geo-coordinate in WGS-84, returns false on error
    bool cartesian2WGS84(double x_pos_m, double y_pos_m, double& latitude_deg,
                         double& longitude_deg);

    // azimuth_deg as math angle
    bool wgs842PolarHorizontal(double latitude_deg, double longitude_deg, double& azimuth_deg, double& ground_range_m);

protected:

    mutable std::unique_ptr<GeographicLib::LocalCartesian> proj_;

    mutable std::unique_ptr<GeographicLib::LocalCartesian> proj_wo_alt_;
};

