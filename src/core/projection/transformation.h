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

#include <memory>

#include <boost/optional.hpp>

class OGRSpatialReference;
class OGRCoordinateTransformation;

/**
*/
class Transformation
{
public:
    Transformation(const boost::optional<double>& max_wgs_dist_custom = boost::optional<double>());
    Transformation(const Transformation& a); // only called during creation, slightly hacky
    virtual ~Transformation();

    // ok, dist x, dist y
    std::tuple<bool, double, double> distanceCart (double lat1, double lon1, double lat2, double lon2);

    // ok, dist
    std::tuple<bool, double> distanceL2Cart (double lat1, double lon1, double lat2, double lon2);
    
    // ok, dist, angle
    std::tuple<bool, double, double> distanceAngleCart (double lat1, double lon1, double lat2, double lon2);
    
    // ok, lat, long
    std::tuple<bool, double, double> wgsAddCartOffset (double lat1, double lon1, double x_pos2, double y_pos2);
    
protected:
    void updateCenter(double lat1, double lon1);

    // static bool in_appimage_;
    static const double MaxWGS84Distance;

    std::unique_ptr<OGRSpatialReference> wgs84_;
    std::unique_ptr<OGRSpatialReference> local_;
    std::unique_ptr<OGRCoordinateTransformation> ogr_geo2cart_;
    std::unique_ptr<OGRCoordinateTransformation> ogr_cart2geo_;

    bool   has_pos1_ {false};
    double lat1_;
    double lon1_;

    double max_wgs_dist_;
};

/**
*/
class FixedTransformation
{
public:
    FixedTransformation(double lat1, double lon1);
    virtual ~FixedTransformation();

    std::tuple<bool, double, double> distanceCart (double lat2, double lon2);
    // ok, dist x, dist y
    std::tuple<bool, double, double> wgsAddCartOffset (double x_pos2, double y_pos2);
    // ok, lat, long

protected:
    // static bool in_appimage_;
    double lat1_, lon1_;

    std::unique_ptr<OGRSpatialReference>         wgs84_;
    std::unique_ptr<OGRSpatialReference>         local_;
    std::unique_ptr<OGRCoordinateTransformation> ogr_geo2cart_;
    std::unique_ptr<OGRCoordinateTransformation> ogr_cart2geo_;
};
