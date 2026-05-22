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

#include "transformation.h"
#include "logger.h"

#include <ogr_spatialref.h>

#include <iomanip>
#include <cmath>

/********************************************************************************************************
 * Transformation
 ********************************************************************************************************/

//bool Transformation::in_appimage_ = COMPASS::isAppImage();

const double Transformation::MaxWGS84Distance {0.2};

/**
*/
Transformation::Transformation(const boost::optional<double>& max_wgs_dist_custom)
:   wgs84_       (new OGRSpatialReference)
,   local_       (new OGRSpatialReference)
,   max_wgs_dist_(max_wgs_dist_custom.has_value() ? max_wgs_dist_custom.value() : MaxWGS84Distance)
{
    wgs84_->SetWellKnownGeogCS("WGS84");
}

/**
*/
Transformation::Transformation (const Transformation& a)
:   wgs84_(new OGRSpatialReference)
,   local_(new OGRSpatialReference)
{
    wgs84_->SetWellKnownGeogCS("WGS84");
}

/**
*/
Transformation::~Transformation() = default;

/**
*/
std::tuple<bool, double, double> Transformation::distanceCart (double lat1, 
                                                               double lon1, 
                                                               double lat2, 
                                                               double lon2)
{
    logdbg << "lat1 " << lat1 << " lon1 " << lon1 << " lat2 " << lat2 << " lon2 " << lon2;

    updateCenter(lat1, lon1);

    double x_pos1, y_pos1, x_pos2, y_pos2;
    bool ok;
    std::tuple<bool, double, double> ret {false, 0, 0};

    x_pos1 = lon1;
    y_pos1 = lat1;

    x_pos2 = lon2;
    y_pos2 = lat2;

    ok = ogr_geo2cart_->Transform(1, &x_pos1, &y_pos1); // wgs84 to cartesian offsets

    if (!ok)
        return ret;

    ok = ogr_geo2cart_->Transform(1, &x_pos2, &y_pos2); // wgs84 to cartesian offsets

    if (!ok)
        return ret;

    ret = std::tuple<bool, double, double>(true, x_pos2 - x_pos1, y_pos2 - y_pos1);

    logdbg << "start"
           << " p1 " << lat1 << " / " << lon1
           << " p2 " << lat2 << " / " << lon2 
           << " d " << x_pos2-x_pos1 << " / " << y_pos2-y_pos1;

    return ret;
}

/**
*/
std::tuple<bool, double> Transformation::distanceL2Cart (double lat1, 
                                                         double lon1, 
                                                         double lat2, 
                                                         double lon2)
{
    bool   ok;
    double dx, dy;

    std::tie(ok, dx, dy) = distanceCart(lat1, lon1, lat2, lon2);

    if (!ok)
        return std::tuple<bool, double>{false, 0};

    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    return std::tuple<bool, double>{true, distance};
}

/**
*/
std::tuple<bool, double, double> Transformation::distanceAngleCart (double lat1, 
                                                                    double lon1, 
                                                                    double lat2, 
                                                                    double lon2)
{
    bool   ok;
    double dx, dy;

    std::tie(ok, dx, dy) = distanceCart(lat1, lon1, lat2, lon2);

    if (!ok)
        return std::tuple<bool, double, double>{false, 0, 0};

    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double angle    = std::atan2(dy, dx);

    return std::tuple<bool, double, double>{true, distance, angle};
}

/**
*/
std::tuple<bool, double, double> Transformation::wgsAddCartOffset (double lat1, 
                                                                   double lon1, 
                                                                   double x_pos2, 
                                                                   double y_pos2)
{
    logdbg << "lat1 " << lat1 << " long1 " << lon1 << " x_pos2 " << x_pos2 << " y_pos2 " << y_pos2;

    updateCenter(lat1, lon1);

    bool ok;
    std::tuple<bool, double, double> ret {false, 0, 0};

    // calc pos 1 cart
    double x_pos1, y_pos1;
    x_pos1 = lon1;
    y_pos1 = lat1;

    ok = ogr_geo2cart_->Transform(1, &x_pos1, &y_pos1); // wgs84 to cartesian offsets

    if (!ok)
        return ret;

    logdbg << "x_pos1 " << x_pos1 << " y_pos1 " << y_pos1
           << " x_pos2 " << x_pos2 << " y_pos2 " << y_pos2;

    // add origin offset
    x_pos2 += x_pos1;
    y_pos2 += y_pos1;

    ok = ogr_cart2geo_->Transform(1, &x_pos2, &y_pos2);

    if (!ok)
        return ret;

    double lat2, long2;

    lat2 = y_pos2;
    long2 = x_pos2;


    ret = std::tuple<bool, double, double>(true, lat2, long2);

    return ret;
}

/**
*/
void Transformation::updateCenter(double lat1, 
                                  double lon1)
{
    if (!has_pos1_ || 
        std::fabs(lat1 - lat1_) > max_wgs_dist_ ||
        std::fabs(lon1 - lon1_) > max_wgs_dist_) // set
    {
        has_pos1_ = true;
        lat1_     = lat1;
        lon1_     = lon1;

        local_->SetStereographic(lat1_, lon1_, 1.0, 0.0, 0.0);

        ogr_geo2cart_.reset(OGRCreateCoordinateTransformation(wgs84_.get(), local_.get()));
        ogr_cart2geo_.reset(OGRCreateCoordinateTransformation(local_.get(), wgs84_.get()));
    }
}

/********************************************************************************************************
 * FixedTransformation
 ********************************************************************************************************/

//bool FixedTransformation::in_appimage_ = true; //COMPASS::isAppImage();

/**
*/
FixedTransformation::FixedTransformation(double lat1, double lon1)
:   lat1_ (lat1)
,   lon1_ (lon1)
,   wgs84_(new OGRSpatialReference())
,   local_(new OGRSpatialReference())
{
    wgs84_->SetWellKnownGeogCS("WGS84");

    local_->SetStereographic(lat1, lon1, 1.0, 0.0, 0.0);

    ogr_geo2cart_.reset(OGRCreateCoordinateTransformation(wgs84_.get(), local_.get()));
    ogr_cart2geo_.reset(OGRCreateCoordinateTransformation(local_.get(), wgs84_.get()));
}

/**
*/
FixedTransformation::~FixedTransformation() = default;

/**
*/
std::tuple<bool, double, double> FixedTransformation::distanceCart (double lat2, double lon2)
{
    logdbg << "lat2 " << lat2 << " long2 " << lon2;

    double x_pos2, y_pos2;
    bool ok;
    std::tuple<bool, double, double> ret {false, 0, 0};

//    if (in_appimage_) // inside appimage
//    {
        x_pos2 = lon2;
        y_pos2 = lat2;
//    }
//    else
//    {
//        x_pos2 = lat2;
//        y_pos2 = long2;
//    }

    ok = ogr_geo2cart_->Transform(1, &x_pos2, &y_pos2); // wgs84 to cartesian offsets

    if (!ok)
        return ret;

    ret = std::tuple<bool, double, double>(true, x_pos2, y_pos2);

    logdbg << "p1 "
           << std::setprecision(14) << lat1_ << " / " << std::setprecision(14) << lon1_
           << " p2 "
           << std::setprecision(14) << lat2 << " / " << std::setprecision(14) << lon2
           << " d " << x_pos2 << " / " << y_pos2;

    return ret;
}

/**
*/
std::tuple<bool, double, double> FixedTransformation::wgsAddCartOffset (double x_pos2, double y_pos2)
{
    logdbg << "x_pos2 " << x_pos2 << " y_pos2 " << y_pos2;

    bool ok;
    std::tuple<bool, double, double> ret {false, 0, 0};

    ok = ogr_cart2geo_->Transform(1, &x_pos2, &y_pos2);

    if (!ok)
        return ret;

    double lat2, lon2;

//    if (in_appimage_) // inside appimage
//    {
        lat2 = y_pos2;
        lon2 = x_pos2;
//    }
//    else
//    {
//        lat2 = x_pos2;
//        long2 = y_pos2;
//    }

    ret = std::tuple<bool, double, double>(true, lat2, lon2);

    return ret;
}
