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

#include "projector.h"
#include "traced_assert.h"

#include <ogr_spatialref.h>

#include <Eigen/Core>

/********************************************************************************
 * FrameProjector
 ********************************************************************************/

/**
*/
FrameProjector::FrameProjector()
:   center_wgs84_(0, 0)
//,   center_cart_ (0, 0)
{
}

/**
*/
FrameProjector::FrameProjector(const QRectF& coord_frame)
{
    update(coord_frame);
}

/**
*/
FrameProjector::FrameProjector(double center_lat, double center_lon)
{
    update(center_lat, center_lon);
}

/**
*/
FrameProjector::~FrameProjector() = default;

/**
*/
const QPointF& FrameProjector::centerWGS84() const
{
    return center_wgs84_;
}

/**
*/
// const QPointF& FrameProjector::centerCart() const
// {
//     return center_cart_;
// }

/**
*/
double FrameProjector::centerLat() const
{
    return center_wgs84_.x();
}

/**
*/
double FrameProjector::centerLon() const
{
    return center_wgs84_.y();
}

/**
*/
bool FrameProjector::valid() const
{
    return (proj_ != nullptr);
}

/**
*/
void FrameProjector::reset()
{
    center_wgs84_ = QPointF(0, 0);
    //center_cart_  = QPointF(0, 0);

    //proj_.reset();
    // ref_dst_.reset();
    // trafo_fwd_.reset();
    // trafo_bwd_.reset();
}

/**
*/
void FrameProjector::update(const QRectF& coord_frame)
{
    traced_assert(!coord_frame.isEmpty());

    auto center = coord_frame.center();

    update(center.x(), center.y());
}

/**
*/
void FrameProjector::update(double center_lat, double center_lon)
{
    //projection is up to date?
    if (valid() && center_lat == center_wgs84_.x() && center_lon == center_wgs84_.y())
        return;

    reset();

    if(proj_)
        proj_->Reset(center_lat, center_lon, 0.0);
    else
        proj_.reset(new GeographicLib::LocalCartesian (center_lat, center_lon, 0.0));

    // ref_src_.reset(new OGRSpatialReference);
    // ref_src_->SetWellKnownGeogCS("WGS84");

    // ref_dst_.reset(new OGRSpatialReference);
    // ref_dst_->SetStereographic(center_lat, center_lon, 1.0, 0.0, 0.0); //@TODO: give some options as to what map proj to use

    // trafo_fwd_.reset(OGRCreateCoordinateTransformation(ref_src_.get(), ref_dst_.get()));
    // trafo_bwd_.reset(OGRCreateCoordinateTransformation(ref_dst_.get(), ref_src_.get()));

    //compute cartesian center of projection (!sometimes not mapped to (0,0)!)
    double cx_cart, cy_cart;
    project(cx_cart, cy_cart, center_lat, center_lon);
    traced_assert(sqrt(pow(cx_cart, 2)+pow(cy_cart,2)) < 1E-6);

    //center_cart_  = QPointF(cx_cart, cy_cart);
    center_wgs84_ = QPointF(center_lat, center_lon);
}

/**
*/
bool FrameProjector::project(double& x,
                             double& y, 
                             double lat, 
                             double lon) const
{
    // x = lon;
    // y = lat;
    // trafo_fwd_->Transform(1, &x, &y);

    // x -= center_cart_.x();
    // y -= center_cart_.y();

    double z;

    traced_assert(proj_);
    proj_->Forward(lat, lon, 0.0, x, y, z);

    return true;
}

/**
*/
bool FrameProjector::unproject(double& lat, 
                               double& lon, 
                               double x, 
                               double y) const
{
    // x += center_cart_.x();
    // y += center_cart_.y();

    // lon = x;
    // lat = y;
    // trafo_bwd_->Transform(1, &lon, &lat);

    double h_back;

    traced_assert(proj_);

    proj_->Reverse(x, y, 0.0, lat, lon, h_back);

    return true;
}
