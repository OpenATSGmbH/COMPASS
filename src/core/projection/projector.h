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

#include <GeographicLib/LocalCartesian.hpp>

#include <QRectF>

#include <memory>

class OGRSpatialReference;
class OGRCoordinateTransformation;

/**
*/
class Projector
{
public:
    Projector() = default;
    virtual ~Projector() = default;

    virtual bool valid() const = 0;
    virtual void reset() = 0;

    virtual bool project(double& x, 
                         double& y, 
                         double lat, 
                         double lon) const = 0;
    virtual bool unproject(double& lat, 
                           double& lon, 
                           double x, 
                           double y) const = 0;
};

/**
 * Projector using a map projection computed for a given wgs84 coordinate frame.
 */
class FrameProjector : public Projector
{
public:
    FrameProjector();
    FrameProjector(const QRectF& coord_frame);
    FrameProjector(double center_lat, double center_lon);
    virtual ~FrameProjector();

    void update(const QRectF& coord_frame);
    void update(double center_lat, double center_lon);

    const QPointF& centerWGS84() const;
    //const QPointF& centerCart() const;
    double centerLat() const;
    double centerLon() const;

    bool valid() const override;
    void reset() override;

    bool project(double& x, 
                 double& y, 
                 double lat, 
                 double lon) const override;
    bool unproject(double& lat, 
                   double& lon, 
                   double x, 
                   double y) const override;
private:
    QPointF center_wgs84_; //x = lat, y = lon
    //QPointF center_cart_;
    
    // mutable std::unique_ptr<OGRSpatialReference>         ref_src_;
    // mutable std::unique_ptr<OGRSpatialReference>         ref_dst_;
    // mutable std::unique_ptr<OGRCoordinateTransformation> trafo_fwd_;
    // mutable std::unique_ptr<OGRCoordinateTransformation> trafo_bwd_;

    mutable std::unique_ptr<GeographicLib::LocalCartesian> proj_;
};
