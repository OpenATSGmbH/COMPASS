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

#include "json.hpp"
#include "geotiff_defs.h"
#include "traced_assert.h"

#include <string>
#include <vector>
#include <sstream>

#include <QRectF>

/**
*/
struct RasterReference
{
    RasterReference() {};

    void set(const std::string& srs_str,
             const QRectF& roi,
             int w,
             int h,
             bool srs_is_north_up,
             bool srs_str_is_wkt = false)
    {
        srs              = srs_str;
        img_origin_x     = roi.left();
        img_origin_y     = srs_is_north_up ? roi.bottom() : roi.top();
        img_pixel_size_x = roi.width()  / w;
        img_pixel_size_y = roi.height() / h;
        is_north_up      = srs_is_north_up;
        srs_is_wkt       = srs_str_is_wkt;
    }

    void set(const std::string& srs_str,
             const std::vector<double>& geo_transform,
             bool srs_str_is_wkt = false)
    {
        traced_assert(geo_transform.size() == 6);

        srs = srs_str;

        is_north_up = geo_transform[ 5 ] < 0;

        img_origin_x     = geo_transform[ 0 ];
        img_pixel_size_x = geo_transform[ 1 ];
        img_origin_y     = geo_transform[ 3 ];
        img_pixel_size_y = is_north_up ? -geo_transform[ 5 ] : geo_transform[ 5 ];

        srs_is_wkt = srs_str_is_wkt;
    }

    void set(const GeoTIFFInfo& info)
    {
        set(info.geo_srs, info.geo_transform, true);
    }

    QRectF getROI(int w, int h) const
    {
        double roi_w = w * img_pixel_size_x;
        double roi_h = h * img_pixel_size_y;
        double roi_x = img_origin_x;
        double roi_y = is_north_up ? img_origin_y - roi_h : img_origin_y;

        return QRectF(roi_x, roi_y, roi_w, roi_h);
    }

    std::vector<double> geoTransform(int offsx = 0, int offsy = 0) const
    {
        const double dy = is_north_up ? -img_pixel_size_y : img_pixel_size_y;

        return { img_origin_x + offsx * img_pixel_size_x, 
                 img_pixel_size_x, 
                 0, 
                 img_origin_y + offsy * dy, 
                 0, 
                 dy };
    }

    RasterReference subsampled(int samples) const
    {
        RasterReference r = *this;
        if (samples <= 1)
            return r;
        
        r.img_pixel_size_x /= samples;
        r.img_pixel_size_y /= samples;

        return r;
    }

    bool fromJSON(const nlohmann::json& json_ref)
    {
        *this = {};

        if (!json_ref.contains("srs")              ||
            !json_ref.contains("img_origin_x")     ||
            !json_ref.contains("img_origin_y")     ||
            !json_ref.contains("img_pixel_size_x") ||
            !json_ref.contains("img_pixel_size_y"))
            return false;

        srs              = json_ref[ "srs" ];
        img_origin_x     = json_ref[ "img_origin_x"   ];
        img_origin_y     = json_ref[ "img_origin_y"    ];
        img_pixel_size_x = json_ref[ "img_pixel_size_x" ];
        img_pixel_size_y = json_ref[ "img_pixel_size_y" ];

        is_north_up = true;
        if (json_ref.contains("is_north_up"))
            is_north_up = json_ref[ "is_north_up" ];

        srs_is_wkt = false;
        if (json_ref.contains("srs_is_wkt"))
            srs_is_wkt = json_ref[ "srs_is_wkt" ];

        return true;
    }

    nlohmann::json toJSON() const
    {
        nlohmann::json j;
        j[ "srs"              ] = srs;
        j[ "img_origin_x"     ] = img_origin_x;
        j[ "img_origin_y"     ] = img_origin_y;
        j[ "img_pixel_size_x" ] = img_pixel_size_x;
        j[ "img_pixel_size_y" ] = img_pixel_size_y;
        j[ "is_north_up"      ] = is_north_up;
        j[ "srs_is_wkt"       ] = srs_is_wkt;

        return j;
    }

    std::string asString() const 
    {
        std::stringstream ss;
        ss << "srs:              " << srs              << std::endl;
        ss << "img_origin_x:     " << img_origin_x     << std::endl;
        ss << "img_origin_y:     " << img_origin_y     << std::endl;
        ss << "img_pixel_size_x: " << img_pixel_size_x << std::endl;
        ss << "img_pixel_size_y: " << img_pixel_size_y << std::endl;
        ss << "is_north_up:      " << is_north_up      << std::endl;
        ss << "srs_is_wkt:       " << srs_is_wkt;

        return ss.str();
    }

    std::string srs;                  // srs name, either a well-known system identifier (e.g. "wgs84") or an epsg code (e.g. "epsg:3857")
    double      img_origin_x;         // x-coordinate of the left-upper corner of the image
    double      img_origin_y;         // y-coordinate of the left-upper corner of the image
    double      img_pixel_size_x;     // width of a pixel in srs units
    double      img_pixel_size_y;     // height of a pixel in srs units
    bool        is_north_up = true;   // is the srs north-up (most likely)
    bool        srs_is_wkt = false;   // if true the srs name can directly be interpreted as a wkt string
};
