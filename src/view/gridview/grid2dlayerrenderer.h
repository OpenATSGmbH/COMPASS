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

#include <utility>

#include <QRectF>

class Grid2DLayer;
struct Grid2DRenderSettings;
struct RasterReference;

class QImage;

/**
*/
class Grid2DLayerRenderer
{
public:
    static std::pair<QImage,RasterReference> render(const Grid2DLayer& layer,
                                                    const Grid2DRenderSettings& settings);

    // Geographic bbox of the opaque pixels of `img`, in the
    // (lat, lon, lat_extent, lon_extent) convention expected by
    // ViewPointGenVP::setROI (see ViewPointGenVP::toJSON). Null QRectF if no
    // opaque pixels exist. `ref` must be geographic (x is longitude, y is
    // latitude); the raster's `is_north_up` flag is honored.
    //
    // Used by analysis inspectors that emit a ViewPointGenFeatureGeoImage to
    // frame the Geographic View on the populated region of the rendered
    // raster instead of falling back to the full loaded-data extent.
    static QRectF geoROIOfOpaquePixels(const QImage& img,
                                       const RasterReference& ref);
};
