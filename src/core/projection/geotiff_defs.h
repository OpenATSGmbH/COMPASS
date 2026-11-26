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

#include <string>
#include <vector>

#include <QRectF>

/**
*/
struct GeoTIFFInfo
{
    enum class ErrCode
    {
        NoError = 0,
        NoInfo,
        InvalidFormat,
        Empty,
        NoReference
    };

    QRectF roi() const;
    QRectF roiAsWGS84() const;

    bool isValid() const;

    ErrCode             error        = ErrCode::NoInfo;
    int                 img_w        = -1;
    int                 img_h        = -1;
    int                 bands        = -1;
    int                 raster_bytes = 0;
    std::vector<double> geo_transform;
    std::string         geo_srs; // most likely wkt!
};

/**
 * Raw rgba channel data.
 */
class RawRasterData
{
public:
    void init(size_t nx, size_t ny, size_t subsampling = 1);

    std::vector<unsigned char>& channelR() { return channel_r; }
    const std::vector<unsigned char>& channelR() const { return channel_r; }

    std::vector<unsigned char>& channelG() { return channel_g; }
    const std::vector<unsigned char>& channelG() const { return channel_g; }

    std::vector<unsigned char>& channelB() { return channel_b; }
    const std::vector<unsigned char>& channelB() const { return channel_b; }

    std::vector<unsigned char>& channelA() { return channel_a; }
    const std::vector<unsigned char>& channelA() const { return channel_a; }

    size_t rasterSizeX() const { return raster_x_size; }
    size_t rasterSizeY() const { return raster_y_size; }
    size_t rasterPixels() const { return raster_n; }

    bool empty() const
    {
        return (raster_x_size == 0 || raster_y_size == 0 || raster_n == 0);
    }

private:
    std::vector<unsigned char> channel_r;
    std::vector<unsigned char> channel_g;
    std::vector<unsigned char> channel_b;
    std::vector<unsigned char> channel_a;

    size_t raster_x_size = 0;
    size_t raster_y_size = 0;
    size_t raster_n      = 0;
};
