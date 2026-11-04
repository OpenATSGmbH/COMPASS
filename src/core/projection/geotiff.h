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

#include "geotiff_defs.h"

#include <string>
#include <vector>

#include <QRectF>

struct RasterReference;

class QImage;
class QSize; 

/**
*/
class GeoTIFF
{
public:
    enum class GDAL2RAWError
    {
        NoError = 0,
        InvalidData,
        NotSupported,
        ConversionFailed
    };

    static std::string generateUniqueMemFilename(const std::string& descr = "");

    static GeoTIFFInfo getInfo(const std::string& fn, bool vmem = false);
    static GeoTIFFInfo getInfo(void* gdal_dataset);
    static bool isValidGeoTIFF(const std::string& fn);

    static bool writeGeoTIFF(const std::string& fn,
                             const QImage& img,
                             const RasterReference& ref,
                             const std::string& warp_to_srs = "",
                             size_t subsampling = 1,
                             bool verbose = false,
                             std::string* error = nullptr);
    static bool warpGeoTIFF(const std::string& fn,
                            const std::string& fn_out,
                            const std::string& warp_to_srs,
                            size_t subsampling = 1,
                            bool subsampling_critical = false,
                            bool verbose = false,
                            std::string* error = nullptr);

    static std::string wktStringFromSRSName(const std::string& srs_name);
    static std::string wktStringFromRasterReference(const RasterReference& ref);
    static std::string wktString(void* gdal_dataset);

    static bool qImageToRaw(RawRasterData& raw_data,
                            const QImage& img);
    static GDAL2RAWError gtiffToRaw(RawRasterData& raw_data,
                                    void* gdal_dataset);
    static void* gtiffFromRaw(const std::string& fn,
                              RawRasterData& raw_data,
                              const RasterReference& ref);
    static bool subsampleRaw(RawRasterData& raw_data_subsampled,
                             const RawRasterData& raw_data,
                             size_t subsampling);
    static bool subsampleRaw(RawRasterData& raw_data,
                             size_t subsampling);
    static void* subsampleGTiff(const std::string& fn_out,
                                void* gdal_dataset,
                                size_t subsampling);
    static void* upsampleGTiff(const std::string& fn_out,
                               void* gdal_dataset,
                               size_t subsampling);

    static size_t maximumSubsamples(size_t w, size_t h);

    static const size_t MaxSize;
    static const size_t MaxPixels;
    static const size_t DefaultSubsamples;
    
private:
    static size_t geotiff_id_;
}; 
