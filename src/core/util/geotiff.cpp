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


#include "geotiff.h"

#include "rasterreference.h"

#include "logger.h"
#include "files.h"

#include "traced_assert.h"

#include <QImage>

#include <gdal.h>
#include <gdalwarper.h>
#include <ogr_spatialref.h>

#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

const size_t GeoTIFF::MaxSize           = 10000u;
const size_t GeoTIFF::MaxPixels         = MaxSize * MaxSize;
const size_t GeoTIFF::DefaultSubsamples = 5u;

size_t GeoTIFF::geotiff_id_ = 0;

namespace
{
    /**
     */
    void logError(const std::string& prefix,
                  const std::string& err,
                  std::string* err_ptr)
    {
        logerr << prefix << err;

        if (err_ptr)
            *err_ptr = err;
    }
}

/**
 */
size_t GeoTIFF::maximumSubsamples(size_t w, size_t h)
{
    if (w >= MaxSize || h >= MaxSize)
        return 1;

    size_t max_x = std::max((size_t)1, (size_t)std::floor(MaxSize / w));
    size_t max_y = std::max((size_t)1, (size_t)std::floor(MaxSize / h));

    return std::min(DefaultSubsamples, std::min(max_x, max_y));
}

/**
 * Creates a unique filename for a geoimage in gdals vsimem memory file system.
 */
std::string GeoTIFF::generateUniqueMemFilename(const std::string& descr)
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::string uuid_str = boost::lexical_cast<std::string>(uuid);

    std::string fn = "/vsimem/geoimage" + std::to_string(geotiff_id_++) + "_" + uuid_str + (descr.empty() ? "" : "_" + descr) + ".tif";

    return fn;
}

/**
 * Retrieves info about the given geotiff file.
*/
GeoTIFFInfo GeoTIFF::getInfo(const std::string& fn, 
                             bool vmem)
{
    if (!vmem && !Utils::Files::fileExists(fn))
        return GeoTIFFInfo();

    //try to open
    auto dataset = GDALOpen(fn.c_str(), GDALAccess::GA_ReadOnly);
    if (!dataset)
        return GeoTIFFInfo();

    auto info = GeoTIFF::getInfo(dataset);

    GDALClose(dataset);

    return info;
}

/**
 * Retrieves info about the given gdal dataset.
 */
GeoTIFFInfo GeoTIFF::getInfo(void* gdal_dataset)
{
    if (!gdal_dataset)
        return GeoTIFFInfo();

    GeoTIFFInfo info;

    auto driver = GDALGetDatasetDriver(gdal_dataset);
    if (!driver)
    {
        info.error = GeoTIFFInfo::ErrCode::InvalidFormat;
        return info;
    }

    const char* driver_name = GDALGetDriverShortName(driver);
    if (!driver_name || std::string(driver_name) != "GTiff")
    {
        info.error = GeoTIFFInfo::ErrCode::InvalidFormat;
        return info;
    }

    //try to get size
    info.img_w = GDALGetRasterXSize(gdal_dataset);
    info.img_h = GDALGetRasterYSize(gdal_dataset);
    info.bands = GDALGetRasterCount(gdal_dataset);
    if (info.img_w < 1 || info.img_h < 1 || info.bands < 1)
    {
        info.error = GeoTIFFInfo::ErrCode::Empty;
        return info;
    }

    auto rBand = GDALGetRasterBand(gdal_dataset, 1);
    if (!rBand)
    {
        info.error = GeoTIFFInfo::ErrCode::InvalidFormat;
        return info;
    }

    auto data_type = GDALGetRasterDataType(rBand);
    info.raster_bytes = GDALGetDataTypeSizeBytes(data_type);
    
    //try to get geo transform
    info.geo_transform.resize(6);
    if (GDALGetGeoTransform(gdal_dataset, info.geo_transform.data()) != CPLErr::CE_None)
    {
        info.error = GeoTIFFInfo::ErrCode::NoReference;
        return info;
    }

    //try to get projection
    info.geo_srs = GeoTIFF::wktString(gdal_dataset);
    if (info.geo_srs.empty())
    {
        info.error = GeoTIFFInfo::ErrCode::NoReference;
        return info;
    }

    info.error = GeoTIFFInfo::ErrCode::NoError;

    return info;
}

/**
 * Checks if the given geotiff file seems to be valid (obtains meaningful data and a projection)
*/
bool GeoTIFF::isValidGeoTIFF(const std::string& fn)
{
    if (!Utils::Files::fileExists(fn))
        return false;

    //try to open
    auto dataset = GDALOpen(fn.c_str(), GDALAccess::GA_ReadOnly);
    if (!dataset)
        return false;

    bool valid = GeoTIFF::getInfo(dataset).isValid();

    GDALClose(dataset);

    return valid;
}

/**
 * Gets the wkt string stored in the passed dataset.
*/
std::string GeoTIFF::wktString(void* gdal_dataset)
{
    if (!gdal_dataset)
        return "";

    const char* ref = GDALGetProjectionRef(gdal_dataset);
    if (!ref)
        return "";

    return QString::fromUtf8(ref).toStdString();
}

/**
 * Creates a wkt string given a srs name or code.
 * (e.g. a well-known system such as wgs84, or an epsg code such as epsg:4326)
*/
std::string GeoTIFF::wktStringFromSRSName(const std::string& srs_name)
{
    char* srs_wkt = NULL;
    OGRSpatialReferenceH srs = OSRNewSpatialReference( NULL );

    auto srs_string_norm = QString::fromStdString(srs_name).toLower();

    if (srs_string_norm.startsWith("epsg:"))
    {
        //set from epsg
        auto parts = srs_string_norm.split(':');
        if (parts.size() == 2 && !parts[ 0 ].isEmpty() && !parts[ 1 ].isEmpty())
        {
            bool ok;
            int epsg = parts[ 1 ].toInt(&ok);

            if (ok)
                OSRImportFromEPSG(srs, epsg);
        }
    }
    else
    {
        //set from well-known system
        OSRSetWellKnownGeogCS(srs, srs_name.c_str());
    }

    OSRExportToWkt(srs, &srs_wkt);
    OSRDestroySpatialReference(srs);

    return std::string(srs_wkt);
}

/**
 * Creates a wkt string given a raster reference.
 */
std::string GeoTIFF::wktStringFromRasterReference(const RasterReference& ref)
{
    if (ref.srs_is_wkt)
        return ref.srs;

    return GeoTIFF::wktStringFromSRSName(ref.srs);
}

/**
 * Converts the given QImage to raw rgbs channels.
*/
bool GeoTIFF::qImageToRaw(RawRasterData& raw_data,
                          const QImage& img)
{
    if (img.isNull())
        return false;

    //convert to common format
    //@TODO: is this working for every type of image data?
    QImage img_in = img.format() == QImage::Format_ARGB32 ? img : img.convertToFormat(QImage::Format_ARGB32);
    if (img_in.isNull())
        return false;

    //split image data into layers
    size_t w = (size_t)img_in.width();
    size_t h = (size_t)img_in.height();

    raw_data.init(w, h);

    uint8_t r, g, b, a;
    size_t y, x;
    size_t idx;

    auto& channel_r = raw_data.channelR();
    auto& channel_g = raw_data.channelG();
    auto& channel_b = raw_data.channelB();
    auto& channel_a = raw_data.channelA();

    //iterate over input image and split into channels
    idx = 0;
    for (y = 0; y < h; ++y)
    {
        //get input image's current line
        QRgb* line = (QRgb*)img_in.scanLine(y);

        for (x = 0; x < w; ++x, ++line, ++idx)
        {
            //get input image's current sample
            a = qAlpha(*line);
            r = qRed  (*line);
            g = qGreen(*line);
            b = qBlue (*line);

            channel_r[ idx ] = r;
            channel_g[ idx ] = g;
            channel_b[ idx ] = b;
            channel_a[ idx ] = a;
        }
    }

    return true;
}

/**
 * Converts the given gtiff dataset to raw rgba channels (if supported).
 * @TODO: reencoding dataset to rgba using gdal possible?
*/
GeoTIFF::GDAL2RAWError GeoTIFF::gtiffToRaw(RawRasterData& raw_data,
                                           void* gdal_dataset)
{
    GDALDatasetH dataset = gdal_dataset;

    int w  = GDALGetRasterXSize(dataset);
    int h  = GDALGetRasterYSize(dataset);
    int nb = GDALGetRasterCount(dataset);

    if (w < 1 || h < 1 || nb < 1)
        return GDAL2RAWError::InvalidData;

    //only rgba supported
    if (nb != 4)
        return GDAL2RAWError::NotSupported;

    auto rBand = GDALGetRasterBand(dataset, 1);
    auto gBand = GDALGetRasterBand(dataset, 2);
    auto bBand = GDALGetRasterBand(dataset, 3);
    auto aBand = GDALGetRasterBand(dataset, 4);

    auto r_interpr_ok = GDALGetRasterColorInterpretation(rBand) == GDALColorInterp::GCI_RedBand;
    auto g_interpr_ok = GDALGetRasterColorInterpretation(gBand) == GDALColorInterp::GCI_GreenBand;
    auto b_interpr_ok = GDALGetRasterColorInterpretation(bBand) == GDALColorInterp::GCI_BlueBand;
    auto a_interpr_ok = GDALGetRasterColorInterpretation(aBand) == GDALColorInterp::GCI_AlphaBand;

    //only rgba supported
    if (!r_interpr_ok || !g_interpr_ok || !b_interpr_ok || !a_interpr_ok)
        return GDAL2RAWError::NotSupported;

    //read in gdal raster
    raw_data.init(w, h);

    auto& channel_r = raw_data.channelR();
    auto& channel_g = raw_data.channelG();
    auto& channel_b = raw_data.channelB();
    auto& channel_a = raw_data.channelA();

    auto err_r = GDALRasterIO(rBand, GF_Read, 0, 0, w, h, channel_r.data(), w, h, GDT_Byte, 0, 0 );
    auto err_g = GDALRasterIO(gBand, GF_Read, 0, 0, w, h, channel_g.data(), w, h, GDT_Byte, 0, 0 );
    auto err_b = GDALRasterIO(bBand, GF_Read, 0, 0, w, h, channel_b.data(), w, h, GDT_Byte, 0, 0 );
    auto err_a = GDALRasterIO(aBand, GF_Read, 0, 0, w, h, channel_a.data(), w, h, GDT_Byte, 0, 0 );

    if (err_r != CPLErr::CE_None ||
        err_g != CPLErr::CE_None ||
        err_b != CPLErr::CE_None ||
        err_a != CPLErr::CE_None)
        return GDAL2RAWError::ConversionFailed;

    return GDAL2RAWError::NoError;
}

/**
 * Converts the given raw channel data to a geotiff image and returns the created dataset.
*/
void* GeoTIFF::gtiffFromRaw(const std::string& fn,
                            RawRasterData& raw_data,
                            const RasterReference& ref)
{
    if (fn.empty() || raw_data.empty())
        return nullptr;

    int w_out = (int)raw_data.rasterSizeX();
    int h_out = (int)raw_data.rasterSizeY();

    //create gtiff dataset in gdal virtual filesystem
    auto gtiff_driver = GDALGetDriverByName("GTiff");
    if (!gtiff_driver)
        return nullptr;

    char **papszOptions = NULL;
    auto dataset = GDALCreate(gtiff_driver, fn.c_str(), w_out, h_out, 4, GDT_Byte, papszOptions);

    if (!dataset)
        return nullptr;

    //write image data to raster bands
    auto rBand = GDALGetRasterBand(dataset, 1);
    auto gBand = GDALGetRasterBand(dataset, 2);
    auto bBand = GDALGetRasterBand(dataset, 3);
    auto aBand = GDALGetRasterBand(dataset, 4);

    auto& channel_r = raw_data.channelR();
    auto& channel_g = raw_data.channelG();
    auto& channel_b = raw_data.channelB();
    auto& channel_a = raw_data.channelA();

    auto err_r = GDALRasterIO(rBand, GF_Write, 0, 0, w_out, h_out, channel_r.data(), w_out, h_out, GDT_Byte, 0, 0 );
    auto err_g = GDALRasterIO(gBand, GF_Write, 0, 0, w_out, h_out, channel_g.data(), w_out, h_out, GDT_Byte, 0, 0 );
    auto err_b = GDALRasterIO(bBand, GF_Write, 0, 0, w_out, h_out, channel_b.data(), w_out, h_out, GDT_Byte, 0, 0 );
    auto err_a = GDALRasterIO(aBand, GF_Write, 0, 0, w_out, h_out, channel_a.data(), w_out, h_out, GDT_Byte, 0, 0 );

    if (err_r != CPLErr::CE_None ||
        err_g != CPLErr::CE_None ||
        err_b != CPLErr::CE_None ||
        err_a != CPLErr::CE_None)
    {
        GDALClose(dataset);
        GDALDeleteDataset(gtiff_driver, fn.c_str());
        return nullptr;
    }

    GDALSetRasterColorInterpretation(rBand, GDALColorInterp::GCI_RedBand);
    GDALSetRasterColorInterpretation(gBand, GDALColorInterp::GCI_GreenBand);
    GDALSetRasterColorInterpretation(bBand, GDALColorInterp::GCI_BlueBand);
    GDALSetRasterColorInterpretation(aBand, GDALColorInterp::GCI_AlphaBand);

    //set geo transform
    auto gt = ref.geoTransform();
    GDALSetGeoTransform(dataset, gt.data());

    //obtain and set wkt string of projection
    auto wkt_string = wktStringFromRasterReference(ref);
    GDALSetProjection(dataset, wkt_string.c_str());

    return dataset;
}

/**
 * Subsamples the given raw channel data using the given number of samples.
 */
bool GeoTIFF::subsampleRaw(RawRasterData& raw_data_subsampled,
                           const RawRasterData& raw_data,
                           size_t subsampling)
{
    if (subsampling < 1)
        return false;

    if (subsampling < 2 || raw_data.empty())
    {
        raw_data_subsampled = raw_data;
        return true;
    }

    size_t w = raw_data.rasterSizeX();
    size_t h = raw_data.rasterSizeY();

    raw_data_subsampled.init(w, h, subsampling);

    size_t w_out = raw_data_subsampled.rasterSizeX();
    size_t n_out = raw_data_subsampled.rasterPixels();

    const size_t chunk_size = subsampling * w_out;

    uint8_t r, g, b, a;
    size_t y_in, x_in;       // position in input image
    size_t idx_in;           // index into input image
    size_t x_block, y_block; // position in subblock
    size_t line_idx_out;     // output index of current subblock's origin line
    size_t idx_out;          // output index of current subblock's origin sample
    size_t line_idx_block;   // output index of current line in subblock
    size_t idx_block;        // output index of current sample in subblock
    
    //iterate over input image
    const uint8_t* r_in = raw_data.channelR().data();
    const uint8_t* g_in = raw_data.channelG().data();
    const uint8_t* b_in = raw_data.channelB().data();
    const uint8_t* a_in = raw_data.channelA().data();

    uint8_t* r_out = raw_data_subsampled.channelR().data();
    uint8_t* g_out = raw_data_subsampled.channelG().data();
    uint8_t* b_out = raw_data_subsampled.channelB().data();
    uint8_t* a_out = raw_data_subsampled.channelA().data();
    
    idx_in = 0;
    for (y_in = 0, line_idx_out = 0; y_in < h; ++y_in, line_idx_out += chunk_size)
    {
        //get input image's current line
        for (x_in = 0, idx_out = line_idx_out; x_in < w; ++x_in, idx_out += subsampling, ++idx_in)
        {
            //get input image's current sample
            a = a_in [ idx_in ];
            r = r_in [ idx_in ];
            g = g_in [ idx_in ];
            b = b_in [ idx_in ];

            //write subsampling block to output channels
            for (y_block = 0, line_idx_block = idx_out; y_block < subsampling; ++y_block, line_idx_block += w_out)
            {
                for (x_block = 0, idx_block = line_idx_block; x_block < subsampling; ++x_block, ++idx_block)
                {
                    traced_assert(idx_block < n_out);
                    r_out[ idx_block ] = r;
                    g_out[ idx_block ] = g;
                    b_out[ idx_block ] = b;
                    a_out[ idx_block ] = a;
                }
            }
        }
    }

    return true;
}

/**
 * Subsamples the given raw channel data using the given number of samples.
*/
bool GeoTIFF::subsampleRaw(RawRasterData& raw_data,
                          size_t subsampling)
{
    RawRasterData raw_data_subsampled;
    if (!GeoTIFF::subsampleRaw(raw_data_subsampled, raw_data, subsampling))
        return false;

    raw_data = raw_data_subsampled;

    return true;
}

/**
 * Upsamples the given gdal dataset using the given number of samples, 
 * creating a new dataset under the provided filename.
 */
void* GeoTIFF::upsampleGTiff(const std::string& fn_out,
                             void* gdal_dataset,
                             size_t subsampling)
{
    if (!gdal_dataset || subsampling < 1)
        return nullptr;

    //@TODO: potential horror
    if (subsampling == 1)
        return gdal_dataset;

    //get info about original dataset
    GeoTIFFInfo info = GeoTIFF::getInfo(gdal_dataset);
    if (!info.isValid())
        return nullptr;

    int wout = info.img_w * (int)subsampling;
    int hout = info.img_h * (int)subsampling;

    auto gtiff_driver = GDALGetDriverByName("GTiff");
    if (!gtiff_driver)
        return nullptr;
    
    //create output dataset
    char **papszOptions = NULL;
    auto dataset_out = GDALCreate(gtiff_driver, fn_out.c_str(), wout, hout, info.bands, GDT_Byte, papszOptions);
    if (!dataset_out)
        return nullptr;

    //get subsampled reference
    RasterReference ref;
    ref.set(info);
    ref = ref.subsampled(subsampling);

    //set geo-info
    auto gt = ref.geoTransform();
    GDALSetGeoTransform(dataset_out, gt.data());

    auto wkt_string = wktStringFromRasterReference(ref);
    GDALSetProjection(dataset_out, wkt_string.c_str());

    if (GDALReprojectImage(gdal_dataset,
                            nullptr,
                            dataset_out,
                            nullptr,
                            GRA_NearestNeighbour,
                            0.0,
                            0.0,
                            nullptr,
                            nullptr,
                            nullptr) != CE_None) 
    {
        GDALClose(dataset_out);
        GDALDeleteDataset(gtiff_driver, fn_out.c_str());
        return nullptr;
    }

    return dataset_out;
}

/**
 * Subsamples the given gdal dataset using the given number of samples, 
 * creating a new dataset under the provided filename.
 */
void* GeoTIFF::subsampleGTiff(const std::string& fn_out,
                              void* gdal_dataset,
                              size_t subsampling)
{
    if (!gdal_dataset || subsampling < 1)
        return nullptr;

    //@TODO: potential horror
    if (subsampling == 1)
        return gdal_dataset;

    RawRasterData raw_data;
    auto err = gtiffToRaw(raw_data, gdal_dataset);

    if (err != GDAL2RAWError::NoError)
        return nullptr;

    //get dataset info
    auto info = GeoTIFF::getInfo(gdal_dataset);
    if (!info.isValid())
        return nullptr;

    //configure reference
    RasterReference ref;
    ref.set(info);
    ref = ref.subsampled(subsampling); // apply subsampling to ref

    //subsample raw data
    if (!subsampleRaw(raw_data, subsampling))
        return nullptr;

    //create intermediate subsampled geotiff in virtual mem
    GDALDatasetH dataset_out = GeoTIFF::gtiffFromRaw(fn_out, raw_data, ref);
    if (!dataset_out)
        return nullptr;

    return dataset_out;
}

/**
 * Writes the given QImage to a new geotiff file.
 * Optionally handles
 * - subsampling
 * - warping to a desired srs
*/
bool GeoTIFF::writeGeoTIFF(const std::string& fn,
                           const QImage& img,
                           const RasterReference& ref,
                           const std::string& warp_to_srs,
                           size_t subsampling,
                           bool verbose,
                           std::string* error)
{
    if (img.isNull())
    {
        logError("GeoTIFF: writeGeoTIFF: ", "input data is null", error);
        return false;
    }

    subsampling = std::max(subsampling, (size_t)1);

    if (verbose)
        loginf << "start" << "fn = " << fn;
    if (verbose)
        loginf << "ref in =\n" << ref.asString();
 
    //split image data into layers
    size_t w_in = (size_t)img.width();
    size_t h_in = (size_t)img.height();

    if (verbose)
        loginf << "img w = " << w_in << ", h = " << h_in << ", depth = " << img.depth() << ", subsampling = " << subsampling;

    //QImage -> raw 4-channel data
    RawRasterData raw_data;
    if (!GeoTIFF::qImageToRaw(raw_data, img))
    {
        logError("GeoTIFF: writeGeoTIFF: ", "could not convert to raw data", error);
        return false;
    }

    RasterReference ref_out = ref;

    if (verbose)
        loginf << "converted to raw";

    //subsample if desired
    if (subsampling > 1)
    {
        if (!GeoTIFF::subsampleRaw(raw_data, subsampling))
        {
            logError("", "subsampling failed", error);
            return false;
        }

        //modify reference due to subsampling
        ref_out = ref.subsampled(subsampling);
    }

    size_t w_out = raw_data.rasterSizeX();
    size_t h_out = raw_data.rasterSizeY();

    if (verbose)
        loginf << "raw data w = " << w_out << ", h = " << h_out;
    if (verbose)
        loginf << "ref out =\n" << ref_out.asString();

    //configure paths
    std::string fn_in  = fn;
    std::string fn_out = fn;

    if (!warp_to_srs.empty())
    {
        //in case of warping, generate an intermediate file in memory to be warped
        fn_in = GeoTIFF::generateUniqueMemFilename("input");
    }

    if (verbose)
        loginf << "input filename = " << fn_in;

    GDALDatasetH dataset;

    //create dataset (either final one or intermediate to be warped)
    {
        dataset = GeoTIFF::gtiffFromRaw(fn_in, raw_data, ref_out);

        if (!dataset)
        {
            logError("GeoTIFF: writeGeoTIFF: ", "could not create geotiff from raw data", error);
            return false;
        }

        // auto dataset_tmp = GDALCreateCopy(gtiff_driver, "/home/mcphatty/grid.png", dataset, 0, NULL, NULL, NULL);
        // GDALClose(dataset_tmp);
    }

    if (verbose)
        loginf << "wkt = " << GeoTIFF::wktString(dataset);

    //warp dataset to other srs
    if (!warp_to_srs.empty())
    {
        if (verbose)
            loginf << "warping...\n\n" << GDALGetProjectionRef(dataset) << "\n =>\n" << warp_to_srs << "\n";

        //GDALWarpOptions *psWarpOptions = GDALCreateWarpOptions();

        auto dataset_warped = GDALAutoCreateWarpedVRT(dataset, 
                                                      GDALGetProjectionRef(dataset), 
                                                      warp_to_srs.c_str(), 
                                                      GDALResampleAlg::GRA_NearestNeighbour,
                                                      0.001,
                                                      0);
        if (!dataset_warped)
        {
            GDALClose(dataset);
            logError("GeoTIFF: writeGeoTIFF: ", "warping failed", error);
            return false;
        }

        //create a copy of the warped system using gtiff driver (not sure if this step is needed)
        auto gtiff_driver  = GDALGetDriverByName("GTiff");
        if (!gtiff_driver)
        {
            GDALClose(dataset);
            logError("GeoTIFF: writeGeoTIFF: ", "geotiff driver not found", error);
            return false;
        }

        auto dataset_layer = GDALCreateCopy(gtiff_driver, fn_out.c_str(), dataset_warped, 0, NULL, NULL, NULL);

        // auto dataset_tmp = GDALCreateCopy(gtiff_driver, "/home/mcphatty/grid_warped.png", dataset_warped, 0, NULL, NULL, NULL);
        // GDALClose(dataset_tmp);

        //close all datasets
        GDALClose(dataset_warped);
        GDALClose(dataset);
        GDALClose(dataset_layer);

        //delete unwarped intermediate file from (virtual) mem
        GDALDeleteDataset(gtiff_driver, fn_in.c_str());

        //GDALDestroyWarpOptions(psWarpOptions);

        if (verbose)
            loginf << "warped";
    }
    else
    {
        //just close created dataset
        GDALClose(dataset);
    }

    return true;
}

/**
 * Warps the given geotiff file to a new geotiff file in the given SRS.
 * Optionally handles
 * - subsampling
*/
bool GeoTIFF::warpGeoTIFF(const std::string& fn,
                          const std::string& fn_out,
                          const std::string& warp_to_srs,
                          size_t subsampling,
                          bool subsampling_critical,
                          bool verbose,
                          std::string* error)
{
    //open dataset
    auto dataset = GDALOpen(fn.c_str(), GDALAccess::GA_ReadOnly);
    if (!dataset)
    {
        logError("GeoTIFF: warpGeoTIFF: ", "input data is null", error);
        return false;
    }

    std::function<void()> cleanupDataset = [ = ] ()
    {
        GDALClose(dataset);
    };

    if (verbose)
        loginf << "fn = " << fn;
    if (verbose)
        loginf << "fn_out = " << fn_out;

    int w  = GDALGetRasterXSize(dataset);
    int h  = GDALGetRasterYSize(dataset);
    int nb = GDALGetRasterCount(dataset);

    if (verbose)
        loginf << "w = " << w << ", h = " << h << ", bands = " << nb;

    if (w < 1 || h < 1 || nb < 1)
    {
        logError("GeoTIFF: warpGeoTIFF: ", "input data is empty", error);
        cleanupDataset();
        return false;
    }

    auto gtiff_driver = GDALGetDriverByName("GTiff");
    if (!gtiff_driver)
    {
        logError("GeoTIFF: writeGeoTIFF: ", "geotiff driver not found", error);
        cleanupDataset();
        return false;
    }

    bool enforce_upsample = true;

    if (subsampling > 1)
    {
        GDALDatasetH dataset_interm = nullptr;
        std::string fn_interm = GeoTIFF::generateUniqueMemFilename("subsampled");

        //subsample first
        if (!enforce_upsample)
        {
            if (verbose)
                loginf << "trying to subsample using " << subsampling << " samples...";

            dataset_interm = GeoTIFF::subsampleGTiff(fn_interm, dataset, subsampling);
        }

        //try to upsample in case subsampling failed
        if (!dataset_interm)
        {
            if (verbose)
                loginf << "trying to upsample using " << subsampling << " samples...";

            dataset_interm = GeoTIFF::upsampleGTiff(fn_interm, dataset, subsampling);
        }

        //finish
        if (dataset_interm)
        {
            //close input dataset
            cleanupDataset();

            //set intermediate set as new dataset
            dataset = dataset_interm;

            //set new cleanup callback
            cleanupDataset = [ dataset, gtiff_driver, fn_interm ] () 
            {
                GDALClose(dataset);

                //intermediate data needs to be removed in the end
                GDALDeleteDataset(gtiff_driver, fn_interm.c_str());
            };
        }
        else
        {
            if (subsampling_critical)
            {
                logError("GeoTIFF: writeGeoTIFF: ", "subsampling could not be applied", error);
                cleanupDataset();
                return false;
            }

            logwrn << "subsampling could not be applied, skipping...";
        }
    }
    
    if (verbose)
        loginf << "warping...\n\n" << GDALGetProjectionRef(dataset) << "\n =>\n" << warp_to_srs << "\n";

    //GDALWarpOptions *psWarpOptions = GDALCreateWarpOptions();

    //warp geotiff
    auto dataset_warped = GDALAutoCreateWarpedVRT(dataset, 
                                                  GDALGetProjectionRef(dataset), 
                                                  warp_to_srs.c_str(), 
                                                  GDALResampleAlg::GRA_NearestNeighbour,
                                                  0.001,
                                                  0);
    //GDALDestroyWarpOptions(psWarpOptions);

    if (!dataset_warped)
    {
        logError("GeoTIFF: warpGeoTIFF: ", "warping failed", error);
        cleanupDataset();
        return false;
    }

    //create a copy of the warped system using gtiff driver (not sure if this intermdiate step is needed)
    auto dataset_out = GDALCreateCopy(gtiff_driver, fn_out.c_str(), dataset_warped, 0, NULL, NULL, NULL);

    //cleanup everything
    GDALClose(dataset_out);
    GDALClose(dataset_warped);
    cleanupDataset();

    if (verbose)
        loginf << "warped";

    return true;
}
