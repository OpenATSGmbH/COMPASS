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

#include "gridbackground_defs.h"

#include "files.h"
#include "logger.h"

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

#include <QDir>
#include <QFile>
#include <QXmlStreamReader>

#include <algorithm>
#include <cmath>

namespace gridbackground
{

/**
 */
bool isEligibleMapFile(const std::string& earth_xml)
{
    QXmlStreamReader reader(QString::fromStdString(earth_xml));

    bool map_found    = false;
    bool projected    = false;
    int  image_layers = 0;

    while (!reader.atEnd())
    {
        reader.readNext();

        if (!reader.isStartElement())
            continue;

        const QString name = reader.name().toString();

        if (!map_found)
        {
            //the first element has to be the map
            if (name != "map")
                return false;

            map_found = true;
            projected = reader.attributes().value("type").toString() == "projected";

            if (!projected)
                return false;

            continue;
        }

        //shaders need OpenGL
        if (name == "shader")
            return false;

        //image layers, e.g. XYZImage, ArcGISServerImage, MBTilesImage, or the older <image driver="...">
        if (name.endsWith("Image") || name == "image")
            ++image_layers;
    }

    if (reader.hasError())
        return false;

    return map_found && projected && image_layers > 0;
}

/**
 */
std::string mapDirectory()
{
    return Utils::Files::dataDirectory() + "maps/";
}

/**
 */
std::string mapTitle(const std::string& file_name)
{
    auto pos = file_name.rfind('.');
    return pos == std::string::npos ? file_name : file_name.substr(0, pos);
}

/**
 */
std::vector<MapFile> eligibleMapFiles(const std::string& directory)
{
    std::vector<MapFile> files;

    QDir dir(QString::fromStdString(directory));
    const QStringList entries = dir.entryList(QStringList() << "*.earth", QDir::Files, QDir::Name);

    for (const QString& entry : entries)
    {
        QFile file(dir.filePath(entry));
        if (!file.open(QIODevice::ReadOnly))
            continue;

        const std::string content = file.readAll().toStdString();

        if (!isEligibleMapFile(content))
            continue;

        files.push_back({ entry.toStdString(), mapTitle(entry.toStdString()) });
    }

    return files;
}

namespace mercator
{

/**
 */
double lonToTileX(double lon, int zoom)
{
    const double n = std::ldexp(1.0, zoom);
    return (lon + 180.0) / 360.0 * n;
}

/**
 */
double latToTileY(double lat, int zoom)
{
    const double n     = std::ldexp(1.0, zoom);
    const double lat_r = std::max(-MaxLatitude, std::min(MaxLatitude, lat)) * M_PI / 180.0;

    return (1.0 - std::asinh(std::tan(lat_r)) / M_PI) / 2.0 * n;
}

/**
 */
double tileXToLon(double tile_x, int zoom)
{
    const double n = std::ldexp(1.0, zoom);
    return tile_x / n * 360.0 - 180.0;
}

/**
 */
double tileYToLat(double tile_y, int zoom)
{
    const double n = std::ldexp(1.0, zoom);
    return std::atan(std::sinh(M_PI * (1.0 - 2.0 * tile_y / n))) * 180.0 / M_PI;
}

/**
 */
int zoomForResolution(double lon_deg_per_px, int max_zoom)
{
    if (lon_deg_per_px <= 0.0 || !std::isfinite(lon_deg_per_px))
        return max_zoom;

    const double z = std::ceil(std::log2(360.0 / (TileSize * lon_deg_per_px)));

    return (int)std::max(0.0, std::min((double)max_zoom, z));
}

/**
 */
TileRange tileRange(const QRectF& lonlat, int zoom, int max_tiles)
{
    TileRange range;

    const double lon_min = std::max(-180.0, lonlat.left());
    const double lon_max = std::min( 180.0, lonlat.right());
    const double lat_min = std::max(-MaxLatitude, lonlat.top());
    const double lat_max = std::min( MaxLatitude, lonlat.bottom());

    if (lon_min >= lon_max || lat_min >= lat_max)
        return range; // outside of the Mercator area => invalid

    zoom = std::max(0, zoom);

    while (true)
    {
        const int n = 1 << zoom;

        auto clampTile = [ & ] (double v) { return std::max(0, std::min(n - 1, (int)std::floor(v))); };

        range.zoom = zoom;
        range.x0   = clampTile(lonToTileX(lon_min, zoom));
        range.x1   = clampTile(lonToTileX(lon_max, zoom) - 1e-9);
        range.y0   = clampTile(latToTileY(lat_max, zoom));
        range.y1   = clampTile(latToTileY(lat_min, zoom) - 1e-9);

        if (range.numTiles() <= max_tiles || zoom == 0)
            break;

        --zoom;
    }

    return range;
}

/**
 */
QImage remapToLonLat(const QImage& mosaic,
                     const TileRange& range,
                     int tile_size,
                     const QRectF& lonlat,
                     const QSize& size)
{
    if (size.width() <= 0 || size.height() <= 0)
        return {};

    QImage out(size, QImage::Format_ARGB32);
    out.fill(Qt::transparent);

    if (mosaic.isNull() || !range.valid() || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return out;

    const QImage src = mosaic.format() == QImage::Format_ARGB32 ? mosaic : mosaic.convertToFormat(QImage::Format_ARGB32);

    const int w = size.width();
    const int h = size.height();

    //longitude is linear in both projections => one source column per target column
    std::vector<int> src_x(w, -1);

    for (int i = 0; i < w; ++i)
    {
        const double lon = lonlat.left() + (i + 0.5) / w * lonlat.width();
        const double tx  = (lonToTileX(lon, range.zoom) - range.x0) * tile_size;
        const int    sx  = (int)std::floor(tx);

        if (lon >= -180.0 && lon <= 180.0 && sx >= 0 && sx < src.width())
            src_x[ i ] = sx;
    }

    //latitude => one source row per target row (top row = north)
    for (int j = 0; j < h; ++j)
    {
        const double lat = lonlat.bottom() - (j + 0.5) / h * lonlat.height();

        if (std::fabs(lat) > MaxLatitude)
            continue;

        const double ty = (latToTileY(lat, range.zoom) - range.y0) * tile_size;
        const int    sy = (int)std::floor(ty);

        if (sy < 0 || sy >= src.height())
            continue;

        const QRgb* src_line = reinterpret_cast<const QRgb*>(src.constScanLine(sy));
        QRgb*       out_line = reinterpret_cast<QRgb*>(out.scanLine(j));

        for (int i = 0; i < w; ++i)
            if (src_x[ i ] >= 0)
                out_line[ i ] = src_line[ src_x[ i ] ];
    }

    return out;
}

} // namespace mercator

/**
 */
LonLatLine makeLine(QPolygonF points)
{
    LonLatLine line;
    line.bounds = points.boundingRect();
    line.points = std::move(points);

    return line;
}

namespace
{
    void addCurve(const OGRSimpleCurve& curve, std::vector<LonLatLine>& lines)
    {
        const int n = curve.getNumPoints();
        if (n < 2)
            return;

        QPolygonF points;
        points.reserve(n);

        for (int i = 0; i < n; ++i)
            points.append(QPointF(curve.getX(i), curve.getY(i)));

        lines.push_back(makeLine(std::move(points)));
    }

    void addGeometry(const OGRGeometry* geometry, std::vector<LonLatLine>& lines)
    {
        if (!geometry)
            return;

        switch (wkbFlatten(geometry->getGeometryType()))
        {
            case wkbPolygon:
            {
                auto polygon = geometry->toPolygon();

                if (auto ring = polygon->getExteriorRing())
                    addCurve(*ring, lines);

                for (int i = 0; i < polygon->getNumInteriorRings(); ++i)
                    addCurve(*polygon->getInteriorRing(i), lines);

                break;
            }
            case wkbLineString:
            case wkbLinearRing:
            {
                addCurve(*geometry->toLineString(), lines);
                break;
            }
            case wkbMultiPolygon:
            case wkbMultiLineString:
            case wkbGeometryCollection:
            {
                auto collection = geometry->toGeometryCollection();

                for (int i = 0; i < collection->getNumGeometries(); ++i)
                    addGeometry(collection->getGeometryRef(i), lines);

                break;
            }
            default:
                break;
        }
    }
}

/**
 */
std::vector<LonLatLine> readVectorFileLines(const std::string& path)
{
    std::vector<LonLatLine> lines;

    GDALAllRegister();

    GDALDataset* data_set = (GDALDataset*)GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);

    if (!data_set)
    {
        logerr << "could not open '" << path << "'";
        return lines;
    }

    for (int l = 0; l < data_set->GetLayerCount(); ++l)
    {
        OGRLayer* layer = data_set->GetLayer(l);
        if (!layer)
            continue;

        layer->ResetReading();

        OGRFeature* feature = nullptr;

        while ((feature = layer->GetNextFeature()) != nullptr)
        {
            addGeometry(feature->GetGeometryRef(), lines);
            OGRFeature::DestroyFeature(feature);
        }
    }

    GDALClose(data_set);

    loginf << "read " << lines.size() << " lines from '" << path << "'";

    return lines;
}

namespace viewport
{

namespace
{
    QRectF scaled(const QRectF& r, double fx, double fy)
    {
        const QPointF c = r.center();
        const double  w = r.width()  * fx;
        const double  h = r.height() * fy;

        return QRectF(c.x() - w / 2.0, c.y() - h / 2.0, w, h);
    }

    double lonScale(double lat)
    {
        return std::max(0.01, std::cos(lat * M_PI / 180.0));
    }
}

/**
 */
QRectF adjustAspect(const QRectF& lonlat, const QSizeF& plot_size)
{
    if (plot_size.width() <= 0.0 || plot_size.height() <= 0.0 || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return lonlat;

    const double c       = lonScale(lonlat.center().y());
    const double desired = plot_size.width() / plot_size.height();
    const double current = lonlat.width() * c / lonlat.height();

    if (current < desired)
        return scaled(lonlat, desired / current, 1.0);

    return scaled(lonlat, 1.0, current / desired);
}

/**
 */
QRectF clamp(const QRectF& lonlat,
             const QRectF& max_extent,
             double min_width,
             double min_height)
{
    if (lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return lonlat;

    QRectF r = lonlat;

    //minimum size, aspect kept
    {
        const double f = std::max(min_width / r.width(), min_height / r.height());
        if (f > 1.0)
            r = scaled(r, f, f);
    }

    if (max_extent.width() <= 0.0 || max_extent.height() <= 0.0)
        return r;

    //maximum size, aspect kept
    {
        const double f = std::min(max_extent.width() / r.width(), max_extent.height() / r.height());
        if (f < 1.0)
            r = scaled(r, f, f);
    }

    //move inside
    double dx = 0.0;
    if (r.left() < max_extent.left())
        dx = max_extent.left() - r.left();
    else if (r.right() > max_extent.right())
        dx = max_extent.right() - r.right();

    double dy = 0.0;
    if (r.top() < max_extent.top())
        dy = max_extent.top() - r.top();
    else if (r.bottom() > max_extent.bottom())
        dy = max_extent.bottom() - r.bottom();

    r.translate(dx, dy);

    return r;
}

/**
 */
QRectF maxExtent(const QRectF& grid_bounds, const QSizeF& plot_size, double margin)
{
    QRectF r = grid_bounds.adjusted(-grid_bounds.width()  * margin,
                                    -grid_bounds.height() * margin,
                                     grid_bounds.width()  * margin,
                                     grid_bounds.height() * margin);

    return adjustAspect(r, plot_size);
}

/**
 */
double minWidth(double cell_width, double plot_width_px)
{
    const double tile_limit = plot_width_px * 360.0 / (TileSize * std::ldexp(1.0, MaxTileZoom));

    return std::max(cell_width / 4.0, tile_limit);
}

} // namespace viewport

} // namespace gridbackground
