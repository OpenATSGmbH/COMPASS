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

#include <QImage>
#include <QPolygonF>
#include <QRectF>
#include <QSize>

#include <string>
#include <vector>

class QPainter;

/**
 * Background map of the Grid View (longitude / latitude grids), helpers without state.
 * Notice: lon/lat rectangles are in axis values, x = longitude, y = latitude,
 * top() = minimum latitude, bottom() = maximum latitude.
 */
namespace gridbackground
{

/// waiting for the map tiles when the view is rendered for an export
enum class ExportWait
{
    NoWait = 0,
    WaitMaxTime,
    WaitNoTimeout
};

/// used until the export dialogs set the waiting
constexpr ExportWait   DefaultExportWait     = ExportWait::WaitMaxTime;
constexpr unsigned int DefaultExportWaitTime = 10; // seconds

constexpr double ExtentMargin   = 0.1; // relative margin around the grid bounds for the maximum extent
constexpr int    MaxTileZoom    = 18;
constexpr int    TileSize       = 256; // pixels of a map tile
constexpr int    MaxTiles       = 150; // per map image, the zoom is lowered above
constexpr int    RequestDelayMs = 250; // delay after the last viewport change before the map is requested
constexpr int    TileThreads    = 4;   // tiles loaded at the same time, as the osgEarth terrain default (concurrency)

/**
 * An eligible map file in the maps folder.
 */
struct MapFile
{
    std::string file_name; // e.g. "openstreetmap_2d.earth"
    std::string title;     // file name without extension
};

/// true if the earth file content is a projected map with at least one image layer and without shaders
bool isEligibleMapFile(const std::string& earth_xml);
/// the eligible earth files in the folder, sorted by name
std::vector<MapFile> eligibleMapFiles(const std::string& directory);
/// folder of the map files (data directory)
std::string mapDirectory();
/// title of a map file (file name without extension)
std::string mapTitle(const std::string& file_name);

namespace mercator
{
    constexpr double MaxLatitude = 85.0511287798066;

    /// fractional tile x of the longitude at the zoom level
    double lonToTileX(double lon, int zoom);
    /// fractional tile y of the latitude at the zoom level (0 at the north border)
    double latToTileY(double lat, int zoom);
    /// longitude of the tile x
    double tileXToLon(double tile_x, int zoom);
    /// latitude of the tile y
    double tileYToLat(double tile_y, int zoom);

    /// lowest zoom level whose tile pixels are at least as fine as the given longitude resolution, max_zoom at most
    int zoomForResolution(double lon_deg_per_px, int max_zoom = MaxTileZoom);

    /**
     * Tile range covering a lon/lat area at a zoom level (inclusive, clamped to the valid tiles).
     */
    struct TileRange
    {
        int zoom = 0;
        int x0   = 0;
        int x1   = -1;
        int y0   = 0;
        int y1   = -1;

        int numX() const { return x1 - x0 + 1; }
        int numY() const { return y1 - y0 + 1; }
        int numTiles() const { return numX() * numY(); }
        bool valid() const { return numX() > 0 && numY() > 0; }
    };

    /// tile range of the lon/lat area, the zoom is lowered until max_tiles fit
    TileRange tileRange(const QRectF& lonlat, int zoom, int max_tiles = MaxTiles);

    /**
     * Resamples a mosaic of Mercator tiles (tile range, tile_size pixels per tile, top row = north) to an
     * image of the lon/lat area (equal steps in longitude and latitude, top row = north).
     * Pixels outside the Mercator range or the mosaic stay transparent.
     */
    QImage remapToLonLat(const QImage& mosaic,
                         const TileRange& range,
                         int tile_size,
                         const QRectF& lonlat,
                         const QSize& size);
}

/**
 * Polyline in lon/lat (x = longitude, y = latitude) with its bounds.
 */
struct LonLatLine
{
    QPolygonF points;
    QRectF    bounds;
};

/// lines of the polygon rings and line strings of a vector file (e.g. a shapefile), empty on error
std::vector<LonLatLine> readVectorFileLines(const std::string& path);
/// line with its bounds
LonLatLine makeLine(QPolygonF points);

/**
 * Viewport adjustments for a lon/lat plot.
 */
namespace viewport
{
    /// widens one side of the area so that the plot shows equal distances in both directions
    QRectF adjustAspect(const QRectF& lonlat, const QSizeF& plot_size);
    /**
     * Clamps the area to the limits:
     * - at most the maximum extent (scaled down, aspect kept), moved inside of it
     * - at least min_width / min_height (scaled up around the center)
     */
    QRectF clamp(const QRectF& lonlat,
                 const QRectF& max_extent,
                 double min_width,
                 double min_height);
    /// maximum extent: the grid bounds plus the relative margin, with the plot aspect ratio
    QRectF maxExtent(const QRectF& grid_bounds, const QSizeF& plot_size, double margin = ExtentMargin);
    /// minimum width of the view: the larger of a quarter cell and the tile zoom limit
    double minWidth(double cell_width, double plot_width_px);
}

} // namespace gridbackground
