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

#include "gridbackground_defs.h"
#include "gridbackgroundtilesource.h"

#include <QColor>
#include <QImage>
#include <QObject>
#include <QRectF>
#include <QSize>
#include <QTimer>

#include <boost/date_time/posix_time/ptime.hpp>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class GridView;
class QPainter;

/**
 * Background of the Grid View for longitude / latitude grids:
 * - below the grid: background color, map (osgEarth tiles, see GridBackgroundTileSource)
 * - above the grid: national borders, sectors (outlines)
 *
 * Map images are requested on a worker thread a short time after the last viewport change.
 * Until the new image is ready, the old image is drawn at its own area (moved and scaled).
 * Results of older requests are discarded (generation counter).
 */
class GridBackgroundRenderer : public QObject
{
    Q_OBJECT
signals:
    /// the display changed (e.g. a map image arrived)
    void updateSignal();

public:
    explicit GridBackgroundRenderer(GridView& view, QObject* parent = nullptr);
    ~GridBackgroundRenderer() override;

    /// the grid is a longitude / latitude grid, set by the data widget
    void setAvailable(bool available);
    bool available() const { return available_; }
    /// available and switched on
    bool active() const;

    /// the view's background settings changed
    void settingsChanged();
    /// the sectors of the data context changed
    void sectorsChanged();

    /// background color and map, requests a new map image if the viewport changed
    void paintBelow(QPainter& painter, const QRectF& plot_rect, const QRectF& lonlat);
    /// national borders and sectors
    void paintAbove(QPainter& painter, const QRectF& plot_rect, const QRectF& lonlat);

    /// before an export: updates the map image of the viewport, waiting as given (max_time in seconds)
    void prepareExport(const QRectF& lonlat,
                       const QSize& size,
                       gridbackground::ExportWait wait = gridbackground::DefaultExportWait,
                       unsigned int max_time = gridbackground::DefaultExportWaitTime);

    /// the map could not be loaded, until the map is switched off and on or the file changes
    bool mapUnavailable() const { return map_unavailable_; }

private:
    struct Guard;

    struct SectorLine
    {
        gridbackground::LonLatLine line;
        QColor                     color;
        std::string                layer;
        unsigned int               id = 0;
    };

    bool mapShown() const;
    bool ensureTileSource();
    void requestMap(const QRectF& lonlat, const QSize& size);
    void startMapRequest();
    void mapRequestDone(unsigned int generation, const GridBackgroundTileSource::Result& result);
    void cancelMapRequest();

    void updateSectors();
    void paintAboveImage(QImage& image, const QRectF& lonlat);

    GridView& view_;

    bool available_ = false;

    // map
    std::shared_ptr<GridBackgroundTileSource> tile_source_;
    std::string                               tile_source_file_;
    bool                                      map_unavailable_ = false;
    bool                                      last_map_shown_  = false;

    QImage map_image_;
    QRectF map_image_lonlat_;

    QRectF requested_lonlat_;
    QSize  requested_size_;

    QTimer                             request_timer_;
    boost::posix_time::ptime           request_start_; // for the timing log
    unsigned int                       generation_ = 0;
    std::shared_ptr<std::atomic<bool>> cancel_flag_;
    std::shared_ptr<Guard>             guard_;

    // vectors
    std::optional<std::vector<gridbackground::LonLatLine>> borders_;
    std::optional<std::vector<SectorLine>>                 sectors_;

    // cached image of the layers above the grid
    QImage       above_image_;
    QRectF       above_lonlat_;
    unsigned int above_revision_ = 0;
    unsigned int revision_       = 1; // increased on every settings / sector change
};
