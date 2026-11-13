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

#include "gridbackgroundrenderer.h"
#include "gridview.h"

#include "viewmanager.h"
#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"

#include "logger.h"

#include <QApplication>
#include <QFileInfo>
#include <QPainter>
#include <QTransform>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <chrono>
#include <mutex>
#include <thread>

namespace
{
    //as in the minimal 2D map of the Geographic View (minimal_2d.earth)
    const QColor BorderColor(255, 255, 0);
    const double BorderWidth = 2.0;

    /// pixel rectangle of a lon/lat area in the plot showing the lon/lat viewport
    QRectF lonLatToPlot(const QRectF& area, const QRectF& plot_rect, const QRectF& lonlat)
    {
        const double sx = plot_rect.width()  / lonlat.width();
        const double sy = plot_rect.height() / lonlat.height();

        const double x0 = plot_rect.left() + (area.left()   - lonlat.left()) * sx;
        const double x1 = plot_rect.left() + (area.right()  - lonlat.left()) * sx;
        const double y0 = plot_rect.top()  + (lonlat.bottom() - area.bottom()) * sy; // north
        const double y1 = plot_rect.top()  + (lonlat.bottom() - area.top())    * sy; // south

        return QRectF(x0, y0, x1 - x0, y1 - y0);
    }

    /// transform from lon/lat to the pixels of an image of the given size showing the viewport
    QTransform lonLatTransform(const QSizeF& size, const QRectF& lonlat)
    {
        const double sx = size.width()  / lonlat.width();
        const double sy = size.height() / lonlat.height();

        return QTransform(sx, 0.0, 0.0, -sy, -lonlat.left() * sx, lonlat.bottom() * sy);
    }

    QColor toColor(const std::string& color_str, const QColor& fallback)
    {
        QColor c(QString::fromStdString(color_str));
        return c.isValid() ? c : fallback;
    }
}

/**
 * Lets a worker reach the renderer as long as it exists.
 */
struct GridBackgroundRenderer::Guard
{
    std::mutex              mutex;
    GridBackgroundRenderer* owner = nullptr;
};

/**
 */
GridBackgroundRenderer::GridBackgroundRenderer(GridView& view, QObject* parent)
:   QObject(parent)
,   view_  (view)
,   guard_ (std::make_shared<Guard>())
{
    guard_->owner = this;

    request_timer_.setSingleShot(true);
    request_timer_.setInterval(gridbackground::RequestDelayMs);

    connect(&request_timer_, &QTimer::timeout, this, &GridBackgroundRenderer::startMapRequest);

    auto& ctx_man = view_.viewManager().dbContextManager();

    connect(&ctx_man, &context::DBContextManager::sectorsChangedSignal, this, &GridBackgroundRenderer::sectorsChanged);
    connect(&ctx_man, &context::DBContextManager::activeContextChangedSignal, this, &GridBackgroundRenderer::sectorsChanged);

    last_map_shown_ = view_.settings().background_map_show;
}

/**
 */
GridBackgroundRenderer::~GridBackgroundRenderer()
{
    cancelMapRequest();

    std::lock_guard<std::mutex> lock(guard_->mutex);
    guard_->owner = nullptr;
}

/**
 */
void GridBackgroundRenderer::setAvailable(bool available)
{
    available_ = available;
}

/**
 */
bool GridBackgroundRenderer::active() const
{
    return available_ && view_.settings().background_show;
}

/**
 */
bool GridBackgroundRenderer::mapShown() const
{
    return active() && view_.settings().background_map_show;
}

/**
 */
void GridBackgroundRenderer::settingsChanged()
{
    const auto& settings = view_.settings();

    //switching the map off and on, or another file => try again
    const bool file_changed = !tile_source_file_.empty() &&
                              QFileInfo(QString::fromStdString(tile_source_file_)).fileName().toStdString() != settings.background_map_file;

    if (file_changed || (settings.background_map_show && !last_map_shown_))
    {
        map_unavailable_ = false;
        requested_lonlat_ = QRectF();
        requested_size_   = QSize();
    }

    if (file_changed)
    {
        cancelMapRequest();
        map_image_ = QImage();
    }

    last_map_shown_ = settings.background_map_show;

    ++revision_;

    emit updateSignal();
}

/**
 */
void GridBackgroundRenderer::sectorsChanged()
{
    sectors_.reset();
    ++revision_;

    emit updateSignal();
}

/**
 */
bool GridBackgroundRenderer::ensureTileSource()
{
    const std::string path = gridbackground::mapDirectory() + view_.settings().background_map_file;

    if (!tile_source_ || tile_source_file_ != path)
    {
        tile_source_      = std::make_shared<GridBackgroundTileSource>(path);
        tile_source_file_ = path;
    }

    return tile_source_->valid();
}

/**
 */
void GridBackgroundRenderer::requestMap(const QRectF& lonlat, const QSize& size)
{
    if (map_unavailable_ || size.isEmpty() || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return;

    if (lonlat == requested_lonlat_ && size == requested_size_)
        return;

    requested_lonlat_ = lonlat;
    requested_size_   = size;

    //restarted on every change => requested after the last one
    request_timer_.start();
}

/**
 */
void GridBackgroundRenderer::cancelMapRequest()
{
    request_timer_.stop();

    if (cancel_flag_)
        *cancel_flag_ = true;

    cancel_flag_.reset();

    //a pending result is discarded
    ++generation_;
}

/**
 */
void GridBackgroundRenderer::startMapRequest()
{
    if (!mapShown() || map_unavailable_)
        return;

    if (!ensureTileSource())
    {
        logwrn << "map '" << view_.settings().background_map_file << "' not available";

        map_unavailable_ = true;
        emit updateSignal();
        return;
    }

    cancelMapRequest();

    const unsigned int generation = generation_;
    const QRectF       lonlat     = requested_lonlat_;
    const QSize        size       = requested_size_;

    auto cancel = std::make_shared<std::atomic<bool>>(false);
    cancel_flag_ = cancel;

    auto source = tile_source_;
    auto guard  = guard_;

    request_start_ = boost::posix_time::microsec_clock::local_time();

    loginf << "request " << generation << " lon " << lonlat.left() << " " << lonlat.right()
           << " lat " << lonlat.top() << " " << lonlat.bottom()
           << " size " << size.width() << "x" << size.height();

    std::thread([ source, guard, cancel, generation, lonlat, size ] ()
    {
        auto result = source->render(lonlat, size, [ cancel ] () { return cancel->load(); });

        std::lock_guard<std::mutex> lock(guard->mutex);

        if (!guard->owner)
            return;

        //queued => runs in the main thread, dropped if the renderer is deleted before
        GridBackgroundRenderer* owner = guard->owner;

        QMetaObject::invokeMethod(owner,
                                  [ owner, generation, result ] () { owner->mapRequestDone(generation, result); },
                                  Qt::QueuedConnection);
    }).detach();
}

/**
 */
void GridBackgroundRenderer::mapRequestDone(unsigned int generation, const GridBackgroundTileSource::Result& result)
{
    if (generation != generation_)
    {
        loginf << "request " << generation << " discarded, newer request " << generation_;
        return; // an older request
    }

    loginf << "request " << generation << " done after "
           << (boost::posix_time::microsec_clock::local_time() - request_start_).total_milliseconds() << " ms"
           << " tiles " << result.tiles_total << " failed " << result.tiles_failed << " canceled " << result.canceled;

    cancel_flag_.reset();

    if (result.canceled)
        return;

    if (result.failed())
    {
        logwrn << "no map tiles of '" << view_.settings().background_map_file << "' available";

        map_unavailable_ = true;
    }
    else
    {
        map_image_        = result.image;
        map_image_lonlat_ = result.lonlat;
    }

    emit updateSignal();
}

/**
 */
void GridBackgroundRenderer::prepareExport(const QRectF& lonlat,
                                           const QSize& size,
                                           gridbackground::ExportWait wait,
                                           unsigned int max_time)
{
    if (!mapShown() || map_unavailable_ || size.isEmpty())
        return;

    if (!map_image_.isNull() && map_image_lonlat_ == lonlat && map_image_.size() == size)
        return; // up to date

    if (wait == gridbackground::ExportWait::NoWait)
        return;

    if (!ensureTileSource())
        return;

    cancelMapRequest();

    const auto start = std::chrono::steady_clock::now();

    auto cancel = [ & ] ()
    {
        if (wait != gridbackground::ExportWait::WaitMaxTime)
            return false;

        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() > max_time;
    };

    QApplication::setOverrideCursor(Qt::WaitCursor);
    auto result = tile_source_->render(lonlat, size, cancel);
    QApplication::restoreOverrideCursor();

    loginf << "export map done after "
           << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << " ms"
           << " tiles " << result.tiles_total << " failed " << result.tiles_failed << " timeout " << result.canceled;

    if (result.failed())
        return;

    map_image_        = result.image;
    map_image_lonlat_ = result.lonlat;

    //complete => no new request for this viewport
    if (!result.canceled)
    {
        requested_lonlat_ = lonlat;
        requested_size_   = size;
    }
}

/**
 */
void GridBackgroundRenderer::paintBelow(QPainter& painter, const QRectF& plot_rect, const QRectF& lonlat)
{
    if (!active() || plot_rect.isEmpty() || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return;

    const auto& settings = view_.settings();

    painter.save();
    painter.setClipRect(plot_rect);

    painter.fillRect(plot_rect, toColor(settings.background_color, Qt::white));

    if (settings.background_map_show)
    {
        requestMap(lonlat, plot_rect.size().toSize());

        if (!map_image_.isNull() && map_image_lonlat_.width() > 0.0 && map_image_lonlat_.height() > 0.0)
        {
            painter.setOpacity(settings.background_map_opacity / 100.0);
            painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
            painter.drawImage(lonLatToPlot(map_image_lonlat_, plot_rect, lonlat), map_image_);
        }
    }

    painter.restore();
}

/**
 */
void GridBackgroundRenderer::updateSectors()
{
    if (sectors_.has_value())
        return;

    auto t0 = boost::posix_time::microsec_clock::local_time();

    sectors_ = std::vector<SectorLine>();

    auto& ctx_man = view_.viewManager().dbContextManager();

    if (!ctx_man.hasActiveContext() || !ctx_man.sectorsLoaded())
        return;

    for (const auto& layer : ctx_man.sectorLayers())
    {
        for (const auto& sector : layer->sectors())
        {
            const auto& points = sector->points();
            if (points.size() < 2)
                continue;

            QPolygonF poly;
            poly.reserve((int)points.size() + 1);

            for (const auto& p : points)
                poly.append(QPointF(p.second, p.first)); // first = latitude, second = longitude

            if (poly.first() != poly.last())
                poly.append(poly.first());

            sectors_->push_back({ gridbackground::makeLine(std::move(poly)),
                                  toColor(sector->colorStr(), Qt::black),
                                  layer->name(),
                                  sector->id() });
        }
    }

    loginf << "sectors " << sectors_->size() << " in "
           << (boost::posix_time::microsec_clock::local_time() - t0).total_milliseconds() << " ms";
}

/**
 */
void GridBackgroundRenderer::paintAboveImage(QImage& image, const QRectF& lonlat)
{
    const auto& settings = view_.settings();

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setBrush(Qt::NoBrush);

    const QTransform t = lonLatTransform(image.size(), lonlat);

    if (settings.background_show && settings.background_borders_show)
    {
        if (!borders_.has_value())
        {
            auto t0 = boost::posix_time::microsec_clock::local_time();

            borders_ = gridbackground::readVectorFileLines(gridbackground::mapDirectory() + "shapefiles/TM_WORLD_BORDERS-0.3.shp");

            loginf << "borders " << borders_->size() << " lines in "
                   << (boost::posix_time::microsec_clock::local_time() - t0).total_milliseconds() << " ms";
        }

        QPen pen(BorderColor);
        pen.setWidthF(BorderWidth);
        pen.setCosmetic(true);
        painter.setPen(pen);

        for (const auto& line : *borders_)
            if (line.bounds.intersects(lonlat))
                painter.drawPolyline(t.map(line.points));
    }

    if (settings.background_sectors_show)
    {
        updateSectors();

        const auto hidden_layers  = view_.backgroundHiddenSectorLayers();
        const auto hidden_sectors = view_.backgroundHiddenSectors();

        for (const auto& sector : *sectors_)
        {
            if (!sector.line.bounds.intersects(lonlat) && !lonlat.contains(sector.line.bounds))
                continue;

            if (hidden_layers.count(sector.layer) || hidden_sectors.count(sector.id))
                continue;

            QPen pen(sector.color);
            pen.setWidthF(2.0);
            pen.setCosmetic(true);
            painter.setPen(pen);

            painter.drawPolyline(t.map(sector.line.points)); // outlines only, never filled
        }
    }
}

/**
 */
void GridBackgroundRenderer::paintAbove(QPainter& painter, const QRectF& plot_rect, const QRectF& lonlat)
{
    if (!available_ || plot_rect.isEmpty() || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return;

    const auto& settings = view_.settings();

    //sectors are a top level item, independent of the Background Map
    const bool borders = settings.background_show && settings.background_borders_show;

    if (!borders && !settings.background_sectors_show)
        return;

    const QSize size = plot_rect.size().toSize();
    if (size.isEmpty())
        return;

    //redrawn only on changes of viewport, size, settings or sectors
    if (above_image_.size() != size || above_lonlat_ != lonlat || above_revision_ != revision_)
    {
        auto t0 = boost::posix_time::microsec_clock::local_time();

        above_image_ = QImage(size, QImage::Format_ARGB32_Premultiplied);
        above_image_.fill(Qt::transparent);

        paintAboveImage(above_image_, lonlat);

        above_lonlat_   = lonlat;
        above_revision_ = revision_;

        logdbg << "borders and sectors drawn in "
               << (boost::posix_time::microsec_clock::local_time() - t0).total_milliseconds() << " ms";
    }

    painter.save();
    painter.setClipRect(plot_rect);
    painter.drawImage(plot_rect.topLeft(), above_image_);
    painter.restore();
}
