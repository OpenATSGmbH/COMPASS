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

#include "gridbackgroundtilesource.h"
#include "gridbackground_defs.h"

#include "logger.h"

#include <osgEarth/Common>
#include <osgEarth/Registry>
#include <osgEarth/MapNode>
#include <osgEarth/ImageLayer>
#include <osgEarth/ImageUtils>
#include <osgEarth/Progress>
#include <osgEarth/TileKey>
#include <osgEarth/Profile>
#include <osgDB/ReadFile>

#include <QPainter>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

namespace
{
    /**
     * Progress callback which asks the cancel function.
     */
    class CancelProgress : public osgEarth::ProgressCallback
    {
    public:
        explicit CancelProgress(const GridBackgroundTileSource::CancelFunc& cancel) : cancel_(cancel) {}

    protected:
        bool shouldCancel() const override { return cancel_ && cancel_(); }

    private:
        GridBackgroundTileSource::CancelFunc cancel_;
    };

    /// copy of the osg image as QImage (top row = north)
    QImage toQImage(const osg::Image& image)
    {
        osg::ref_ptr<osg::Image> rgba = osgEarth::ImageUtils::convertToRGBA8(&image);
        if (!rgba.valid() || rgba->s() <= 0 || rgba->t() <= 0)
            return {};

        QImage img(rgba->data(), rgba->s(), rgba->t(), rgba->s() * 4, QImage::Format_RGBA8888);
        QImage copy = img.copy();

        //osg images start at the bottom row by default
        if (image.getOrigin() == osg::Image::BOTTOM_LEFT)
            copy = copy.mirrored(false, true);

        return copy;
    }
}

/**
 */
struct GridBackgroundTileSource::Impl
{
    osg::ref_ptr<osg::Node>                            node;
    osg::ref_ptr<const osgEarth::Profile>              profile;
    std::vector<osg::ref_ptr<osgEarth::ImageLayer>>    layers;
};

/**
 */
GridBackgroundTileSource::GridBackgroundTileSource(const std::string& earth_file_path)
:   file_path_(earth_file_path)
,   impl_     (new Impl)
{
    //the geographic view initializes osgEarth on its own as well, a second call is harmless
    static std::once_flag init_flag;
    std::call_once(init_flag, [] () { osgEarth::initialize(); });

    auto t0 = boost::posix_time::microsec_clock::local_time();

    impl_->node = osgDB::readNodeFile(file_path_);

    osgEarth::MapNode* map_node = impl_->node.valid() ? osgEarth::MapNode::get(impl_->node.get()) : nullptr;

    if (!map_node || !map_node->getMap())
    {
        logerr << "could not read map file '" << file_path_ << "'";
        impl_->node = nullptr;
        return;
    }

    osgEarth::Map* map = map_node->getMap();

    impl_->profile = map->getProfile();

    if (!impl_->profile.valid() || !impl_->profile->getSRS() || !impl_->profile->getSRS()->isSphericalMercator())
    {
        logerr << "map file '" << file_path_ << "' has no spherical mercator profile";
        impl_->node    = nullptr;
        impl_->profile = nullptr;
        return;
    }

    osgEarth::ImageLayerVector layers;
    map->getLayers(layers);

    for (const auto& layer : layers)
        if (layer.valid() && layer->isOpen() && layer->getVisible())
            impl_->layers.push_back(layer);

    auto t1 = boost::posix_time::microsec_clock::local_time();

    loginf << "map file '" << file_path_ << "' image layers " << impl_->layers.size()
           << " loaded in " << (t1 - t0).total_milliseconds() << " ms";
}

/**
 */
GridBackgroundTileSource::~GridBackgroundTileSource() = default;

/**
 */
bool GridBackgroundTileSource::valid() const
{
    return impl_->node.valid() && impl_->profile.valid() && !impl_->layers.empty();
}

/**
 */
GridBackgroundTileSource::Result GridBackgroundTileSource::render(const QRectF& lonlat,
                                                                  const QSize& size,
                                                                  const CancelFunc& cancel) const
{
    namespace merc = gridbackground::mercator;

    Result result;
    result.lonlat = lonlat;

    if (!valid() || size.width() <= 0 || size.height() <= 0 || lonlat.width() <= 0.0 || lonlat.height() <= 0.0)
        return result;

    const int zoom  = merc::zoomForResolution(lonlat.width() / size.width());
    const auto range = merc::tileRange(lonlat, zoom);

    if (!range.valid())
    {
        //outside of the mercator area
        result.image = QImage(size, QImage::Format_ARGB32);
        result.image.fill(Qt::transparent);
        return result;
    }

    const int ts = gridbackground::TileSize;

    auto t0 = boost::posix_time::microsec_clock::local_time();

    QImage mosaic(range.numX() * ts, range.numY() * ts, QImage::Format_ARGB32_Premultiplied);
    mosaic.fill(Qt::transparent);

    const int num_tiles   = range.numTiles();
    const int num_threads = std::max(1, std::min(gridbackground::TileThreads, num_tiles));

    {
        QPainter painter(&mosaic);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

        for (const auto& layer : impl_->layers)
        {
            //tiles of the layer loaded by a few threads at the same time (as in the Geographic View),
            //painted afterwards in this thread
            std::vector<QImage> tiles(num_tiles);
            std::atomic<int>    next_tile(0);
            std::atomic<bool>   canceled(false);
            std::atomic<size_t> loaded(0); // requested tiles
            std::atomic<size_t> failed(0);

            auto load = [ & ] ()
            {
                //own progress per thread, osgEarth progress callbacks have state
                osg::ref_ptr<CancelProgress> progress = new CancelProgress(cancel);

                while (!canceled)
                {
                    const int idx = next_tile++;
                    if (idx >= num_tiles)
                        break;

                    if (cancel && cancel())
                    {
                        canceled = true;
                        break;
                    }

                    const int x = range.x0 + idx % range.numX();
                    const int y = range.y0 + idx / range.numX();

                    osgEarth::TileKey key(range.zoom, x, y, impl_->profile.get());
                    osgEarth::GeoImage geo_image = layer->createImage(key, progress.get());

                    if (geo_image.valid() && geo_image.getImage())
                        tiles[ idx ] = toQImage(*geo_image.getImage());

                    //a tile canceled during the request counts as not requested
                    if (progress->isCanceled())
                    {
                        canceled = true;
                        break;
                    }

                    ++loaded;

                    if (tiles[ idx ].isNull())
                        ++failed;
                }
            };

            std::vector<std::thread> threads;
            threads.reserve(num_threads - 1);

            for (int i = 1; i < num_threads; ++i)
                threads.emplace_back(load);

            load();

            for (auto& t : threads)
                t.join();

            result.tiles_total  += loaded;
            result.tiles_failed += failed;

            //loaded tiles are kept on a cancel (e.g. export wait time exceeded)
            painter.setOpacity(layer->getOpacity());

            for (int idx = 0; idx < num_tiles; ++idx)
            {
                if (tiles[ idx ].isNull())
                    continue;

                const int x = idx % range.numX();
                const int y = idx / range.numX();

                painter.drawImage(QRect(x * ts, y * ts, ts, ts), tiles[ idx ]);
            }

            if (canceled)
            {
                result.canceled = true;
                break;
            }
        }
    }

    auto t1 = boost::posix_time::microsec_clock::local_time();

    result.image = merc::remapToLonLat(mosaic, range, ts, lonlat, size);

    auto t2 = boost::posix_time::microsec_clock::local_time();

    loginf << "zoom " << range.zoom
           << " tiles " << result.tiles_total << " failed " << result.tiles_failed << " canceled " << result.canceled
           << " size " << size.width() << "x" << size.height()
           << " tiles " << (t1 - t0).total_milliseconds() << " ms"
           << " remap " << (t2 - t1).total_milliseconds() << " ms";

    return result;
}
