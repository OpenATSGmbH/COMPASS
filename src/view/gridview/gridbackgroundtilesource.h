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
#include <QRectF>
#include <QSize>

#include <functional>
#include <memory>
#include <string>

/**
 * Map tiles of the image layers of an osgEarth earth file (spherical-mercator profile), rendered to
 * lon/lat images for the Grid View background.
 * - the earth file is read by osgEarth without a viewer (no OpenGL), same layer config and cache
 *   folders as in the Geographic View
 * - render() may run on a worker thread, several renders may run at the same time
 */
class GridBackgroundTileSource
{
public:
    /**
     * Result of a render.
     */
    struct Result
    {
        QImage      image;              // lon/lat image, transparent where no tile was available
        QRectF      lonlat;             // area of the image
        size_t      tiles_total  = 0;
        size_t      tiles_failed = 0;
        bool        canceled     = false;

        bool failed() const { return tiles_total > 0 && tiles_failed == tiles_total; }
    };

    using CancelFunc = std::function<bool()>;

    explicit GridBackgroundTileSource(const std::string& earth_file_path);
    ~GridBackgroundTileSource();

    bool valid() const;
    const std::string& filePath() const { return file_path_; }

    /// renders the lon/lat area to an image of the size, the cancel function stops between and in tile requests
    Result render(const QRectF& lonlat, const QSize& size, const CancelFunc& cancel) const;

private:
    struct Impl;

    std::string           file_path_;
    std::unique_ptr<Impl> impl_;
};
