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

#include "osgtools.h"

#include "logger.h"

#include <QImage>
#include <QPixmap>
#include <QIcon>
#include <QSize>

#include <osg/Image>

namespace Utils
{
namespace OSG
{

/**
 */
extern osg::Image* createImage(const QImage& img)
{
    QImage img_rgba = img.convertToFormat(QImage::Format_RGBA8888).mirrored(false, true);

    auto osg_image = new osg::Image();
    osg_image->allocateImage(img_rgba.width(), img_rgba.height(), 1, GL_RGBA, GL_UNSIGNED_BYTE);

    memcpy(osg_image->data(), img_rgba.bits(), img_rgba.width() * img_rgba.height() * 4);

    return osg_image;
}

/**
 */
extern osg::Image* createImage(const QPixmap& pixmap)
{
    return createImage(pixmap.toImage());
}

/**
 */
extern osg::Image* createImage(const QIcon& icon, const QSize& size)
{
    return createImage(icon.pixmap(size));
}

}  // namespace OSG
}  // namespace Utils
