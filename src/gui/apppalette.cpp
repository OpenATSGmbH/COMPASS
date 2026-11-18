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

#include "apppalette.h"

#include <QStyle>
#include <QStyleFactory>

#include <memory>

namespace app_palette
{

/**
 * The standard palette of the Fusion style, not a palette captured at startup: a captured palette
 * has only some roles set, so the other roles of a dark palette would stay after a switch to light.
 */
QPalette light()
{
    std::unique_ptr<QStyle> fusion(QStyleFactory::create("Fusion"));

    return fusion ? fusion->standardPalette() : QPalette();
}

}
