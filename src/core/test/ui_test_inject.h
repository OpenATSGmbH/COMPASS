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

#include "ui_test_common.h"

#include <boost/optional.hpp>

#include <QRect>

class QWidget;
class QString;

namespace ui_test
{
    struct Coord
    {
        enum class CoordSystem
        {   
            Unknown = 0,
            Pixels,
            Percent
        };

        bool valid() const
        {
            return (value.has_value() && coord_system != CoordSystem::Unknown);
        }

        int getValue(double range_min = 0.0, double range_max = 1.0) const
        {
            if (!valid())
                return 0;

            if (coord_system == CoordSystem::Pixels)
                return (int)value.value();
            
            //percentage given
            return (int)((1.0 - value.value()) * range_min + value.value() * range_max);
        }

        boost::optional<double> value;
        CoordSystem             coord_system = CoordSystem::Unknown;
    };
    
    bool injectUIEvent(QWidget* parent, 
                       const QString& obj_name, 
                       const QString& event, 
                       int delay = -1);

    bool injectMouseClick(QWidget* w,
                          Qt::MouseButton button,
                          const Coord& xcoord,
                          const Coord& ycoord,
                          int delay = -1);
    bool injectMouseRect(QWidget* w,
                         Qt::MouseButton button,
                         const Coord& xcoord0,
                         const Coord& ycoord0,
                         const Coord& xcoord1,
                         const Coord& ycoord1,
                         int delay = -1);

} // namespace ui_test
