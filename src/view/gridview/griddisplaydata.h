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

#include "grid2dlayer.h"
#include "grid2drendersettings.h"
#include "gridbackgroundsettings.h"
#include "property.h"
#include "representation.h"

#include <QColor>
#include <QPointF>
#include <QRectF>

#include <boost/optional.hpp>

#include <string>
#include <vector>

/**
 * Text label on a geographic grid, from a text feature of the view point.
 */
struct GridTextLabel
{
    /// side of the position the text is drawn to, as the text feature defines it
    enum class Direction
    {
        RightUp = 0,
        RightDown,
        LeftUp,
        LeftDown
    };

    QPointF     position;            // x = longitude, y = latitude
    std::string text;
    QColor      color     = Qt::black;
    double      font_size = 10.0;    // pixels of the layout
    Direction   direction = Direction::RightUp;
};

/**
 * Output of GridViewModel, the only data the grid render widget draws from.
 */
struct GridDisplayData
{
    boost::optional<Grid2DLayer>            layer;           // values and reference
    boost::optional<Grid2DRenderSettings>   render_settings; // of the annotation, wins over the render options
    bool                                    is_geographic = false; // longitude / latitude grid, has a background
    std::string                             title;
    std::string                             x_axis_label;
    std::string                             y_axis_label;

    // Data type and representation per axis (0 = x, 1 = y), for the tick labels
    PropertyDataType                        axis_data_type[ 2 ] = { PropertyDataType::DOUBLE,
                                                                    PropertyDataType::DOUBLE };
    dbContent::Representation               axis_repr[ 2 ]      = { dbContent::Representation::STANDARD,
                                                                    dbContent::Representation::STANDARD };

    // of the legend values
    PropertyDataType                        legend_data_type = PropertyDataType::DOUBLE;
    dbContent::Representation               legend_repr      = dbContent::Representation::STANDARD;

    QRectF                                  database_extent; // empty chart and zoom limits, empty if not used
    std::vector<GridSectorLine>             sectors;         // all sectors of the data context, as lines
    std::vector<GridTextLabel>              labels;          // text labels of a geographic grid
    boost::optional<GridBackgroundSettings> background;      // from the view point, wins over the options

    // Identity of the value domain of the axes. The render widget keeps the zoom of the user
    // while the id stays the same.
    std::string                             domain_id;

    bool                                    valid = false;   // false: the render widget shows the empty chart
};

/**
 * Settings of the grid drawing that come from the owner of the render widget, not from the data.
 * The defaults are the defaults of the Grid View.
 */
struct GridRenderOptions
{
    GridRenderOptions()
    {
        render_settings.color_map.create(colorscale::ColorScale::Green2Red, 10);
    }

    Grid2DRenderSettings   render_settings;    // color scale, steps, log scale, minimum, maximum
    unsigned int           grid_opacity = 100; // percent
    GridBackgroundSettings background;         // color, map, map opacity, borders, sectors, hidden sectors
};
