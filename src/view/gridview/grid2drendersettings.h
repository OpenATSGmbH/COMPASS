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

#include "colormap.h"
#include "colorlegend.h"

#include <boost/optional.hpp>

#include "json.hpp"

/**
*/
struct Grid2DRenderSettings
{
    /// Order of the value entries in the legend.
    enum class LegendOrder
    {
        Normal = 0, // first color of the color map at the top
        Reversed    // last color of the color map at the top
    };

    ColorMap                color_map;
    boost::optional<double> min_value;
    boost::optional<double> max_value;
    int                     pixels_per_cell = 1;

    LegendOrder             legend_order = LegendOrder::Normal;
    boost::optional<int>    legend_precision; // number of decimals in the legend, display default if not set
    std::string             legend_unit;      // unit after each number in the legend (e.g. "m", "%"), none if empty
    ColorLegend             legend_entries;   // fixed legend entries (e.g. names of categories), replace the
                                              // entries from the color map if not empty

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& obj);

    /// Legend of the color map, or legend_entries as given if not empty (order, precision and unit
    /// are not applied to them). The value range is min_value / max_value if both are set,
    /// else the value range of the color map. Number format: legend_precision if set,
    /// else the given decorator, else 2 decimals, followed by " " + legend_unit if it is set.
    /// The selected / null entries are added after the value entries, independent of the legend order.
    ColorLegend legend(bool add_sel_color = false,
                       bool add_null_color = false,
                       const ColorMap::ValueDecorator& decorator = ColorMap::ValueDecorator()) const;

    bool show_selected = true;
};
