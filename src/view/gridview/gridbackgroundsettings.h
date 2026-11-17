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
#include "json_fwd.hpp"

#include <QColor>

#include <set>
#include <string>

/**
 * Background of a longitude / latitude grid: what GridBackgroundRenderer draws.
 * The defaults are the defaults of the Grid View.
 */
struct GridBackgroundSettings
{
    bool                   show         = true;      // background color, map and national borders
    std::string            color        = "#FFFFFF";
    bool                   map_show     = false;     // offline PCs never wait for tiles by default
    std::string            map_file     = "openstreetmap_2d.earth";
    unsigned int           map_opacity  = 100;       // percent
    bool                   borders_show = true;
    bool                   sectors_show = true;      // independent of show
    std::set<std::string>  hidden_sector_layers;     // names of the sector layers that are not drawn
    std::set<unsigned int> hidden_sectors;           // ids of the sectors that are not drawn

    bool operator==(const GridBackgroundSettings& other) const;
    bool operator!=(const GridBackgroundSettings& other) const { return !(*this == other); }

    nlohmann::json toJSON() const;
    /// A missing field keeps its default. Returns false if obj is no object or a field has a wrong type.
    bool fromJSON(const nlohmann::json& obj);

    /// The background of a report figure without its own background set: national borders on black,
    /// map off (the first map file of the maps folder), all sectors.
    static GridBackgroundSettings reportDefaults();

    static const std::string TagShow;
    static const std::string TagColor;
    static const std::string TagMapShow;
    static const std::string TagMapFile;
    static const std::string TagMapOpacity;
    static const std::string TagBordersShow;
    static const std::string TagSectorsShow;
    static const std::string TagHiddenSectorLayers;
    static const std::string TagHiddenSectors;
};

/**
 * Outline of a sector of the data context, as the background draws it.
 */
struct GridSectorLine
{
    gridbackground::LonLatLine line;          // closed polygon, x = longitude, y = latitude
    QColor                     color;
    std::string                layer_name;
    std::string                sector_name;
    unsigned int               sector_id = 0; // the hidden sectors of the settings are ids
};
