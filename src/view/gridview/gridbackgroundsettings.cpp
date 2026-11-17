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

#include "gridbackgroundsettings.h"
#include "logger.h"

#include "json.hpp"

const std::string GridBackgroundSettings::TagShow               = "show";
const std::string GridBackgroundSettings::TagColor              = "color";
const std::string GridBackgroundSettings::TagMapShow            = "map_show";
const std::string GridBackgroundSettings::TagMapFile            = "map_file";
const std::string GridBackgroundSettings::TagMapOpacity         = "map_opacity";
const std::string GridBackgroundSettings::TagBordersShow        = "borders_show";
const std::string GridBackgroundSettings::TagSectorsShow        = "sectors_show";
const std::string GridBackgroundSettings::TagHiddenSectorLayers = "hidden_sector_layers";
const std::string GridBackgroundSettings::TagHiddenSectors      = "hidden_sectors";

/**
 */
bool GridBackgroundSettings::operator==(const GridBackgroundSettings& other) const
{
    return show                 == other.show                 &&
           color                == other.color                &&
           map_show             == other.map_show             &&
           map_file             == other.map_file             &&
           map_opacity          == other.map_opacity          &&
           borders_show         == other.borders_show         &&
           sectors_show         == other.sectors_show         &&
           hidden_sector_layers == other.hidden_sector_layers &&
           hidden_sectors       == other.hidden_sectors;
}

/**
 */
nlohmann::json GridBackgroundSettings::toJSON() const
{
    nlohmann::json obj;

    obj[ TagShow               ] = show;
    obj[ TagColor              ] = color;
    obj[ TagMapShow            ] = map_show;
    obj[ TagMapFile            ] = map_file;
    obj[ TagMapOpacity         ] = map_opacity;
    obj[ TagBordersShow        ] = borders_show;
    obj[ TagSectorsShow        ] = sectors_show;
    obj[ TagHiddenSectorLayers ] = hidden_sector_layers;
    obj[ TagHiddenSectors      ] = hidden_sectors;

    return obj;
}

/**
 */
bool GridBackgroundSettings::fromJSON(const nlohmann::json& obj)
{
    if (!obj.is_object())
    {
        logerr << "background settings are no JSON object";
        return false;
    }

    GridBackgroundSettings read;

    try
    {
        auto readField = [ & ] (const std::string& tag, auto& value)
        {
            auto it = obj.find(tag);
            if (it != obj.end())
                value = it->template get<std::remove_reference_t<decltype(value)>>();
        };

        readField(TagShow              , read.show                );
        readField(TagColor             , read.color               );
        readField(TagMapShow           , read.map_show            );
        readField(TagMapFile           , read.map_file            );
        readField(TagMapOpacity        , read.map_opacity         );
        readField(TagBordersShow       , read.borders_show        );
        readField(TagSectorsShow       , read.sectors_show        );
        readField(TagHiddenSectorLayers, read.hidden_sector_layers);
        readField(TagHiddenSectors     , read.hidden_sectors      );
    }
    catch (const nlohmann::json::exception& e)
    {
        logerr << "background settings not readable: " << e.what();
        return false;
    }

    *this = read;

    return true;
}
