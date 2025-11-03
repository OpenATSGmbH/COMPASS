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

#include "grid2drendersettings.h"

#include "logger.h"

/**
 * Converts the Grid2DRenderSettings to a JSON object.
*/
nlohmann::json Grid2DRenderSettings::toJSON() const
{
    nlohmann::json obj;
    // obj["color_map"] = color_map.toJSON();
    // if (min_value)
    //     obj["min_value"] = *min_value;
    // if (max_value)
    //     obj["max_value"] = *max_value;
    // obj["pixels_per_cell"] = pixels_per_cell;
    // obj["show_selected"] = show_selected;
    return obj;
}

/**
 * Parses the Grid2DRenderSettings from a JSON object.
 * Returns false if parsing fails.
*/
bool Grid2DRenderSettings::fromJSON(const nlohmann::json& obj)
{
    // if (!obj.contains("color_map"))
    // {
    //     logerr << "color_map not found";
    //     return false;
    // }
    // if (!color_map.fromJSON(obj.at("color_map")))
    // {
    //     logerr << "color_map parsing failed";
    //     return false;
    // }

    // if (obj.contains("min_value"))
    //     min_value = obj.at("min_value").get<double>();
    // else
    //     min_value.reset();

    // if (obj.contains("max_value"))
    //     max_value = obj.at("max_value").get<double>();
    // else
    //     max_value.reset();

    // pixels_per_cell = obj.value("pixels_per_cell", 1);
    // show_selected = obj.value("show_selected", true);

    return true;
}
