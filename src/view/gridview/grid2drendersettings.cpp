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

#include <QColor>

/**
 * Converts the Grid2DRenderSettings to a JSON object.
 *
 * The full `ColorMap` is not round-tripped (it carries sampled QColor arrays);
 * instead the scale enum and step count are emitted so the consumer can rebuild
 * an equivalent map. `min_value` / `max_value` are emitted only when set.
*/
nlohmann::json Grid2DRenderSettings::toJSON() const
{
    nlohmann::json obj;

    if (min_value)
        obj["min_value"] = *min_value;
    if (max_value)
        obj["max_value"] = *max_value;

    if (color_map.valid())
    {
        obj["color_scale"] = static_cast<int>(color_map.colorScale());
        obj["color_steps"] = static_cast<int>(color_map.colorSteps());

        // Emit the resolved color stops so non-Qt consumers (compass_web)
        // don't have to recreate a ColorMap to colour their renderings.
        // Hex form (#RRGGBB) is straight Plotly-compatible; alpha is dropped
        // because Grid2DLayerRenderer's RGBA blending doesn't carry across
        // wire formats consistently (selection / null colours travel as
        // separate fields below).
        nlohmann::json colors = nlohmann::json::array();
        for (std::size_t i = 0; i < color_map.numColors(); ++i)
            colors.push_back(color_map.getColor(i).name(QColor::HexRgb).toStdString());
        obj["colors"] = std::move(colors);
    }

    obj["pixels_per_cell"] = pixels_per_cell;
    obj["show_selected"]   = show_selected;

    return obj;
}

/**
 * Parses the Grid2DRenderSettings from a JSON object.
 * Returns false if parsing fails. All fields are optional - a missing field
 * leaves the corresponding member unchanged from its default-constructed value.
*/
bool Grid2DRenderSettings::fromJSON(const nlohmann::json& obj)
{
    if (!obj.is_object())
    {
        logerr << "render_settings is not a JSON object";
        return false;
    }

    min_value.reset();
    max_value.reset();

    if (obj.contains("min_value") && obj.at("min_value").is_number())
        min_value = obj.at("min_value").get<double>();

    if (obj.contains("max_value") && obj.at("max_value").is_number())
        max_value = obj.at("max_value").get<double>();

    if (obj.contains("color_scale") && obj.at("color_scale").is_number_integer())
    {
        int scale_int  = obj.at("color_scale").get<int>();
        int steps      = obj.value("color_steps", 10);
        if (steps < 1) steps = 1;

        color_map.create(static_cast<colorscale::ColorScale>(scale_int),
                         static_cast<std::size_t>(steps));
    }

    if (obj.contains("pixels_per_cell") && obj.at("pixels_per_cell").is_number_integer())
        pixels_per_cell = obj.at("pixels_per_cell").get<int>();

    if (obj.contains("show_selected") && obj.at("show_selected").is_boolean())
        show_selected = obj.at("show_selected").get<bool>();

    return true;
}
