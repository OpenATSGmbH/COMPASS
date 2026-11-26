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
#include "stringconv.h"

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
        obj["color_type"]  = static_cast<int>(color_map.type());
        obj["log_scale"]   = color_map.logScale();
        if (color_map.logScale())
            obj["log_floor"] = color_map.logFloor();

        // Emit the resolved color stops so non-Qt consumers (compass_web)
        // don't have to recreate a ColorMap to colour their renderings.
        // fromJSON() reads them back for a custom color scale.
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

    obj["legend_order"] = static_cast<int>(legend_order);
    if (legend_precision)
        obj["legend_precision"] = *legend_precision;
    if (!legend_unit.empty())
        obj["legend_unit"] = legend_unit;
    if (!legend_entries.empty())
        obj["legend_entries"] = legend_entries.toJSON();

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
        int scale_int = obj.at("color_scale").get<int>();
        int steps     = obj.value("color_steps", 10);
        if (steps < 1) steps = 1;

        auto type = ColorMap::Type::LinearSamples;
        if (obj.contains("color_type") && obj.at("color_type").is_number_integer())
        {
            int type_int = obj.at("color_type").get<int>();
            if (type_int >= static_cast<int>(ColorMap::Type::LinearSamples) &&
                type_int <= static_cast<int>(ColorMap::Type::Binary))
                type = static_cast<ColorMap::Type>(type_int);
            else
                logwrn << "unknown color_type " << type_int << ", linear samples used";
        }

        if (scale_int == static_cast<int>(colorscale::ColorScale::Custom))
        {
            //custom scale: the colors are stored
            std::vector<QColor> colors;
            if (obj.contains("colors") && obj.at("colors").is_array())
            {
                for (const auto& c : obj.at("colors"))
                {
                    QColor col = c.is_string() ? QColor(QString::fromStdString(c.get<std::string>())) : QColor();
                    if (!col.isValid())
                    {
                        colors.clear();
                        break;
                    }
                    colors.push_back(col);
                }
            }

            const size_t n = colors.size();
            const bool colors_ok = (type == ColorMap::Type::Binary       && n == 2) ||
                                   (type == ColorMap::Type::LinearRanges && n >= 3) ||
                                   (type == ColorMap::Type::LinearSamples && n >= 1) ||
                                   (type == ColorMap::Type::Discrete      && n >= 1);
            if (colors_ok)
                color_map.create(colors, type);
            else
                logwrn << "custom color scale without valid colors, default color map used";
        }
        else if (scale_int >= 0 && scale_int < static_cast<int>(colorscale::ColorScale::Custom))
        {
            color_map.create(static_cast<colorscale::ColorScale>(scale_int),
                             static_cast<std::size_t>(steps),
                             type);
        }
        else
        {
            logwrn << "unknown color_scale " << scale_int << ", default color map used";
        }
    }

    if (obj.contains("log_scale") && obj.at("log_scale").is_boolean())
        color_map.setLogScale(obj.at("log_scale").get<bool>(), obj.value("log_floor", 0.0));

    if (obj.contains("pixels_per_cell") && obj.at("pixels_per_cell").is_number_integer())
        pixels_per_cell = obj.at("pixels_per_cell").get<int>();

    if (obj.contains("show_selected") && obj.at("show_selected").is_boolean())
        show_selected = obj.at("show_selected").get<bool>();

    legend_order = LegendOrder::Normal;
    legend_precision.reset();
    legend_unit.clear();
    legend_entries.clear();

    if (obj.contains("legend_order") && obj.at("legend_order").is_number_integer())
    {
        int order = obj.at("legend_order").get<int>();
        if (order == static_cast<int>(LegendOrder::Reversed))
            legend_order = LegendOrder::Reversed;
        else if (order != static_cast<int>(LegendOrder::Normal))
            logwrn << "unknown legend_order " << order << ", normal order used";
    }

    if (obj.contains("legend_precision") && obj.at("legend_precision").is_number_integer())
    {
        int precision = obj.at("legend_precision").get<int>();
        if (precision >= 0)
            legend_precision = precision;
        else
            logwrn << "negative legend_precision " << precision << " ignored";
    }

    if (obj.contains("legend_unit") && obj.at("legend_unit").is_string())
        legend_unit = obj.at("legend_unit").get<std::string>();

    if (obj.contains("legend_entries") && !legend_entries.fromJSON(obj.at("legend_entries")))
    {
        logwrn << "legend_entries unparseable, legend from the color map used";
        legend_entries.clear();
    }

    return true;
}

/**
 * Builds the legend from the color map, the value range and the legend fields.
 * Returns an empty legend if the color map is invalid or no value range is available.
*/
ColorLegend Grid2DRenderSettings::legend(bool add_sel_color,
                                         bool add_null_color,
                                         const ColorMap::ValueDecorator& decorator) const
{
    if (!legend_entries.empty())
    {
        //fixed entries as given
        ColorLegend legend = legend_entries;

        if (add_sel_color)
            legend.addEntry(color_map.specialColor(ColorMap::SpecialColorSelected), "Selected");
        if (add_null_color)
            legend.addEntry(color_map.specialColor(ColorMap::SpecialColorNull), "NULL");

        return legend;
    }

    if (!color_map.valid())
        return ColorLegend();

    ColorMap cmap = color_map;

    if (min_value.has_value() && max_value.has_value() && *min_value <= *max_value)
        cmap.setValueRange(std::make_pair(*min_value, *max_value));

    if (!cmap.canSampleValues())
        return ColorLegend();

    ColorMap::ValueDecorator active_decorator;
    if (legend_precision.has_value())
    {
        const unsigned int precision = static_cast<unsigned int>(*legend_precision);
        active_decorator = [ precision ] (double v) { return Utils::String::doubleToStringPrecision(v, precision); };
    }
    else if (decorator)
    {
        active_decorator = decorator;
    }
    else
    {
        active_decorator = [] (double v) { return Utils::String::doubleToStringPrecision(v, 2); };
    }

    if (!legend_unit.empty())
    {
        auto number_decorator = active_decorator;
        const std::string suffix = " " + legend_unit;
        active_decorator = [ number_decorator, suffix ] (double v) { return number_decorator(v) + suffix; };
    }

    auto value_legend = cmap.colorLegend(false, false, active_decorator);

    ColorLegend legend;

    const auto& entries = value_legend.entries();
    if (legend_order == LegendOrder::Reversed)
    {
        for (auto it = entries.rbegin(); it != entries.rend(); ++it)
            legend.addEntry(it->first, it->second);
    }
    else
    {
        for (const auto& e : entries)
            legend.addEntry(e.first, e.second);
    }

    if (add_sel_color)
        legend.addEntry(cmap.specialColor(ColorMap::SpecialColorSelected), "Selected");

    if (add_null_color)
        legend.addEntry(cmap.specialColor(ColorMap::SpecialColorNull), "NULL");

    return legend;
}
