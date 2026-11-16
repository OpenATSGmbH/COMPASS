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

#include "catch.hpp"

#include "gridview/grid2drendersettings.h"

#include "json.hpp"

TEST_CASE("Grid2DRenderSettings JSON round trip", "[grid2d]")
{
    SECTION("legend fields round trip")
    {
        Grid2DRenderSettings rs;
        rs.color_map.create(ColorMap::ColorScale::Red2Green, 5);
        rs.min_value        = 0.5;
        rs.max_value        = 0.99;
        rs.pixels_per_cell  = 3;
        rs.show_selected    = false;
        rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
        rs.legend_precision = 3;
        rs.legend_unit      = "m";

        auto j = rs.toJSON();

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(j));

        REQUIRE(rs2.min_value.has_value());
        REQUIRE(*rs2.min_value == Approx(0.5));
        REQUIRE(rs2.max_value.has_value());
        REQUIRE(*rs2.max_value == Approx(0.99));
        REQUIRE(rs2.pixels_per_cell == 3);
        REQUIRE(rs2.show_selected == false);
        REQUIRE(rs2.legend_order == Grid2DRenderSettings::LegendOrder::Reversed);
        REQUIRE(rs2.legend_precision.has_value());
        REQUIRE(*rs2.legend_precision == 3);
        REQUIRE(rs2.legend_unit == "m");

        REQUIRE(rs2.color_map.valid());
        REQUIRE(rs2.color_map.colorScale() == ColorMap::ColorScale::Red2Green);
        REQUIRE(rs2.color_map.colorSteps() == 5);

        // second round trip gives the same JSON
        REQUIRE(rs2.toJSON() == j);
    }

    SECTION("JSON without the legend fields loads the defaults")
    {
        nlohmann::json j;
        j["min_value"]       = 1.0;
        j["max_value"]       = 2.0;
        j["color_scale"]     = static_cast<int>(ColorMap::ColorScale::Green2Red);
        j["color_steps"]     = 4;
        j["pixels_per_cell"] = 1;

        Grid2DRenderSettings rs;
        rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
        rs.legend_precision = 4;
        rs.legend_unit      = "%";

        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.legend_order == Grid2DRenderSettings::LegendOrder::Normal);
        REQUIRE(!rs.legend_precision.has_value());
        REQUIRE(rs.legend_unit.empty());

        auto j2 = rs.toJSON();
        REQUIRE(j2.at("legend_order").get<int>() == 0);
        REQUIRE(!j2.contains("legend_precision"));
        REQUIRE(!j2.contains("legend_unit"));
    }

    SECTION("invalid legend fields load the defaults")
    {
        nlohmann::json j;
        j["legend_order"]     = 7;
        j["legend_precision"] = -1;

        Grid2DRenderSettings rs;
        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.legend_order == Grid2DRenderSettings::LegendOrder::Normal);
        REQUIRE(!rs.legend_precision.has_value());
    }

    SECTION("map type round trip")
    {
        Grid2DRenderSettings rs;
        rs.color_map.create(ColorMap::ColorScale::Viridis, 4, ColorMap::Type::LinearRanges);

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(rs.toJSON()));
        REQUIRE(rs2.color_map.type() == ColorMap::Type::LinearRanges);
        REQUIRE(rs2.color_map.colorScale() == ColorMap::ColorScale::Viridis);
        REQUIRE(rs2.color_map.getColors() == rs.color_map.getColors());
    }

    SECTION("custom colors round trip (binary red / green)")
    {
        Grid2DRenderSettings rs;
        rs.min_value = 0.0;
        rs.max_value = 1.0;
        rs.color_map.create(QColor(Qt::red), QColor(Qt::green), 2, ColorMap::Type::Binary);

        auto j = rs.toJSON();
        REQUIRE(j.at("color_scale").get<int>() == static_cast<int>(ColorMap::ColorScale::Custom));

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(j));
        REQUIRE(rs2.color_map.type() == ColorMap::Type::Binary);
        REQUIRE(rs2.color_map.getColors() == rs.color_map.getColors());
    }

    SECTION("custom colors round trip (categorical palette)")
    {
        std::vector<QColor> palette = { QColor(31, 119, 180), QColor(255, 127, 14), QColor(44, 160, 44), QColor(170, 170, 170) };

        Grid2DRenderSettings rs;
        rs.color_map.create(palette, ColorMap::Type::LinearSamples, ColorMap::ValueRange(0.0, 4.0));

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(rs.toJSON()));
        REQUIRE(rs2.color_map.type() == ColorMap::Type::LinearSamples);
        REQUIRE(rs2.color_map.getColors() == palette);
    }

    SECTION("old JSON without the map type: linear samples")
    {
        nlohmann::json j;
        j["color_scale"] = static_cast<int>(ColorMap::ColorScale::Green2Red);
        j["color_steps"] = 2;

        Grid2DRenderSettings rs;
        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.color_map.type() == ColorMap::Type::LinearSamples);
        REQUIRE(rs.color_map.numColors() == 2);
    }

    SECTION("custom scale without valid colors keeps the default color map")
    {
        nlohmann::json j;
        j["color_scale"] = static_cast<int>(ColorMap::ColorScale::Custom);
        j["color_type"]  = static_cast<int>(ColorMap::Type::Binary);
        j["colors"]      = { "#ff0000" }; // binary needs 2

        Grid2DRenderSettings rs;
        const auto default_colors = rs.color_map.getColors();

        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.color_map.getColors() == default_colors);

        j["colors"] = { "#ff0000", "not a color" };
        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.color_map.getColors() == default_colors);
    }

    SECTION("unknown color scale keeps the default color map")
    {
        nlohmann::json j;
        j["color_scale"] = 999;

        Grid2DRenderSettings rs;
        const auto default_colors = rs.color_map.getColors();

        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.color_map.getColors() == default_colors);
    }

    SECTION("fixed legend entries round trip")
    {
        Grid2DRenderSettings rs;
        rs.legend_entries.addEntry(QColor(31, 119, 180), "RU 1");
        rs.legend_entries.addEntry(QColor(170, 170, 170), "Other");

        auto j = rs.toJSON();
        REQUIRE(j.contains("legend_entries"));

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(j));
        REQUIRE(rs2.legend_entries.entries() == rs.legend_entries.entries());

        // not written if empty, cleared on load without them
        Grid2DRenderSettings rs3;
        REQUIRE(!rs3.toJSON().contains("legend_entries"));
        REQUIRE(rs2.fromJSON(rs3.toJSON()));
        REQUIRE(rs2.legend_entries.empty());
    }

    SECTION("unparseable legend entries are ignored")
    {
        nlohmann::json j;
        j["legend_entries"] = { {"entries", { { {"color", "#ff0000"} } } } }; // no description

        Grid2DRenderSettings rs;
        REQUIRE(rs.fromJSON(j));
        REQUIRE(rs.legend_entries.empty());
    }

    SECTION("non-object JSON fails")
    {
        Grid2DRenderSettings rs;
        REQUIRE(!rs.fromJSON(nlohmann::json::array()));
    }
}
