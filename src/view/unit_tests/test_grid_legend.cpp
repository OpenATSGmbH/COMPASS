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

#include <string>
#include <vector>

namespace
{

std::vector<std::string> labels(const ColorLegend& legend)
{
    std::vector<std::string> l;
    for (const auto& e : legend.entries())
        l.push_back(e.second);
    return l;
}

std::vector<QColor> colors(const ColorLegend& legend)
{
    std::vector<QColor> c;
    for (const auto& e : legend.entries())
        c.push_back(e.first);
    return c;
}

} // namespace

TEST_CASE("Grid2DRenderSettings legend", "[grid2d]")
{
    Grid2DRenderSettings rs;
    rs.color_map.create(ColorMap::ColorScale::Red2Green, 3);
    rs.min_value = 0.0;
    rs.max_value = 1.0;

    SECTION("entries from the color map, normal order")
    {
        auto legend = rs.legend();

        REQUIRE(labels(legend) == std::vector<std::string>({"0.00", "0.50", "1.00"}));
        REQUIRE(colors(legend) == rs.color_map.getColors());
    }

    SECTION("reversed order")
    {
        rs.legend_order = Grid2DRenderSettings::LegendOrder::Reversed;

        auto legend = rs.legend();

        REQUIRE(labels(legend) == std::vector<std::string>({"1.00", "0.50", "0.00"}));

        auto c = rs.color_map.getColors();
        REQUIRE(colors(legend) == std::vector<QColor>(c.rbegin(), c.rend()));
    }

    SECTION("number precision")
    {
        rs.legend_precision = 0;
        REQUIRE(labels(rs.legend()) == std::vector<std::string>({"0", "0", "1"}));

        rs.legend_precision = 3;
        REQUIRE(labels(rs.legend()) == std::vector<std::string>({"0.000", "0.500", "1.000"}));
    }

    SECTION("precision has priority over the decorator")
    {
        auto deco = [] (double v) { return std::to_string(static_cast<int>(v * 100.0)) + " %"; };

        REQUIRE(labels(rs.legend(false, false, deco)) == std::vector<std::string>({"0 %", "50 %", "100 %"}));

        rs.legend_precision = 1;
        REQUIRE(labels(rs.legend(false, false, deco)) == std::vector<std::string>({"0.0", "0.5", "1.0"}));
    }

    SECTION("unit after each number")
    {
        rs.legend_unit = "m";
        REQUIRE(labels(rs.legend()) == std::vector<std::string>({"0.00 m", "0.50 m", "1.00 m"}));

        rs.legend_precision = 0;
        rs.min_value        = 0.0;
        rs.max_value        = 100.0;
        rs.legend_unit      = "%";
        REQUIRE(labels(rs.legend()) == std::vector<std::string>({"0 %", "50 %", "100 %"}));

        // unit also after the numbers of a decorator, not after selected / null
        rs.legend_precision.reset();
        auto deco = [] (double v) { return std::to_string(static_cast<int>(v)); };
        REQUIRE(labels(rs.legend(true, true, deco)) == std::vector<std::string>({"0 %", "50 %", "100 %", "Selected", "NULL"}));
    }

    SECTION("fixed legend entries replace the color map entries")
    {
        ColorLegend fixed;
        fixed.addEntry(QColor(220, 50, 50),   "Contested");
        fixed.addEntry(QColor(220, 220, 220), "Uncontested");

        rs.legend_entries   = fixed;
        rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed; // not applied
        rs.legend_precision = 3;                                           // not applied
        rs.legend_unit      = "m";                                         // not applied

        auto legend = rs.legend();
        REQUIRE(labels(legend) == std::vector<std::string>({"Contested", "Uncontested"}));
        REQUIRE(colors(legend) == std::vector<QColor>({QColor(220, 50, 50), QColor(220, 220, 220)}));

        REQUIRE(labels(rs.legend(true, true)) == std::vector<std::string>({"Contested", "Uncontested", "Selected", "NULL"}));

        // also without a value range
        Grid2DRenderSettings rs2;
        rs2.legend_entries = fixed;
        REQUIRE(labels(rs2.legend()) == std::vector<std::string>({"Contested", "Uncontested"}));
    }

    SECTION("min / max from the render settings")
    {
        rs.min_value = 10.0;
        rs.max_value = 30.0;
        REQUIRE(labels(rs.legend()) == std::vector<std::string>({"10.00", "20.00", "30.00"}));
    }

    SECTION("value range of the color map without min / max")
    {
        Grid2DRenderSettings rs2;
        rs2.color_map.create(ColorMap::ColorScale::Green2Red, 3, ColorMap::Type::LinearSamples,
                             ColorMap::ValueRange(2.0, 4.0));
        REQUIRE(labels(rs2.legend()) == std::vector<std::string>({"2.00", "3.00", "4.00"}));

        // min / max override the color map range
        rs2.min_value = 0.0;
        rs2.max_value = 8.0;
        REQUIRE(labels(rs2.legend()) == std::vector<std::string>({"0.00", "4.00", "8.00"}));
    }

    SECTION("selected and null entries at the end, also in reversed order")
    {
        rs.legend_order = Grid2DRenderSettings::LegendOrder::Reversed;

        auto legend = rs.legend(true, true);

        REQUIRE(labels(legend) == std::vector<std::string>({"1.00", "0.50", "0.00", "Selected", "NULL"}));
        REQUIRE(legend.entries()[ 3 ].first == rs.color_map.specialColor(ColorMap::SpecialColorSelected));
        REQUIRE(legend.entries()[ 4 ].first == rs.color_map.specialColor(ColorMap::SpecialColorNull));
    }

    SECTION("no value range gives an empty legend")
    {
        Grid2DRenderSettings rs2;
        rs2.color_map.create(ColorMap::ColorScale::Green2Red, 3);
        REQUIRE(rs2.legend().empty());

        rs2.min_value = 1.0; // max missing
        REQUIRE(rs2.legend().empty());
    }

    SECTION("default color map with min / max gives one entry per color")
    {
        Grid2DRenderSettings rs2;
        rs2.min_value = 0.0;
        rs2.max_value = 1.0;
        REQUIRE(rs2.legend().entries().size() == rs2.color_map.numColors());
    }

    SECTION("JSON round trip keeps the legend")
    {
        rs.legend_order     = Grid2DRenderSettings::LegendOrder::Reversed;
        rs.legend_precision = 1;
        rs.legend_unit      = "m";

        Grid2DRenderSettings rs2;
        REQUIRE(rs2.fromJSON(rs.toJSON()));

        REQUIRE(labels(rs2.legend()) == labels(rs.legend()));
        REQUIRE(colors(rs2.legend()) == colors(rs.legend()));
    }
}
