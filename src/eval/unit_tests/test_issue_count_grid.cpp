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

#include "results/base/featuredefinition_grid.h"

#include "json.hpp"

#include <string>
#include <vector>

using EvaluationRequirementResult::FeatureDefinitionIssueCountGrid;

namespace
{
    std::vector<std::string> texts(const ColorLegend& legend)
    {
        std::vector<std::string> t;
        for (const auto& e : legend.entries())
            t.push_back(e.second);
        return t;
    }
}

TEST_CASE("issue count grid colors and legend", "[eval][issuecountgrid]")
{
    const QColor green (0, 255, 0);
    const QColor yellow(255, 255, 0);
    const QColor red   (255, 0, 0);

    SECTION("no issue: green only, the legend holds 0")
    {
        const auto rs = FeatureDefinitionIssueCountGrid::renderSettings(0);

        CHECK(rs.color_map.sampleValue(0.0) == green);
        CHECK(texts(rs.legend_entries) == std::vector<std::string>{ "0" });
        CHECK(rs.legend_entries.entries().front().first == green);
    }

    SECTION("a single issue is yellow")
    {
        const auto rs = FeatureDefinitionIssueCountGrid::renderSettings(1);

        CHECK(rs.color_map.sampleValue(0.0) == green);
        CHECK(rs.color_map.sampleValue(1.0) == yellow);
        CHECK(texts(rs.legend_entries) == std::vector<std::string>{ "0", "1" });
    }

    SECTION("many issues: logarithmic steps from yellow to red, the legend covers all counts")
    {
        const auto rs = FeatureDefinitionIssueCountGrid::renderSettings(1000);

        CHECK(rs.min_value.value() == Approx(1.0));
        CHECK(rs.max_value.value() == Approx(1000.0));
        CHECK(rs.color_map.logScale());

        CHECK(rs.color_map.sampleValue(0.0)    == green);
        CHECK(rs.color_map.sampleValue(1.0)    == yellow);
        CHECK(rs.color_map.sampleValue(1000.0) == red);

        //1000^0.2 = 3.98: 1 - 3, 4 - 15, 16 - 63, 64 - 251, 252 - 1000 (the maximum has the color of the last step)
        CHECK(texts(rs.legend_entries) == std::vector<std::string>{ "0", "1 - 3", "4 - 15", "16 - 63", "64 - 251", "252 - 1000" });

        //each legend entry has the color the grid gives its counts
        CHECK(rs.legend_entries.entries()[ 1 ].first == rs.color_map.sampleValue(3.0));
        CHECK(rs.legend_entries.entries()[ 2 ].first == rs.color_map.sampleValue(4.0));
        CHECK(rs.legend_entries.entries()[ 5 ].first == rs.color_map.sampleValue(999.0));
    }

    SECTION("few issues: the legend lists only the counts that can occur")
    {
        const auto rs = FeatureDefinitionIssueCountGrid::renderSettings(3);

        const auto t = texts(rs.legend_entries);

        REQUIRE(t.size() >= 2);
        CHECK(t.front() == "0");
        CHECK(t[ 1 ].rfind("1", 0) == 0);
        CHECK(t.back().find("3") != std::string::npos);
        CHECK(t.size() <= 4);
    }

    SECTION("the render settings survive the view point JSON")
    {
        const auto rs = FeatureDefinitionIssueCountGrid::renderSettings(1000);

        Grid2DRenderSettings read;
        REQUIRE(read.fromJSON(rs.toJSON()));

        //the JSON holds no value range of the color map, the readers (grid renderer, grid render widget) use
        //min_value / max_value
        REQUIRE(read.min_value.has_value());
        REQUIRE(read.max_value.has_value());

        for (double v : { 0.0, 1.0, 5.0, 100.0, 1000.0 })
            CHECK(read.color_map.sample(read.color_map.factorOfValue(v, *read.min_value, *read.max_value))
                  == rs.color_map.sampleValue(v));

        CHECK(texts(read.legend_entries) == texts(rs.legend_entries));
    }
}
