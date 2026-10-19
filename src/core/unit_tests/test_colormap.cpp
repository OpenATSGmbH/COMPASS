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
#include "colormap.h"
#include "colorlegend.h"

TEST_CASE("ColorMap linear value mapping is unchanged by default", "[colormap]")
{
    ColorMap cm;
    cm.create(ColorMap::ColorScale::Green2Red, 10, ColorMap::Type::LinearSamples,
              ColorMap::ValueRange(0.0, 100.0));

    REQUIRE_FALSE(cm.logScale());
    REQUIRE(cm.factorOfValue(0.0,   0.0, 100.0) == Approx(0.0));
    REQUIRE(cm.factorOfValue(25.0,  0.0, 100.0) == Approx(0.25));
    REQUIRE(cm.factorOfValue(100.0, 0.0, 100.0) == Approx(1.0));
    REQUIRE(cm.valueAtFactor(0.5,   0.0, 100.0) == Approx(50.0));
}

TEST_CASE("ColorMap logarithmic value mapping", "[colormap]")
{
    ColorMap cm;
    cm.create(ColorMap::ColorScale::Green2Red, 10, ColorMap::Type::LinearSamples,
              ColorMap::ValueRange(1.0, 100.0));
    cm.setLogScale(true);

    REQUIRE(cm.logScale());

    SECTION("decades are equally spaced")
    {
        REQUIRE(cm.factorOfValue(1.0,   1.0, 100.0) == Approx(0.0));
        REQUIRE(cm.factorOfValue(10.0,  1.0, 100.0) == Approx(0.5));
        REQUIRE(cm.factorOfValue(100.0, 1.0, 100.0) == Approx(1.0));
    }

    SECTION("values at or below the lower bound get the first color")
    {
        REQUIRE(cm.factorOfValue(0.5, 1.0, 100.0) == Approx(0.0));
        REQUIRE(cm.factorOfValue(0.0, 1.0, 100.0) == Approx(0.0));
        REQUIRE(cm.sampleValue(0.2) == cm.sampleValue(1.0));
    }

    SECTION("valueAtFactor is the inverse")
    {
        for (double v : {1.0, 2.0, 5.0, 20.0, 99.0})
            REQUIRE(cm.valueAtFactor(cm.factorOfValue(v, 1.0, 100.0), 1.0, 100.0) == Approx(v));
    }

    SECTION("a lower bound <= 0 uses the floor")
    {
        ColorMap cm0;
        cm0.setLogScale(true, 1.0);
        REQUIRE(cm0.factorOfValue(10.0, 0.0, 100.0) == Approx(0.5));

        ColorMap cm1;
        cm1.setLogScale(true);  // no floor: 1e-3 of the upper bound
        REQUIRE(cm1.factorOfValue(0.1, 0.0, 100.0) == Approx(0.0));
        REQUIRE(cm1.factorOfValue(100.0, 0.0, 100.0) == Approx(1.0));
    }

    SECTION("legend entries are spaced logarithmically")
    {
        ColorMap cm3;
        cm3.create(ColorMap::ColorScale::Green2Red, 3, ColorMap::Type::LinearSamples,
                   ColorMap::ValueRange(1.0, 100.0));
        cm3.setLogScale(true);

        std::vector<double> values;
        auto legend = cm3.colorLegend(false, false, [&] (double v) {
            values.push_back(v);
            return std::to_string(v);
        });

        REQUIRE(values.size() == 3);
        REQUIRE(values[0] == Approx(1.0));
        REQUIRE(values[1] == Approx(10.0));
        REQUIRE(values[2] == Approx(100.0));
    }
}
