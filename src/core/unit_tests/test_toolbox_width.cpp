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
#include "toolboxwidth.h"

using namespace toolbox;

TEST_CASE("Flight Deck width clamp", "[toolbox_width]")
{
    REQUIRE(clampWidthPercent(-5) == 10);
    REQUIRE(clampWidthPercent(0) == 10);
    REQUIRE(clampWidthPercent(10) == 10);
    REQUIRE(clampWidthPercent(37) == 37);
    REQUIRE(clampWidthPercent(70) == 70);
    REQUIRE(clampWidthPercent(71) == 70);
    REQUIRE(clampWidthPercent(200) == 70);
}

TEST_CASE("Flight Deck width step down", "[toolbox_width]")
{
    SECTION("from a multiple of the step")
    {
        REQUIRE(widthPercentStepDown(70) == 60);
        REQUIRE(widthPercentStepDown(30) == 20);
        REQUIRE(widthPercentStepDown(20) == 10);
    }
    SECTION("from a dragged value")
    {
        REQUIRE(widthPercentStepDown(37) == 30);
        REQUIRE(widthPercentStepDown(31) == 30);
        REQUIRE(widthPercentStepDown(69) == 60);
        REQUIRE(widthPercentStepDown(11) == 10);
    }
    SECTION("at and beyond the limits")
    {
        REQUIRE(widthPercentStepDown(10) == 10);
        REQUIRE(widthPercentStepDown(5) == 10);
        REQUIRE(widthPercentStepDown(90) == 60);
    }
}

TEST_CASE("Flight Deck width step up", "[toolbox_width]")
{
    SECTION("from a multiple of the step")
    {
        REQUIRE(widthPercentStepUp(10) == 20);
        REQUIRE(widthPercentStepUp(30) == 40);
        REQUIRE(widthPercentStepUp(60) == 70);
    }
    SECTION("from a dragged value")
    {
        REQUIRE(widthPercentStepUp(37) == 40);
        REQUIRE(widthPercentStepUp(31) == 40);
        REQUIRE(widthPercentStepUp(69) == 70);
        REQUIRE(widthPercentStepUp(11) == 20);
    }
    SECTION("at and beyond the limits")
    {
        REQUIRE(widthPercentStepUp(70) == 70);
        REQUIRE(widthPercentStepUp(90) == 70);
        REQUIRE(widthPercentStepUp(5) == 20);
    }
}

TEST_CASE("Flight Deck width default lookup", "[toolbox_width]")
{
    REQUIRE(widthPercentOrDefault(std::nullopt, 30) == 30);
    REQUIRE(widthPercentOrDefault(std::nullopt, 50) == 50);
    REQUIRE(widthPercentOrDefault(45, 30) == 50);
    REQUIRE(widthPercentOrDefault(44, 30) == 40);
    REQUIRE(widthPercentOrDefault(3, 30) == 10);
    REQUIRE(widthPercentOrDefault(95, 30) == 70);
}

TEST_CASE("Flight Deck width snap", "[toolbox_width]")
{
    REQUIRE(snapWidthPercent(30) == 30);
    REQUIRE(snapWidthPercent(34) == 30);
    REQUIRE(snapWidthPercent(35) == 40);
    REQUIRE(snapWidthPercent(66) == 70);
    REQUIRE(snapWidthPercent(4) == 10);
    REQUIRE(snapWidthPercent(76) == 70);
    REQUIRE(snapWidthPercent(100) == 70);
}
