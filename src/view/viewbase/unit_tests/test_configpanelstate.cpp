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
#include "configpanelstate.h"

using namespace config_panel;

namespace
{
    const Thresholds T = thresholdsFor(400);

    State resized(State state, int previous_width, int width)
    {
        return nextState(state, previous_width, width, T, UserAction::None);
    }
}

TEST_CASE("config panel thresholds of a 400 pixel panel", "[view][config_panel]")
{
    REQUIRE(T.collapse_below == 1000);
    REQUIRE(T.expand_above == 1050);
}

TEST_CASE("config panel collapses when the view gets narrow", "[view][config_panel]")
{
    REQUIRE(resized(State::Expanded, 1200, 1000) == State::Expanded);
    REQUIRE(resized(State::Expanded, 1200, 999) == State::AutoCollapsed);
    REQUIRE(resized(State::Expanded, 1000, 300) == State::AutoCollapsed);
}

TEST_CASE("config panel expands above the hysteresis", "[view][config_panel]")
{
    REQUIRE(resized(State::AutoCollapsed, 900, 1001) == State::AutoCollapsed);
    REQUIRE(resized(State::AutoCollapsed, 900, 1050) == State::AutoCollapsed);
    REQUIRE(resized(State::AutoCollapsed, 900, 1051) == State::Expanded);
}

TEST_CASE("config panel does not toggle while the width moves around the collapse width", "[view][config_panel]")
{
    State s = State::Expanded;
    int previous = 1100;

    for (int width : { 990, 1010, 995, 1040, 990, 1049 })
    {
        s = resized(s, previous, width);
        previous = width;

        REQUIRE(s == State::AutoCollapsed);
    }
}

TEST_CASE("config panel collapsed by the user stays collapsed", "[view][config_panel]")
{
    REQUIRE(nextState(State::Expanded, 1500, 1500, T, UserAction::Close) == State::UserCollapsed);
    REQUIRE(nextState(State::AutoCollapsed, 800, 800, T, UserAction::Close) == State::UserCollapsed);

    REQUIRE(resized(State::UserCollapsed, 800, 2000) == State::UserCollapsed);
    REQUIRE(resized(State::UserCollapsed, 2000, 300) == State::UserCollapsed);
}

TEST_CASE("config panel opened by the user stays open in a narrow view", "[view][config_panel]")
{
    State s = nextState(State::AutoCollapsed, 700, 700, T, UserAction::Open);
    REQUIRE(s == State::Expanded);

    s = resized(s, 700, 680);
    REQUIRE(s == State::Expanded);

    s = nextState(State::UserCollapsed, 700, 700, T, UserAction::Open);
    REQUIRE(s == State::Expanded);

    s = resized(s, 700, 1200);
    s = resized(s, 1200, 900);
    REQUIRE(s == State::AutoCollapsed);
}

TEST_CASE("config panel is shown only when expanded", "[view][config_panel]")
{
    REQUIRE(isShown(State::Expanded));
    REQUIRE(!isShown(State::AutoCollapsed));
    REQUIRE(!isShown(State::UserCollapsed));
}
