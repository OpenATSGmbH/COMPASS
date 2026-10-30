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
#include "layertreestate.h"

#include "json.hpp"

#include <string>
#include <vector>

TEST_CASE("LayerTreeState moves the configs of a renamed key and its subtree", "[view][layerpanel]")
{
    LayerTreeState state;

    nlohmann::json config = { { "A/a",   { { "hidden", true } } },
                              { "A/a/x", { { "hidden", true } } },
                              { "A/ab",  { { "hidden", true } } } };
    state.bindConfig(&config);

    state.renameItemConfigs("A/a", "A/b");

    REQUIRE(config.contains("A/b"));
    REQUIRE(config.contains("A/b/x"));
    REQUIRE(!config.contains("A/a"));
    REQUIRE(!config.contains("A/a/x"));
    REQUIRE(config.contains("A/ab")); // a name prefix, not below the key
}

TEST_CASE("LayerTreeState clears the hidden ids on database close", "[view][layerpanel]")
{
    LayerTreeState state;

    nlohmann::json config = { { "a", { { "hidden", true } } } };
    state.bindConfig(&config);

    state.setEffectiveHidden("a", true);
    state.setEffectiveHidden("b", true);
    state.setEffectiveHidden("b", false);

    REQUIRE(state.hiddenIds() == std::set<std::string>{ "a" });

    state.clear();

    REQUIRE(state.hiddenIds().empty());
    REQUIRE(state.configRoot().contains("a")); // config stays
}

TEST_CASE("LayerTreeState uses a scratch config while unbound", "[view][layerpanel]")
{
    LayerTreeState state;

    state.configRoot()[ "a" ][ "x" ] = 1;
    REQUIRE(state.configRoot().at("a").at("x") == 1);

    nlohmann::json config; // null => made an object on bind
    state.bindConfig(&config);

    REQUIRE(config.is_object());
    REQUIRE(!state.configRoot().contains("a"));

    state.bindConfig(nullptr);
    REQUIRE(state.configRoot().contains("a"));
}
