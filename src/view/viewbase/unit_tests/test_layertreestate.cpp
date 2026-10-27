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

TEST_CASE("LayerTreeState keeps only non-empty item states", "[view][layerpanel]")
{
    LayerTreeState state;

    state.updateItemState("a", [] (LayerTreeState::ItemState&) {});
    REQUIRE(!state.itemState("a"));

    state.updateItemState("a", [] (LayerTreeState::ItemState& s) { s.hidden = true; s.expanded = true; });
    REQUIRE(state.itemState("a"));

    state.clearItemState("a", LayerTreeState::PersistHidden);
    REQUIRE(!state.itemState("a")->hidden);
    REQUIRE(*state.itemState("a")->expanded);

    state.clearItemState("a", LayerTreeState::PersistExpansion);
    REQUIRE(!state.itemState("a"));
    REQUIRE(!state.hasItemStates());

    state.updateItemState("", [] (LayerTreeState::ItemState& s) { s.hidden = true; }); // invisible root
    REQUIRE(!state.hasItemStates());
}

TEST_CASE("LayerTreeState keeps a single pending current item", "[view][layerpanel]")
{
    LayerTreeState state;

    state.setPendingCurrent("a");
    state.setPendingCurrent("b");

    REQUIRE(!state.itemState("a"));
    REQUIRE(state.itemState("b")->current);

    state.updateItemState("c", [] (LayerTreeState::ItemState& s) { s.selected = true; s.hidden = true; });
    state.clearPendingSelection();

    REQUIRE(!state.itemState("b"));
    REQUIRE(!state.itemState("c")->selected);
    REQUIRE(*state.itemState("c")->hidden);
}

TEST_CASE("LayerTreeState drops disabled components", "[view][layerpanel]")
{
    LayerTreeState state;

    state.updateItemState("a", [] (LayerTreeState::ItemState& s) { s.hidden = true; });
    state.updateItemState("b", [] (LayerTreeState::ItemState& s) { s.hidden = true; s.expanded = false; });

    state.setPersistFlags(LayerTreeState::PersistAll & ~LayerTreeState::PersistHidden);

    REQUIRE(!state.persists(LayerTreeState::PersistHidden));
    REQUIRE(!state.itemState("a"));
    REQUIRE(state.itemState("b"));
    REQUIRE(!state.itemState("b")->hidden);
}

TEST_CASE("LayerTreeState prunes only the entries with the key prefix", "[view][layerpanel]")
{
    LayerTreeState state;

    for (const std::string key : { "A", "A/a", "A/b", "AB/c", "B/d" })
        state.updateItemState(key, [] (LayerTreeState::ItemState& s) { s.hidden = true; });

    std::vector<std::string> visited;
    state.prune([ & ] (const std::string& key, const LayerTreeState::ItemState&)
    {
        visited.push_back(key);
        return true;
    }, "A/");

    REQUIRE(visited == std::vector<std::string>{ "A/a", "A/b" });
    REQUIRE(state.itemState("A"));
    REQUIRE(state.itemState("AB/c"));
    REQUIRE(state.itemState("B/d"));
}

TEST_CASE("LayerTreeState moves the states of a renamed key and its subtree", "[view][layerpanel]")
{
    LayerTreeState state;

    nlohmann::json config = { { "A/a",   { { "hidden", true } } },
                              { "A/a/x", { { "hidden", true } } },
                              { "A/ab",  { { "hidden", true } } } };
    state.bindConfig(&config);

    for (const std::string key : { "A/a", "A/a/x", "A/ab" })
        state.updateItemState(key, [] (LayerTreeState::ItemState& s) { s.expanded = true; });

    state.renameItemStates("A/a", "A/b");

    REQUIRE(state.itemState("A/b"));
    REQUIRE(state.itemState("A/b/x"));
    REQUIRE(!state.itemState("A/a"));
    REQUIRE(!state.itemState("A/a/x"));
    REQUIRE(state.itemState("A/ab")); // a name prefix, not below the key

    REQUIRE(config.contains("A/b"));
    REQUIRE(config.contains("A/b/x"));
    REQUIRE(!config.contains("A/a"));
    REQUIRE(!config.contains("A/a/x"));
    REQUIRE(config.contains("A/ab"));
}

TEST_CASE("LayerTreeState prunes by predicate and clears on database close", "[view][layerpanel]")
{
    LayerTreeState state;

    nlohmann::json config = { { "a", { { "hidden", true } } } };
    state.bindConfig(&config);

    state.updateItemState("a", [] (LayerTreeState::ItemState& s) { s.hidden = true; });
    state.updateItemState("b", [] (LayerTreeState::ItemState& s) { s.hidden = true; });
    state.setEffectiveHidden("a", true);

    state.prune([] (const std::string& key, const LayerTreeState::ItemState&) { return key == "b"; });

    REQUIRE(state.itemState("a"));
    REQUIRE(!state.itemState("b"));

    state.clear();

    REQUIRE(!state.hasItemStates());
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
