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

#include "db_context_manager.h"

#include <json.hpp>

using context::DBContextManager;
using AsterixInfoMap = DBContextManager::AsterixInfoMap;
using AsterixCategoryStats = DBContextManager::AsterixCategoryStats;
using AsterixItemStats = DBContextManager::AsterixItemStats;
using nlohmann::json;

namespace
{

AsterixItemStats makeItem(std::size_t count, json min_v = json(), json max_v = json())
{
    AsterixItemStats s;
    s.count = count;
    s.min = std::move(min_v);
    s.max = std::move(max_v);
    return s;
}

}

TEST_CASE("asterix_info merge: empty + first import populates", "[asterix_info]")
{
    AsterixInfoMap store;

    AsterixInfoMap delta;
    auto& cat = delta[1234][48];
    cat.total_count = 100;
    cat.items["010.SAC"] = makeItem(100, json(7), json(7));
    cat.items["070.Mode-3/A"] = makeItem(80, json(0), json(7777));

    DBContextManager::mergeAsterixInfoInto(store, delta);

    REQUIRE(store.size() == 1);
    REQUIRE(store.count(1234) == 1);
    REQUIRE(store.at(1234).count(48) == 1);

    const auto& cat_dst = store.at(1234).at(48);
    REQUIRE(cat_dst.total_count == 100);
    REQUIRE(cat_dst.items.size() == 2);
    REQUIRE(cat_dst.items.at("010.SAC").count == 100);
    REQUIRE(cat_dst.items.at("070.Mode-3/A").count == 80);
    REQUIRE(cat_dst.items.at("070.Mode-3/A").min == json(0));
    REQUIRE(cat_dst.items.at("070.Mode-3/A").max == json(7777));
}

TEST_CASE("asterix_info merge: second import sums and refines min/max", "[asterix_info]")
{
    AsterixInfoMap store;

    AsterixInfoMap delta1;
    {
        auto& cat = delta1[1234][48];
        cat.total_count = 100;
        cat.items["070.Mode-3/A"] = makeItem(80, json(100), json(5000));
    }

    AsterixInfoMap delta2;
    {
        auto& cat = delta2[1234][48];
        cat.total_count = 50;
        cat.items["070.Mode-3/A"] = makeItem(40, json(50), json(7777));
        // a new item not seen before
        cat.items["220.Aircraft Address"] = makeItem(40, json(0x000001), json(0xFFFFFF));
    }

    DBContextManager::mergeAsterixInfoInto(store, delta1);
    DBContextManager::mergeAsterixInfoInto(store, delta2);

    const auto& cat = store.at(1234).at(48);
    REQUIRE(cat.total_count == 150);

    REQUIRE(cat.items.at("070.Mode-3/A").count == 120);
    REQUIRE(cat.items.at("070.Mode-3/A").min == json(50));   // refined down
    REQUIRE(cat.items.at("070.Mode-3/A").max == json(7777)); // refined up

    REQUIRE(cat.items.at("220.Aircraft Address").count == 40);
}

TEST_CASE("asterix_info merge: null min/max absorbs into a numeric value", "[asterix_info]")
{
    AsterixInfoMap store;

    AsterixInfoMap delta1;
    {
        auto& cat = delta1[1234][48];
        cat.total_count = 10;
        cat.items["X"] = makeItem(10, json(), json()); // both null
    }

    AsterixInfoMap delta2;
    {
        auto& cat = delta2[1234][48];
        cat.total_count = 5;
        cat.items["X"] = makeItem(5, json(7), json(42));
    }

    DBContextManager::mergeAsterixInfoInto(store, delta1);
    DBContextManager::mergeAsterixInfoInto(store, delta2);

    const auto& it = store.at(1234).at(48).items.at("X");
    REQUIRE(it.count == 15);
    REQUIRE(it.min == json(7));
    REQUIRE(it.max == json(42));
}

TEST_CASE("asterix_info merge: unknown SAC/SIC bucket (ds_id 0) is ignored", "[asterix_info]")
{
    AsterixInfoMap store;

    AsterixInfoMap delta;
    auto& cat = delta[0][48]; // ds_id 0 = unknown
    cat.total_count = 999;
    cat.items["010.SAC"] = makeItem(999);

    DBContextManager::mergeAsterixInfoInto(store, delta);

    REQUIRE(store.empty());
}

TEST_CASE("asterix_info merge: existing keys outside delta are untouched", "[asterix_info]")
{
    AsterixInfoMap store;

    {
        AsterixInfoMap delta;
        auto& cat = delta[1234][48];
        cat.total_count = 100;
        cat.items["A"] = makeItem(100);
        DBContextManager::mergeAsterixInfoInto(store, delta);
    }

    {
        // delta only touches a different (ds, cat)
        AsterixInfoMap delta;
        auto& cat = delta[5678][62];
        cat.total_count = 50;
        cat.items["B"] = makeItem(50);
        DBContextManager::mergeAsterixInfoInto(store, delta);
    }

    REQUIRE(store.at(1234).at(48).total_count == 100);
    REQUIRE(store.at(1234).at(48).items.at("A").count == 100);
    REQUIRE(store.at(5678).at(62).total_count == 50);
    REQUIRE(store.at(5678).at(62).items.at("B").count == 50);
}

TEST_CASE("asterix_info JSON round-trip", "[asterix_info]")
{
    AsterixInfoMap store;
    {
        auto& cat = store[1234][48];
        cat.total_count = 100;
        cat.items["010.SAC"] = makeItem(100, json(7), json(7));
        cat.items["070.Mode-3/A"] = makeItem(80, json(100), json(5000));
        cat.items["220.Aircraft Address"] = makeItem(80); // null min/max
    }
    {
        auto& cat = store[1234][62];
        cat.total_count = 30;
        cat.items["010.SAC"] = makeItem(30, json(7), json(7));
    }

    json j = DBContextManager::asterixInfoToJSON(store);
    AsterixInfoMap roundtrip = DBContextManager::asterixInfoFromJSON(j);

    REQUIRE(roundtrip.size() == store.size());
    REQUIRE(roundtrip.at(1234).size() == 2);

    const auto& cat48 = roundtrip.at(1234).at(48);
    REQUIRE(cat48.total_count == 100);
    REQUIRE(cat48.items.size() == 3);
    REQUIRE(cat48.items.at("010.SAC").count == 100);
    REQUIRE(cat48.items.at("010.SAC").min == json(7));
    REQUIRE(cat48.items.at("010.SAC").max == json(7));
    REQUIRE(cat48.items.at("070.Mode-3/A").count == 80);
    REQUIRE(cat48.items.at("070.Mode-3/A").min == json(100));
    REQUIRE(cat48.items.at("070.Mode-3/A").max == json(5000));
    REQUIRE(cat48.items.at("220.Aircraft Address").count == 80);
    REQUIRE(cat48.items.at("220.Aircraft Address").min.is_null());
    REQUIRE(cat48.items.at("220.Aircraft Address").max.is_null());

    const auto& cat62 = roundtrip.at(1234).at(62);
    REQUIRE(cat62.total_count == 30);
    REQUIRE(cat62.items.at("010.SAC").count == 30);
}

TEST_CASE("asterix_info from invalid JSON returns empty", "[asterix_info]")
{
    AsterixInfoMap r1 = DBContextManager::asterixInfoFromJSON(json::array());
    REQUIRE(r1.empty());

    AsterixInfoMap r2 = DBContextManager::asterixInfoFromJSON(json("foo"));
    REQUIRE(r2.empty());

    // bad ds_id key (non-numeric) is skipped, valid keys are kept
    json j;
    j["not_a_number"] = json::object();
    j["1234"]["48"]["total_count"] = 5;
    AsterixInfoMap r3 = DBContextManager::asterixInfoFromJSON(j);
    REQUIRE(r3.size() == 1);
    REQUIRE(r3.at(1234).at(48).total_count == 5);
}
