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

#pragma once

#include "traced_assert.h"

#include <map>
#include <vector>

/**
 * Declares a creator registry inside the owning class: the type Name##Type and the static accessor Name().
 * The accessor is defined out of line in the owner's .cpp (function-local static: one instance per process,
 * also on Windows where a function-local static in a header exists once per DLL):
 *
 *   Owner::exporterRegistryType& Owner::exporterRegistry()
 *   {
 *       static exporterRegistryType registry;
 *       return registry;
 *   }
 */
#define DECLARE_CREATOR_REGISTRY(Name, Key, Creator)  \
    typedef CreatorRegistry<Key, Creator> Name##Type; \
    static Name##Type& Name();

/**
 * Registry of creators by key, an extension point filled at runtime (e.g. by plugins in their init steps).
 * - keys are unique, creators non-empty (asserted)
 * - declared in its owner with DECLARE_CREATOR_REGISTRY
 */
template<typename Key, typename Creator>
class CreatorRegistry
{
public:
    void add(const Key& key, const Creator& creator);
    bool has(const Key& key) const { return creators_.count(key) > 0; }
    const Creator& creator(const Key& key) const;
    std::vector<Key> keys() const;

private:
    std::map<Key, Creator> creators_;
};

/**
*/
template<typename Key, typename Creator>
void CreatorRegistry<Key, Creator>::add(const Key& key, const Creator& creator)
{
    traced_assert(creator);
    traced_assert_msg(!has(key), "CreatorRegistry: key added twice");

    creators_[ key ] = creator;
}

/**
*/
template<typename Key, typename Creator>
const Creator& CreatorRegistry<Key, Creator>::creator(const Key& key) const
{
    auto it = creators_.find(key);
    traced_assert_msg(it != creators_.end(), "CreatorRegistry: key not registered");

    return it->second;
}

/**
*/
template<typename Key, typename Creator>
std::vector<Key> CreatorRegistry<Key, Creator>::keys() const
{
    std::vector<Key> keys;
    keys.reserve(creators_.size());

    for (const auto& [ key, creator ] : creators_)
        keys.push_back(key);

    return keys;
}
