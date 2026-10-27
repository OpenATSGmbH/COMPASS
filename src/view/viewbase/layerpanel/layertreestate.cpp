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

#include "layertreestate.h"

#include "json.hpp"

#include <utility>
#include <vector>

/**
 */
LayerTreeState::LayerTreeState()
:   scratch_config_(new nlohmann::json(nlohmann::json::object()))
{
}

/**
 */
LayerTreeState::~LayerTreeState() = default;

/**
 */
void LayerTreeState::setPersistFlags(int flags)
{
    persist_flags_ = flags;

    int dropped = 0;
    if (!persists(PersistHidden))
        dropped |= PersistHidden;
    if (!persists(PersistExpansion))
        dropped |= PersistExpansion;
    if (!persists(PersistSelection))
        dropped |= PersistSelection;

    for (auto it = item_states_.begin(); it != item_states_.end();)
    {
        const std::string key = (it++)->first; // clearItemState() may erase the entry

        clearItemState(key, dropped);
    }
}

/**
 */
const LayerTreeState::ItemState* LayerTreeState::itemState(const std::string& key) const
{
    if (key.empty())
        return nullptr;

    auto it = item_states_.find(key);

    return it != item_states_.end() ? &it->second : nullptr;
}

/**
 * Notice: no entry is created if func leaves the state empty.
 */
void LayerTreeState::updateItemState(const std::string& key, const std::function<void(ItemState&)>& func)
{
    if (key.empty())
        return;

    auto it = item_states_.find(key);
    if (it == item_states_.end())
    {
        ItemState state;
        func(state);

        if (!state.isEmpty())
            item_states_[ key ] = state;

        return;
    }

    func(it->second);

    if (it->second.isEmpty())
        item_states_.erase(it);
}

/**
 */
void LayerTreeState::clearItemState(const std::string& key, int flags)
{
    auto it = item_states_.find(key);
    if (it == item_states_.end())
        return;

    if (flags & PersistHidden)
        it->second.hidden.reset();

    if (flags & PersistExpansion)
        it->second.expanded.reset();

    if (flags & PersistSelection)
    {
        it->second.selected = false;
        it->second.current  = false;
    }

    if (it->second.isEmpty())
        item_states_.erase(it);
}

/**
 * Remembers the given key as the only pending current item.
 */
void LayerTreeState::setPendingCurrent(const std::string& key)
{
    for (auto it = item_states_.begin(); it != item_states_.end();)
    {
        it->second.current = false;

        if (it->second.isEmpty())
            it = item_states_.erase(it);
        else
            ++it;
    }

    updateItemState(key, [] (ItemState& state) { state.current = true; });
}

/**
 */
void LayerTreeState::clearPendingSelection()
{
    for (auto it = item_states_.begin(); it != item_states_.end();)
    {
        it->second.selected = false;
        it->second.current  = false;

        if (it->second.isEmpty())
            it = item_states_.erase(it);
        else
            ++it;
    }
}

/**
 * Notice: only visits the entries whose key starts with key_prefix.
 */
void LayerTreeState::prune(const PruneFunc& func, const std::string& key_prefix)
{
    // keys are ordered => the prefix range starts at lower_bound and is contiguous
    for (auto it = item_states_.lower_bound(key_prefix); it != item_states_.end();)
    {
        if (it->first.compare(0, key_prefix.size(), key_prefix) != 0)
            break;

        if (func(it->first, it->second))
            it = item_states_.erase(it);
        else
            ++it;
    }
}

/**
 * Notice: entries already present under the new keys are replaced.
 */
void LayerTreeState::renameItemStates(const std::string& old_key, const std::string& new_key)
{
    if (old_key.empty() || new_key.empty() || old_key == new_key)
        return;

    // the key itself or a key below it (old_key is a complete key => the next '/' is a separator)
    auto renamedKey = [ & ] (const std::string& key) -> std::optional<std::string>
    {
        if (key.compare(0, old_key.size(), old_key) != 0)
            return {};

        if (key.size() == old_key.size())
            return new_key;

        if (key[ old_key.size() ] == '/')
            return new_key + key.substr(old_key.size());

        return {};
    };

    // item states: the candidates are a contiguous range of the ordered map
    std::vector<std::pair<std::string, ItemState>> moved_states;

    for (auto it = item_states_.lower_bound(old_key); it != item_states_.end();)
    {
        if (it->first.compare(0, old_key.size(), old_key) != 0)
            break;

        auto key = renamedKey(it->first);
        if (!key)
        {
            ++it;
            continue;
        }

        moved_states.emplace_back(*key, it->second);
        it = item_states_.erase(it);
    }

    for (auto& [ key, state ] : moved_states)
        item_states_[ key ] = state;

    // item configs
    auto& root = configRoot();
    if (!root.is_object())
        return;

    std::vector<std::pair<std::string, std::string>> moved_configs;

    for (auto it = root.begin(); it != root.end(); ++it)
    {
        if (auto key = renamedKey(it.key()))
            moved_configs.emplace_back(it.key(), *key);
    }

    for (const auto& [ from, to ] : moved_configs)
    {
        nlohmann::json config = std::move(root[ from ]);
        root.erase(from);
        root[ to ] = std::move(config);
    }
}

/**
 */
void LayerTreeState::clear()
{
    item_states_.clear();
    hidden_ids_.clear();
}

/**
 */
void LayerTreeState::bindConfig(nlohmann::json* root)
{
    config_root_ = root;

    if (config_root_ && !config_root_->is_object())
        *config_root_ = nlohmann::json::object();
}

/**
 */
nlohmann::json& LayerTreeState::configRoot()
{
    return config_root_ ? *config_root_ : *scratch_config_;
}

/**
 */
const nlohmann::json& LayerTreeState::configRoot() const
{
    return config_root_ ? *config_root_ : *scratch_config_;
}

/**
 */
void LayerTreeState::setEffectiveHidden(const std::string& id, bool hidden)
{
    if (hidden)
        hidden_ids_.insert(id);
    else
        hidden_ids_.erase(id);
}
