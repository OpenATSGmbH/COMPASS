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

#include <optional>
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
 * Notice: entries already present under the new keys are replaced.
 */
void LayerTreeState::renameItemConfigs(const std::string& old_key, const std::string& new_key)
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
