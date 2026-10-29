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

#include "json_fwd.hpp"

#include <memory>
#include <set>
#include <string>

/**
 * Remembered state of a layer tree beyond its items, keyed by LayerTreeItem::stateKey().
 * - item config root (flat { key: config }), bound to a view config or a scratch object
 * - effective-hidden ids for view-side data filtering
 * Notice: the session state (checked, expanded, selected) lives in the items and the tree view,
 * vanished items are forgotten.
 */
class LayerTreeState
{
public:
    LayerTreeState();
    ~LayerTreeState();

    /// moves the item configs of the key and the keys below it to the new key (renamed item)
    void renameItemConfigs(const std::string& old_key, const std::string& new_key);

    /// drops the effective-hidden ids (database close), the config stays
    void clear();

    // ---- item config -------------------------------------------------------

    /// flat object { state key: item config } owned by the view, nullptr => scratch object
    void bindConfig(nlohmann::json* root);
    nlohmann::json& configRoot();
    const nlohmann::json& configRoot() const;

    // ---- effective-hidden ids ----------------------------------------------

    /// item hidden itself or via an unchecked ancestor; ids without an item in the tree are kept until clear()
    const std::set<std::string>& hiddenIds() const { return hidden_ids_; }
    void setEffectiveHidden(const std::string& id, bool hidden);

private:
    nlohmann::json*                 config_root_ = nullptr; // view config, see bindConfig()
    std::unique_ptr<nlohmann::json> scratch_config_;        // used while unbound
    std::set<std::string>           hidden_ids_;            // effective-hidden ids, see hiddenIds()
};
