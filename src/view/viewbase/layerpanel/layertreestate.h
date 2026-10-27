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

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>

/**
 * Remembered state of a layer tree, keyed by LayerTreeItem::stateKey().
 * - session state per item (only deviations from the defaults), switchable per component
 * - item config root (flat { key: config }), bound to a view config or a scratch object
 * - effective-hidden ids for view-side data filtering
 * Notice: knows nothing about the tree, LayerTreeModel feeds and applies it.
 */
class LayerTreeState
{
public:
    /// remembered item state components, combined in persistFlags()
    enum PersistFlag
    {
        PersistHidden    = 1 << 0,
        PersistExpansion = 1 << 1,
        PersistSelection = 1 << 2,
        PersistAll       = PersistHidden | PersistExpansion | PersistSelection
    };

    /**
     * Session state of an item. Only deviations from the defaults are kept, empty entries are erased.
     */
    struct ItemState
    {
        std::optional<bool> hidden;           // own (checkbox) hidden flag, if != defaultHidden()
        std::optional<bool> expanded;         // if != the panel's default expansion
        bool                selected = false; // selected when removed, reselected on return
        bool                current  = false; // current item when removed

        bool isEmpty() const { return !hidden && !expanded && !selected && !current; }
    };

    /// true => drop the entry
    typedef std::function<bool(const std::string& key, const ItemState& state)> PruneFunc;

    LayerTreeState();
    ~LayerTreeState();

    // ---- persist flags -----------------------------------------------------

    /// combination of PersistFlag, disabled components are dropped from the item states
    void setPersistFlags(int flags);
    int persistFlags() const { return persist_flags_; }
    bool persists(PersistFlag flag) const { return (persist_flags_ & flag) != 0; }

    // ---- item states -------------------------------------------------------

    bool hasItemStates() const { return !item_states_.empty(); }
    const ItemState* itemState(const std::string& key) const;
    /// creates the entry if needed, erases it if func leaves it empty
    void updateItemState(const std::string& key, const std::function<void(ItemState&)>& func);
    /// drops the given components (combination of PersistFlag) of an item state
    void clearItemState(const std::string& key, int flags);

    void setPendingCurrent(const std::string& key);
    void clearPendingSelection();

    /// func decides on the entries with the given key prefix (empty => all)
    void prune(const PruneFunc& func, const std::string& key_prefix = std::string());

    /// moves the item states and item configs of the key and the keys below it to the new key (renamed item)
    void renameItemStates(const std::string& old_key, const std::string& new_key);

    /// drops the item states and the effective-hidden ids (database close), the config stays
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
    std::map<std::string, ItemState> item_states_;                  // state key -> session state
    int                              persist_flags_ = PersistAll;   // remembered components, see PersistFlag
    nlohmann::json*                  config_root_   = nullptr;      // view config, see bindConfig()
    std::unique_ptr<nlohmann::json>  scratch_config_;               // used while unbound
    std::set<std::string>            hidden_ids_;                   // effective-hidden ids, see hiddenIds()
};
