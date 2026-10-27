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

#include <QColor>
#include <QIcon>
#include <QVariant>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class LayerTreeModel;
class QAction;
class QMenu;

/**
 * Polymorphic base for every item in a LayerTreeModel.
 *
 * Items own their children (unique_ptr). `model_` is set when the item is
 * attached to a LayerTreeModel and propagates down to children as they are
 * appended; items emit visibility / color signals through the model.
 *
 * Subclasses may override virtuals to provide per-column data, their own
 * context menu, or react to visibility changes (onEffectiveHiddenChanged).
 * Color editing from the tree is intentionally not supported.
 */
class LayerTreeItem
{
public:
    explicit LayerTreeItem(const std::string& name, LayerTreeItem* parent_item = nullptr);
    virtual ~LayerTreeItem();

    // ---- structural --------------------------------------------------------

    LayerTreeItem* parentItem() const { return parent_item_; }
    bool hasParent() const { return parent_item_ != nullptr; }

    int childCount() const { return (int)children_.size(); }
    LayerTreeItem* child(int row) const;
    int indexOf(const LayerTreeItem* child) const;
    int row() const;

    /// takes ownership, propagates model_ down
    LayerTreeItem* appendChild(std::unique_ptr<LayerTreeItem> child);
    void clearChildren();

    /// Remove and destroy a single child by row index.
    void removeChildAt(int row);

    /// Transfer all children out as a flat vector, detaching their parent
    /// pointers. Used to build a subtree that will be reattached via
    /// LayerTreeModel::refreshSubtree.
    std::vector<std::unique_ptr<LayerTreeItem>> moveChildrenOut();

    typedef std::function<std::vector<std::unique_ptr<LayerTreeItem>>()> BuildFunc;

    /// structural changes with model notification if attached (plain operations otherwise)
    LayerTreeItem* appendChildNotified(std::unique_ptr<LayerTreeItem> child);
    /// the removed subtree's remembered states are dropped
    void removeChildNotified(int row);
    /// replaces all children by the built ones (build runs after the old ones are removed)
    /// complete => the remembered states of items that did not come back are dropped
    void replaceChildren(const BuildFunc& build, bool complete = true);
    /// notifies a changed display of the item (e.g. name), if attached
    void notifyChanged();

    /// model attachment; LayerTreeModel calls setModel on the invisible root
    /// which propagates to every descendant.
    void setModel(LayerTreeModel* model);
    LayerTreeModel* model() const { return model_; }

    // ---- identity / persistence -------------------------------------------

    const std::string& name() const { return name_; }
    /// plain name change, for detached items or items whose key does not follow the name
    void setName(const std::string& name) { name_ = name; }
    /// renames an attached item: moves the remembered states of its subtree to the new key, notifies the change
    void rename(const std::string& name);

    /// Stable identifier used for viewpoint round-trip of hidden state. Empty
    /// string means "do not persist".
    virtual std::string persistenceId() const { return {}; }

    /// identifier of the item among its siblings (plain, escaped by stateKey()), empty => the name
    virtual std::string stateKeyId() const { return persistenceId(); }

    /// Unique key of the item state across rebuilds: the parent's key plus the item's escaped segment
    /// (stateKeyId(), else the name with "#n" for repeated sibling names) => keys start with their parent's key.
    /// Notice: also called on items about to be removed.
    std::string stateKey() const;

    // ---- display -----------------------------------------------------------

    /// Base Name column. Subclasses override for col >= 1 (Count, custom cols).
    /// Groups that want the model to aggregate a column should return an
    /// invalid QVariant for that column and rely on the ColumnSpec aggregator.
    virtual QVariant itemData(int column) const;
    virtual QVariant icon() const;
    virtual QVariant tooltip() const { return {}; }
    virtual QVariant sortValue() const { return QString::fromStdString(name_); }

    virtual bool isExpandable() const { return true; }
    virtual bool isReorderable() const { return false; }

    // ---- color (display only, no editing) ---------------------------------

    QColor color() const { return color_; }
    void setColor(const QColor& color);

    // ---- visibility --------------------------------------------------------

    virtual bool canHide() const { return true; }
    bool hidden() const { return hidden_; }
    /// hidden state when first attached to a model, only deviations from it are remembered
    bool defaultHidden() const { return default_hidden_; }
    /// self or any ancestor hidden
    bool effectiveHidden() const;

    /// Flip hidden state on self, cascade updateHidden() through subtree, emit
    /// `hiddenChangedSignal` via model once at the end (unless suppressed).
    /// Virtual so subclasses can replace the toggle with custom semantics
    /// (e.g. radio-style single-select). Subclasses overriding this method
    /// should call the base via `LayerTreeItem::setHidden(...)` for any path
    /// that should retain the default behavior.
    virtual void setHidden(bool value, bool emit_signal = true);

    /// Flip all descendants hidden (includes self). Emits once at end.
    void hideSubtree(bool emit_signal = true);
    void showSubtree(bool emit_signal = true);

    // ---- item config -------------------------------------------------------

    /// true => the hidden state is written to config(), set per subtree via persistsSubtreeToConfig()
    bool persistHiddenStateToConfig() const;
    /// true => this item and all items below persist their hidden state to the config
    virtual bool persistsSubtreeToConfig() const { return false; }

    /// the item's entry in the model's config, created on first access
    /// Notice: only after attach, the key depends on the parent chain
    nlohmann::json& config();
    /// read-only, an empty object if there is no entry or not attached (none is created)
    const nlohmann::json& config() const;

    // ---- context menu -----------------------------------------------------

    /// Append this item's own actions to `menu`. Default: no-op.
    virtual void buildContextMenu(QMenu& menu) { (void)menu; }

    /// runs func on the item with this item's key when the action is triggered, nothing if there is none
    /// Notice: menus may stay open across rebuilds (QMenu::exec()) => never capture the item itself
    void connectItemAction(QAction* action, const std::function<void(LayerTreeItem&)>& func) const;

    // ---- hooks -------------------------------------------------------------

    /// Called on every descendant after a visibility change bubbles through.
    /// Subclasses override to push state to owned data (e.g. chart series).
    virtual void onEffectiveHiddenChanged() {}

    /// called once after the item got attached to a model (e.g. config-dependent setup)
    /// Notice: may run inside model insert/reset notifications => no structural changes
    virtual void onAttached() {}
    /// called after the model's config was replaced (e.g. preset applied)
    virtual void onConfigChanged() {}

protected:
    friend class LayerTreeModel; // restores the state of attached items

    /// Recompute rounded-square color icon (14x14) from color_.
    void rebuildColorIcon();

    /// Apply updateHidden hook to self and all descendants.
    void cascadeEffectiveHidden();

    void storeHiddenState();
    /// hidden state from the config, if persisted there
    std::optional<bool> configuredHidden() const;

    std::string    name_;
    QColor         color_;
    QIcon          color_icon_;
    bool           hidden_{false};
    bool           default_hidden_{false};

    LayerTreeItem* parent_item_{nullptr};
    LayerTreeModel* model_{nullptr};

    std::vector<std::unique_ptr<LayerTreeItem>> children_;
};
