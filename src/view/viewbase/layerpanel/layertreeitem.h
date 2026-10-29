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
    /// as appendChild(), at the given row
    LayerTreeItem* insertChild(int row, std::unique_ptr<LayerTreeItem> child);
    /// moves the child at row from to row to (its final row)
    void moveChild(int from, int to);
    void clearChildren();

    /// Remove and destroy a single child by row index.
    void removeChildAt(int row);
    /// removes and destroys the children in the row range [first, last]
    void removeChildrenAt(int first, int last);

    /// structural changes with model notification if attached (plain operations otherwise)
    LayerTreeItem* appendChildNotified(std::unique_ptr<LayerTreeItem> child);
    /// the removed subtree is forgotten
    void removeChildNotified(int row);
    /// notifies a changed display of the item (e.g. name), if attached
    void notifyChanged();

    // ---- update in place ---------------------------------------------------

    /// Updates the subtree from the domain: dynamic items (childKeys()) keep their children which still
    /// exist, drop the vanished ones (forgotten) and create the new ones; static items pass the update on.
    /// Notified via LayerTreeModel::updateSubtree() if attached, plain operations otherwise.
    void update();

    /// desired child keys (updateKey() of the children) in display order, nullopt => static children
    virtual std::optional<std::vector<std::string>> childKeys() const { return std::nullopt; }
    /// creates the detached child for a key not present yet (its subtree is filled by its own update)
    virtual std::unique_ptr<LayerTreeItem> createChild(const std::string& key) { (void)key; return nullptr; }
    /// refreshes a kept child from the domain, true if its display changed
    virtual bool updateChild(LayerTreeItem& child) { (void)child; return false; }
    /// called after the item's subtree was updated (bottom-up), e.g. to recompute derived display
    virtual void onSubtreeUpdated() {}

    /// identifies the item among its siblings on updates, default stateKeyId(), else the name
    /// Notice: separate from stateKey(), e.g. a session-only id matches while the persisted key stays
    virtual std::string updateKey() const;

    /// model attachment; LayerTreeModel calls setModel on the invisible root
    /// which propagates to every descendant.
    void setModel(LayerTreeModel* model);
    LayerTreeModel* model() const { return model_; }

    // ---- identity / persistence -------------------------------------------

    const std::string& name() const { return name_; }
    /// plain name change, for detached items or items whose key does not follow the name
    void setName(const std::string& name) { name_ = name; }
    /// renames an attached item: moves the item configs of its subtree to the new key, notifies the change
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

    /// shows outdated content (e.g. data being reloaded), painted as configured in LayerTreeItemDelegate
    virtual bool isStale() const { return false; }

    // ---- color (display only, no editing) ---------------------------------

    QColor color() const { return color_; }
    void setColor(const QColor& color);

    // ---- visibility --------------------------------------------------------

    virtual bool canHide() const { return true; }
    bool hidden() const { return hidden_; }
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

    // ---- panel handover ----------------------------------------------------

    /**
     * View state a destroyed panel leaves on the item for the next panel on the same model.
     */
    struct ViewHandover
    {
        bool expanded = false;
        bool selected = false;
        bool current  = false; // the tree view's current row
    };

    void setViewHandover(const ViewHandover& handover) { view_handover_ = handover; }
    const std::optional<ViewHandover>& viewHandover() const { return view_handover_; }
    void clearViewHandover() { view_handover_.reset(); }

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
    friend class LayerTreeModel; // applies the config to attached items

    /// Recompute rounded-square color icon (14x14) from color_.
    void rebuildColorIcon();

    /// Apply updateHidden hook to self and all descendants.
    void cascadeEffectiveHidden();

    /// writes the hidden state to the config of persisted subtrees
    void storeHiddenState();
    /// hidden state from the config, if persisted there
    std::optional<bool> configuredHidden() const;

    /// update() of the subtree, true if the display of the subtree changed
    bool updateChildren();
    /// reconciles the own children with the keys, collects the kept ones
    bool reconcileChildren(const std::vector<std::string>& keys,
                           std::vector<LayerTreeItem*>& kept,
                           bool& kept_changed);

    /// structural changes with model notification if attached (plain operations otherwise)
    void insertChildrenNotified(int row, std::vector<std::unique_ptr<LayerTreeItem>> children);
    void moveChildNotified(int from, int to);
    void removeChildrenNotified(int first, int last);

    std::string    name_;
    QColor         color_;
    QIcon          color_icon_;
    bool           hidden_{false};

    LayerTreeItem* parent_item_{nullptr};
    LayerTreeModel* model_{nullptr};

    std::vector<std::unique_ptr<LayerTreeItem>> children_;

    std::optional<ViewHandover> view_handover_; // see setViewHandover()
};
