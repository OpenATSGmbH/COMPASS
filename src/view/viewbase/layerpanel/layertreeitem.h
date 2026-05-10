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

#include <QColor>
#include <QIcon>
#include <QVariant>

#include <memory>
#include <string>
#include <vector>

class LayerTreeModel;
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

    /// model attachment; LayerTreeModel calls setModel on the invisible root
    /// which propagates to every descendant.
    void setModel(LayerTreeModel* model);
    LayerTreeModel* model() const { return model_; }

    // ---- identity / persistence -------------------------------------------

    const std::string& name() const { return name_; }
    void setName(const std::string& name) { name_ = name; }

    /// Stable identifier used for viewpoint round-trip of hidden state. Empty
    /// string means "do not persist".
    virtual std::string persistenceId() const { return {}; }

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
    /// self or any ancestor hidden
    bool effectiveHidden() const;

    /// Flip hidden state on self, cascade updateHidden() through subtree, emit
    /// `hiddenChangedSignal` via model once at the end (unless suppressed).
    /// Virtual so subclasses can replace the toggle with custom semantics
    /// (e.g. radio-style single-select). Subclasses overriding this method
    /// should call the base via `LayerTreeItem::setHidden(...)` for any path
    /// that should retain the default behaviour.
    virtual void setHidden(bool value, bool emit_signal = true);

    /// Flip all descendants hidden (includes self). Emits once at end.
    void hideSubtree(bool emit_signal = true);
    void showSubtree(bool emit_signal = true);

    // ---- context menu -----------------------------------------------------

    /// Append this item's own actions to `menu`. Default: no-op.
    virtual void buildContextMenu(QMenu& menu) { (void)menu; }

    // ---- hooks -------------------------------------------------------------

    /// Called on every descendant after a visibility change bubbles through.
    /// Subclasses override to push state to owned data (e.g. chart series).
    virtual void onEffectiveHiddenChanged() {}

protected:
    /// Recompute rounded-square color icon (14x14) from color_.
    void rebuildColorIcon();

    /// Apply updateHidden hook to self and all descendants.
    void cascadeEffectiveHidden();

    std::string    name_;
    QColor         color_;
    QIcon          color_icon_;
    bool           hidden_{false};

    LayerTreeItem* parent_item_{nullptr};
    LayerTreeModel* model_{nullptr};

    std::vector<std::unique_ptr<LayerTreeItem>> children_;
};
