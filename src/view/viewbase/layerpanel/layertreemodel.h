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

#include "layertreedefs.h"
#include "layertreestate.h"

#include <QAbstractItemModel>
#include <QHeaderView>
#include <QString>
#include <QVariant>

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

class LayerTreeItem;
class QMenu;

/**
 * QAbstractItemModel over a LayerTreeItem tree.
 *
 * Built-in columns:
 *   0 - Name
 *   1 - Count (sum-aggregated across descendants by default)
 *
 * Views may register additional columns via addColumn(). The model tracks
 * column 0 width as Stretch, column 1 fixed, and applies each custom spec's
 * resize/width settings to the QTreeView header once wired via
 * applyHeaderSettings().
 *
 * Signals emitted by items (via model_ pointer) are re-emitted here so panel
 * / view code can subscribe.
 *
 * Remembered state (item states, item config, effective-hidden ids) lives in state(),
 * the model applies it to the tree (attach, config changes, pruning).
 */
class LayerTreeModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void hiddenChangedSignal();
    void colorChangedSignal();
    void modelChangedSignal();
    /// Emitted at the end of refreshSubtree() with the parent whose children
    /// were just rebuilt. Lets panel-level code (re-)apply expansion or other
    /// view state without having to subscribe to begin/endInsertRows manually.
    void subtreeRefreshedSignal(LayerTreeItem* parent);
    /// a destroyed panel stored its view state, e.g. for its successor
    void viewStateHandedOverSignal();

public:
    enum DataRole
    {
        IconRole = Qt::UserRole + 100
    };

    explicit LayerTreeModel(QObject* parent = nullptr);
    ~LayerTreeModel() override;

    // Expose reset-scope methods so views can wrap non-trivial tree rewrites.
    using QAbstractItemModel::beginResetModel;
    void endResetModel();

    // ---- tree construction -------------------------------------------------

    /// Add a top-level item under the invisible root. Takes ownership.
    /// Must be called before the panel is shown with data, or wrapped in
    /// beginResetModel/endResetModel.
    LayerTreeItem* addRootItem(std::unique_ptr<LayerTreeItem> item);

    LayerTreeItem* invisibleRootItem() const { return root_item_.get(); }

    // ---- columns -----------------------------------------------------------

    /// Register a custom column (appended after Name, Count).
    int addColumn(const LayerColumnSpec& spec);

    int totalColumnCount() const { return 2 + (int)custom_columns_.size(); }
    const LayerColumnSpec* columnSpec(int column) const;

    /// Apply each column's resize/width/alignment to the given header.
    void applyHeaderSettings(QHeaderView* header) const;

    // ---- scoped refresh ---------------------------------------------------

    /// Scoped refill of a subtree without nuking expansion/selection elsewhere.
    /// build_children must return the new children detached (not yet attached
    /// to parent); the model performs begin/endRemoveRows and
    /// begin/endInsertRows around the mutation.
    /// complete => the remembered states of items below parent that did not come back are dropped,
    /// false for intermediate rebuilds (e.g. emptied at loading start), pruned later via pruneItemStates()
    void refreshSubtree(LayerTreeItem* parent,
                        const std::function<std::vector<std::unique_ptr<LayerTreeItem>>()>& build_children,
                        bool complete);

    /// appends a child to an attached parent, with insert notification
    LayerTreeItem* appendChildItem(LayerTreeItem* parent, std::unique_ptr<LayerTreeItem> item);
    /// removes a child of an attached parent, with remove notification, and drops its subtree's states
    void removeChildItem(LayerTreeItem* parent, int row);
    /// notifies a changed display of an attached item (e.g. name)
    void itemChanged(LayerTreeItem* item);

    // ---- traversal ---------------------------------------------------------

    /// Walk every item depth-first. max_depth < 0 means unlimited.
    void traverse(const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                  int max_depth = -1) const;

    // ---- context menu -----------------------------------------------------

    /// Optional model-level context-menu extension. Lets a view-specific
    /// model subclass append custom actions for `item` (e.g. geometry node
    /// actions) without subclassing the tree items. Invoked by
    /// LayerPanelWidget after item->buildContextMenu(). Default: no-op.
    virtual void buildCustomItemContextMenu(QMenu& menu, LayerTreeItem* item)
    {
        (void)menu;
        (void)item;
    }

    // ---- QAbstractItemModel -----------------------------------------------

    QVariant data(const QModelIndex& index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex& index) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column,
                      const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;

    // ---- remembered state --------------------------------------------------

    /// item states, item config root, effective-hidden ids (kept current on every hiddenChangedSignal
    /// and after each refreshSubtree(), cleared on database close via state().clear())
    LayerTreeState& state() { return state_; }
    const LayerTreeState& state() const { return state_; }

    /// drops the states below scope (nullptr => whole tree) doPruneItem() rejects, call after a complete rebuild
    void pruneItemStates(const LayerTreeItem* scope = nullptr);
    /// drops the given components (combination of LayerTreeState::PersistFlag) of the subtree's item states
    void clearItemStates(const LayerTreeItem* subtree, int flags);

    /// the item with the given state key, nullptr if there is none
    LayerTreeItem* findItem(const std::string& key) const;

    /// binds the view's item config (see LayerTreeState::bindConfig()) and applies it to the current items
    void bindConfig(nlohmann::json* root);
    /// re-applies the config to all items, e.g. after a preset was applied
    void configChanged();

    /// called by items on hidden changes
    void storeHiddenState(const LayerTreeItem& item);
    /// called by items when a subtree got attached to this model
    void subtreeAttached(LayerTreeItem* subtree);

    // ---- icon-change re-queries ------------------------------------------

    /// Emit dataChanged for `item` and its ancestor chain (column 0 only,
    /// icon-related roles). Used after color mutations propagate through.
    void notifyIconChanged(LayerTreeItem* item);

protected:
    /// prune criterion, default: the item is not in the pruned scope
    virtual bool doPruneItem(const std::string& key,
                             const LayerTreeState::ItemState& state,
                             const std::set<std::string>& present_keys) const;

private:
    QModelIndex indexOfItem(const LayerTreeItem* item) const;

    void captureHiddenState();
    void flushHiddenNotification();
    void checkUniqueKeys(const LayerTreeItem* parent) const;

    QVariant aggregateColumnFor(LayerTreeItem* item, int column,
                                const LayerColumnSpec& spec) const;
    void collectChildValues(LayerTreeItem* item, int column,
                            std::vector<QVariant>& out) const;

    void traverseImpl(const QModelIndex& index, LayerTreeItem* item,
                      const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                      int depth, int max_depth) const;

    std::unique_ptr<LayerTreeItem>  root_item_;
    std::vector<LayerColumnSpec>    custom_columns_;

    LayerTreeState state_;
    bool           restoring_state_       = false; // suppresses storing while restoring
    bool           hidden_notify_pending_ = false; // restored hidden states not signalled yet
};
