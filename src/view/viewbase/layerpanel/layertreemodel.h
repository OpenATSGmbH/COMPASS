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

/**
 * Spec for a custom column. Custom columns start at index 2 (Name=0, Count=1).
 *
 * group_aggregator: optional — when a group row has no own value for this
 * column (itemData returns invalid), the model collects valid values from the
 * full descendant subtree and passes them to this callback. Without an
 * aggregator the group row stays empty.
 */
struct LayerColumnSpec
{
    QString                 header;
    int                     default_width = 80;
    QHeaderView::ResizeMode resize_mode   = QHeaderView::Interactive;
    Qt::Alignment           alignment     = Qt::AlignLeft | Qt::AlignVCenter;
    std::function<QVariant(const std::vector<QVariant>&)> group_aggregator;
};

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
 */
class LayerTreeModel : public QAbstractItemModel
{
    Q_OBJECT

signals:
    void hiddenChangedSignal();
    void colorChangedSignal();

public:
    enum DataRole
    {
        IconRole = Qt::UserRole + 100
    };

    explicit LayerTreeModel(QObject* parent = nullptr);
    ~LayerTreeModel() override;

    // Expose reset-scope methods so views can wrap non-trivial tree rewrites.
    using QAbstractItemModel::beginResetModel;
    using QAbstractItemModel::endResetModel;

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
    void refreshSubtree(LayerTreeItem* parent,
                        const std::function<std::vector<std::unique_ptr<LayerTreeItem>>()>& build_children);

    // ---- traversal ---------------------------------------------------------

    /// Walk every item depth-first. max_depth < 0 means unlimited.
    void traverse(const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                  int max_depth = -1) const;

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

    // ---- persistence helpers ---------------------------------------------

    /// Collect persistenceId() of every descendant leaf currently in an
    /// effective-hidden state. Empty ids are skipped.
    std::set<std::string> persistedHiddenIds() const;

    /// For every descendant leaf whose persistenceId() is in `ids`, set
    /// hidden=true. Group hidden state is untouched.
    void applyPersistedHiddenIds(const std::set<std::string>& ids);

    // ---- icon-change re-queries ------------------------------------------

    /// Emit dataChanged for `item` and its ancestor chain (column 0 only,
    /// icon-related roles). Used after color mutations propagate through.
    void notifyIconChanged(LayerTreeItem* item);

private:
    QVariant aggregateColumnFor(LayerTreeItem* item, int column,
                                const LayerColumnSpec& spec) const;
    void collectChildValues(LayerTreeItem* item, int column,
                            std::vector<QVariant>& out) const;

    void traverseImpl(const QModelIndex& index, LayerTreeItem* item,
                      const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                      int depth, int max_depth) const;

    std::unique_ptr<LayerTreeItem>  root_item_;
    std::vector<LayerColumnSpec>    custom_columns_;
};
