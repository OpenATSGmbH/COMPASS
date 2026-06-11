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

#include "layertreemodel.h"
#include "layertreeitem.h"
#include "traced_assert.h"

LayerTreeModel::LayerTreeModel(QObject* parent)
    : QAbstractItemModel(parent)
    , root_item_(std::make_unique<LayerTreeItem>("<root>", nullptr))
{
    // invisible root's model_ = this; children inherit on appendChild.
    root_item_->setModel(this);

    connect(this, &LayerTreeModel::hiddenChangedSignal,
            this, [this]{ emit layoutChanged(); });

    // Keep the hidden-state memory current on every visibility change.
    // Connected here, before any view-side hiddenChangedSignal handlers
    // attach, so those handlers see an up-to-date storedHiddenIds().
    connect(this, &LayerTreeModel::hiddenChangedSignal,
            this, &LayerTreeModel::captureHiddenState);
}

LayerTreeModel::~LayerTreeModel() = default;

void LayerTreeModel::endResetModel()
{
    QAbstractItemModel::endResetModel();
    emit modelChangedSignal();
}

LayerTreeItem* LayerTreeModel::addRootItem(std::unique_ptr<LayerTreeItem> item)
{
    const int row = root_item_->childCount();
    beginInsertRows(QModelIndex(), row, row);
    LayerTreeItem* raw = root_item_->appendChild(std::move(item));
    endInsertRows();
    return raw;
}

int LayerTreeModel::addColumn(const LayerColumnSpec& spec)
{
    const int new_col = 2 + (int)custom_columns_.size();
    beginInsertColumns(QModelIndex(), new_col, new_col);
    custom_columns_.push_back(spec);
    endInsertColumns();
    return new_col;
}

const LayerColumnSpec* LayerTreeModel::columnSpec(int column) const
{
    const int idx = column - 2;
    if (idx < 0 || idx >= (int)custom_columns_.size())
        return nullptr;
    return &custom_columns_[idx];
}

void LayerTreeModel::applyHeaderSettings(QHeaderView* header) const
{
    if (!header)
        return;

    header->setStretchLastSection(false);
    header->setSectionResizeMode(0, QHeaderView::Stretch);
    header->resizeSection(0, 230);  // initial width; Stretch redistributes remaining space
    header->setSectionResizeMode(1, QHeaderView::Interactive);
    header->resizeSection(1, 70);

    for (int i = 0; i < (int)custom_columns_.size(); ++i)
    {
        const int col = 2 + i;
        const auto& spec = custom_columns_[i];
        header->setSectionResizeMode(col, spec.resize_mode);
        if (spec.resize_mode == QHeaderView::Interactive ||
            spec.resize_mode == QHeaderView::Fixed)
        {
            header->resizeSection(col, spec.default_width);
        }
    }
}

void LayerTreeModel::refreshSubtree(LayerTreeItem* parent,
                                    const std::function<std::vector<std::unique_ptr<LayerTreeItem>>()>& build_children)
{
    traced_assert(parent);

    const QModelIndex parent_idx =
        (parent == root_item_.get()) ? QModelIndex()
                                     : createIndex(parent->row(), 0, parent);

    const int old_count = parent->childCount();
    if (old_count > 0)
    {
        beginRemoveRows(parent_idx, 0, old_count - 1);
        parent->clearChildren();
        endRemoveRows();
    }

    auto new_children = build_children();
    const int new_count = (int)new_children.size();
    if (new_count > 0)
    {
        beginInsertRows(parent_idx, 0, new_count - 1);
        for (auto& c : new_children)
            parent->appendChild(std::move(c));
        endInsertRows();
    }

    // Restore remembered unchecked state on the freshly built items - new
    // items default to visible/checked. Applying the own-hidden ids (groups
    // and leaves alike) reproduces the literal checkbox states; descendants
    // of an unchecked group follow via effectiveHidden(). Guarded so the
    // hiddenChangedSignal emitted by the apply does not run a redundant
    // mid-apply capture; the capture afterwards re-syncs both stored sets to
    // the new tree (e.g. picks up new leaves under a restored group).
    if (!stored_unchecked_ids_.empty())
    {
        applying_hidden_state_ = true;
        applyPersistedHiddenIds(stored_unchecked_ids_);
        applying_hidden_state_ = false;
    }
    captureHiddenState();

    emit subtreeRefreshedSignal(parent);
}

void LayerTreeModel::traverse(const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                              int max_depth) const
{
    for (int i = 0; i < root_item_->childCount(); ++i)
    {
        auto* child = root_item_->child(i);
        QModelIndex idx = const_cast<LayerTreeModel*>(this)->createIndex(i, 0, child);
        traverseImpl(idx, child, fn, 0, max_depth);
    }
}

void LayerTreeModel::traverseImpl(const QModelIndex& index, LayerTreeItem* item,
                                  const std::function<void(const QModelIndex&, LayerTreeItem*)>& fn,
                                  int depth, int max_depth) const
{
    fn(index, item);
    if (max_depth >= 0 && depth >= max_depth)
        return;
    for (int i = 0; i < item->childCount(); ++i)
    {
        auto* c = item->child(i);
        QModelIndex child_idx = const_cast<LayerTreeModel*>(this)->createIndex(i, 0, c);
        traverseImpl(child_idx, c, fn, depth + 1, max_depth);
    }
}

// ---- QAbstractItemModel -------------------------------------------------

QVariant LayerTreeModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return {};

    auto* item = static_cast<LayerTreeItem*>(index.internalPointer());
    traced_assert(item);

    const int col = index.column();

    switch (role)
    {
    case IconRole:
        if (col == 0) return item->icon();
        return {};

    case Qt::DisplayRole:
    {
        // col 0: name (no aggregation; the delegate paints icon+text itself).
        if (col == 0) return item->itemData(0);

        // col >= 1: own value, else aggregate
        QVariant own = item->itemData(col);
        if (own.isValid()) return own;

        if (col == 1)
        {
            // built-in Count - sum over descendants' own col-1 values
            std::vector<QVariant> vals;
            collectChildValues(item, 1, vals);
            unsigned long long sum = 0;
            bool any = false;
            for (const auto& v : vals)
            {
                bool ok = false;
                unsigned long long n = v.toULongLong(&ok);
                if (ok) { sum += n; any = true; }
            }
            return any ? QVariant((qulonglong)sum) : QVariant();
        }

        const auto* spec = columnSpec(col);
        if (!spec || !spec->group_aggregator) return {};
        std::vector<QVariant> vals;
        collectChildValues(item, col, vals);
        return spec->group_aggregator(vals);
    }

    case Qt::TextAlignmentRole:
    {
        if (col == 1)
            return int(Qt::AlignRight | Qt::AlignVCenter);
        if (const auto* spec = columnSpec(col))
            return int(spec->alignment);
        return {};
    }

    case Qt::ToolTipRole:
        if (col == 0) return item->tooltip();
        return {};

    default:
        return {};
    }
}

Qt::ItemFlags LayerTreeModel::flags(const QModelIndex& index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;
    return QAbstractItemModel::flags(index);
}

QVariant LayerTreeModel::headerData(int section, Qt::Orientation orientation,
                                    int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
        return {};

    if (section == 0) return QString("Name");
    if (section == 1) return QString("Count");
    const auto* spec = columnSpec(section);
    if (spec) return spec->header;
    return {};
}

QModelIndex LayerTreeModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return {};

    LayerTreeItem* parent_item = parent.isValid()
        ? static_cast<LayerTreeItem*>(parent.internalPointer())
        : root_item_.get();

    LayerTreeItem* child_item = parent_item->child(row);
    if (!child_item) return {};
    return createIndex(row, column, child_item);
}

QModelIndex LayerTreeModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return {};

    auto* child_item = static_cast<LayerTreeItem*>(index.internalPointer());
    auto* parent_item = child_item->parentItem();

    if (!parent_item || parent_item == root_item_.get())
        return {};

    return createIndex(parent_item->row(), 0, parent_item);
}

int LayerTreeModel::rowCount(const QModelIndex& parent) const
{
    LayerTreeItem* parent_item = parent.isValid()
        ? static_cast<LayerTreeItem*>(parent.internalPointer())
        : root_item_.get();
    return parent_item->childCount();
}

int LayerTreeModel::columnCount(const QModelIndex& /*parent*/) const
{
    return totalColumnCount();
}

// ---- persistence helpers --------------------------------------------------

std::set<std::string> LayerTreeModel::persistedHiddenIds() const
{
    std::set<std::string> ids;
    std::function<void(LayerTreeItem*)> walk = [&](LayerTreeItem* it)
    {
        const std::string id = it->persistenceId();
        if (!id.empty() && it->effectiveHidden())
            ids.insert(id);
        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));
    };
    walk(root_item_.get());
    return ids;
}

void LayerTreeModel::applyPersistedHiddenIds(const std::set<std::string>& ids)
{
    if (ids.empty())
        return;

    // Collect first so the layout/hidden signals are skipped entirely when no
    // shown item matches (ids may refer to layers absent from the current load).
    std::vector<LayerTreeItem*> to_hide;
    std::function<void(LayerTreeItem*)> walk = [&](LayerTreeItem* it)
    {
        const std::string id = it->persistenceId();
        if (!id.empty() && !it->hidden() && ids.count(id))
            to_hide.push_back(it);
        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));
    };
    walk(root_item_.get());

    if (to_hide.empty())
        return;

    // Mutate hidden_ on matching items in place; emit layoutChanged so views
    // re-render without the header losing section widths (which a full
    // modelReset would trigger).
    emit layoutAboutToBeChanged();

    for (auto* it : to_hide)
        it->setHidden(true, /*emit_signal=*/false);

    emit layoutChanged();
    emit hiddenChangedSignal();
}

/**
 * Keeps the two stored hidden sets aligned with the tree's checked state:
 * own-hidden ids (stored_unchecked_ids_, the literal checkbox states restored
 * in refreshSubtree) and effective-hidden ids (stored_hidden_ids_, consumed
 * by view-side data filters). Merge semantics: ids with an item in the
 * current tree are re-captured from that item's state; stored ids without a
 * matching item (e.g. a data source not part of the current load) are kept,
 * so their unchecked state survives until the next load that contains them -
 * or until clearStoredHiddenState().
 */
void LayerTreeModel::captureHiddenState()
{
    if (applying_hidden_state_)
        return;

    std::function<void(LayerTreeItem*)> walk = [&](LayerTreeItem* it)
    {
        const std::string id = it->persistenceId();
        if (!id.empty())
        {
            if (it->hidden())
                stored_unchecked_ids_.insert(id);
            else
                stored_unchecked_ids_.erase(id);

            if (it->effectiveHidden())
                stored_hidden_ids_.insert(id);
            else
                stored_hidden_ids_.erase(id);
        }
        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));
    };
    walk(root_item_.get());
}

/**
 */
void LayerTreeModel::clearStoredHiddenState()
{
    stored_hidden_ids_.clear();
    stored_unchecked_ids_.clear();
}

void LayerTreeModel::notifyIconChanged(LayerTreeItem* item)
{
    if (!item) return;
    for (auto* a = item; a && a != root_item_.get(); a = a->parentItem())
    {
        QModelIndex idx = createIndex(a->row(), 0, a);
        emit dataChanged(idx, idx, {Qt::DecorationRole, IconRole});
    }
}

// ---- helpers --------------------------------------------------------------

void LayerTreeModel::collectChildValues(LayerTreeItem* item, int column,
                                        std::vector<QVariant>& out) const
{
    for (int i = 0; i < item->childCount(); ++i)
    {
        auto* c = item->child(i);
        QVariant v = c->itemData(column);
        if (v.isValid())
            out.push_back(v);
        else
            collectChildValues(c, column, out);
    }
}

QVariant LayerTreeModel::aggregateColumnFor(LayerTreeItem* item, int column,
                                            const LayerColumnSpec& spec) const
{
    if (!spec.group_aggregator)
        return {};
    std::vector<QVariant> vals;
    collectChildValues(item, column, vals);
    return spec.group_aggregator(vals);
}
