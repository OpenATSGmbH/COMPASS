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

#include "logger.h"
#include "traced_assert.h"

#include "json.hpp"

LayerTreeModel::LayerTreeModel(QObject* parent)
    : QAbstractItemModel(parent)
    , root_item_(std::make_unique<LayerTreeItem>("<root>", nullptr))
{
    // invisible root's model_ = this; children inherit on appendChild.
    root_item_->setModel(this);

    // Keep the hidden-state memory current on every visibility change.
    // Connected here, before any view-side hiddenChangedSignal handlers
    // attach, so those handlers see up-to-date state().hiddenIds().
    connect(this, &LayerTreeModel::hiddenChangedSignal,
            this, &LayerTreeModel::captureHiddenState);
}

LayerTreeModel::~LayerTreeModel() = default;

void LayerTreeModel::endResetModel()
{
    QAbstractItemModel::endResetModel();
    flushHiddenNotification();
    emit modelChangedSignal();
}

LayerTreeItem* LayerTreeModel::addRootItem(std::unique_ptr<LayerTreeItem> item)
{
    return appendChildItem(root_item_.get(), std::move(item));
}

/**
 */
LayerTreeItem* LayerTreeModel::appendChildItem(LayerTreeItem* parent, std::unique_ptr<LayerTreeItem> item)
{
    traced_assert(parent);

    return insertChildItem(parent, parent->childCount(), std::move(item));
}

/**
 */
LayerTreeItem* LayerTreeModel::insertChildItem(LayerTreeItem* parent, int row, std::unique_ptr<LayerTreeItem> item)
{
    std::vector<std::unique_ptr<LayerTreeItem>> items;
    items.push_back(std::move(item));

    insertChildItems(parent, row, std::move(items));

    return parent->child(row);
}

/**
 */
void LayerTreeModel::insertChildItems(LayerTreeItem* parent, int row, std::vector<std::unique_ptr<LayerTreeItem>> items)
{
    traced_assert(parent);
    traced_assert(parent->model() == this);
    traced_assert(row >= 0 && row <= parent->childCount());

    if (items.empty())
        return;

    const int n = (int)items.size();

    beginInsertRows(indexOfItem(parent), row, row + n - 1);
    for (int i = 0; i < n; ++i)
        parent->insertChild(row + i, std::move(items[ i ]));
    endInsertRows();

    // the configured hidden states were applied on attach, see subtreeAttached()
    checkUniqueKeys(parent);
    structureChanged();
}

/**
 */
void LayerTreeModel::moveChildItem(LayerTreeItem* parent, int from, int to)
{
    traced_assert(parent);
    traced_assert(parent->model() == this);
    traced_assert(from >= 0 && from < parent->childCount());
    traced_assert(to >= 0 && to < parent->childCount());

    if (from == to)
        return;

    const QModelIndex parent_idx = indexOfItem(parent);

    // Qt counts the destination before the move
    const bool ok = beginMoveRows(parent_idx, from, from, parent_idx, to < from ? to : to + 1);
    traced_assert(ok);

    parent->moveChild(from, to);

    endMoveRows();
}

/**
 */
void LayerTreeModel::removeChildItem(LayerTreeItem* parent, int row)
{
    removeChildItems(parent, row, row);
}

/**
 * Notice: the removed items are forgotten, the effective-hidden ids follow with structureChanged().
 */
void LayerTreeModel::removeChildItems(LayerTreeItem* parent, int first, int last)
{
    traced_assert(parent);
    traced_assert(parent->model() == this);
    traced_assert(first >= 0 && first <= last && last < parent->childCount());

    beginRemoveRows(indexOfItem(parent), first, last);
    parent->removeChildrenAt(first, last);
    endRemoveRows();
}

/**
 */
void LayerTreeModel::itemChanged(LayerTreeItem* item)
{
    traced_assert(item && item != root_item_.get());
    traced_assert(item->model() == this);

    const QModelIndex idx = indexOfItem(item);

    emit dataChanged(idx, idx.sibling(idx.row(), columnCount() - 1));
}

/**
 */
void LayerTreeModel::childItemsChanged(LayerTreeItem* parent)
{
    traced_assert(parent);
    traced_assert(parent->model() == this);

    const int n = parent->childCount();
    if (n == 0)
        return;

    emit dataChanged(createIndex(0, 0, parent->child(0)),
                     createIndex(n - 1, columnCount() - 1, parent->child(n - 1)));
}

/**
 * Notice: the hidden signal and the effective-hidden ids follow once at the end.
 */
void LayerTreeModel::updateSubtree(LayerTreeItem* item)
{
    traced_assert(item);
    traced_assert(item->model() == this);

    ++update_depth_;
    const bool changed = item->updateChildren();
    --update_depth_;

    // the display of the item and its ancestors depends on the subtree (aggregates)
    if (changed)
        for (LayerTreeItem* it = item; it && it != root_item_.get(); it = it->parentItem())
            itemChanged(it);

    structureChanged();
}

/**
 */
void LayerTreeModel::updateAll()
{
    updateSubtree(root_item_.get());
}

/**
 */
void LayerTreeModel::structureChanged()
{
    if (update_depth_ > 0)
        return;

    flushHiddenNotification();
    captureHiddenState();
}

/**
 */
QModelIndex LayerTreeModel::indexOfItem(const LayerTreeItem* item) const
{
    if (item == root_item_.get())
        return {};

    return createIndex(item->row(), 0, const_cast<LayerTreeItem*>(item));
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

// ---- config and hidden ids ------------------------------------------------

/**
 * Keeps the effective-hidden ids (consumed by view-side data filters) aligned with the tree.
 */
void LayerTreeModel::captureHiddenState()
{
    std::function<void(LayerTreeItem*)> walk = [&](LayerTreeItem* it)
    {
        const std::string id = it->persistenceId();
        if (!id.empty())
            state_.setEffectiveHidden(id, it->effectiveHidden());

        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));
    };
    walk(root_item_.get());
}

/**
 * Descends only into items whose key is a prefix of the searched key.
 */
LayerTreeItem* LayerTreeModel::findItem(const std::string& key) const
{
    if (key.empty())
        return nullptr;

    std::function<LayerTreeItem*(LayerTreeItem*)> search = [&](LayerTreeItem* parent) -> LayerTreeItem*
    {
        for (int i = 0; i < parent->childCount(); ++i)
        {
            LayerTreeItem*    child     = parent->child(i);
            const std::string child_key = child->stateKey();

            if (child_key == key)
                return child;

            const bool below = key.size() > child_key.size() &&
                               key.compare(0, child_key.size(), child_key) == 0 &&
                               key[ child_key.size() ] == '/';
            if (!below)
                continue;

            if (LayerTreeItem* hit = search(child))
                return hit;
        }

        return nullptr;
    };

    return search(root_item_.get());
}

/**
 * Applies the configured hidden states (persisted subtrees) to a newly attached subtree, the
 * other items keep their constructed state.
 * Notice: may run inside begin/endInsertRows or a reset => the hidden signal is deferred.
 */
void LayerTreeModel::subtreeAttached(LayerTreeItem* subtree)
{
    traced_assert(subtree);

    bool changed = false;

    std::function<void(LayerTreeItem*)> restore = [&](LayerTreeItem* it)
    {
        if (it->canHide())
        {
            const std::optional<bool> target = it->configuredHidden();

            if (target && *target != it->hidden())
            {
                it->setHidden(*target, /*emit_signal=*/false);
                changed = true;
            }
        }

        checkUniqueKeys(it);

        for (int i = 0; i < it->childCount(); ++i)
            restore(it->child(i));
    };
    restore(subtree);

    // below an unchecked ancestor => new items start effectively hidden
    const LayerTreeItem* parent = subtree->parentItem();
    if (parent && parent->effectiveHidden())
        subtree->cascadeEffectiveHidden();

    std::function<void(LayerTreeItem*)> notifyAttached = [&](LayerTreeItem* it)
    {
        it->onAttached();

        for (int i = 0; i < it->childCount(); ++i)
            notifyAttached(it->child(i));
    };
    notifyAttached(subtree);

    if (changed)
        hidden_notify_pending_ = true;
}

/**
 */
void LayerTreeModel::bindConfig(nlohmann::json* root)
{
    state_.bindConfig(root);

    configChanged();
}

/**
 * Applies the persisted hidden states of the config and lets all items re-read their settings.
 */
void LayerTreeModel::configChanged()
{
    bool changed = false;

    std::function<void(LayerTreeItem*)> apply = [&](LayerTreeItem* it)
    {
        if (it != root_item_.get())
        {
            const std::optional<bool> hidden = it->configuredHidden();
            if (hidden && it->canHide() && *hidden != it->hidden())
            {
                it->setHidden(*hidden, /*emit_signal=*/false);
                changed = true;
            }

            it->onConfigChanged();
        }

        for (int i = 0; i < it->childCount(); ++i)
            apply(it->child(i));
    };
    apply(root_item_.get());

    if (changed)
        emit hiddenChangedSignal();
}

/**
 * Emits the hidden signal deferred by subtreeAttached().
 */
void LayerTreeModel::flushHiddenNotification()
{
    if (!hidden_notify_pending_)
        return;

    hidden_notify_pending_ = false;

    emit hiddenChangedSignal();
}

/**
 */
void LayerTreeModel::checkUniqueKeys(const LayerTreeItem* parent) const
{
    if (parent->childCount() < 2)
        return;

    std::set<std::string> keys;
    for (int i = 0; i < parent->childCount(); ++i)
    {
        const std::string key = parent->child(i)->stateKey();
        if (!key.empty() && !keys.insert(key).second)
            logwrn << "duplicate item state key '" << key << "'";
    }
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
