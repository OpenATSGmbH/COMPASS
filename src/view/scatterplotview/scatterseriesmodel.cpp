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

#include "scatterseriesmodel.h"
#include "scatterseriestreeitem.h"
#include "logger.h"
#include "traced_assert.h"

#include "compass.h"

#include <functional>
#include <vector>

using namespace std;

ScatterSeriesModel::ScatterSeriesModel()
    : QAbstractItemModel(nullptr)
{
    root_item_.reset(new ScatterSeriesTreeItem("Data", QColor(), *this, nullptr,
                                               ScatterSeriesTreeItem::Level::Root));
}

ScatterSeriesModel::~ScatterSeriesModel()
{

}

QVariant ScatterSeriesModel::data(const QModelIndex& index, int role) const
{
    logdbg << "index row " << index.row() << " col " << index.column()
           << " valid " << index.isValid();

    if (!index.isValid())
        return QVariant();

    switch (role)
    {
    case IconRole:
    {
        if (index.column() == 1)  // only col 0 have icons
            return QVariant();

        logdbg << "icon role";
        ScatterSeriesTreeItem* item = static_cast<ScatterSeriesTreeItem*>(index.internalPointer());
        return item->icon();
    }
    case Qt::DisplayRole:
    {
        logdbg << "display role";
        ScatterSeriesTreeItem* item = static_cast<ScatterSeriesTreeItem*>(index.internalPointer());
        return item->data(index.column());
    }
    //    case Qt::BackgroundRole:
    //    {
    //        return QVariant(QColor(Qt::black));
    //    }
    default:
    {
        logdbg << "default";
        return QVariant();
    }
    }
}
Qt::ItemFlags ScatterSeriesModel::flags(const QModelIndex& index) const
{
    logdbg << "index valid " << index.isValid();

    if (!index.isValid())
        return Qt::NoItemFlags;

    return QAbstractItemModel::flags(index);
}
QVariant ScatterSeriesModel::headerData(int section, Qt::Orientation orientation,
                    int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        if (section == 0) return QString("Name");
        if (section == 1) return QString("Count");
        return QVariant();
    }

    logdbg << "wrong role";
    return QVariant();
}
QModelIndex ScatterSeriesModel::index(int row, int column,
                  const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
    {
        logerr << "row " << row << " col " << column << " not existing";
        return QModelIndex();
    }

    ScatterSeriesTreeItem* parent_item;

    logdbg << "parent valid " << parent.isValid();

    if (!parent.isValid())
        parent_item = root_item_.get();
    else
        parent_item = static_cast<ScatterSeriesTreeItem*>(parent.internalPointer());

    ScatterSeriesTreeItem* childItem = parent_item->child(row);
    if (childItem)
    {
        logdbg << "returning create index row " << row << " col " << column;
        return createIndex(row, column, childItem);
    }
    else
    {
        logerr << "child row " << row << " not existing";
        return QModelIndex();
    }
}
QModelIndex ScatterSeriesModel::parent(const QModelIndex& index) const
{
    logdbg << "index valid " << index.isValid();

    if (!index.isValid())
        return QModelIndex();

    ScatterSeriesTreeItem* child_item = static_cast<ScatterSeriesTreeItem*>(index.internalPointer());
    traced_assert(child_item);
    ScatterSeriesTreeItem* parent_item = child_item->parentItem();

    if (parent_item == root_item_.get())
        return QModelIndex();

    if (!parent_item)
    {
        logerr << "null parent in " << child_item->name();
    }

    traced_assert(parent_item);
    logdbg << "returning create index";
    return createIndex(parent_item->row(), 0, parent_item);
}
int ScatterSeriesModel::rowCount(const QModelIndex& parent) const
{
    ScatterSeriesTreeItem* parent_item;

    if (!parent.isValid())
        parent_item = root_item_.get();
    else
        parent_item = static_cast<ScatterSeriesTreeItem*>(parent.internalPointer());

    logdbg << "row " << parent.row() << " col " << parent.column()
           << " child count " << parent_item->childCount();

    return parent_item->childCount();
}
int ScatterSeriesModel::columnCount(const QModelIndex& parent) const
{
    logdbg << "index valid " << parent.isValid();

    if (parent.isValid())
        return static_cast<ScatterSeriesTreeItem*>(parent.internalPointer())->columnCount();
    else
    {
        logdbg << "root count " << root_item_->columnCount();
        return root_item_->columnCount();
    }
}

namespace
{

/// split "DSType:DS Name:L<n>:DBContent" into its four parts. returns false if
/// the key is malformed (not exactly four colon-separated fields).
bool splitSeriesKey(const std::string& key,
                    std::string& ds_type,
                    std::string& ds_name,
                    std::string& line_tok,
                    std::string& dbcontent)
{
    std::vector<std::string> parts;
    size_t start = 0;
    for (size_t i = 0; i <= key.size(); ++i)
    {
        if (i == key.size() || key[i] == ':')
        {
            parts.push_back(key.substr(start, i - start));
            start = i + 1;
        }
    }
    if (parts.size() != 4) return false;
    ds_type   = parts[0];
    ds_name   = parts[1];
    line_tok  = parts[2];
    dbcontent = parts[3];
    return true;
}

ScatterSeriesTreeItem* findOrCreateGroup(ScatterSeriesTreeItem* parent,
                                        const std::string& name,
                                        const QColor& color,
                                        ScatterSeriesModel& model,
                                        ScatterSeriesTreeItem::Level level)
{
    for (int i = 0; i < parent->childCount(); ++i)
    {
        auto* c = parent->child(i);
        if (c && c->name() == name)
            return c;
    }
    auto* item = new ScatterSeriesTreeItem(name, color, model, parent, level);
    parent->appendChild(item);
    return item;
}

} // anonymous

void ScatterSeriesModel::updateFrom(ScatterSeriesCollection& collection, COMPASS& compass)
{
    (void) compass; // kept for API stability; colors now come from the series themselves

    beginResetModel();

    root_item_->clear();

    using L = ScatterSeriesTreeItem::Level;

    for (auto& col_it : collection.dataSeries())
    {
        std::string ds_type, ds_name, line_tok, dbcontent;
        if (!splitSeriesKey(col_it.first, ds_type, ds_name, line_tok, dbcontent))
        {
            // malformed key — drop flat under root so it's still visible
            auto* leaf = new ScatterSeriesTreeItem(col_it.first, col_it.second.color,
                                                   *this, &col_it.second, root_item_.get(),
                                                   L::DBContent);
            root_item_->appendChild(leaf);
            continue;
        }

        // groups start with invalid color; it gets derived from descendant
        // leaves below. Only leaves carry an authoritative color (the chart
        // color assigned via resolveSeriesColor).
        auto* ds_type_item = findOrCreateGroup(root_item_.get(), ds_type,
                                               QColor(), *this, L::DSType);
        auto* ds_item      = findOrCreateGroup(ds_type_item, ds_name,
                                               QColor(), *this, L::DataSource);
        auto* line_item    = findOrCreateGroup(ds_item, line_tok,
                                               QColor(), *this, L::Line);

        auto* leaf = new ScatterSeriesTreeItem(dbcontent, col_it.second.color,
                                               *this, &col_it.second, line_item,
                                               L::DBContent);
        line_item->appendChild(leaf);
    }

    // propagate leaf colors up: a group's icon shows the common color of its
    // descendant leaves, or no icon if they disagree.
    root_item_->recomputeEffectiveColorRecursive();

    endResetModel();
}

void ScatterSeriesModel::notifyIconChangedSubtreeAndAncestors(ScatterSeriesTreeItem* item)
{
    if (!item) return;

    // self + descendants
    std::function<void(ScatterSeriesTreeItem*)> walk = [&](ScatterSeriesTreeItem* it)
    {
        if (it != root_item_.get())
        {
            QModelIndex idx = createIndex(it->row(), 0, it);
            emit dataChanged(idx, idx, {Qt::DecorationRole, IconRole});
        }
        for (int i = 0; i < it->childCount(); ++i)
            walk(it->child(i));
    };
    walk(item);

    // ancestors
    for (auto* a = item->parentItem(); a && a != root_item_.get(); a = a->parentItem())
    {
        QModelIndex idx = createIndex(a->row(), 0, a);
        emit dataChanged(idx, idx, {Qt::DecorationRole, IconRole});
    }
}

void ScatterSeriesModel::deselectAll()
{
    traced_assert(root_item_);
    root_item_->hideAll();
    root_item_->hide(false);
}

namespace
{

/// depth-first traversal collecting full paths ("a:b:c:d") for every leaf whose
/// effective hidden state (self or any ancestor hidden) is true.
void collectHiddenLeafPaths(ScatterSeriesTreeItem* item, const std::string& path,
                            std::set<std::string>& out)
{
    for (int i = 0; i < item->childCount(); ++i)
    {
        auto* c = item->child(i);
        const std::string p = path.empty() ? c->name() : (path + ":" + c->name());
        if (c->hasDataSeries())
        {
            if (c->itemHidden())
                out.insert(p);
        }
        else
        {
            collectHiddenLeafPaths(c, p, out);
        }
    }
}

void hideMatchingLeaves(ScatterSeriesTreeItem* item, const std::string& path,
                        const std::set<std::string>& names, bool& changed)
{
    for (int i = 0; i < item->childCount(); ++i)
    {
        auto* c = item->child(i);
        const std::string p = path.empty() ? c->name() : (path + ":" + c->name());
        if (c->hasDataSeries())
        {
            if (names.count(p) && !c->hidden())
            {
                c->hide(true);
                changed = true;
            }
        }
        else
        {
            hideMatchingLeaves(c, p, names, changed);
        }
    }
}

} // anonymous

std::set<std::string> ScatterSeriesModel::hiddenSeriesNames() const
{
    std::set<std::string> names;
    collectHiddenLeafPaths(root_item_.get(), "", names);
    return names;
}

void ScatterSeriesModel::applyHiddenSeriesNames(const std::set<std::string>& names)
{
    loginf << "applying " << names.size() << " hidden leaf paths";

    if (names.empty())
        return;

    bool changed = false;

    // block signals on the model to prevent visibilityChangedSignal from
    // firing per-item inside hide() — it emits on model_, not on the item
    blockSignals(true);

    beginResetModel();
    hideMatchingLeaves(root_item_.get(), "", names, changed);
    endResetModel();

    blockSignals(false);

    loginf << "changed: " << changed;

    if (changed)
        emit visibilityChangedSignal();
}

