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

using namespace std;

ScatterSeriesModel::ScatterSeriesModel()
    : QAbstractItemModel(nullptr)
{
    root_item_.reset(new ScatterSeriesTreeItem("Data", *this));
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
        logdbg << "returning root data";
        return root_item_->data(section);
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

void ScatterSeriesModel::updateFrom (ScatterSeriesCollection& collection)
{
    beginResetModel();

    root_item_->clear();

    for (auto& col_it : collection.dataSeries())
    {
        auto* item = new ScatterSeriesTreeItem(col_it.first, *this, &col_it.second, root_item_.get()); // to be added

        root_item_->appendChild(item);
    }

    endResetModel();
}

void ScatterSeriesModel::deselectAll()
{
    traced_assert(root_item_);
    root_item_->hideAll();
    root_item_->hide(false);
}
