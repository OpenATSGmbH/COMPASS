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

#include "task/result/report/treemodel.h"
#include "task/result/report/treeitem.h"
#include "task/result/report/report.h"

#include "logger.h"

namespace ResultReport
{

/**
 */
TreeModel::TreeModel()
:   QAbstractItemModel(nullptr)
{
}

void TreeModel::setReport(const std::shared_ptr<Report>& report)
{
    beginResetModel();

    traced_assert(report);
    report_ = report;

    endResetModel();
}

void TreeModel::clear()
{
    beginResetModel();

    report_ = nullptr;

    endResetModel();
}

/**
 */
int TreeModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return static_cast<TreeItem*>(parent.internalPointer())->columnCount();

    if (!report_)
        return 0;

    return report_->columnCount();
}

/**
 */
QVariant TreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    TreeItem *item = static_cast<TreeItem*>(index.internalPointer());
    traced_assert(item);

    if (role == Qt::DisplayRole)
        return item->data(index.column());
    if (role == Qt::UserRole)
        return item->id().c_str();
    else
        return QVariant();
}

/**
 */
Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    return QAbstractItemModel::flags(index);
}

/**
 */
QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                                int role) const
{
    if (report_ && orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return report_->data(section);

    return QVariant();
}

/**
 */
QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!report_ || !hasIndex(row, column, parent))
        return QModelIndex();

    TreeItem *parentItem;

    if (!parent.isValid())
        parentItem = report_.get();
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    TreeItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    return QModelIndex();
}

/**
 */
QModelIndex TreeModel::parent(const QModelIndex &index) const
{
    if (!report_ || !index.isValid())
        return QModelIndex();

    TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
    TreeItem *parentItem = childItem->parentItem();

    if (parentItem == report_.get())
        return QModelIndex();

    return createIndex(parentItem->row(), 0, parentItem);
}

/**
 */
int TreeModel::rowCount(const QModelIndex &parent) const
{
    TreeItem *parentItem;
    if (!report_ || parent.column() > 0)
        return 0;

    if (!parent.isValid())
        parentItem = report_.get();
    else
        parentItem = static_cast<TreeItem*>(parent.internalPointer());

    return parentItem->childCount();
}

/**
 */
QModelIndex TreeModel::findItem(const std::string& id) const
{
    QModelIndexList items = match(
                index(0, 0),
                Qt::UserRole,
                QVariant(id.c_str()),
                2, // look *
                Qt::MatchRecursive); // look *

    if (items.size() == 1)
    {
        loginf << "id '" << id << "' found";
        return items.at(0);
    }
    else
    {
        loginf << "id '" << id << " found " << items.size() << " matches";
        return QModelIndex(); // none or too many found
    }
}

/**
 */
void TreeModel::beginReset()
{
    beginResetModel();
}

/**
 */
void TreeModel::endReset()
{
    endResetModel();
}

/**
 */
const Report* TreeModel::rootItem() const
{
    //assert (report_);
    return report_.get();
}

}
