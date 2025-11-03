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

#include "eval/results/report/treemodel.h"
#include "eval/results/report/treeitem.h"
#include "eval/results/report/rootitem.h"
#include "logger.h"

namespace EvaluationResultsReport
{
    TreeModel::TreeModel(EvaluationManager& eval_man)
     : QAbstractItemModel(nullptr), eval_man_(eval_man)
    {
        root_item_ = make_shared<RootItem>(eval_man_);
    }

    int TreeModel::columnCount(const QModelIndex &parent) const
    {
        if (parent.isValid())
            return static_cast<TreeItem*>(parent.internalPointer())->columnCount();

        return root_item_->columnCount();
    }

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

    Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
    {
        if (!index.isValid())
            return Qt::NoItemFlags;

        return QAbstractItemModel::flags(index);
    }

    QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                                   int role) const
    {
        if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
            return root_item_->data(section);

        return QVariant();
    }

    QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const
    {
        if (!hasIndex(row, column, parent))
            return QModelIndex();

        TreeItem *parentItem;

        if (!parent.isValid())
            parentItem = root_item_.get();
        else
            parentItem = static_cast<TreeItem*>(parent.internalPointer());

        TreeItem *childItem = parentItem->child(row);
        if (childItem)
            return createIndex(row, column, childItem);
        return QModelIndex();
    }

    QModelIndex TreeModel::parent(const QModelIndex &index) const
    {
        if (!index.isValid())
            return QModelIndex();

        TreeItem *childItem = static_cast<TreeItem*>(index.internalPointer());
        TreeItem *parentItem = childItem->parentItem();

        if (parentItem == root_item_.get())
            return QModelIndex();

        return createIndex(parentItem->row(), 0, parentItem);
    }

    int TreeModel::rowCount(const QModelIndex &parent) const
    {
        TreeItem *parentItem;
        if (parent.column() > 0)
            return 0;

        if (!parent.isValid())
            parentItem = root_item_.get();
        else
            parentItem = static_cast<TreeItem*>(parent.internalPointer());

        return parentItem->childCount();
    }

    QModelIndex TreeModel::findItem (const string& id)
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

    void TreeModel::beginReset()
    {
        beginResetModel();
    }

    void TreeModel::clear()
    {
        loginf;

        root_item_ = make_shared<RootItem>(eval_man_);
    }

    void TreeModel::endReset()
    {
        endResetModel();
    }

    shared_ptr<RootItem> TreeModel::rootItem() const
    {
        return root_item_;
    }
}
