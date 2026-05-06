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

#include "db_context_edit_tree_model.h"
#include "db_context_edit_tree_item.h"
#include "db_context_manager.h"
#include "sector.h"
#include "sectorlayer.h"

#include <map>

namespace context
{

DBContextEditTreeModel::DBContextEditTreeModel(DBContextManager& manager, QObject* parent)
    : QAbstractItemModel(parent), manager_(manager)
{
    buildTree();
}

DBContextEditTreeModel::~DBContextEditTreeModel() = default;

void DBContextEditTreeModel::rebuild()
{
    beginResetModel();
    buildTree();
    endResetModel();
}

void DBContextEditTreeModel::buildTree()
{
    root_ = std::make_unique<RootItem>();

    if (!manager_.hasActiveContext())
        return;

    auto& ctx = manager_.activeContext();

    // Data Sources group, sub-grouped by dstype
    auto ds_group = std::make_unique<GroupItem>(GroupItem::DataSources, root_.get());
    {
        // collect unique ds types in order
        std::map<std::string, std::vector<const DataSource*>> by_type;
        for (const auto& [ds_id, ds] : ctx.dataSources())
            by_type[ds.dsType()].push_back(&ds);

        for (const auto& [type, sources] : by_type)
        {
            auto type_item = std::make_unique<DSTypeGroupItem>(type, ds_group.get());
            for (const auto* ds : sources)
            {
                type_item->appendChild(std::make_unique<DataSourceItem>(
                    ds->id(), ds->name(), ds->sac(), ds->sic(), type_item.get()));
            }
            ds_group->appendChild(std::move(type_item));
        }
    }
    root_->appendChild(std::move(ds_group));

    // ASTERIX Configuration (single leaf)
    root_->appendChild(std::make_unique<ASTERIXConfigLeafItem>(root_.get()));

    // Sector Layers group
    auto sl_group = std::make_unique<GroupItem>(GroupItem::SectorLayers, root_.get());
    for (const auto& layer : manager_.sectorLayers())
    {
        auto layer_item = std::make_unique<SectorLayerItem>(layer->name(), sl_group.get());
        for (const auto& sector : layer->sectors())
        {
            layer_item->appendChild(std::make_unique<SectorItem>(
                sector->id(), sector->name(), layer_item.get()));
        }
        sl_group->appendChild(std::move(layer_item));
    }
    root_->appendChild(std::move(sl_group));

    // FFTs group
    auto fft_group = std::make_unique<GroupItem>(GroupItem::FFTs, root_.get());
    for (const auto& fft : ctx.ffts())
    {
        fft_group->appendChild(std::make_unique<FFTItem>(fft.name(), fft_group.get()));
    }
    root_->appendChild(std::move(fft_group));

    // Colors group — leaf, no sub-items (the ColorsEditWidget covers all
    // sub-sections inline with its own group boxes)
    root_->appendChild(std::make_unique<GroupItem>(GroupItem::Colors, root_.get()));
}

QVariant DBContextEditTreeModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || role != Qt::DisplayRole)
        return {};

    auto* item = static_cast<DBContextEditTreeItem*>(index.internalPointer());
    return item->data(index.column());
}

Qt::ItemFlags DBContextEditTreeModel::flags(const QModelIndex& index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;

    return QAbstractItemModel::flags(index);
}

QVariant DBContextEditTreeModel::headerData(int /*section*/, Qt::Orientation orientation,
                                            int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return "Context Items";

    return {};
}

QModelIndex DBContextEditTreeModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return {};

    DBContextEditTreeItem* parent_item;

    if (!parent.isValid())
        parent_item = root_.get();
    else
        parent_item = static_cast<DBContextEditTreeItem*>(parent.internalPointer());

    auto* child_item = parent_item->child(row);
    if (child_item)
        return createIndex(row, column, child_item);

    return {};
}

QModelIndex DBContextEditTreeModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return {};

    auto* child_item = static_cast<DBContextEditTreeItem*>(index.internalPointer());
    auto* parent_item = child_item->parentItem();

    if (parent_item == root_.get())
        return {};

    return createIndex(parent_item->row(), 0, parent_item);
}

int DBContextEditTreeModel::rowCount(const QModelIndex& parent) const
{
    if (parent.column() > 0)
        return 0;

    DBContextEditTreeItem* parent_item;

    if (!parent.isValid())
        parent_item = root_.get();
    else
        parent_item = static_cast<DBContextEditTreeItem*>(parent.internalPointer());

    return parent_item->childCount();
}

int DBContextEditTreeModel::columnCount(const QModelIndex& /*parent*/) const
{
    return 1;
}

} // namespace context
