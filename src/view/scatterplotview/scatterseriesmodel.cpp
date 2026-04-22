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
#include "color_provider.h"
#include "data_source.h"
#include "db_context.h"
#include "db_context_manager.h"

#include <vector>

using namespace std;

ScatterSeriesModel::ScatterSeriesModel()
    : QAbstractItemModel(nullptr)
{
    root_item_.reset(new ScatterSeriesTreeItem("Data", QColor(), *this, nullptr));
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

QColor dsTypeIconColor(const std::string& ds_type, COMPASS& compass)
{
    if (compass.dbContextManager().hasActiveContext())
    {
        const auto& palette = compass.dbContextManager().activeContext().colors().ds_type_colors;
        auto it = palette.find(ds_type);
        if (it != palette.end() && it->second.isValid())
            return it->second;
    }
    return context::ColorProvider::defaultDSTypeColor(ds_type);
}

QColor dbContentIconColor(const std::string& dbcontent, COMPASS& compass)
{
    if (compass.dbContextManager().hasActiveContext())
    {
        const auto& palette = compass.dbContextManager().activeContext().colors().dbcontent_colors;
        auto it = palette.find(dbcontent);
        if (it != palette.end() && it->second.isValid())
            return it->second;
    }
    return context::ColorProvider::defaultDBContentColor(dbcontent);
}

const context::DataSource* lookupDataSource(const std::string& ds_name, COMPASS& compass)
{
    auto& ctx_mgr = compass.dbContextManager();
    if (!ctx_mgr.hasActiveContext() || !ctx_mgr.hasDataSource(ds_name))
        return nullptr;
    return ctx_mgr.dataSource(ctx_mgr.getDataSourceId(ds_name));
}

QColor dsIconColor(const context::DataSource* ds, const std::string& ds_name)
{
    if (ds && ds->baseColor().isValid())
        return ds->baseColor();
    return QColor(QString::fromStdString(std::string())); // invalid -> empty icon slot
    (void)ds_name;
}

QColor lineIconColor(const context::DataSource* ds, unsigned int line_id)
{
    if (ds)
    {
        QColor c = ds->lineColor(line_id);
        if (c.isValid())
            return c;
        if (ds->baseColor().isValid())
        {
            auto shades = context::ColorProvider::autoLineColors(ds->baseColor());
            return shades[line_id];
        }
    }
    return QColor(); // invalid -> empty icon slot
}

int lineIndexFromToken(const std::string& line_tok)
{
    if (line_tok.size() >= 2 && line_tok[0] == 'L')
    {
        try { return std::stoi(line_tok.substr(1)) - 1; } catch (...) {}
    }
    return 0;
}

ScatterSeriesTreeItem* findOrCreateGroup(ScatterSeriesTreeItem* parent,
                                        const std::string& name,
                                        const QColor& color,
                                        ScatterSeriesModel& model)
{
    for (int i = 0; i < parent->childCount(); ++i)
    {
        auto* c = parent->child(i);
        if (c && c->name() == name)
            return c;
    }
    auto* item = new ScatterSeriesTreeItem(name, color, model, parent);
    parent->appendChild(item);
    return item;
}

} // anonymous

void ScatterSeriesModel::updateFrom(ScatterSeriesCollection& collection, COMPASS& compass)
{
    beginResetModel();

    root_item_->clear();

    for (auto& col_it : collection.dataSeries())
    {
        std::string ds_type, ds_name, line_tok, dbcontent;
        if (!splitSeriesKey(col_it.first, ds_type, ds_name, line_tok, dbcontent))
        {
            // malformed key — drop flat under root so it's still visible
            auto* leaf = new ScatterSeriesTreeItem(col_it.first, col_it.second.color,
                                                   *this, &col_it.second, root_item_.get());
            root_item_->appendChild(leaf);
            continue;
        }

        const int line_index   = lineIndexFromToken(line_tok);
        const unsigned int lid = (line_index >= 0 && line_index < 4) ? (unsigned int)line_index : 0;
        const context::DataSource* ds = lookupDataSource(ds_name, compass);

        auto* ds_type_item = findOrCreateGroup(root_item_.get(), ds_type,
                                               dsTypeIconColor(ds_type, compass), *this);
        auto* ds_item      = findOrCreateGroup(ds_type_item, ds_name,
                                               dsIconColor(ds, ds_name), *this);
        auto* line_item    = findOrCreateGroup(ds_item, line_tok,
                                               lineIconColor(ds, lid), *this);

        const QColor leaf_color = dbContentIconColor(dbcontent, compass);
        auto* leaf = new ScatterSeriesTreeItem(dbcontent, leaf_color,
                                               *this, &col_it.second, line_item);
        line_item->appendChild(leaf);
    }

    endResetModel();
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

