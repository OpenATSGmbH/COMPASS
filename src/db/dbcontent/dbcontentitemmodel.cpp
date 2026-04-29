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

#include "dbcontentitemmodel.h"

#include <QMenu>

/**
 */
DBContentItemModel::DBContentItemModel(DBContentItemProvider& provider, QObject* parent)
    : QAbstractListModel(parent)
    , provider_(provider)
{
    connect(&provider_, &DBContentItemProvider::dataResetSignal,
            this, &DBContentItemModel::dataResetSlot,
            Qt::QueuedConnection);

    connect(&provider_, &DBContentItemProvider::dataRefreshedSignal,
            this, &DBContentItemModel::dataRefreshedSlot,
            Qt::QueuedConnection);

    rebuild();
}

// --- QAbstractListModel interface ---

/**
 */
int DBContentItemModel::rowCount(const QModelIndex& parent) const
{
    if (parent.isValid())
        return 0;
    return static_cast<int>(item_ids_.size());
}

/**
 */
QVariant DBContentItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole && section == 0)
        return tr("Item");
    return {};
}

/**
 */
QVariant DBContentItemModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= (int)item_ids_.size())
        return {};

    const auto& item_id = item_ids_.at(index.row());

    if (role == Qt::DisplayRole)
        return itemName(item_id);

    if (role == Qt::CheckStateRole)
    {
        auto it = visibility_.find(item_id);
        bool visible = (it != visibility_.end()) ? it->second : true;
        return visible ? Qt::Checked : Qt::Unchecked;
    }

    return {};
}

/**
 */
bool DBContentItemModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (!index.isValid() || index.row() < 0 || index.row() >= (int)item_ids_.size())
        return false;

    if (role == Qt::CheckStateRole)
    {
        const auto& item_id = item_ids_.at(index.row());
        bool visible = (value.toInt() == Qt::Checked);
        visibility_[item_id] = visible;
        emit dataChanged(index, index, {Qt::CheckStateRole});
        setItemVisible_impl(item_id, visible);
        return true;
    }

    return false;
}

/**
 */
Qt::ItemFlags DBContentItemModel::flags(const QModelIndex& index) const
{
    if (!index.isValid())
        return Qt::NoItemFlags;
    return Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
}

// --- Public API ---

/**
 */
QString DBContentItemModel::itemName(const nlohmann::json& item_id) const
{
    return QString::fromStdString(provider_.itemName(item_id));
}

/**
 */
bool DBContentItemModel::isVisible(const nlohmann::json& item_id) const
{
    auto it = visibility_.find(item_id);
    return (it != visibility_.end()) ? it->second : true;
}

/**
 */
void DBContentItemModel::setVisible(const nlohmann::json& item_id, bool visible)
{
    auto entry = visibility_.find(item_id);
    if (entry == visibility_.end())
        return;

    entry->second = visible;

    auto it = std::find(item_ids_.begin(), item_ids_.end(), item_id);
    if (it != item_ids_.end())
    {
        int row = static_cast<int>(std::distance(item_ids_.begin(), it));
        QModelIndex idx = createIndex(row, 0);
        emit dataChanged(idx, idx, {Qt::CheckStateRole});
    }

    setItemVisible_impl(item_id, visible);
}

/**
 */
void DBContentItemModel::setAllVisible(bool visible)
{
    for (auto& [id, vis] : visibility_)
        vis = visible;

    if (!item_ids_.empty())
    {
        emit dataChanged(createIndex(0, 0),
                         createIndex(static_cast<int>(item_ids_.size()) - 1, 0),
                         {Qt::CheckStateRole});
    }

    setAllItemsVisible_impl(visible);
}

/**
 */
void DBContentItemModel::setSiblingsVisible(const nlohmann::json& item_id, bool visible)
{
    for (auto& [id, vis] : visibility_)
        if (id != item_id)
            vis = visible;

    if (!item_ids_.empty())
    {
        emit dataChanged(createIndex(0, 0),
                         createIndex(static_cast<int>(item_ids_.size()) - 1, 0),
                         {Qt::CheckStateRole});
    }

    std::vector<nlohmann::json> items;
    items.reserve(provider_.itemLocations().size());

    for (const auto& loc : provider_.itemLocations())
        if (loc.first != item_id)
            items.push_back(loc.first);

    setItemsVisible_impl(items, visible);
}

/**
 */
void DBContentItemModel::showContextMenu(const QModelIndex& index, 
                                         const QPoint& global_pos,
                                         QWidget* parent)
{
    QMenu menu(parent);

    if (index.isValid() && index.row() < (int)item_ids_.size())
    {
        const auto& item_id = item_ids_.at(index.row());
        auto item_name = itemName(item_id);

        auto show_item = menu.addAction("Show " + item_name);
        auto hide_item = menu.addAction("Hide " + item_name);

        connect(show_item, &QAction::triggered, [ this, item_id ] () { this->setVisible(item_id, true); });
        connect(hide_item, &QAction::triggered, [ this, item_id ] () { this->setVisible(item_id, false); });
    }

    auto show_all = menu.addAction("Show All");
    auto hide_all = menu.addAction("Hide All");

    connect(show_all, &QAction::triggered, [ this ] () { this->setAllVisible(true); });
    connect(hide_all, &QAction::triggered, [ this ] () { this->setAllVisible(false); });

    if (index.isValid() && index.row() < (int)item_ids_.size())
    {
        const auto& item_id = item_ids_.at(index.row());

        if (item_ids_.size() >= 2)
        {
            auto show_others = menu.addAction("Show Others");
            auto hide_others = menu.addAction("Hide Others");

            connect(show_others, &QAction::triggered, [ this, item_id ] () { this->setSiblingsVisible(item_id, true); });
            connect(hide_others, &QAction::triggered, [ this, item_id ] () { this->setSiblingsVisible(item_id, false); });
        }

        menu.addSeparator();

        fillItemContextMenu(menu, item_id);
    }

    menu.exec(global_pos);
}

// --- Public slots ---

/**
 */
void DBContentItemModel::itemDoubleClickedSlot(const QModelIndex& index)
{
    if (!index.isValid() || index.row() < 0 || index.row() >= (int)item_ids_.size())
        return;

    onItemDoubleClicked(item_ids_.at(index.row()));
}

// --- Private slots ---

/**
 */
void DBContentItemModel::dataResetSlot()
{
    beginResetModel();
    item_ids_.clear();
    visibility_.clear();
    endResetModel();
}

/**
 */
void DBContentItemModel::dataRefreshedSlot()
{
    rebuild();
}

// --- Private ---

/**
 */
void DBContentItemModel::rebuild()
{
    // Snapshot existing visibility so we can preserve it for items that remain.
    std::map<nlohmann::json, bool> snapshot = std::move(visibility_);

    beginResetModel();

    item_ids_.clear();
    visibility_.clear();

    std::map<nlohmann::json, nlohmann::json> name_cache;

    for (const auto& [item_id, locations] : provider_.itemLocations())
    {
        item_ids_.push_back(item_id);

        auto it = snapshot.find(item_id);
        visibility_[item_id] = (it != snapshot.end()) ? it->second : true;

        name_cache[item_id] = provider_.itemSortValue(item_id);
    }

    std::sort(item_ids_.begin(), item_ids_.end(),
              [&name_cache](const nlohmann::json& a, const nlohmann::json& b)
              { return name_cache.at(a) < name_cache.at(b); });

    endResetModel();
}
