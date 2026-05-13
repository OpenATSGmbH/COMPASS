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

#include "logger.h"

#include <QMenu>

/**
 */
DBContentItemModel::DBContentItemModel(DBContentItemProvider& provider, QObject* parent)
    : QAbstractListModel(parent)
    , provider_(provider)
{
    connect(&provider_, &DBContentItemProvider::dataAboutToBeResetSignal,
            this, &DBContentItemModel::dataResetSlot,
            Qt::QueuedConnection);

    connect(&provider_, &DBContentItemProvider::dataRefreshedSignal,
            this, &DBContentItemModel::dataRefreshedSlot,
            Qt::QueuedConnection);

    connect(&provider_, &DBContentItemProvider::itemVisibilityChangedSignal,
            this, &DBContentItemModel::itemVisibilityChangedSlot);

    connect(&provider_, &DBContentItemProvider::showItemColorsChangedSignal,
            this, &DBContentItemModel::showItemColorsChangedSlot);

    connect(&provider_, &DBContentItemProvider::groupingChangedSignal,
            this, &DBContentItemModel::groupingChangedSlot);

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
int DBContentItemModel::columnCount(const QModelIndex& parent) const
{
    if (parent.isValid())
        return 0;
    return provider_.grouping() == DBContentItemProvider::Grouping::UTN ? 2 : 1;
}

/**
 */
QVariant DBContentItemModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        if (section == 0) return tr("Item");
        if (section == 1) return tr("Best Available Identification");
    }
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
    {
        if (index.column() == 0)
            return itemName(item_id);
        if (index.column() == 1)
            return QString::fromStdString(provider_.itemBestAvailableIdentification(item_id));
        return {};
    }

    if (index.column() != 0)
        return {};

    if (role == Qt::CheckStateRole)
        return provider_.itemVisible(item_id) ? Qt::Checked : Qt::Unchecked;

    if (role == Qt::DecorationRole && provider_.showItemColors())
    {
        QColor c = provider_.itemColor(item_id);
        if (c.isValid())
            return c;
    }

    return {};
}

/**
 */
bool DBContentItemModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    if (!index.isValid() || index.row() < 0 || index.row() >= (int)item_ids_.size())
        return false;

    if (index.column() != 0)
        return false;

    if (role == Qt::CheckStateRole)
    {
        const auto& item_id = item_ids_.at(index.row());
        bool visible = (value.toInt() == Qt::Checked);
        provider_.setItemVisible(item_id, visible);
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
    if (index.column() == 0)
        return Qt::ItemIsEnabled | Qt::ItemIsUserCheckable;
    return Qt::ItemIsEnabled;
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
    return provider_.itemVisible(item_id);
}

/**
 */
void DBContentItemModel::setVisible(const nlohmann::json& item_id, bool visible)
{
    provider_.setItemVisible(item_id, visible);
}

/**
 */
void DBContentItemModel::setAllVisible(bool visible)
{
    provider_.setAllItemsVisible(visible);
}

/**
 */
void DBContentItemModel::setSiblingsVisible(const nlohmann::json& item_id, bool visible)
{
    provider_.setSiblingItemsVisible(item_id, visible);
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
        auto item_name = provider_.groupingAsString() + " " + provider_.itemName(item_id);

        auto show_item = menu.addAction("Show " + QString::fromStdString(item_name));
        auto hide_item = menu.addAction("Hide " + QString::fromStdString(item_name));

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
    endResetModel();
}

/**
 */
void DBContentItemModel::dataRefreshedSlot()
{
    rebuild();
}

/**
 * The provider mutated its visibility cache (either directly via setItemVisible
 * from this model, or indirectly via subclass-side scope mutations); refresh
 * the entire CheckStateRole column.
 */
void DBContentItemModel::itemVisibilityChangedSlot()
{
    if (item_ids_.empty())
        return;

    emit dataChanged(createIndex(0, 0),
                     createIndex(static_cast<int>(item_ids_.size()) - 1, 0),
                     {Qt::CheckStateRole});
}

/**
 */
void DBContentItemModel::showItemColorsChangedSlot()
{
    if (item_ids_.empty())
        return;

    emit dataChanged(createIndex(0, 0),
                     createIndex(static_cast<int>(item_ids_.size()) - 1, 0),
                     {Qt::DecorationRole});
}

/**
 */
void DBContentItemModel::groupingChangedSlot()
{
    // item_ids_ still holds entries from the previous grouping; clear them so
    // the view does not query itemName() with an old-shaped item_id against
    // the new grouping_ (the queued dataRefreshedSlot will rebuild later).
    beginResetModel();
    item_ids_.clear();
    endResetModel();
}

// --- Private ---

/**
 */
void DBContentItemModel::rebuild()
{
    beginResetModel();

    item_ids_.clear();

    std::map<nlohmann::json, nlohmann::json> name_cache;

    for (const auto& [item_id, locations] : provider_.itemLocations())
    {
        item_ids_.push_back(item_id);
        name_cache[item_id] = provider_.itemSortValue(item_id);
    }

    std::sort(item_ids_.begin(), item_ids_.end(),
              [&name_cache](const nlohmann::json& a, const nlohmann::json& b)
              { return name_cache.at(a) < name_cache.at(b); });

    endResetModel();
}
