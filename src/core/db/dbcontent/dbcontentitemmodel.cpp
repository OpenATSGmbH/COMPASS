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
#include "sortedlistdiff.h"

#include "logger.h"
#include "traced_assert.h"

#include <algorithm>

namespace
{
    const char* StaleTooltip = "Data is being reloaded"; // as the layer tree's stale rows
}

#include <QApplication>
#include <QMenu>

/**
 */
DBContentItemModel::DBContentItemModel(DBContentItemProvider& provider, QObject* parent)
    : QAbstractListModel(parent)
    , provider_(provider)
{
    // no clearing on a provider reset: the rows stay readable (stale) until the refresh updates them in
    // place. Both queued, the stale mark only while the refresh is outstanding (see dataAboutToBeResetSlot())
    connect(&provider_, &DBContentItemProvider::dataAboutToBeResetSignal,
            this, &DBContentItemModel::dataAboutToBeResetSlot,
            Qt::QueuedConnection);

    connect(&provider_, &DBContentItemProvider::dataRefreshedSignal,
            this, &DBContentItemModel::dataRefreshedSlot,
            Qt::QueuedConnection);

    connect(&provider_, &DBContentItemProvider::itemVisibilityChangedSignal,
            this, &DBContentItemModel::itemVisibilityChangedSlot);

    connect(&provider_, &DBContentItemProvider::showItemColorsChangedSignal,
            this, &DBContentItemModel::showItemColorsChangedSlot);

    connect(&provider_, &DBContentItemProvider::itemStatesClearedSignal,
            this, &DBContentItemModel::itemStatesClearedSlot);

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

    if (stale_ && role == Qt::ForegroundRole)
        return QApplication::palette().color(QPalette::Disabled, QPalette::Text);

    if (stale_ && role == Qt::ToolTipRole)
        return QString(StaleTooltip);

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
 * The provider resets its data => the rows are stale until the next refresh.
 * Notice: Queued => only if the refresh is still outstanding: a live tick already refreshed in the same turn,
 * a database close emptied the list in between.
 */
void DBContentItemModel::dataAboutToBeResetSlot()
{
    if (provider_.refreshPending())
        setStale(true);
}

/**
 */
void DBContentItemModel::dataRefreshedSlot()
{
    update();
}

/**
 * Notice: Repaints all rows (greyed text, tooltip).
 */
void DBContentItemModel::setStale(bool stale)
{
    if (stale_ == stale)
        return;

    stale_ = stale;

    if (!item_ids_.empty())
        emit dataChanged(index(0, 0),
                         index((int)item_ids_.size() - 1, columnCount() - 1),
                         { Qt::ForegroundRole, Qt::ToolTipRole });

    emit staleChangedSignal();
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
 * The provider forgot its item states (grouping change, database close) => the list is emptied, the next
 * refresh builds it from scratch.
 */
void DBContentItemModel::itemStatesClearedSlot()
{
    beginResetModel();
    item_ids_.clear();
    endResetModel();

    setStale(false);
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
 * Collects all item ids of the provider, sorted by the strict key (sort value, id): different ids may
 * share a sort value (e.g. ACIDs once spaces are removed). The sort values of the given extra ids
 * (e.g. the current rows) are added to sort_values as well.
 */
std::vector<nlohmann::json> DBContentItemModel::sortedItemIds(SortValues& sort_values,
                                                              const std::vector<nlohmann::json>& extra_ids) const
{
    std::vector<nlohmann::json> item_ids;
    item_ids.reserve(provider_.itemLocations().size());

    for (const auto& [ item_id, locations ] : provider_.itemLocations())
    {
        item_ids.push_back(item_id);
        sort_values[ item_id ] = provider_.itemSortValue(item_id);
    }

    for (const auto& item_id : extra_ids)
    {
        if (!sort_values.count(item_id))
            sort_values[ item_id ] = provider_.itemSortValue(item_id);
    }

    std::sort(item_ids.begin(), item_ids.end(), lessThan(sort_values));

    return item_ids;
}

/**
 */
std::function<bool(const nlohmann::json&, const nlohmann::json&)> DBContentItemModel::lessThan(const SortValues& sort_values)
{
    return [ &sort_values ] (const nlohmann::json& a, const nlohmann::json& b)
    {
        const auto& sa = sort_values.at(a);
        const auto& sb = sort_values.at(b);

        if (sa != sb)
            return sa < sb;

        return a < b;
    };
}

/**
 * Collects + sorts all item ids of the provider and resets the model (initial build, grouping change).
 * Sets a wait cursor itself: the triggering signals are queued (see ctor), so this
 * runs in a later event loop turn, outside of any cursor the original caller (e.g.
 * the grouping combo) might have set around its own work.
 */
void DBContentItemModel::rebuild()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    beginResetModel();

    SortValues sort_values;
    item_ids_ = sortedItemIds(sort_values, {});

    endResetModel();

    setStale(false);

    QApplication::restoreOverrideCursor();
}

/**
 * Updates the rows in place from the provider's items: vanished rows are removed, new ones inserted,
 * kept rows stay (the view keeps their selection, current row and scroll position) and are refreshed.
 * An empty list (initial build, grouping change) is built as a full reset.
 */
void DBContentItemModel::update()
{
    if (item_ids_.empty())
    {
        // nothing to list (e.g. an empty live data window) => no full reset and no wait cursor per tick
        if (provider_.itemLocations().empty())
            setStale(false);
        else
            rebuild();

        return;
    }

    SortValues sort_values;
    const auto item_ids = sortedItemIds(sort_values, item_ids_);

    const auto ops = dbContent::diffSortedLists(item_ids_, item_ids, lessThan(sort_values));

    for (const auto& op : ops)
    {
        const int first = (int)op.row;
        const int last  = (int)(op.row + op.count) - 1;

        if (op.insert)
        {
            beginInsertRows(QModelIndex(), first, last);
            item_ids_.insert(item_ids_.begin() + op.row,
                             item_ids.begin() + op.target_begin,
                             item_ids.begin() + op.target_begin + op.count);
            endInsertRows();
        }
        else
        {
            beginRemoveRows(QModelIndex(), first, last);
            item_ids_.erase(item_ids_.begin() + op.row, item_ids_.begin() + op.row + op.count);
            endRemoveRows();
        }
    }

    traced_assert(item_ids_ == item_ids);

    // check state, color and the identification column of kept rows may have changed
    if (!item_ids_.empty())
        emit dataChanged(index(0, 0), index((int)item_ids_.size() - 1, columnCount() - 1));

    setStale(false);
}
