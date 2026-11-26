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

#include "catch.hpp"
#include "sortedlistdiff.h"

#include "json.hpp"

#include <string>
#include <utility>
#include <vector>

#include <QAbstractListModel>
#include <QItemSelectionModel>
#include <QSignalSpy>
#include <QTreeView>

namespace
{

/**
 * Applies the ops to a copy of the current list.
 */
template<typename T>
std::vector<T> applyOps(std::vector<T>                                current,
                        const std::vector<T>&                         target,
                        const std::vector<dbContent::SortedListOp>&   ops)
{
    for (const auto& op : ops)
    {
        if (op.insert)
            current.insert(current.begin() + op.row,
                           target.begin() + op.target_begin,
                           target.begin() + op.target_begin + op.count);
        else
            current.erase(current.begin() + op.row, current.begin() + op.row + op.count);
    }

    return current;
}

auto intLess = [] (int a, int b) { return a < b; };

/**
 * List model updated in place as DBContentItemModel::update() does.
 */
class IntListModel : public QAbstractListModel
{
public:
    explicit IntListModel(const std::vector<int>& values) : values_(values) {}

    int rowCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : (int)values_.size();
    }

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override
    {
        if (!index.isValid() || role != Qt::DisplayRole)
            return {};

        return values_.at(index.row());
    }

    void updateTo(const std::vector<int>& target)
    {
        for (const auto& op : dbContent::diffSortedLists(values_, target, intLess))
        {
            const int first = (int)op.row;
            const int last  = (int)(op.row + op.count) - 1;

            if (op.insert)
            {
                beginInsertRows(QModelIndex(), first, last);
                values_.insert(values_.begin() + op.row,
                               target.begin() + op.target_begin,
                               target.begin() + op.target_begin + op.count);
                endInsertRows();
            }
            else
            {
                beginRemoveRows(QModelIndex(), first, last);
                values_.erase(values_.begin() + op.row, values_.begin() + op.row + op.count);
                endRemoveRows();
            }
        }
    }

    int value(int row) const { return values_.at(row); }

private:
    std::vector<int> values_;
};

} // namespace

TEST_CASE("Sorted list diff: keep, remove and add in contiguous ranges", "[sortedlistdiff]")
{
    const std::vector<int> current = { 1, 2, 3, 5, 8, 9 };
    const std::vector<int> target  = { 0, 2, 3, 4, 6, 7, 9, 10 };

    const auto ops = dbContent::diffSortedLists(current, target, intLess);

    REQUIRE(applyOps(current, target, ops) == target);

    // 0 in, 1 out, 4 in, 5 out, 6 7 in, 8 out, 10 in => one op per run
    REQUIRE(ops.size() == 7);

    REQUIRE(ops[ 0 ].insert);
    REQUIRE(ops[ 0 ].row   == 0);
    REQUIRE(ops[ 0 ].count == 1);

    REQUIRE(!ops[ 1 ].insert);
    REQUIRE(ops[ 1 ].row   == 1);
    REQUIRE(ops[ 1 ].count == 1);

    REQUIRE(ops[ 4 ].insert);
    REQUIRE(ops[ 4 ].count        == 2);
    REQUIRE(ops[ 4 ].target_begin == 4);
}

TEST_CASE("Sorted list diff: identical, empty and disjoint lists", "[sortedlistdiff]")
{
    const std::vector<int> values = { 1, 2, 3 };
    const std::vector<int> empty;

    SECTION("identical => no ops")
    {
        REQUIRE(dbContent::diffSortedLists(values, values, intLess).empty());
    }

    SECTION("from empty => one insert")
    {
        const auto ops = dbContent::diffSortedLists(empty, values, intLess);

        REQUIRE(ops.size() == 1);
        REQUIRE(ops[ 0 ].insert);
        REQUIRE(ops[ 0 ].count == 3);
        REQUIRE(applyOps(empty, values, ops) == values);
    }

    SECTION("to empty => one remove")
    {
        const auto ops = dbContent::diffSortedLists(values, empty, intLess);

        REQUIRE(ops.size() == 1);
        REQUIRE(!ops[ 0 ].insert);
        REQUIRE(ops[ 0 ].count == 3);
        REQUIRE(applyOps(values, empty, ops).empty());
    }

    SECTION("disjoint => everything replaced")
    {
        const std::vector<int> other = { 4, 5 };

        const auto ops = dbContent::diffSortedLists(values, other, intLess);

        REQUIRE(ops.size() == 2);
        REQUIRE(applyOps(values, other, ops) == other);
    }
}

TEST_CASE("Sorted list diff: equal sort values with different ids", "[sortedlistdiff]")
{
    // (sort value, id) as the item list sorts: ACIDs "AB 1" and "AB1" share the sort value "AB1"
    typedef std::pair<std::string, nlohmann::json> Key;

    auto keyLess = [] (const Key& a, const Key& b)
    {
        if (a.first != b.first)
            return a.first < b.first;

        return a.second < b.second;
    };

    const std::vector<Key> current = { { "AB1", "AB 1" }, { "AB1", "AB1" }, { "CD2", "CD2" } };
    const std::vector<Key> target  = { { "AB1", "AB1" }, { "CD2", "CD2" }, { "EF3", "EF3" } };

    const auto ops = dbContent::diffSortedLists(current, target, keyLess);

    REQUIRE(applyOps(current, target, ops) == target);
    REQUIRE(ops.size() == 2); // "AB 1" out, "EF3" in
}

TEST_CASE("Sorted list diff: a view keeps the selection and current row of kept rows", "[sortedlistdiff]")
{
    IntListModel model({ 1, 2, 3, 4, 5, 6 });

    QTreeView view;
    view.setModel(&model);

    auto* selection = view.selectionModel();
    REQUIRE(selection);

    // select 2, 4 and 5, current 4
    selection->select(model.index(1, 0), QItemSelectionModel::Select);
    selection->select(model.index(3, 0), QItemSelectionModel::Select);
    selection->select(model.index(4, 0), QItemSelectionModel::Select);
    selection->setCurrentIndex(model.index(3, 0), QItemSelectionModel::NoUpdate);

    QSignalSpy reset_spy(&model, &QAbstractItemModel::modelReset);

    // 1 and 5 vanish, 0 and 7 arrive
    model.updateTo({ 0, 2, 3, 4, 6, 7 });

    REQUIRE(reset_spy.count() == 0);

    std::vector<int> selected;
    for (const auto& index : selection->selectedRows())
        selected.push_back(model.value(index.row()));

    std::sort(selected.begin(), selected.end());

    REQUIRE(selected == std::vector<int>{ 2, 4 }); // 5 removed, 2 and 4 kept
    REQUIRE(selection->currentIndex().isValid());
    REQUIRE(model.value(selection->currentIndex().row()) == 4);
}
