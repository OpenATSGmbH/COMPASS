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

#pragma once

#include "dbcontentitem.h"

#include <optional>

#include <QWidget>

class DBContentItemModel;
class QComboBox;
class QTreeView;

/**
 * Widget combining a grouping selector and a flat item list for a DBContentItemModel.
 *
 * Layout (top → bottom):
 *   [ QComboBox  — grouping mode selector ]
 *   [ QTreeView  — flat item list with checkbox + context menu ]
 *
 * The combo is pre-populated with all DBContentItemProvider grouping modes and
 * wired to call provider().setGrouping() on change. The tree view is wired
 * for context menus and double-click automatically.
 */
class DBContentItemView : public QWidget
{
    Q_OBJECT

public:
    explicit DBContentItemView(DBContentItemModel& model, QWidget* parent = nullptr);
    virtual ~DBContentItemView() = default;

    QTreeView* treeView() const { return tree_view_; }

    dbContent::Grouping currentGrouping() const;

    void setActiveGroupings(unsigned int flags);
    bool isGroupingActive(dbContent::Grouping grouping) const;

private slots:
    void groupingChangedSlot(const QString& text);

private:
    void updateGrouping();

    DBContentItemModel& model_;

    QComboBox* grouping_box_ {nullptr};
    QTreeView* tree_view_    {nullptr};

    unsigned int active_groupings_ = std::numeric_limits<unsigned int>::max();
};
