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
class QLineEdit;
class QSortFilterProxyModel;
class QToolButton;
class QTreeView;
class WaitSpinnerWidget;

/**
 * Widget combining a grouping selector and a flat item list for a DBContentItemModel.
 *
 * Layout (top → bottom):
 *   [ QComboBox  - grouping mode selector ] [ filter button ] [ wait icon while the list is stale ]
 *   [ QLineEdit  - filter, lists only matching items (* and ? as wildcards), toggled by the filter
 *                  button, opened by Ctrl+F (focus inside the widget, text selected), closed by
 *                  Escape; closing clears the filter ]
 *   [ QTreeView  - flat item list with checkbox + context menu ]
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

    void clearFilter();

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
    void groupingChangedSlot(const QString& text);
    void filterChangedSlot(const QString& text);
    void setFilterVisible(bool visible);
    void contextMenuSlot(const QPoint& pos);

private:
    void updateGrouping();
    void updateStaleIndication();
    void openFilter();

    DBContentItemModel& model_;

    QComboBox*             grouping_box_  {nullptr};
    QToolButton*           filter_button_ {nullptr};  // checked => filter field open
    QLineEdit*             filter_edit_   {nullptr};
    QSortFilterProxyModel* proxy_model_   {nullptr};  // filter between model_ and tree_view_
    QTreeView*             tree_view_     {nullptr};  // shows proxy_model_ => map its indexes to model_
    WaitSpinnerWidget*     stale_spinner_ {nullptr};

    unsigned int active_groupings_ = std::numeric_limits<unsigned int>::max();
};
