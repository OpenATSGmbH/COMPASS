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

#include <QDialog>

class DBContentManager;

class QSplitter;
class QStackedWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace dbContent
{

class MetaVariableDetailWidget;
class VariableDetailWidget;

/**
 * Edit dialog for DBContent variables and MetaVariables.
 *
 * Layout:
 *   - left: tree with top-level "Meta", "CAT001", ..., "RefTraj" entries.
 *     At most one top-level item is expanded at any time. Leaves are
 *     MetaVariables / DBContent Variables sorted alphabetically.
 *   - right: stacked widget showing the detail editor for the selected
 *     leaf (MetaVariableDetailWidget or VariableDetailWidget) or a
 *     placeholder when nothing is selected.
 *
 * Read-only outside expert mode (the detail widgets enforce this).
 *
 * Available regardless of DB-open state: all data shown comes from
 * the DBContentManager configuration loaded at startup.
 */
class DBContentEditDialog : public QDialog
{
    Q_OBJECT

signals:
    void okSignal();

private slots:
    void itemSelectedSlot(QTreeWidgetItem* current, QTreeWidgetItem* previous);
    void itemExpandedSlot(QTreeWidgetItem* item);
    void okClickedSlot();

public:
    explicit DBContentEditDialog(DBContentManager& dbcont_man, QWidget* parent = nullptr);
    ~DBContentEditDialog() override;

    void rebuildTree();

private:
    void showPlaceholder();

    DBContentManager& dbcont_man_;

    QSplitter* splitter_ {nullptr};
    QTreeWidget* tree_widget_ {nullptr};
    QStackedWidget* detail_stack_ {nullptr};

    QWidget* placeholder_ {nullptr};
    MetaVariableDetailWidget* meta_detail_widget_ {nullptr};
    VariableDetailWidget* var_detail_widget_ {nullptr};
};

}
