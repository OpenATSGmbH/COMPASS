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

#include "toolboxwidget.h"

#include "json_fwd.hpp"

class ViewManager;
class ViewPointsTableModel;
class ViewPoint;

class QTableView;
class QPushButton;
class QSortFilterProxyModel;

/**
 */
class ViewPointsWidget : public ToolBoxWidget
{
    Q_OBJECT

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

    void exportSlot();
    void exportPDFSlot();
    void deleteAllSlot();
    void importSlot();

    void selectPreviousSlot();
    void selectNextSlot();

    void setSelectedOpenSlot();
    void setSelectedClosedSlot();
    void setSelectedTodoSlot();
    void editCommentSlot();

    void currentRowChanged(const QModelIndex& current, const QModelIndex& previous);
    //void onTableClickedSlot(const QModelIndex& current);

    void typesChangedSlot(QStringList types);
    void statusesChangedSlot(QStringList statuses);

public:
    ViewPointsWidget(ViewManager& view_manager);
    virtual ~ViewPointsWidget();

    //ToolBoxWidget
    QIcon toolIcon() const override final;
    std::string toolName() const override final;
    std::string toolInfo() const override final;
    std::vector<std::string> toolLabels() const override final;
    int defaultWidthPercent() const override final;
    void addToConfigMenu(QMenu* menu) override final;
    void addToToolBar(QToolBar* tool_bar) override final;
    void loadingStarted() override final;
    void loadingDone() override final;

    void loadViewPoints();
    void clearViewPoints();
    void addViewPoints(const std::vector <nlohmann::json>& viewpoints);

    void resizeColumnsToContents();

    //    void selectNextOpen();
//    void openCurrentSelectNext();
//    void closeCurrentSelectNext();

    ViewPointsTableModel* tableModel() const;

    // filter columns
    QStringList columns() const;
    QStringList filteredColumns() const;

    void filterColumn (QString name);
    void showOnlyMainColumns ();
    void showAllColumns ();
    void showNoColumns ();

    void updateFilteredColumns ();

    // filter type
    QStringList types() const;
    QStringList filteredTypes() const;

    void filterType (QString type);
    void showAllTypes ();
    void showNoTypes ();

    void updateFilteredTypes ();

    // filter statuses
    QStringList statuses() const;
    QStringList filteredStatuses() const;

    void filterStatus (QString status);
    void showAllStatuses ();
    void showNoStatuses ();

    void updateFilteredStatuses ();

    std::vector<unsigned int> viewPoints(); // all
    std::vector<unsigned int> viewedViewPoints(); // only viewed

private:
    ViewManager& view_manager_;

    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};
    ViewPointsTableModel* table_model_{nullptr};

    QPushButton* import_button_{nullptr};
    QPushButton* delete_all_button_{nullptr};
    QPushButton* export_button_{nullptr};
    QPushButton* export_pdf_button_{nullptr};

    QStringList types_;
    QStringList filtered_types_;

    QStringList statuses_;
    QStringList filtered_statuses_;

    QStringList columns_;
    QStringList filtered_columns_;

    bool load_in_progress_ {false};
    bool restore_focus_ {false};
};
