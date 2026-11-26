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

#include <map>
#include <string>
#include <vector>

class AnalyzeDataSourceTask;
class DataSourceInspectorBase;

class QTreeWidget;
class QTreeWidgetItem;
class QStackedWidget;
class QWidget;
class QPushButton;
class QLineEdit;
class QComboBox;
class QLabel;

class AnalyzeDataSourceDialog : public QDialog
{
    Q_OBJECT

public:
    AnalyzeDataSourceDialog(AnalyzeDataSourceTask& task, QWidget* parent = nullptr);
    ~AnalyzeDataSourceDialog() override = default;

private slots:
    void treeSelectionChangedSlot();
    void treeItemChangedSlot(QTreeWidgetItem* item, int column);
    void runSlot();

private:
    void buildTree();
    QWidget* buildDataSourcesWidget();
    QWidget* buildReportWidget();
    QWidget* buildInspectorWidget(DataSourceInspectorBase* inspector);
    void refreshInspectorRows();
    void updateRunEnabled();
    void updateSuggestedReportName();

    AnalyzeDataSourceTask&  task_;

    QTreeWidget*    tree_         = nullptr;
    QStackedWidget* stack_        = nullptr;
    QPushButton*    run_button_   = nullptr;

    QTreeWidgetItem* ds_item_     = nullptr;
    QTreeWidgetItem* report_item_ = nullptr;

    int ds_stack_idx_     = -1;
    int report_stack_idx_ = -1;

    QLineEdit*       report_name_edit_ = nullptr;
    QComboBox*       ref_line_combo_   = nullptr;
    QComboBox*       tst_line_combo_   = nullptr;

    bool             report_name_user_edited_ = false;

    struct InspectorRow
    {
        QTreeWidgetItem* item = nullptr;
        DataSourceInspectorBase* inspector = nullptr;
        QLabel* reason_label = nullptr;   // unmet prerequisite, on top of the inspector page
    };
    std::vector<InspectorRow> inspector_rows_;

    void updateInspectorRow(InspectorRow& row);

    bool updating_ui_ = false;
};
