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

#include "asteriximportprobeaggregator.h"

#include <QWidget>

#include <boost/optional.hpp>

class ASTERIXImportTask;
class DataSourceEditWidget;

class QLabel;
class QSplitter;
class QStackedWidget;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

/**
 * Tab widget showing data sources of the active context joined with the
 * sensors detected by jASTERIX during the import probe (testFileDecoding()).
 *
 * Left:  three-level tree DSType -> DataSource -> Category
 * Right: stacked widget — DataSourceEditWidget on DS click,
 *        items table on Category click, placeholder otherwise.
 */
class ASTERIXImportDataSourcesWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ASTERIXImportDataSourcesWidget(ASTERIXImportTask& task,
                                            QWidget* parent = nullptr);
    ~ASTERIXImportDataSourcesWidget() override;

    /// Current warning state (matches the most recent warningsChanged signal).
    bool hasWarnings() const { return has_warnings_; }

signals:
    /// Emitted whenever the warning state of the displayed tree changes.
    /// `any` is true when at least one data source has a warning (missing
    /// radar position, DSType "Other", or detected-but-not-in-context).
    void warningsChanged(bool any);

private slots:
    void rebuildAll();
    void onTreeSelectionChanged();

private:
    void buildUI();
    void rebuildTree();
    void populateItemsTable(unsigned int ds_id, unsigned int category);
    void showDetailWidget(QWidget* widget);

    ASTERIXImportTask& task_;
    ASTERIXImportProbeAggregator::Result last_result_;

    QSplitter*       splitter_     = nullptr;
    QTreeWidget*     tree_widget_  = nullptr;
    QStackedWidget*  detail_stack_ = nullptr;

    DataSourceEditWidget* ds_edit_widget_ = nullptr;
    QWidget*              items_page_     = nullptr;
    QLabel*               items_header_   = nullptr;
    QTableWidget*         items_table_    = nullptr;
    QLabel*               placeholder_    = nullptr;

    // remember last selection across rebuilds
    boost::optional<unsigned int> selected_ds_id_;
    boost::optional<unsigned int> selected_category_;

    bool has_warnings_ = false;   // last reported state to limit signal emission
};
