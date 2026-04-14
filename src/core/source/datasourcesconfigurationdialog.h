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

#include "datasourcecreatedialog.h"

#include <QDialog>

#include <memory>

class DataSourceManager;
class DataSourceTableModel;
class DataSourceEditWidget;

class QTableView;
class QSortFilterProxyModel;

class DataSourcesConfigurationDialog : public QDialog
{
    Q_OBJECT

signals:
    void doneSignal();

public slots:

    void currentRowChanged(const QModelIndex& current, const QModelIndex& previous);

    void newDSClickedSlot();
    void newDSDoneSlot();

    void importClickedSlot();
    void deleteAllClickedSlot();
    void exportClickedSlot();
    void doneClickedSlot();

public:
    DataSourcesConfigurationDialog(DataSourceManager& ds_man);

protected:
    DataSourceManager& ds_man_;

    QTableView* table_view_{nullptr};
    QSortFilterProxyModel* proxy_model_{nullptr};
    DataSourceTableModel* table_model_;

    DataSourceEditWidget* edit_widget_ {nullptr};

    std::unique_ptr<DataSourceCreateDialog> create_dialog_;
};
