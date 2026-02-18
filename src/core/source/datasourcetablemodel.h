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

#include <QAbstractItemModel>
#include <QIcon>

class DataSourceManager;
class DataSourcesConfigurationDialog;

class DataSourceTableModel : public QAbstractItemModel
{
public:
    DataSourceTableModel(DataSourceManager& ds_man, DataSourcesConfigurationDialog& dialog);

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    int columnCount(const QModelIndex& parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex& index) const override;

    Qt::ItemFlags flags(const QModelIndex &index) const override;

    //bool setData(const QModelIndex &index, const QVariant &value, int role) override;

    unsigned int getIdOf (const QModelIndex& index);

    QModelIndex dataSourceIndex(unsigned int ds_id); // returns row
    void updateDataSource(unsigned int ds_id);
    void beginModelReset();
    void endModelReset();

protected:
    DataSourceManager& ds_man_;
    DataSourcesConfigurationDialog& dialog_;

    QStringList table_columns_ {"Name", "Short Name", "DSType", "SAC", "SIC", "In DB", "In Cfg"};

    QIcon db_icon_;
    QIcon config_icon_;
};
