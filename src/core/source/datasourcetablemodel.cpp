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

#include "datasourcetablemodel.h"
#include "datasourcemanager.h"
#include "logger.h"
#include "files.h"

using namespace Utils;
using namespace std;

DataSourceTableModel::DataSourceTableModel(DataSourceManager& ds_man, DataSourcesConfigurationDialog& dialog)
    : ds_man_(ds_man), dialog_(dialog)
{
    db_icon_ = Files::IconProvider::getIcon("db.png");
    config_icon_ = Files::IconProvider::getIcon("configuration.png");
}

int DataSourceTableModel::rowCount(const QModelIndex& parent) const
{
    return ds_man_.getAllDsIDs().size();
}

int DataSourceTableModel::columnCount(const QModelIndex& parent) const
{
    return table_columns_.size();
}

QVariant DataSourceTableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    switch (role)
    {
    case Qt::DisplayRole:
    {
        logdbg << "display role: row " << index.row() << " col " << index.column();

        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < ds_man_.getAllDsIDs().size());

        unsigned int ds_id = ds_man_.getAllDsIDs().at(index.row());

        logdbg << "got ds_id " << ds_id;

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (ds_man_.hasDBDataSource(ds_id))
        {
            dbContent::DBDataSource& ds = ds_man_.dbDataSource(ds_id);
            traced_assert(ds_man_.hasConfigDataSource(ds_id));

            if (col_name == "Name")
                return ds.name().c_str();
            if (col_name == "Short Name")
            {
                if (ds.hasShortName())
                    return ds.shortName().c_str();
                else
                    return QVariant();
            }
            if (col_name == "DSType")
                return ds.dsType().c_str();
            if (col_name == "SAC")
                return ds.sac();
            if (col_name == "SIC")
                return ds.sic();
            else
                return QVariant();
        }
        else // cfg only
        {
            traced_assert(ds_man_.hasConfigDataSource(ds_id));

            dbContent::ConfigurationDataSource& ds = ds_man_.configDataSource(ds_id);

            if (col_name == "Name")
                return ds.name().c_str();
            if (col_name == "Short Name")
            {
                if (ds.hasShortName())
                    return ds.shortName().c_str();
                else
                    return QVariant();
            }
            if (col_name == "DSType")
                return ds.dsType().c_str();
            if (col_name == "SAC")
                return ds.sac();
            if (col_name == "SIC")
                return ds.sic();
            else
                return QVariant();
        }
    }
    case Qt::DecorationRole:
    {
        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < ds_man_.getAllDsIDs().size());

        unsigned int ds_id = ds_man_.getAllDsIDs().at(index.row());

        logdbg << "got ds_id " << ds_id;

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (col_name != "In DB" && col_name != "In Cfg")
            return QVariant();

        if (col_name == "In DB" && ds_man_.hasDBDataSource(ds_id))
            return db_icon_;
        else if (col_name == "In Cfg" && ds_man_.hasConfigDataSource(ds_id))
            return config_icon_;
        else
            return QVariant();

    }
    default:
    {
        return QVariant();
    }
    }
}

QVariant DataSourceTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

QModelIndex DataSourceTableModel::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

QModelIndex DataSourceTableModel::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags DataSourceTableModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    //    if (table_columns_.at(index.column()) == "comment")
    //        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    //    else

    return QAbstractItemModel::flags(index);
}

unsigned int DataSourceTableModel::getIdOf (const QModelIndex& index)
{
    traced_assert(index.isValid());

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < ds_man_.getAllDsIDs().size());

    return ds_man_.getAllDsIDs().at(index.row());
}

QModelIndex DataSourceTableModel::dataSourceIndex(unsigned int ds_id)
{
    loginf << "ds_id " << ds_id;

    auto ds_ids = ds_man_.getAllDsIDs();

    auto itr = std::find(ds_ids.begin(), ds_ids.end(), ds_id);
    traced_assert(itr != ds_ids.end());

    unsigned int row = std::distance(ds_ids.begin(), itr);

    return index(row, 0);
}

void DataSourceTableModel::updateDataSource(unsigned int ds_id)
{
    loginf << "ds_id " << ds_id;

    auto ds_ids = ds_man_.getAllDsIDs();

    auto itr = std::find(ds_ids.begin(), ds_ids.end(), ds_id);
    traced_assert(itr != ds_ids.end());

    unsigned int row = std::distance(ds_ids.begin(), itr);

    emit dataChanged(index(row, 0), index(row, table_columns_.size()-1), QVector<int>(Qt::DisplayRole));
}

void DataSourceTableModel::beginModelReset()
{
    beginResetModel();
}
void DataSourceTableModel::endModelReset()
{
    endResetModel();
}
