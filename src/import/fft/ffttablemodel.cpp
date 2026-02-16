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

#include "ffttablemodel.h"
#include "fftmanager.h"
#include "logger.h"
#include "files.h"
#include "fft/dbfft.h"
#include "fft/configurationfft.h"

using namespace Utils;
using namespace std;

FFTTableModel::FFTTableModel(FFTManager& ds_man, FFTsConfigurationDialog& dialog)
    : fft_man_(ds_man), dialog_(dialog)
{
    db_icon_ = Files::IconProvider::getIcon("db.png");
    config_icon_ = Files::IconProvider::getIcon("configuration.png");
}

int FFTTableModel::rowCount(const QModelIndex& parent) const
{
    return fft_man_.getAllFFTNames().size();
}

int FFTTableModel::columnCount(const QModelIndex& parent) const
{
    return table_columns_.size();
}

QVariant FFTTableModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid())
        return QVariant();

    switch (role)
    {
    case Qt::DisplayRole:
    {
        logdbg << "display role: row " << index.row() << " col " << index.column();

        traced_assert(index.row() >= 0);
        traced_assert(index.row() < static_cast<int>(fft_man_.getAllFFTNames().size()));

        string fft_name = fft_man_.getAllFFTNames().at(index.row());

        logdbg << "got fft_name " << fft_name;

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (fft_man_.hasDBFFT(fft_name))
        {
            DBFFT& fft = fft_man_.dbFFT(fft_name);

            if (col_name == "Name")
                return fft.name().c_str();
            else
                return QVariant();
        }
        else // cfg only
        {
            traced_assert(fft_man_.hasConfigFFT(fft_name));

            ConfigurationFFT& fft = fft_man_.configFFT(fft_name);

            if (col_name == "Name")
                return fft.name().c_str();
            else
                return QVariant();
        }
    }
    case Qt::DecorationRole:
    {
        traced_assert(index.row() >= 0);
        traced_assert(index.row() < static_cast<int>(fft_man_.getAllFFTNames().size()));

        string fft_name = fft_man_.getAllFFTNames().at(index.row());

        logdbg << "got fft_name " << fft_name;

        traced_assert(index.column() < table_columns_.size());
        std::string col_name = table_columns_.at(index.column()).toStdString();

        if (col_name != "In DB" && col_name != "In Cfg")
            return QVariant();

        if (col_name == "In DB" && fft_man_.hasDBFFT(fft_name))
            return db_icon_;
        else if (col_name == "In Cfg" && fft_man_.hasConfigFFT(fft_name))
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

QVariant FFTTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
    {
        traced_assert(section < table_columns_.size());
        return table_columns_.at(section);
    }

    return QVariant();
}

QModelIndex FFTTableModel::index(int row, int column, const QModelIndex& parent) const
{
    return createIndex(row, column);
}

QModelIndex FFTTableModel::parent(const QModelIndex& index) const
{
    return QModelIndex();
}

Qt::ItemFlags FFTTableModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    traced_assert(index.column() < table_columns_.size());

    return QAbstractItemModel::flags(index);
}

std::string FFTTableModel::getNameOf (const QModelIndex& index)
{
    traced_assert(index.isValid());

    traced_assert(index.row() >= 0);
    traced_assert(index.row() < static_cast<int>(fft_man_.getAllFFTNames().size()));

    return fft_man_.getAllFFTNames().at(index.row());
}

QModelIndex FFTTableModel::fftIndex(const std::string& fft_name)
{
    loginf << "fft_name " << fft_name;

    auto fft_names = fft_man_.getAllFFTNames();

    auto itr = std::find(fft_names.begin(), fft_names.end(), fft_name);
    traced_assert(itr != fft_names.end());

    unsigned int row = std::distance(fft_names.begin(), itr);

    return index(row, 0);
}

void FFTTableModel::updateFFT(const std::string& fft_name)
{
    loginf << "fft_name " << fft_name;

    auto fft_names = fft_man_.getAllFFTNames();

    auto itr = std::find(fft_names.begin(), fft_names.end(), fft_name);
    traced_assert(itr != fft_names.end());

    unsigned int row = std::distance(fft_names.begin(), itr);

    emit dataChanged(index(row, 0), index(row, table_columns_.size()-1), QVector<int>(Qt::DisplayRole));
}

void FFTTableModel::beginModelReset()
{
    beginResetModel();
}
void FFTTableModel::endModelReset()
{
    endResetModel();
}
