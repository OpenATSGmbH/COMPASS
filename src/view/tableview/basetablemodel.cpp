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

#include "basetablemodel.h"
#include "basetablewidget.h"
#include "buffer.h"
#include "buffer_value_string.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "datasourcemanager.h"
#include "datasourceremoteunit.h"
#include "global.h"
#include "tableview.h"
#include "tableviewdatasource.h"

#include "json.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>

#include <algorithm>
#include <numeric>

namespace
{

/// Returns a raw QVariant value from the buffer for efficient sort key extraction.
/// Numeric types are returned as their native QVariant type (int, double, qlonglong, etc.)
/// rather than formatted strings, avoiding expensive string conversion during sort.
QVariant getRawSortValue(dbContent::Variable& var, Buffer& buffer, unsigned int idx)
{
    PropertyDataType dt = var.dataType();
    const std::string& name = var.name();

    switch (dt)
    {
    case PropertyDataType::BOOL:
        if (!buffer.has<bool>(name) || buffer.get<bool>(name).isNull(idx))
            return QVariant();
        return QVariant(buffer.get<bool>(name).get(idx));

    case PropertyDataType::CHAR:
        if (!buffer.has<char>(name) || buffer.get<char>(name).isNull(idx))
            return QVariant();
        return QVariant(static_cast<int>(buffer.get<char>(name).get(idx)));

    case PropertyDataType::UCHAR:
        if (!buffer.has<unsigned char>(name) || buffer.get<unsigned char>(name).isNull(idx))
            return QVariant();
        return QVariant(static_cast<int>(buffer.get<unsigned char>(name).get(idx)));

    case PropertyDataType::INT:
        if (!buffer.has<int>(name) || buffer.get<int>(name).isNull(idx))
            return QVariant();
        return QVariant(buffer.get<int>(name).get(idx));

    case PropertyDataType::UINT:
        if (!buffer.has<unsigned int>(name) || buffer.get<unsigned int>(name).isNull(idx))
            return QVariant();
        return QVariant(buffer.get<unsigned int>(name).get(idx));

    case PropertyDataType::LONGINT:
        if (!buffer.has<long int>(name) || buffer.get<long int>(name).isNull(idx))
            return QVariant();
        return QVariant(static_cast<qlonglong>(buffer.get<long int>(name).get(idx)));

    case PropertyDataType::ULONGINT:
        if (!buffer.has<unsigned long int>(name) || buffer.get<unsigned long int>(name).isNull(idx))
            return QVariant();
        return QVariant(static_cast<qulonglong>(buffer.get<unsigned long int>(name).get(idx)));

    case PropertyDataType::FLOAT:
        if (!buffer.has<float>(name) || buffer.get<float>(name).isNull(idx))
            return QVariant();
        return QVariant(static_cast<double>(buffer.get<float>(name).get(idx)));

    case PropertyDataType::DOUBLE:
        if (!buffer.has<double>(name) || buffer.get<double>(name).isNull(idx))
            return QVariant();
        return QVariant(buffer.get<double>(name).get(idx));

    case PropertyDataType::STRING:
        if (!buffer.has<std::string>(name) || buffer.get<std::string>(name).isNull(idx))
            return QVariant();
        return QVariant(QString::fromStdString(buffer.get<std::string>(name).get(idx)));

    case PropertyDataType::TIMESTAMP:
    {
        if (!buffer.has<boost::posix_time::ptime>(name) ||
            buffer.get<boost::posix_time::ptime>(name).isNull(idx))
            return QVariant();
        boost::posix_time::ptime ts = buffer.get<boost::posix_time::ptime>(name).get(idx);
        static const boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
        return QVariant(static_cast<qlonglong>((ts - epoch).total_microseconds()));
    }

    case PropertyDataType::JSON:
        if (!buffer.has<nlohmann::json>(name) || buffer.get<nlohmann::json>(name).isNull(idx))
            return QVariant();
        return QVariant(QString::fromStdString(
            buffer.get<nlohmann::json>(name).get(idx).dump()));

    default:
        return QVariant();
    }
}

/// Type-aware QVariant comparison matching Qt's internal isVariantLessThan.
bool variantLessThan(const QVariant& l, const QVariant& r)
{
    switch (l.type())
    {
    case QVariant::Int:       return l.toInt() < r.toInt();
    case QVariant::UInt:      return l.toUInt() < r.toUInt();
    case QVariant::LongLong:  return l.toLongLong() < r.toLongLong();
    case QVariant::ULongLong: return l.toULongLong() < r.toULongLong();
    case QVariant::Double:    return l.toDouble() < r.toDouble();
    case QVariant::Bool:      return !l.toBool() && r.toBool();
    case QVariant::String:    return l.toString() < r.toString();
    default:                  return l.toString() < r.toString();
    }
}

} // anonymous namespace

BaseBufferTableModel::BaseBufferTableModel(TableView& view, BaseBufferTableWidget* table_widget,
                                           TableViewDataSource& data_source)
    : QAbstractTableModel(table_widget),
      view_(view),
      table_widget_(table_widget),
      data_source_(data_source)
{
    connect(&data_source_, &TableViewDataSource::setChangedSignal,
            this, &BaseBufferTableModel::setChangedSlot);
}

BaseBufferTableModel::~BaseBufferTableModel() {}

void BaseBufferTableModel::setChangedSlot()
{
    beginResetModel();
    endResetModel();
    traced_assert(table_widget_);
    table_widget_->resizeColumns();
}

int BaseBufferTableModel::rowCount(const QModelIndex& /*parent*/) const
{
    logdbg << "start " << dataRowCount();
    return dataRowCount();
}

int BaseBufferTableModel::columnCount(const QModelIndex& /*parent*/) const
{
    unsigned int count = prefixColumnCount() + dataColumnCount();
    logdbg << "start " << count;
    return count;
}

QVariant BaseBufferTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal)
    {
        logdbg << "section " << section;
        unsigned int col = section;

        if (col < prefixColumnCount())
            return prefixColumnHeader(col);

        unsigned int data_col = col - prefixColumnCount();
        traced_assert(data_col < dataColumnCount());
        return dataColumnHeader(data_col);
    }
    else if (orientation == Qt::Vertical)
        return section;

    return QVariant();
}

Qt::ItemFlags BaseBufferTableModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags flags;

    if (index.column() == 0)
    {
        flags |= Qt::ItemIsEnabled;
        flags |= Qt::ItemIsUserCheckable;
        flags |= Qt::ItemIsEditable;
    }
    else
        return Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    return flags;
}

QVariant BaseBufferTableModel::data(const QModelIndex& index, int role) const
{
    logdbg << "row " << index.row() << " col " << index.column();

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < dataRowCount());

    RowData rd = resolveRow(index.row());
    unsigned int col = index.column();

    if (role == Qt::CheckStateRole)
    {
        if (col == 0)  // selected special case
        {
            traced_assert(rd.buffer->has<bool>(DBContent::selected_var.name()));

            if (rd.buffer->get<bool>(DBContent::selected_var.name()).isNull(rd.buffer_index))
                return Qt::Unchecked;

            if (rd.buffer->get<bool>(DBContent::selected_var.name()).get(rd.buffer_index))
                return Qt::Checked;
            else
                return Qt::Unchecked;
        }
    }
    else if (role == Qt::DisplayRole)
    {
        traced_assert(rd.buffer);

        if (rd.buffer_index >= rd.buffer->size())
        {
            logerr << "index " << rd.buffer_index << " too large for "
                   << rd.dbcontent_name << " size " << rd.buffer->size();
            return QVariant();
        }

        // prefix columns (checkbox, DBContent name, etc.)
        if (col < prefixColumnCount())
            return prefixColumnData(col, rd);

        unsigned int data_col = col - prefixColumnCount();
        traced_assert(data_col < dataColumnCount());

        // resolve the variable for this column + dbcontent
        dbContent::Variable* variable = nullptr;
        if (!resolveVariable(data_col, rd.dbcontent_name, variable))
            return QString();

        traced_assert(variable);

        // check if property exists in buffer
        if (!rd.buffer->properties().hasProperty(variable->name()))
        {
            logdbg << "variable " << variable->name() << " not present in buffer";
            return QVariant();
        }

        // try special representation first
        std::string value_str;
        if (view_.settings().use_presentation_ &&
            getSpecialRepresentation(value_str, *variable, *rd.buffer, rd.buffer_index))
            return QString(value_str.c_str());

        // get value string via shared utility
        bool is_null = false;
        value_str = buffer_utils::getValueString(
            *variable, *rd.buffer, rd.buffer_index,
            view_.settings().use_presentation_, is_null);

        if (is_null)
            return QVariant();
        else
            return QString(value_str.c_str());
    }
    return QVariant();
}

bool BaseBufferTableModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    logdbg << "checked row " << index.row() << " col " << index.column();

    if (role == Qt::CheckStateRole && index.column() == 0)
    {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < dataRowCount());

        RowData rd = resolveRow(index.row());

        traced_assert(rd.buffer);
        traced_assert(rd.buffer->has<bool>(DBContent::selected_var.name()));

        if (value == Qt::Checked)
        {
            logdbg << "checked row index " << rd.buffer_index;
            rd.buffer->get<bool>(DBContent::selected_var.name()).set(rd.buffer_index, true);
        }
        else
        {
            logdbg << "unchecked row index " << rd.buffer_index;
            rd.buffer->get<bool>(DBContent::selected_var.name()).set(rd.buffer_index, false);
        }

        view_.emitSelectionChange();

        if (view_.settings().show_only_selected_)
            rebuild();

        QApplication::restoreOverrideCursor();
    }
    return true;
}

void BaseBufferTableModel::reset()
{
    beginResetModel();
    endResetModel();
}

void BaseBufferTableModel::exportJobObsoleteSlot()
{
    logdbg;
    emit exportDoneSignal(true);
}

void BaseBufferTableModel::exportJobDoneSlot()
{
    logdbg;
    emit exportDoneSignal(false);
}

bool BaseBufferTableModel::getSpecialRepresentation(std::string& repr,
                                                     dbContent::Variable& var,
                                                     Buffer& buffer,
                                                     unsigned int buffer_idx)
{
    auto data_type = var.dataType();
    std::string property_name = var.name();

    if (data_type != PropertyDataType::JSON)
        return false;

    // handle CAT020 contributing receivers
    if (var.dbContent().id() == 20 && var.name() == DBContent::var_cat020_contrib_recv_.name())
    {
        if (buffer.get<nlohmann::json>(property_name).isNull(buffer_idx))
            return false;

        const auto& contrib_receivers = buffer.get<nlohmann::json>(property_name).getRef(buffer_idx);

        if (!contrib_receivers.is_array() || contrib_receivers.empty())
            return false;

        auto& dbcontent_man = COMPASS::instance().dbContentManager();

        traced_assert(dbcontent_man.metaCanGetVariable(var.dbContentName(), DBContent::meta_var_ds_id_));
        auto& ds_var = dbcontent_man.metaGetVariable(var.dbContentName(), DBContent::meta_var_ds_id_);
        traced_assert(buffer.hasAnyPropertyNamed(ds_var.name()));

        auto& ds_vec = buffer.get<unsigned int>(ds_var.name());
        if (ds_vec.isNull(buffer_idx))
            return false;

        auto ds_id = ds_vec.get(buffer_idx);

        auto& ds_man = COMPASS::instance().dataSourceManager();
        traced_assert(ds_man.hasDBDataSource(ds_id));

        auto& ds = ds_man.dbDataSource(ds_id);

        repr = "[";

        size_t i = 0;
        size_t n = contrib_receivers.size();
        for (const auto& j_idx : contrib_receivers)
        {
            traced_assert(j_idx.is_number_integer());
            int idx = j_idx.get<int>();

            if (ds.hasRemoteUnit(idx))
                repr += ds.remoteUnit(idx)->name();
            else
                repr += std::to_string(idx);

            if (i < n - 1)
                repr += ", ";

            ++i;
        }

        repr += "]";

        return true;
    }

    return false;
}

void BaseBufferTableModel::sort(int column, Qt::SortOrder order)
{
    sort_column_ = column;
    sort_order_ = order;

    beginResetModel();
    sortRowIndexes();
    endResetModel();
}

void BaseBufferTableModel::sortRowIndexes()
{
    if (sort_column_ < 0)
        return;

    unsigned int n = dataRowCount();
    if (n == 0)
        return;

    unsigned int col = static_cast<unsigned int>(sort_column_);

    // extract sort keys once (N calls) instead of per-comparison (N*logN calls)
    std::vector<QVariant> keys(n);
    for (unsigned int i = 0; i < n; ++i)
    {
        RowData rd = resolveRow(i);

        if (col == 0)  // checkbox column
        {
            if (rd.buffer->has<bool>(DBContent::selected_var.name()) &&
                !rd.buffer->get<bool>(DBContent::selected_var.name()).isNull(rd.buffer_index))
                keys[i] = QVariant(rd.buffer->get<bool>(DBContent::selected_var.name()).get(rd.buffer_index));
        }
        else if (col < prefixColumnCount())
        {
            keys[i] = prefixColumnData(col, rd);
        }
        else
        {
            unsigned int data_col = col - prefixColumnCount();
            dbContent::Variable* var = nullptr;
            if (resolveVariable(data_col, rd.dbcontent_name, var) && var &&
                rd.buffer->properties().hasProperty(var->name()))
            {
                keys[i] = getRawSortValue(*var, *rd.buffer, rd.buffer_index);
            }
        }
    }

    // build permutation and sort by pre-extracted keys
    std::vector<unsigned int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);

    bool ascending = (sort_order_ == Qt::AscendingOrder);
    std::stable_sort(perm.begin(), perm.end(), [&](unsigned int a, unsigned int b)
    {
        bool a_valid = keys[a].isValid();
        bool b_valid = keys[b].isValid();
        if (!a_valid && !b_valid) return false;
        if (!a_valid) return ascending;   // nulls first in ascending
        if (!b_valid) return !ascending;
        if (ascending)
            return variantLessThan(keys[a], keys[b]);
        else
            return variantLessThan(keys[b], keys[a]);
    });

    applyRowPermutation(perm);
}
