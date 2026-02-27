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
#include "buffer_utils.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "datasourcemanager.h"
#include "datasourceremoteunit.h"
#include "global.h"
#include "tableview.h"
#include "tableviewdatasource.h"

#include "logger.h"
#include "stringconv.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <QApplication>

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
    beginCustomResetModel();
    endCustomResetModel();
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
            getSpecialRepresentation(value_str, *variable, *rd.buffer, rd.buffer_index, view_.compass()))
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
    beginCustomResetModel();
    endCustomResetModel();
}

void BaseBufferTableModel::beginCustomResetModel()
{
    resetting_model_ = true;
    beginResetModel();
}

void BaseBufferTableModel::endCustomResetModel()
{
    endResetModel();
    resetting_model_ = false;
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
                                                     unsigned int buffer_idx,
                                                     COMPASS& compass)
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

        auto& dbcontent_man = compass.dbContentManager();

        traced_assert(dbcontent_man.metaCanGetVariable(var.dbContentName(), DBContent::meta_var_ds_id_));
        auto& ds_var = dbcontent_man.metaGetVariable(var.dbContentName(), DBContent::meta_var_ds_id_);
        traced_assert(buffer.hasAnyPropertyNamed(ds_var.name()));

        auto& ds_vec = buffer.get<unsigned int>(ds_var.name());
        if (ds_vec.isNull(buffer_idx))
            return false;

        auto ds_id = ds_vec.get(buffer_idx);

        auto& ds_man = compass.dataSourceManager();
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
    // sort() can be called by Qt automatically after endResetModel() when sorting is enabled,
    // in which case resetting_model_ is true and cursor/reset are already managed by the caller
    if (resetting_model_)
    {
        sort_column_ = column;
        sort_order_ = order;
        sortRowIndexes();
        return;
    }

    sort_column_ = column;
    sort_order_ = order;

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    unsigned int num_records = dataRowCount();

    if (num_records)
        loginf << "sorting column " << column << " with " << num_records << " records";

    beginCustomResetModel();
    sortRowIndexes();
    endCustomResetModel();

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    if (num_records)
        loginf << "sorting done with " << num_records << " records in "
            << Utils::String::timeStringFromDouble(elapsed_s, true);

    QApplication::restoreOverrideCursor();
}

void BaseBufferTableModel::sortRowIndexes()
{
    // default no-op; subclasses override with type-specific sorting
}
