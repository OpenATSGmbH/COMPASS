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

#include "allbuffertablemodel.h"

#include <QApplication>

#include "allbuffercsvexportjob.h"
#include "allbuffertablewidget.h"
#include "compass.h"
#include "buffer.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "global.h"
#include "jobmanager.h"
#include "tableview.h"
#include "tableviewdatasource.h"
#include "dbcontent/variable/metavariable.h"

AllBufferTableModel::AllBufferTableModel(TableView& view, AllBufferTableWidget* table_widget,
                                         TableViewDataSource& data_source)
    : QAbstractTableModel(table_widget), view_(view), table_widget_(table_widget), data_source_(data_source)
{
    connect(&data_source_, &TableViewDataSource::setChangedSignal, this, &AllBufferTableModel::setChangedSlot);
}

AllBufferTableModel::~AllBufferTableModel() {}

void AllBufferTableModel::setChangedSlot()
{
    beginResetModel();
    endResetModel();
    traced_assert(table_widget_);
    table_widget_->resizeColumns();
}

int AllBufferTableModel::rowCount(const QModelIndex& /*parent*/) const
{
    logdbg << "start" << row_indexes_.size();
    return row_indexes_.size();
}

int AllBufferTableModel::columnCount(const QModelIndex& /*parent*/) const
{
    logdbg << "start" << data_source_.getSet()->getSize();

    // cnt, DBCont
    return data_source_.getSet()->getSize() + 2;
}

QVariant AllBufferTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal)
    {
        logdbg << "section " << section;
        unsigned int col = section;

        if (col == 0)
            return QString();
        if (col == 1)
            return QString("DBContent");

        col -= 2;  // for the actual properties

        traced_assert(col < data_source_.getSet()->getSize());
        std::string variable_name = data_source_.getSet()->variableDefinition(col).second;
        return QString(variable_name.c_str());
    }
    else if (orientation == Qt::Vertical)
        return section;

    return QVariant();
}

Qt::ItemFlags AllBufferTableModel::flags(const QModelIndex& index) const
{
    Qt::ItemFlags flags;

    if (index.column() == 0)
    {
        flags |= Qt::ItemIsEnabled;
        flags |= Qt::ItemIsUserCheckable;
        flags |= Qt::ItemIsEditable;
        // flags |= Qt::ItemIsSelectable;
    }
    else
        return Qt::ItemIsEnabled | Qt::ItemIsSelectable;

    return flags;
}

QVariant AllBufferTableModel::data(const QModelIndex& index, int role) const
{
    logdbg << "row " << index.row() - 1 << " col " << index.column() - 1;

    bool null = false;

    traced_assert(index.row() >= 0);
    traced_assert((unsigned int)index.row() < row_indexes_.size());
    unsigned int dbcont_num = row_indexes_.at(index.row()).first;
    unsigned int buffer_index = row_indexes_.at(index.row()).second;
    unsigned int col = index.column();

    traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
    const std::string& dbcontent_name = number_to_dbcont_.at(dbcont_num);

    traced_assert(buffers_.count(dbcontent_name) == 1);
    std::shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);

    if (role == Qt::CheckStateRole)
    {
        if (col == 0)  // selected special case
        {
            traced_assert(buffer->has<bool>(DBContent::selected_var.name()));

            if (buffer->get<bool>(DBContent::selected_var.name()).isNull(buffer_index))
                return Qt::Unchecked;

            if (buffer->get<bool>(DBContent::selected_var.name()).get(buffer_index))
                return Qt::Checked;
            else
                return Qt::Unchecked;
        }
    }
    else if (role == Qt::DisplayRole)
    {
        traced_assert(buffer);

        std::string value_str;

        const PropertyList& properties = buffer->properties();

        if (buffer_index >= buffer->size())
        {
            logerr << "index " << buffer_index << " too large for "
                   << dbcontent_name << "  size " << buffer->size();
            return QVariant();
        }

        traced_assert(buffer_index < buffer->size());

        if (col == 0)  // selected special case
            return QVariant();
        if (col == 1)  // selected special case
            return QVariant(dbcontent_name.c_str());

        col -= 2;  // for the actual properties

        //        loginf << "col " << col << " set size " <<
        //        data_source_.getSet()->getSize()
        //               << " show assoc " << show_associations_;
        traced_assert(col < data_source_.getSet()->getSize());

        std::string variable_dbcontent_name, variable_name;

        std::tie(variable_dbcontent_name, variable_name) = data_source_.getSet()->variableDefinition(col);

        DBContentManager& manager = COMPASS::instance().dbContentManager();

        // check if data & variables exist
        if (variable_dbcontent_name == META_OBJECT_NAME)
        {
            traced_assert(manager.existsMetaVariable(variable_name));
            if (!manager.metaVariable(variable_name).existsIn(dbcontent_name))  // not data if not exist
                return QString();
        }
        else
        {
            if (dbcontent_name != variable_dbcontent_name)  // check if other dbcont
                return QString();

            traced_assert(manager.existsDBContent(dbcontent_name));
            traced_assert(manager.dbContent(dbcontent_name).hasVariable(variable_name));
        }

        dbContent::Variable& variable = (variable_dbcontent_name == META_OBJECT_NAME)
                                    ? manager.metaVariable(variable_name).getFor(dbcontent_name)
                                    : manager.dbContent(dbcontent_name).variable(variable_name);
        PropertyDataType data_type = variable.dataType();

        value_str = NULL_STRING;

        if (!properties.hasProperty(variable.name()))
        {
            logdbg << "variable " << variable.name()
                   << " not present in buffer";
        }
        else
        {
            std::string property_name = variable.name();

            if (data_type == PropertyDataType::BOOL)
            {
                traced_assert(buffer->has<bool>(property_name));
                null = buffer->get<bool>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<bool>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<bool>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::CHAR)
            {
                traced_assert(buffer->has<char>(property_name));
                null = buffer->get<char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<char>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UCHAR)
            {
                traced_assert(buffer->has<unsigned char>(property_name));
                null = buffer->get<unsigned char>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<unsigned char>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                            buffer->get<unsigned char>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::INT)
            {
                traced_assert(buffer->has<int>(property_name));
                null = buffer->get<int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::UINT)
            {
                traced_assert(buffer->has<unsigned int>(property_name));
                null = buffer->get<unsigned int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<unsigned int>(property_name).getAsString(buffer_index));
                    else
                        value_str =
                            buffer->get<unsigned int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::LONGINT)
            {
                traced_assert(buffer->has<long int>(property_name));
                null = buffer->get<long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<long int>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<long int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::ULONGINT)
            {
                traced_assert(buffer->has<unsigned long int>(property_name));
                null = buffer->get<unsigned long int>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<unsigned long int>(property_name)
                                .getAsString(buffer_index));
                    else
                        value_str =
                            buffer->get<unsigned long int>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::FLOAT)
            {
                traced_assert(buffer->has<float>(property_name));
                null = buffer->get<float>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<float>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<float>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::DOUBLE)
            {
                traced_assert(buffer->has<double>(property_name));
                null = buffer->get<double>(property_name).isNull(buffer_index);
                if (!null)
                {
                    if (view_.settings().use_presentation_)
                        value_str = variable.getRepresentationStringFromValue(
                            buffer->get<double>(property_name).getAsString(buffer_index));
                    else
                        value_str = buffer->get<double>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::STRING)
            {
                traced_assert(buffer->has<std::string>(property_name));
                null = buffer->get<std::string>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer->get<std::string>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::JSON)
            {
                traced_assert(buffer->has<nlohmann::json>(property_name));
                null = buffer->get<nlohmann::json>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer->get<nlohmann::json>(property_name).getAsString(buffer_index);
                }
            }
            else if (data_type == PropertyDataType::TIMESTAMP)
            {
                traced_assert(buffer->has<boost::posix_time::ptime>(property_name));
                null = buffer->get<boost::posix_time::ptime>(property_name).isNull(buffer_index);
                if (!null)
                {
                    value_str = buffer->get<boost::posix_time::ptime>(property_name).getAsString(buffer_index);
                }
            }
            else
                throw std::domain_error("BufferTableWidget: show: unknown property data type");

            if (null)
                return QVariant();
            else
                return QString(value_str.c_str());
        }
    }
    return QVariant();
}

bool AllBufferTableModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
    logdbg << "checked row " << index.row() << " col "
           << index.column();

    if (role == Qt::CheckStateRole && index.column() == 0)
    {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        traced_assert(index.row() >= 0);
        traced_assert((unsigned int)index.row() < row_indexes_.size());
        unsigned int dbcont_num = row_indexes_.at(index.row()).first;
        unsigned int buffer_index = row_indexes_.at(index.row()).second;

        traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
        std::string dbcontent_name = number_to_dbcont_.at(dbcont_num);

        traced_assert(buffers_.count(dbcontent_name) == 1);
        std::shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);

        traced_assert(buffer);
        traced_assert(buffer->has<bool>(DBContent::selected_var.name()));

        if (value == Qt::Checked)
        {
            logdbg << "checked row index" << buffer_index;
            buffer->get<bool>(DBContent::selected_var.name()).set(buffer_index, true);
        }
        else
        {
            logdbg << "unchecked row index " << buffer_index;
            buffer->get<bool>(DBContent::selected_var.name()).set(buffer_index, false);
        }
        traced_assert(table_widget_);
        table_widget_->view().emitSelectionChange();

        if (view_.settings().show_only_selected_)
            rebuild();

        QApplication::restoreOverrideCursor();
    }
    return true;
}

void AllBufferTableModel::clearData()
{
    logdbg << "start";

    beginResetModel();

    time_to_indexes_.clear();
    row_indexes_.clear();
    buffers_.clear();

    endResetModel();
}

void AllBufferTableModel::setData(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{

    beginResetModel();

    for (auto& buf_it : buffers)
    {
        std::string dbcontent_name = buf_it.first;

        if (dbcont_to_number_.count(dbcontent_name) == 0)  // new dbcont from the wild
        {
            unsigned int num = dbcont_to_number_.size();
            number_to_dbcont_[num] = dbcontent_name;
            dbcont_to_number_[dbcontent_name] = num;
        }
    }

    traced_assert(dbcont_to_number_.size() == number_to_dbcont_.size());

    buffers_ = buffers;

    updateTimeIndexes();
    rebuildRowIndexes();

    endResetModel();
}

void AllBufferTableModel::updateTimeIndexes()
{
    logdbg << "start";

    unsigned int buffer_index;
    std::string dbcontent_name;
    unsigned int dbcont_num;
    unsigned int buffer_size;

    unsigned int num_time_none;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto& buf_it : buffers_)
    {
        if (view_.settings().ignore_non_target_reports_
            && !dbcont_man.metaCanGetVariable(buf_it.first, DBContent::meta_var_latitude_))
            continue;

        buffer_index = 0;
        dbcontent_name = buf_it.first;
        num_time_none = 0;

        traced_assert(dbcont_to_number_.count(dbcontent_name) == 1);
        dbcont_num = dbcont_to_number_.at(dbcontent_name);

        buffer_size = buf_it.second->size();

        if (buffer_size > buffer_index + 1)  // new data
        {
            logdbg << "new " << dbcontent_name
                   << " data, last index " << buffer_index << " size " << buf_it.second->size();

            const dbContent::Variable& ts_var =
                    dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).getFor(dbcontent_name);

            traced_assert(buf_it.second->has<boost::posix_time::ptime>(ts_var.name()));
            NullableVector<boost::posix_time::ptime>& ts_vec = buf_it.second->get<boost::posix_time::ptime>(ts_var.name());

            traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
            NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

            boost::posix_time::ptime ts;

            for (; buffer_index < buffer_size; ++buffer_index)
            {
                if (ts_vec.isNull(buffer_index))
                {
                    ts = boost::posix_time::ptime (boost::posix_time::not_a_date_time);

                    num_time_none++;
                    //continue;
                }
                else
                    ts = ts_vec.get(buffer_index);

                if (view_.settings().show_only_selected_)
                {
                    if (selected_vec.isNull(buffer_index))  // check if null, skip if so
                        continue;

                    if (selected_vec.get(buffer_index))  // add if set
                        time_to_indexes_.insert(std::make_pair(ts, std::make_pair(dbcont_num, buffer_index)));
                }
                else
                    time_to_indexes_.insert(std::make_pair(ts, std::make_pair(dbcont_num, buffer_index)));
            }

            if (num_time_none)
                loginf << "new " << dbcontent_name << " skipped "
                       << num_time_none << " indexes with no time";
        }
    }
}

void AllBufferTableModel::rebuildRowIndexes()
{
    row_indexes_.clear();

    for (auto& time_index_it : time_to_indexes_)
    {
        row_indexes_.push_back(time_index_it.second);
    }
}

void AllBufferTableModel::reset()
{
    beginResetModel();
    endResetModel();
}

void AllBufferTableModel::saveAsCSV(const std::string& file_name)
{
    loginf << "into filename " << file_name;

    if (!buffers_.size())
        return;

    AllBufferCSVExportJob* export_job = new AllBufferCSVExportJob(
        buffers_, data_source_.getSet(), number_to_dbcont_, row_indexes_, file_name, true,
        view_.settings().show_only_selected_, view_.settings().use_presentation_);

    export_job_ = std::shared_ptr<AllBufferCSVExportJob>(export_job);
    connect(export_job, &AllBufferCSVExportJob::obsoleteSignal, this,
            &AllBufferTableModel::exportJobObsoleteSlot, Qt::QueuedConnection);
    connect(export_job, &AllBufferCSVExportJob::doneSignal, this,
            &AllBufferTableModel::exportJobDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addBlockingJob(export_job_);
}

void AllBufferTableModel::exportJobObsoleteSlot()
{
    logdbg << "start";

    emit exportDoneSignal(true);
}

void AllBufferTableModel::exportJobDoneSlot()
{
    logdbg << "start";

    emit exportDoneSignal(false);
}

void AllBufferTableModel::rebuild()
{
    beginResetModel();

    time_to_indexes_.clear();
    row_indexes_.clear();

    updateTimeIndexes();
    rebuildRowIndexes();

    endResetModel();
}

std::pair<int,int> AllBufferTableModel::getSelectedRows()
{
    loginf;

    unsigned int dbcont_num;
    unsigned int buffer_index;

    int first_row = -1;
    int last_row = -1;

    for (unsigned int cnt=0; cnt < row_indexes_.size(); ++cnt)
    {
        dbcont_num = row_indexes_.at(cnt).first;
        buffer_index = row_indexes_.at(cnt).second;

        traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
        const std::string& dbcontent_name = number_to_dbcont_.at(dbcont_num);

        traced_assert(buffers_.count(dbcontent_name) == 1);

        std::shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);

        traced_assert(buffer->has<bool>(DBContent::selected_var.name()));
        if (buffer->get<bool>(DBContent::selected_var.name()).isNull(buffer_index))
            continue;

        if (buffer->get<bool>(DBContent::selected_var.name()).get(buffer_index))
        {
            if (first_row == -1)
                first_row = cnt;

            last_row = cnt;
        }
    }

    return std::make_pair(first_row, last_row);
}
