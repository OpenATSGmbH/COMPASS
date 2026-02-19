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
#include "allbuffertablewidget.h"
#include "allbuffercsvexportjob.h"
#include "buffer.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "global.h"
#include "jobmanager.h"
#include "tableview.h"
#include "tableviewdatasource.h"

AllBufferTableModel::AllBufferTableModel(TableView& view, AllBufferTableWidget* table_widget,
                                         TableViewDataSource& data_source)
    : BaseBufferTableModel(view, table_widget, data_source)
{
}

AllBufferTableModel::~AllBufferTableModel() {}

unsigned int AllBufferTableModel::dataRowCount() const
{
    return row_indexes_.size();
}

BaseBufferTableModel::RowData AllBufferTableModel::resolveRow(int row) const
{
    traced_assert(row >= 0);
    traced_assert((unsigned int)row < row_indexes_.size());

    unsigned int dbcont_num = row_indexes_.at(row).first;
    unsigned int buffer_index = row_indexes_.at(row).second;

    traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
    const std::string& dbcontent_name = number_to_dbcont_.at(dbcont_num);

    traced_assert(buffers_.count(dbcontent_name) == 1);

    RowData rd;
    rd.buffer = buffers_.at(dbcontent_name).get();
    rd.buffer_index = buffer_index;
    rd.dbcontent_name = dbcontent_name;
    return rd;
}

unsigned int AllBufferTableModel::prefixColumnCount() const
{
    return 2;  // checkbox + DBContent name
}

unsigned int AllBufferTableModel::dataColumnCount() const
{
    return data_source_.getSet()->getSize();
}

QVariant AllBufferTableModel::prefixColumnData(unsigned int col, const RowData& row_data) const
{
    if (col == 0)  // checkbox column returns empty for DisplayRole
        return QVariant();
    if (col == 1)  // DBContent name column
        return QVariant(row_data.dbcontent_name.c_str());

    return QVariant();
}

QVariant AllBufferTableModel::prefixColumnHeader(unsigned int col) const
{
    if (col == 0)
        return QString();
    if (col == 1)
        return QString("DBContent");

    return QVariant();
}

bool AllBufferTableModel::resolveVariable(unsigned int data_col,
                                          const std::string& dbcontent_name,
                                          dbContent::Variable*& out_var) const
{
    traced_assert(data_col < data_source_.getSet()->getSize());

    std::string variable_dbcontent_name, variable_name;
    std::tie(variable_dbcontent_name, variable_name) = data_source_.getSet()->variableDefinition(data_col);

    DBContentManager& manager = COMPASS::instance().dbContentManager();

    if (variable_dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(manager.existsMetaVariable(variable_name));
        if (!manager.metaVariable(variable_name).existsIn(dbcontent_name))
            return false;
    }
    else
    {
        if (dbcontent_name != variable_dbcontent_name)
            return false;

        traced_assert(manager.existsDBContent(dbcontent_name));
        traced_assert(manager.dbContent(dbcontent_name).hasVariable(variable_name));
    }

    out_var = (variable_dbcontent_name == META_OBJECT_NAME)
                  ? &manager.metaVariable(variable_name).getFor(dbcontent_name)
                  : &manager.dbContent(dbcontent_name).variable(variable_name);
    return true;
}

QVariant AllBufferTableModel::dataColumnHeader(unsigned int data_col) const
{
    traced_assert(data_col < data_source_.getSet()->getSize());
    std::string variable_name = data_source_.getSet()->variableDefinition(data_col).second;
    return QString(variable_name.c_str());
}

void AllBufferTableModel::clearData()
{
    logdbg;

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

        if (dbcont_to_number_.count(dbcontent_name) == 0)
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
    sortRowIndexes();

    endResetModel();
}

void AllBufferTableModel::updateTimeIndexes()
{
    logdbg;

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

        if (buffer_size > buffer_index + 1)
        {
            logdbg << "new " << dbcontent_name
                   << " data, last index " << buffer_index << " size " << buf_it.second->size();

            const dbContent::Variable& ts_var =
                    dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).getFor(dbcontent_name);

            traced_assert(buf_it.second->has<boost::posix_time::ptime>(ts_var.name()));
            NullableVector<boost::posix_time::ptime>& ts_vec =
                buf_it.second->get<boost::posix_time::ptime>(ts_var.name());

            traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));
            NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

            boost::posix_time::ptime ts;

            for (; buffer_index < buffer_size; ++buffer_index)
            {
                if (ts_vec.isNull(buffer_index))
                {
                    ts = boost::posix_time::ptime(boost::posix_time::not_a_date_time);
                    num_time_none++;
                }
                else
                    ts = ts_vec.get(buffer_index);

                if (view_.settings().show_only_selected_)
                {
                    if (selected_vec.isNull(buffer_index))
                        continue;

                    if (selected_vec.get(buffer_index))
                        time_to_indexes_.insert(
                            std::make_pair(ts, std::make_pair(dbcont_num, buffer_index)));
                }
                else
                    time_to_indexes_.insert(
                        std::make_pair(ts, std::make_pair(dbcont_num, buffer_index)));
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

void AllBufferTableModel::rebuild()
{
    beginResetModel();

    time_to_indexes_.clear();
    row_indexes_.clear();

    updateTimeIndexes();
    rebuildRowIndexes();
    sortRowIndexes();

    endResetModel();
}

void AllBufferTableModel::applyRowPermutation(const std::vector<unsigned int>& perm)
{
    std::vector<std::pair<unsigned int, unsigned int>> new_indexes(perm.size());
    for (unsigned int i = 0; i < perm.size(); ++i)
        new_indexes[i] = row_indexes_[perm[i]];
    row_indexes_ = std::move(new_indexes);
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

std::pair<int,int> AllBufferTableModel::getSelectedRows()
{
    loginf;

    int first_row = -1;
    int last_row = -1;

    for (unsigned int cnt = 0; cnt < row_indexes_.size(); ++cnt)
    {
        unsigned int dbcont_num = row_indexes_.at(cnt).first;
        unsigned int buffer_index = row_indexes_.at(cnt).second;

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
