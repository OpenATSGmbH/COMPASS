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
#include "logger.h"
#include "stringconv.h"
#include "tableview.h"
#include "tableviewdatasource.h"

#include "json.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <algorithm>
#include <numeric>

namespace
{

/// Extracts native-typed keys from multiple buffers and sorts row_indexes in-place.
/// Each row references a (dbcont_num, buffer_index) pair; the NullableVector for each
/// dbcont_num is resolved once via nvec_map to avoid per-row map lookups.
template <typename T>
void typedSortPairs(
    std::vector<std::pair<unsigned int, unsigned int>>& row_indexes,
    const std::map<unsigned int, std::string>& number_to_dbcont,
    const std::map<std::string, std::shared_ptr<Buffer>>& buffers,
    const std::map<unsigned int, std::string>& dbcont_num_to_var_name,
    bool ascending)
{
    unsigned int n = row_indexes.size();

    // pre-resolve NullableVector<T>* per dbcont_num
    std::map<unsigned int, NullableVector<T>*> nvec_map;
    for (const auto& p : dbcont_num_to_var_name)
    {
        const auto& dbcont_name = number_to_dbcont.at(p.first);
        Buffer& buf = *buffers.at(dbcont_name);
        if (buf.has<T>(p.second))
            nvec_map[p.first] = &buf.get<T>(p.second);
    }

    // extract typed keys
    std::vector<T> values(n);
    std::vector<bool> nulls(n, true);

    for (unsigned int i = 0; i < n; ++i)
    {
        auto nvec_it = nvec_map.find(row_indexes[i].first);
        if (nvec_it == nvec_map.end())
            continue;

        unsigned int buf_idx = row_indexes[i].second;
        NullableVector<T>& nvec = *nvec_it->second;

        if (!nvec.isNull(buf_idx))
        {
            values[i] = nvec.get(buf_idx);
            nulls[i] = false;
        }
    }

    // sort permutation by typed keys
    std::vector<unsigned int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);

    std::stable_sort(perm.begin(), perm.end(), [&](unsigned int a, unsigned int b)
    {
        if (nulls[a] && nulls[b]) return false;
        if (nulls[a]) return ascending;
        if (nulls[b]) return !ascending;
        return ascending ? (values[a] < values[b]) : (values[b] < values[a]);
    });

    // apply permutation
    std::vector<std::pair<unsigned int, unsigned int>> new_indexes(n);
    for (unsigned int i = 0; i < n; ++i)
        new_indexes[i] = row_indexes[perm[i]];
    row_indexes = std::move(new_indexes);
}

void dispatchTypedSort(
    std::vector<std::pair<unsigned int, unsigned int>>& row_indexes,
    const std::map<unsigned int, std::string>& number_to_dbcont,
    const std::map<std::string, std::shared_ptr<Buffer>>& buffers,
    const std::map<unsigned int, std::string>& dbcont_num_to_var_name,
    PropertyDataType dt, bool ascending)
{
    switch (dt)
    {
    case PropertyDataType::BOOL:      typedSortPairs<bool>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::CHAR:      typedSortPairs<char>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::UCHAR:     typedSortPairs<unsigned char>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::INT:       typedSortPairs<int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::UINT:      typedSortPairs<unsigned int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::LONGINT:   typedSortPairs<long int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::ULONGINT:  typedSortPairs<unsigned long int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::FLOAT:     typedSortPairs<float>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::DOUBLE:    typedSortPairs<double>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::STRING:    typedSortPairs<std::string>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::JSON:      typedSortPairs<nlohmann::json>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    case PropertyDataType::TIMESTAMP: typedSortPairs<boost::posix_time::ptime>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending); break;
    default: break;
    }
}

} // anonymous namespace

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

    DBContentManager& manager = view_.compass().dbContentManager();

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

    beginCustomResetModel();

    row_indexes_.clear();
    buffers_.clear();

    endCustomResetModel();
}

void AllBufferTableModel::setData(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    beginCustomResetModel();

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

    buildRowIndexes();
    sortRowIndexes();

    endCustomResetModel();
}

void AllBufferTableModel::buildRowIndexes()
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    row_indexes_.clear();

    // count total records to reserve vector capacity
    unsigned int total_size = 0;
    for (auto& buf_it : buffers_)
        total_size += buf_it.second->size();

    // build (timestamp, dbcont_num, buffer_index) entries in a flat vector
    using TimedEntry = std::pair<boost::posix_time::ptime, std::pair<unsigned int, unsigned int>>;
    std::vector<TimedEntry> timed_entries;
    timed_entries.reserve(total_size);

    DBContentManager& dbcont_man = view_.compass().dbContentManager();

    for (auto& buf_it : buffers_)
    {
        if (view_.settings().ignore_non_target_reports_
            && !dbcont_man.metaCanGetVariable(buf_it.first, dbcontent_vars::meta_var_latitude_))
            continue;

        const std::string& dbcontent_name = buf_it.first;

        traced_assert(dbcont_to_number_.count(dbcontent_name) == 1);
        unsigned int dbcont_num = dbcont_to_number_.at(dbcontent_name);

        unsigned int buffer_size = buf_it.second->size();
        if (buffer_size == 0)
            continue;

        const dbContent::Variable& ts_var =
                dbcont_man.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).getFor(dbcontent_name);

        traced_assert(buf_it.second->has<boost::posix_time::ptime>(ts_var.name()));
        NullableVector<boost::posix_time::ptime>& ts_vec =
            buf_it.second->get<boost::posix_time::ptime>(ts_var.name());

        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        unsigned int num_time_none = 0;

        for (unsigned int buffer_index = 0; buffer_index < buffer_size; ++buffer_index)
        {
            if (view_.settings().show_only_selected_)
            {
                if (selected_vec.isNull(buffer_index) || !selected_vec.get(buffer_index))
                    continue;
            }

            boost::posix_time::ptime ts;
            if (ts_vec.isNull(buffer_index))
            {
                ts = boost::posix_time::ptime(boost::posix_time::not_a_date_time);
                num_time_none++;
            }
            else
                ts = ts_vec.get(buffer_index);

            timed_entries.emplace_back(ts, std::make_pair(dbcont_num, buffer_index));
        }

        if (num_time_none)
            loginf << dbcontent_name << " skipped " << num_time_none << " indexes with no time";
    }

    // sort by timestamp (stable_sort preserves insertion order for equal timestamps)
    std::stable_sort(timed_entries.begin(), timed_entries.end(),
        [](const TimedEntry& a, const TimedEntry& b)
        {
            return a.first < b.first;
        });

    // extract row index map (strip timestamps)
    row_indexes_.resize(timed_entries.size());
    for (unsigned int i = 0; i < timed_entries.size(); ++i)
        row_indexes_[i] = timed_entries[i].second;

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "built " << row_indexes_.size() << " row indexes in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);
}

void AllBufferTableModel::rebuild()
{
    beginCustomResetModel();

    buildRowIndexes();
    sortRowIndexes();

    endCustomResetModel();
}

void AllBufferTableModel::applyRowPermutation(const std::vector<unsigned int>& perm)
{
    std::vector<std::pair<unsigned int, unsigned int>> new_indexes(perm.size());
    for (unsigned int i = 0; i < perm.size(); ++i)
        new_indexes[i] = row_indexes_[perm[i]];
    row_indexes_ = std::move(new_indexes);
}

void AllBufferTableModel::sortRowIndexes()
{
    if (sort_column_ < 0 || row_indexes_.empty())
        return;

    unsigned int col = static_cast<unsigned int>(sort_column_);
    bool ascending = (sort_order_ == Qt::AscendingOrder);

    if (col == 0)  // checkbox column
    {
        std::map<unsigned int, std::string> var_names;
        for (const auto& p : number_to_dbcont_)
            var_names[p.first] = dbcontent_vars::selected_var_.name();

        typedSortPairs<bool>(row_indexes_, number_to_dbcont_, buffers_, var_names, ascending);
        return;
    }

    if (col == 1)  // DBContent name column
    {
        unsigned int n = row_indexes_.size();

        std::vector<std::string> keys(n);
        for (unsigned int i = 0; i < n; ++i)
            keys[i] = number_to_dbcont_.at(row_indexes_[i].first);

        std::vector<unsigned int> perm(n);
        std::iota(perm.begin(), perm.end(), 0);

        std::stable_sort(perm.begin(), perm.end(), [&](unsigned int a, unsigned int b)
        {
            return ascending ? (keys[a] < keys[b]) : (keys[b] < keys[a]);
        });

        applyRowPermutation(perm);
        return;
    }

    // data column — resolve variable per DBContent, dispatch on type
    unsigned int data_col = col - prefixColumnCount();
    std::map<unsigned int, std::string> dbcont_num_to_var_name;
    PropertyDataType dt = PropertyDataType::BOOL;  // will be overwritten
    bool have_type = false;

    for (const auto& p : number_to_dbcont_)
    {
        dbContent::Variable* var = nullptr;
        if (resolveVariable(data_col, p.second, var) && var &&
            buffers_.count(p.second) && buffers_.at(p.second)->properties().hasProperty(var->name()))
        {
            dbcont_num_to_var_name[p.first] = var->name();
            if (!have_type)
            {
                dt = var->dataType();
                have_type = true;
            }
        }
    }

    if (have_type)
        dispatchTypedSort(row_indexes_, number_to_dbcont_, buffers_, dbcont_num_to_var_name, dt, ascending);
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

    view_.compass().jobManager().addBlockingJob(export_job_);
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

        traced_assert(buffer->has<bool>(dbcontent_vars::selected_var_.name()));
        if (buffer->get<bool>(dbcontent_vars::selected_var_.name()).isNull(buffer_index))
            continue;

        if (buffer->get<bool>(dbcontent_vars::selected_var_.name()).get(buffer_index))
        {
            if (first_row == -1)
                first_row = cnt;

            last_row = cnt;
        }
    }

    return std::make_pair(first_row, last_row);
}

