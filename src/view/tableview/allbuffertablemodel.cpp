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
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "global.h"
#include "jobmanager.h"
#include "logger.h"
#include "number.h"
#include "stringconv.h"
#include "tableview.h"
#include "tableviewdatasource.h"
#include "viewdatawidget.h"

#include <QBrush>
#include <QPainter>
#include <QPen>
#include <QPixmap>

#include "json.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <algorithm>
#include <numeric>

namespace
{

/// Computes the sort permutation by extracting native-typed keys from multiple
/// buffers. Each row references a (dbcont_num, buffer_index) pair; the
/// NullableVector for each dbcont_num is resolved once via nvec_map to avoid
/// per-row map lookups. Returns a permutation perm such that
/// new[i] = old[perm[i]] - caller applies it (row_indexes_ + parallel arrays).
template <typename T>
std::vector<unsigned int> typedSortPerm(
    const std::vector<std::pair<unsigned int, unsigned int>>& row_indexes,
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

    std::vector<unsigned int> perm(n);
    std::iota(perm.begin(), perm.end(), 0);

    std::stable_sort(perm.begin(), perm.end(), [&](unsigned int a, unsigned int b)
    {
        if (nulls[a] && nulls[b]) return false;
        if (nulls[a]) return ascending;
        if (nulls[b]) return !ascending;
        return ascending ? (values[a] < values[b]) : (values[b] < values[a]);
    });

    return perm;
}

std::vector<unsigned int> dispatchTypedSortPerm(
    const std::vector<std::pair<unsigned int, unsigned int>>& row_indexes,
    const std::map<unsigned int, std::string>& number_to_dbcont,
    const std::map<std::string, std::shared_ptr<Buffer>>& buffers,
    const std::map<unsigned int, std::string>& dbcont_num_to_var_name,
    PropertyDataType dt, bool ascending)
{
    switch (dt)
    {
    case PropertyDataType::BOOL:      return typedSortPerm<bool>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::CHAR:      return typedSortPerm<char>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::UCHAR:     return typedSortPerm<unsigned char>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::INT:       return typedSortPerm<int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::UINT:      return typedSortPerm<unsigned int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::LONGINT:   return typedSortPerm<long int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::ULONGINT:  return typedSortPerm<unsigned long int>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::FLOAT:     return typedSortPerm<float>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::DOUBLE:    return typedSortPerm<double>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::STRING:    return typedSortPerm<std::string>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::JSON:      return typedSortPerm<nlohmann::json>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    case PropertyDataType::TIMESTAMP: return typedSortPerm<boost::posix_time::ptime>(row_indexes, number_to_dbcont, buffers, dbcont_num_to_var_name, ascending);
    default: return {};
    }
}

} // anonymous namespace

AllBufferTableModel::AllBufferTableModel(TableView& view, AllBufferTableWidget* table_widget,
                                         TableViewDataSource& data_source)
    : BaseBufferTableModel(view, table_widget, data_source)
{
}

AllBufferTableModel::~AllBufferTableModel() {}

void AllBufferTableModel::setChangedSlot()
{
    // Variable column layout depends on the data source's variable set; clear
    // the (data_col, dbcontent) -> Variable* cache so resolveVariable() picks
    // up the new mapping on next paint.
    variable_cache_.clear();
    BaseBufferTableModel::setChangedSlot();
}

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
    rd.row = (unsigned int)row;
    return rd;
}

unsigned int AllBufferTableModel::prefixColumnCount() const
{
    return 2;  // checkbox+icon (same cell) + DBContent name
}

unsigned int AllBufferTableModel::dataColumnCount() const
{
    return data_source_.getSet()->getSize();
}

QVariant AllBufferTableModel::prefixColumnData(unsigned int col, const RowData& row_data) const
{
    if (col == 0)  // checkbox+icon cell - checkbox via CheckStateRole, icon via
                   // DecorationRole; no DisplayRole text.
        return QVariant();
    if (col == 1)  // DBContent name column
        return QVariant(row_data.dbcontent_name.c_str());

    return QVariant();
}

QVariant AllBufferTableModel::prefixColumnDecoration(unsigned int col, const RowData& row_data) const
{
    if (col != 0)
        return QVariant();

    // Is the row selected? A null selected_var_ counts as not selected.
    bool selected = false;
    if (row_data.buffer &&
        row_data.buffer->has<bool>(dbcontent_vars::selected_var_.name()))
    {
        const auto& selected_vec = row_data.buffer->get<bool>(dbcontent_vars::selected_var_.name());
        if (!selected_vec.isNull(row_data.buffer_index) &&
            selected_vec.get(row_data.buffer_index))
            selected = true;
    }

    // Layer id was resolved once in buildRowIndexes() and stored in
    // row_layer_index_; the empty-string pool slot 0 means "unknown".
    static const std::string empty_layer;
    const std::string& layer_id =
        (row_data.row < row_layer_index_.size() &&
         row_layer_index_[row_data.row] < layer_id_pool_.size())
            ? layer_id_pool_[row_layer_index_[row_data.row]]
            : empty_layer;

    QIcon icon = iconFor(layer_id, selected);
    if (icon.isNull())
        return QVariant();
    return icon;
}

QVariant AllBufferTableModel::prefixColumnHeader(unsigned int col) const
{
    if (col == 0)
        return QString();
    if (col == 1)
        return QString("DBContent");

    return QVariant();
}

QIcon AllBufferTableModel::iconFor(const std::string& layer_id, bool selected) const
{
    auto cache_key = std::make_pair(layer_id, selected);
    auto it = icon_cache_.find(cache_key);
    if (it != icon_cache_.end())
        return it->second;

    QColor color;
    if (selected)
    {
        // Selection yellow wins over the layer's own color, even for layers
        // that would otherwise render blank (non-target-report) - so
        // selection is always visible.
        color = ViewDataWidget::ColorSelected;
    }
    else
    {
        auto layer_it = layer_colors_.find(layer_id);
        if (layer_it != layer_colors_.end())
            color = layer_it->second;
    }

    constexpr int w = 14;
    constexpr int h = 14;
    constexpr qreal radius = 3.0;

    QPixmap pixmap(w, h);
    pixmap.fill(Qt::transparent);

    if (color.isValid())
    {
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(Qt::darkGray, 1, Qt::SolidLine));
        p.setBrush(QBrush(color));
        QRectF r(0.5, 0.5, w - 1.0, h - 1.0);
        p.drawRoundedRect(r, radius, radius);
    }

    QIcon icon = color.isValid() ? QIcon(pixmap) : QIcon();
    icon_cache_[cache_key] = icon;
    return icon;
}

bool AllBufferTableModel::resolveVariable(unsigned int data_col,
                                          const std::string& dbcontent_name,
                                          dbContent::Variable*& out_var) const
{
    traced_assert(data_col < data_source_.getSet()->getSize());

    auto cache_key = std::make_pair(data_col, dbcontent_name);
    auto cache_it = variable_cache_.find(cache_key);
    if (cache_it != variable_cache_.end())
    {
        out_var = cache_it->second;
        return out_var != nullptr;
    }

    std::string variable_dbcontent_name, variable_name;
    std::tie(variable_dbcontent_name, variable_name) = data_source_.getSet()->variableDefinition(data_col);

    DBContentManager& manager = view_.compass().dbContentManager();

    dbContent::Variable* var = nullptr;
    if (variable_dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(manager.existsMetaVariable(variable_name));
        if (manager.metaVariable(variable_name).existsIn(dbcontent_name))
            var = &manager.metaVariable(variable_name).getFor(dbcontent_name);
    }
    else if (dbcontent_name == variable_dbcontent_name)
    {
        traced_assert(manager.existsDBContent(dbcontent_name));
        traced_assert(manager.dbContent(dbcontent_name).hasVariable(variable_name));
        var = &manager.dbContent(dbcontent_name).variable(variable_name);
    }

    variable_cache_[cache_key] = var;
    out_var = var;
    return var != nullptr;
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
    row_layer_index_.clear();
    layer_id_pool_.clear();
    variable_cache_.clear();
    buffers_.clear();

    endCustomResetModel();
}

void AllBufferTableModel::setData(std::map<std::string, std::shared_ptr<Buffer>> buffers)
{
    //synchronous path: snapshot, prepare and apply inline
    auto input = makePrepareInput(std::move(buffers), /*for_load*/ false);

    applyPrepared(prepareData(input));
}

/**
 * Snapshots everything prepareData() reads from the view/model/managers, so the build
 * can run on a worker thread. Main thread only; also extends the dbcontent numbering.
 * for_load drops the layer filter: the layer tree of the fresh load is not built yet
 * at snapshot time (the caller re-filters via rebuild() if layers turn out hidden).
 */
AllBufferTableModel::PrepareInput AllBufferTableModel::makePrepareInput(
    std::map<std::string, std::shared_ptr<Buffer>> buffers,
    bool for_load)
{
    PrepareInput input;

    for (auto& buf_it : buffers)
    {
        const std::string& dbcontent_name = buf_it.first;

        if (dbcont_to_number_.count(dbcontent_name) == 0)
        {
            unsigned int num = dbcont_to_number_.size();
            number_to_dbcont_[num] = dbcontent_name;
            dbcont_to_number_[dbcontent_name] = num;
        }
    }

    traced_assert(dbcont_to_number_.size() == number_to_dbcont_.size());

    input.buffers          = std::move(buffers);
    input.number_to_dbcont = number_to_dbcont_;
    input.dbcont_to_number = dbcont_to_number_;

    input.show_only_selected = view_.settings().show_only_selected_;

    // on the load path the selection ranges are prepared along (saves the extra scan
    // in selectSelectedRows); the layer filter is dropped there - the fresh load's
    // layer tree is not built yet at snapshot time
    input.compute_selected_ranges = for_load;

    if (!for_load)
        input.allowed_layer_ids = allowed_layer_ids_;

    DBContentManager& dbcont_man = view_.compass().dbContentManager();
    auto& ctx_mgr = view_.compass().dbContextManager();

    for (const auto& buf_it : input.buffers)
    {
        const std::string& dbcontent_name = buf_it.first;

        if (view_.settings().ignore_non_target_reports_
            && !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_))
        {
            input.skipped_contents.insert(dbcontent_name);
            continue;
        }

        input.timestamp_columns[ dbcontent_name ] =
            dbcont_man.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).getFor(dbcontent_name).name();

        if (dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_) &&
            dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_))
        {
            input.ds_id_columns[ dbcontent_name ] = dbcont_man.metaGetVariable(
                dbcontent_name, dbcontent_vars::meta_var_ds_id_).name();
            input.line_id_columns[ dbcontent_name ] = dbcont_man.metaGetVariable(
                dbcontent_name, dbcontent_vars::meta_var_line_id_).name();
        }
    }

    if (ctx_mgr.hasActiveContext())
    {
        for (const auto& [ds_id, ds] : ctx_mgr.activeContext().dataSources())
            input.data_sources[ ds_id ] = {ds.dsType(), ds.name()};
    }

    // active sort snapshot
    input.sort_column = sort_column_;
    input.sort_order  = sort_order_;

    if (sort_column_ >= 1)
    {
        if (sort_column_ == 1)  // DBContent name column
        {
            input.sort_by_dbcontent = true;
        }
        else
        {
            unsigned int data_col = (unsigned int)sort_column_ - prefixColumnCount();

            for (const auto& p : input.number_to_dbcont)
            {
                dbContent::Variable* var = nullptr;
                if (resolveVariable(data_col, p.second, var) && var &&
                    input.buffers.count(p.second) &&
                    input.buffers.at(p.second)->properties().hasProperty(var->name()))
                {
                    input.sort_var_names[ p.first ] = var->name();
                    if (!input.sort_var_valid)
                    {
                        input.sort_var_type  = var->dataType();
                        input.sort_var_valid = true;
                    }
                }
            }
        }
    }

    return input;
}

/**
 * Builds the model's row data from the snapshot input: row indexes with per-row layer
 * ids, ordered by timestamp (or the active sort), plus the selected row ranges. Pure
 * over the input - safe on a worker thread. Note the selection read: prepared on load
 * from loadingDone on, when the selection is stable until the load-done edge fires.
 */
AllBufferTableModel::PreparedData AllBufferTableModel::prepareData(const PrepareInput& input)
{
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    PreparedData prepared;
    prepared.buffers          = input.buffers;
    prepared.number_to_dbcont = input.number_to_dbcont;
    prepared.dbcont_to_number = input.dbcont_to_number;

    prepared.layer_id_pool.push_back("");  // index 0 = unknown / no layer

    // count total records to reserve vector capacity
    unsigned int total_size = 0;
    for (const auto& buf_it : input.buffers)
        total_size += buf_it.second->size();

    // (timestamp, dbcont_num, buffer_index, layer_id_index) per accepted row
    struct TimedEntry
    {
        boost::posix_time::ptime ts;
        unsigned int dbcont_num;
        unsigned int buffer_index;
        unsigned int layer_index;
    };
    std::vector<TimedEntry> timed_entries;
    timed_entries.reserve(total_size);

    const bool filter_enabled = input.allowed_layer_ids.has_value();

    // dedup pool across all buffers - typical datasets have a few dozen
    // distinct layer ids vs. millions of rows.
    std::map<std::string, unsigned int> layer_id_to_index;

    for (const auto& buf_it : input.buffers)
    {
        const std::string& dbcontent_name = buf_it.first;

        if (input.skipped_contents.count(dbcontent_name))
            continue;

        traced_assert(input.dbcont_to_number.count(dbcontent_name) == 1);
        unsigned int dbcont_num = input.dbcont_to_number.at(dbcontent_name);

        unsigned int buffer_size = buf_it.second->size();
        if (buffer_size == 0)
            continue;

        traced_assert(input.timestamp_columns.count(dbcontent_name));
        const std::string& ts_col = input.timestamp_columns.at(dbcontent_name);

        traced_assert(buf_it.second->has<boost::posix_time::ptime>(ts_col));
        const NullableVector<boost::posix_time::ptime>& ts_vec =
            buf_it.second->get<boost::posix_time::ptime>(ts_col);

        traced_assert(buf_it.second->has<bool>(dbcontent_vars::selected_var_.name()));
        const NullableVector<bool>& selected_vec =
            buf_it.second->get<bool>(dbcontent_vars::selected_var_.name());

        // Per-row layer id resolution - wire ds_id/line_id columns up once
        // per buffer.
        const NullableVector<unsigned int>* ds_ids_vec   = nullptr;
        const NullableVector<unsigned int>* line_ids_vec = nullptr;

        if (input.ds_id_columns.count(dbcontent_name) &&
            input.line_id_columns.count(dbcontent_name))
        {
            const std::string& ds_id_name   = input.ds_id_columns.at(dbcontent_name);
            const std::string& line_id_name = input.line_id_columns.at(dbcontent_name);

            if (buf_it.second->has<unsigned int>(ds_id_name) &&
                buf_it.second->has<unsigned int>(line_id_name))
            {
                ds_ids_vec   = &buf_it.second->get<unsigned int>(ds_id_name);
                line_ids_vec = &buf_it.second->get<unsigned int>(line_id_name);
            }
        }

        // Per-buffer cache (ds_id, line_id) -> pool index, avoids a
        // data source lookup + layer-id string build per row.
        std::map<std::pair<unsigned int, unsigned int>, unsigned int> ds_line_to_layer_idx;

        unsigned int num_time_none = 0;

        for (unsigned int buffer_index = 0; buffer_index < buffer_size; ++buffer_index)
        {
            if (input.show_only_selected)
            {
                if (selected_vec.isNull(buffer_index) || !selected_vec.get(buffer_index))
                    continue;
            }

            unsigned int layer_index = 0;  // 0 = unknown

            if (ds_ids_vec && line_ids_vec &&
                !ds_ids_vec->isNull(buffer_index) &&
                !line_ids_vec->isNull(buffer_index))
            {
                const unsigned int ds_id   = ds_ids_vec->get(buffer_index);
                const unsigned int line_id = line_ids_vec->get(buffer_index);

                const auto cache_key = std::make_pair(ds_id, line_id);
                auto cache_it = ds_line_to_layer_idx.find(cache_key);
                if (cache_it != ds_line_to_layer_idx.end())
                {
                    layer_index = cache_it->second;
                }
                else
                {
                    std::string ds_type, ds_name;

                    auto ds_it = input.data_sources.find(ds_id);
                    if (ds_it != input.data_sources.end())
                    {
                        ds_type = ds_it->second.first;
                        ds_name = ds_it->second.second;
                    }
                    else
                    {
                        ds_type = "Other";
                        ds_name = std::to_string(Utils::Number::sacFromDsId(ds_id))
                                + "/" + std::to_string(Utils::Number::sicFromDsId(ds_id));
                    }
                    std::string layer_id = ds_type + ":" + ds_name + ":"
                                         + Utils::String::lineStrFrom(line_id) + ":"
                                         + dbcontent_name;

                    auto pool_it = layer_id_to_index.find(layer_id);
                    if (pool_it == layer_id_to_index.end())
                    {
                        layer_index = (unsigned int)prepared.layer_id_pool.size();
                        layer_id_to_index[layer_id] = layer_index;
                        prepared.layer_id_pool.push_back(std::move(layer_id));
                    }
                    else
                    {
                        layer_index = pool_it->second;
                    }
                    ds_line_to_layer_idx[cache_key] = layer_index;
                }
            }

            if (filter_enabled)
            {
                // unknown layer (index 0) can't match anything
                if (layer_index == 0 ||
                    !input.allowed_layer_ids->count(prepared.layer_id_pool[layer_index]))
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

            timed_entries.push_back({ts, dbcont_num, buffer_index, layer_index});
        }

        if (num_time_none)
            loginf << dbcontent_name << " skipped " << num_time_none << " indexes with no time";
    }

    // Default ordering: timestamp. If a sort column is active, the sort below
    // overwrites this anyway - skip the work in that case.
    if (input.sort_column < 0)
    {
        std::stable_sort(timed_entries.begin(), timed_entries.end(),
            [](const TimedEntry& a, const TimedEntry& b)
            {
                return a.ts < b.ts;
            });
    }

    prepared.row_indexes.resize(timed_entries.size());
    prepared.row_layer_index.resize(timed_entries.size());
    for (unsigned int i = 0; i < timed_entries.size(); ++i)
    {
        prepared.row_indexes[i] = std::make_pair(timed_entries[i].dbcont_num,
                                                 timed_entries[i].buffer_index);
        prepared.row_layer_index[i] = timed_entries[i].layer_index;
    }

    // active sort (mirrors sortRowIndexes over the prepared rows)
    if (input.sort_column >= 1 && !prepared.row_indexes.empty())
    {
        bool ascending = (input.sort_order == Qt::AscendingOrder);

        std::vector<unsigned int> perm;

        if (input.sort_by_dbcontent)
        {
            unsigned int n = prepared.row_indexes.size();

            std::vector<std::string> keys(n);
            for (unsigned int i = 0; i < n; ++i)
                keys[i] = prepared.number_to_dbcont.at(prepared.row_indexes[i].first);

            perm.resize(n);
            std::iota(perm.begin(), perm.end(), 0);

            std::stable_sort(perm.begin(), perm.end(), [&](unsigned int a, unsigned int b)
            {
                return ascending ? (keys[a] < keys[b]) : (keys[b] < keys[a]);
            });
        }
        else if (input.sort_var_valid)
        {
            perm = dispatchTypedSortPerm(prepared.row_indexes, prepared.number_to_dbcont,
                                         prepared.buffers, input.sort_var_names,
                                         input.sort_var_type, ascending);
        }

        if (!perm.empty())
        {
            std::vector<std::pair<unsigned int, unsigned int>> new_indexes(perm.size());
            std::vector<unsigned int> new_layer(perm.size());
            for (unsigned int i = 0; i < perm.size(); ++i)
            {
                new_indexes[i] = prepared.row_indexes[perm[i]];
                new_layer[i]   = prepared.row_layer_index[perm[i]];
            }
            prepared.row_indexes     = std::move(new_indexes);
            prepared.row_layer_index = std::move(new_layer);
        }
    }

    // selected row ranges (as getSelectedRows over the prepared rows; load path only)
    if (input.compute_selected_ranges)
    {
        int run_start = -1;
        int run_end   = -1;

        for (unsigned int cnt = 0; cnt < prepared.row_indexes.size(); ++cnt)
        {
            const std::string& dbcontent_name =
                prepared.number_to_dbcont.at(prepared.row_indexes[cnt].first);
            const auto& buffer = prepared.buffers.at(dbcontent_name);

            const auto& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());

            unsigned int buffer_index = prepared.row_indexes[cnt].second;
            bool selected = !selected_vec.isNull(buffer_index) && selected_vec.get(buffer_index);

            if (selected)
            {
                if (run_start == -1)
                    run_start = run_end = (int)cnt;
                else if ((int)cnt == run_end + 1)
                    run_end = (int)cnt;
                else
                {
                    prepared.selected_ranges.emplace_back(run_start, run_end);
                    run_start = run_end = (int)cnt;
                }
            }
        }

        if (run_start != -1)
            prepared.selected_ranges.emplace_back(run_start, run_end);
    }

    boost::posix_time::ptime stop_time = boost::posix_time::microsec_clock::local_time();
    double elapsed_s = (stop_time - start_time).total_milliseconds() / 1000.0;

    loginf << "built " << prepared.row_indexes.size() << " row indexes in "
           << Utils::String::timeStringFromDouble(elapsed_s, true);

    return prepared;
}

/**
 * Commits prepared row data: model reset plus move into the members. Main thread only.
 */
void AllBufferTableModel::applyPrepared(PreparedData&& prepared)
{
    beginCustomResetModel();

    buffers_          = std::move(prepared.buffers);
    number_to_dbcont_ = std::move(prepared.number_to_dbcont);
    dbcont_to_number_ = std::move(prepared.dbcont_to_number);
    row_indexes_      = std::move(prepared.row_indexes);
    row_layer_index_  = std::move(prepared.row_layer_index);
    layer_id_pool_    = std::move(prepared.layer_id_pool);

    endCustomResetModel();
}

void AllBufferTableModel::rebuild()
{
    //synchronous path: snapshot, prepare and apply inline
    auto input = makePrepareInput(buffers_, /*for_load*/ false);

    applyPrepared(prepareData(input));
}

void AllBufferTableModel::setAllowedLayerIds(std::optional<std::set<std::string>> keys)
{
    allowed_layer_ids_ = std::move(keys);
}

void AllBufferTableModel::setLayerColors(std::map<std::string, QColor> layer_colors)
{
    layer_colors_ = std::move(layer_colors);
    icon_cache_.clear();

    // Refresh the icon column so color-mode changes / palette tweaks show up
    // without waiting for a full reload.
    if (!row_indexes_.empty())
    {
        QModelIndex tl = index(0, 0);
        QModelIndex br = index((int)row_indexes_.size() - 1, 0);
        emit dataChanged(tl, br, {Qt::DecorationRole});
    }
}

void AllBufferTableModel::applyRowPermutation(const std::vector<unsigned int>& perm)
{
    std::vector<std::pair<unsigned int, unsigned int>> new_indexes(perm.size());
    for (unsigned int i = 0; i < perm.size(); ++i)
        new_indexes[i] = row_indexes_[perm[i]];
    row_indexes_ = std::move(new_indexes);

    if (row_layer_index_.size() == perm.size())
    {
        std::vector<unsigned int> new_layer(perm.size());
        for (unsigned int i = 0; i < perm.size(); ++i)
            new_layer[i] = row_layer_index_[perm[i]];
        row_layer_index_ = std::move(new_layer);
    }
}

void AllBufferTableModel::sortRowIndexes()
{
    if (sort_column_ < 0 || row_indexes_.empty())
        return;

    unsigned int col = static_cast<unsigned int>(sort_column_);
    bool ascending = (sort_order_ == Qt::AscendingOrder);

    if (col == 0)  // checkbox+icon column - not sortable
        return;

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

    // data column - resolve variable per DBContent, dispatch on type
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
    {
        auto perm = dispatchTypedSortPerm(row_indexes_, number_to_dbcont_, buffers_,
                                          dbcont_num_to_var_name, dt, ascending);
        if (!perm.empty())
            applyRowPermutation(perm);
    }
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
    connect(export_job, &AllBufferCSVExportJob::doneSignal, this,
            &AllBufferTableModel::exportJobDoneSlot, Qt::QueuedConnection);

    view_.compass().jobManager().addBlockingJob(export_job_);
}

std::vector<std::pair<int,int>> AllBufferTableModel::getSelectedRows()
{
    loginf;

    // Collect the actually-selected rows as contiguous [first,last] runs. Returning a
    // single (min,max) span would make a sparse selection (e.g. a geo rectangle spread
    // across the time-sorted table) select every row in between - millions of rows -
    // which is both wrong and extremely slow to apply/render in Qt.
    std::vector<std::pair<int,int>> ranges;
    int run_start = -1;
    int run_end   = -1;

    for (unsigned int cnt = 0; cnt < row_indexes_.size(); ++cnt)
    {
        unsigned int dbcont_num = row_indexes_.at(cnt).first;
        unsigned int buffer_index = row_indexes_.at(cnt).second;

        traced_assert(number_to_dbcont_.count(dbcont_num) == 1);
        const std::string& dbcontent_name = number_to_dbcont_.at(dbcont_num);

        traced_assert(buffers_.count(dbcontent_name) == 1);

        std::shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);

        traced_assert(buffer->has<bool>(dbcontent_vars::selected_var_.name()));
        const auto& selected_vec = buffer->get<bool>(dbcontent_vars::selected_var_.name());

        bool selected = !selected_vec.isNull(buffer_index) && selected_vec.get(buffer_index);

        if (selected)
        {
            if (run_start == -1)                 // begin a new run
                run_start = run_end = (int)cnt;
            else if ((int)cnt == run_end + 1)    // extend the current run
                run_end = (int)cnt;
            else                                 // gap: close the run, start a new one
            {
                ranges.emplace_back(run_start, run_end);
                run_start = run_end = (int)cnt;
            }
        }
    }

    if (run_start != -1)
        ranges.emplace_back(run_start, run_end);

    return ranges;
}

