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

#include "viewlayerscan.h"

#include "buffer.h"
#include "compass.h"
#include "data_source.h"
#include "db_context_manager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "number.h"
#include "stringconv.h"

#include <algorithm>
#include <cstdlib>

namespace view_layer_scan
{

namespace
{
    /// resolves ds type + name from the snapshot, with the shared "Other" + SAC/SIC
    /// fallback for unknown ids (pure, worker-safe)
    void resolveDataSource(const ScanInput& input,
                           unsigned int ds_id,
                           std::string& ds_type,
                           std::string& ds_name)
    {
        auto it = input.data_sources.find(ds_id);
        if (it != input.data_sources.end())
        {
            ds_type = it->second.first;
            ds_name = it->second.second;
            return;
        }

        ds_type = "Other";
        ds_name = std::to_string(Utils::Number::sacFromDsId(ds_id))
                + "/" + std::to_string(Utils::Number::sicFromDsId(ds_id));
    }
}

/**
 */
ScanInput makeScanInput(const std::map<std::string, std::shared_ptr<Buffer>>& data,
                        COMPASS& compass)
{
    ScanInput input;

    auto& dbcont_man = compass.dbContentManager();
    auto& ctx_mgr    = compass.dbContextManager();

    for (const auto& buf_it : data)
    {
        const std::string& dbcontent_name = buf_it.first;

        if (!buf_it.second || buf_it.second->size() == 0)
            continue;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_) ||
            !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_))
            continue;

        const std::string ds_id_name = dbcont_man.metaGetVariable(
            dbcontent_name, dbcontent_vars::meta_var_ds_id_).name();
        const std::string line_id_name = dbcont_man.metaGetVariable(
            dbcontent_name, dbcontent_vars::meta_var_line_id_).name();

        if (!buf_it.second->has<unsigned int>(ds_id_name) ||
            !buf_it.second->has<unsigned int>(line_id_name))
            continue;

        input.buffers.push_back({buf_it.second, dbcontent_name, ds_id_name, line_id_name});
    }

    if (ctx_mgr.hasActiveContext())
    {
        for (const auto& [ds_id, ds] : ctx_mgr.activeContext().dataSources())
            input.data_sources[ ds_id ] = {ds.dsType(), ds.name()};
    }

    return input;
}

/**
 */
std::map<std::string, LayerAgg> aggregateLayers(const ScanInput& input)
{
    std::map<std::string, LayerAgg> agg;

    for (const auto& scan_buf : input.buffers)
    {
        const auto& ds_ids   = scan_buf.buffer->get<unsigned int>(scan_buf.ds_id_column);
        const auto& line_ids = scan_buf.buffer->get<unsigned int>(scan_buf.line_id_column);

        const unsigned int n = scan_buf.buffer->size();
        for (unsigned int i = 0; i < n; ++i)
        {
            if (ds_ids.isNull(i) || line_ids.isNull(i))
                continue;

            const unsigned int ds_id   = ds_ids.get(i);
            const unsigned int line_id = line_ids.get(i);

            std::string ds_type, ds_name;
            resolveDataSource(input, ds_id, ds_type, ds_name);

            const std::string line_str = Utils::String::lineStrFrom(line_id);
            const std::string full_key = ds_type + ":" + ds_name + ":"
                                       + line_str + ":" + scan_buf.dbcontent_name;

            auto it = agg.find(full_key);
            if (it == agg.end())
            {
                LayerAgg a;
                a.ds_type    = ds_type;
                a.ds_name    = ds_name;
                a.line       = line_str;
                a.dbcontent  = scan_buf.dbcontent_name;
                a.count      = 1;
                a.line_index = (line_str.size() >= 2 && line_str[0] == 'L')
                             ? std::max(0, std::atoi(line_str.c_str() + 1) - 1)
                             : 0;
                agg.emplace(full_key, std::move(a));
            }
            else
            {
                it->second.count++;
            }
        }
    }

    return agg;
}

/**
 */
std::map<std::string, std::vector<std::string>> computeRowLayerIds(const ScanInput& input)
{
    std::map<std::string, std::vector<std::string>> row_layer_ids;

    for (const auto& scan_buf : input.buffers)
    {
        const auto& ds_ids   = scan_buf.buffer->get<unsigned int>(scan_buf.ds_id_column);
        const auto& line_ids = scan_buf.buffer->get<unsigned int>(scan_buf.line_id_column);

        const unsigned int n = scan_buf.buffer->size();

        std::vector<std::string> per_row(n);

        // Cache (ds_id, line_id) -> layer id to avoid a lookup + string build per row.
        std::map<std::pair<unsigned int, unsigned int>, std::string> layer_id_cache;

        for (unsigned int i = 0; i < n; ++i)
        {
            if (ds_ids.isNull(i) || line_ids.isNull(i))
                continue;   // leave empty -> unmappable

            const unsigned int ds_id   = ds_ids.get(i);
            const unsigned int line_id = line_ids.get(i);

            const auto cache_key = std::make_pair(ds_id, line_id);
            auto cache_it = layer_id_cache.find(cache_key);
            if (cache_it != layer_id_cache.end())
            {
                per_row[i] = cache_it->second;
                continue;
            }

            std::string ds_type, ds_name;
            resolveDataSource(input, ds_id, ds_type, ds_name);

            std::string lid = ds_type + ":" + ds_name + ":"
                            + Utils::String::lineStrFrom(line_id) + ":"
                            + scan_buf.dbcontent_name;

            layer_id_cache.emplace(cache_key, lid);
            per_row[i] = std::move(lid);
        }

        row_layer_ids.emplace(scan_buf.dbcontent_name, std::move(per_row));
    }

    return row_layer_ids;
}

/**
 */
RowLayerIndex computeRowLayerIndex(const ScanInput& input)
{
    RowLayerIndex result;

    result.pool.push_back(std::string()); // index 0 = unmapped

    std::map<std::string, std::uint16_t> id_to_index;

    for (const auto& scan_buf : input.buffers)
    {
        const auto& ds_ids   = scan_buf.buffer->get<unsigned int>(scan_buf.ds_id_column);
        const auto& line_ids = scan_buf.buffer->get<unsigned int>(scan_buf.line_id_column);

        const unsigned int n = scan_buf.buffer->size();

        std::vector<std::uint16_t> per_row(n, RowLayerIndex::UnmappedIndex);

        // (ds_id, line_id) -> pool index; a layer id is built once per distinct pair
        std::map<std::pair<unsigned int, unsigned int>, std::uint16_t> ds_line_to_index;

        for (unsigned int i = 0; i < n; ++i)
        {
            if (ds_ids.isNull(i) || line_ids.isNull(i))
                continue;   // stays unmapped

            const auto cache_key = std::make_pair(ds_ids.get(i), line_ids.get(i));

            auto cache_it = ds_line_to_index.find(cache_key);
            if (cache_it != ds_line_to_index.end())
            {
                per_row[i] = cache_it->second;
                continue;
            }

            std::string ds_type, ds_name;
            resolveDataSource(input, cache_key.first, ds_type, ds_name);

            std::string lid = ds_type + ":" + ds_name + ":"
                            + Utils::String::lineStrFrom(cache_key.second) + ":"
                            + scan_buf.dbcontent_name;

            std::uint16_t idx;

            auto pool_it = id_to_index.find(lid);
            if (pool_it != id_to_index.end())
            {
                idx = pool_it->second;
            }
            else
            {
                idx = (std::uint16_t)result.pool.size();
                id_to_index.emplace(lid, idx);
                result.pool.push_back(std::move(lid));
            }

            ds_line_to_index.emplace(cache_key, idx);
            per_row[i] = idx;
        }

        result.rows.emplace(scan_buf.dbcontent_name, std::move(per_row));
    }

    return result;
}

} // namespace view_layer_scan
