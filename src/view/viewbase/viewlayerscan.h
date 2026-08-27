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

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class Buffer;
class COMPASS;

/**
 * Shared per-row layer scan over loaded buffers, used by the table and histogram views
 * for their layer panels and per-row layer resolution. Layer ids follow the common
 * "<ds_type>:<ds_name>:L<n>:<dbcontent>" scheme.
 *
 * The scan input is a snapshot taken on the main thread (resolved column names plus the
 * data source registry contents), so the scans themselves are worker-safe: they read
 * only the immutable buffers and the snapshot.
 */
namespace view_layer_scan
{

/// aggregate of one layer: identity split into parts plus the row count
struct LayerAgg
{
    std::string  ds_type;
    std::string  ds_name;
    std::string  line;
    std::string  dbcontent;
    unsigned int count      = 0;
    int          line_index = 0;   // 0..3, for color resolution
};

/// one buffer with its resolved ds id / line id column names
struct ScanBuffer
{
    std::shared_ptr<Buffer> buffer;
    std::string             dbcontent_name;
    std::string             ds_id_column;
    std::string             line_id_column;
};

/// snapshot input for the scans (main thread, see makeScanInput)
struct ScanInput
{
    std::vector<ScanBuffer> buffers; // only buffers with resolvable ds/line columns

    // known data sources: ds_id -> (ds type, ds name); unknown ids fall back to
    // "Other" plus SAC/SIC, computed purely from the id
    std::map<unsigned int, std::pair<std::string, std::string>> data_sources;
};

/// builds the scan input snapshot; main thread only (reads managers)
ScanInput makeScanInput(const std::map<std::string, std::shared_ptr<Buffer>>& data,
                        COMPASS& compass);

/// layer id "<ds_type>:<ds_name>:L<n>:<dbcontent>" aggregation per row; worker-safe
std::map<std::string, LayerAgg> aggregateLayers(const ScanInput& input);

/// per-dbcontent per-row layer id (empty string for unmappable rows); worker-safe.
/// dbcontents without ds/line columns get no entry at all.
std::map<std::string, std::vector<std::string>> computeRowLayerIds(const ScanInput& input);

/**
 * Per-row layer ids as a pool plus one index per row, instead of one string per row.
 * A dataset has a handful of layers and millions of rows, so the string form costs one
 * heap allocation per row (the ids exceed the small-string limit) and makes every
 * consumer compare strings per row. Here a row costs two bytes and consumers can
 * precompute whatever they need per pool entry.
 *
 * Index 0 is always the empty id, i.e. a row whose (ds_id, line_id) could not be mapped.
 * dbcontents without ds/line columns get no entry at all.
 */
struct RowLayerIndex
{
    std::vector<std::string> pool;                                 // pool[0] is empty
    std::map<std::string, std::vector<std::uint16_t>> rows;         // dbcontent -> per row pool index

    static const std::uint16_t UnmappedIndex = 0;

    /// pool index of a row, or UnmappedIndex when the dbcontent or row is not covered
    std::uint16_t indexOf(const std::string& dbcontent_name, unsigned int row) const
    {
        auto it = rows.find(dbcontent_name);
        if (it == rows.end() || row >= it->second.size())
            return UnmappedIndex;
        return it->second[ row ];
    }

    const std::string& id(std::uint16_t idx) const
    {
        static const std::string empty;
        return idx < pool.size() ? pool[ idx ] : empty;
    }
};

/// worker-safe; see RowLayerIndex
RowLayerIndex computeRowLayerIndex(const ScanInput& input);

} // namespace view_layer_scan
