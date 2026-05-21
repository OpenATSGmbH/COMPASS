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

#include "dbcontentdatastore.h"

#include "dbcontentmanager.h"
#include "dbcontentaccessor.h"
#include "targetreportaccessor.h"
#include "buffer.h"

/**
 * Initializes the store with an empty accessor; call update() after the
 * DBContentManager has loaded data to populate buffers and indices.
 */
DBContentDataStore::DBContentDataStore(DBContentManager& dbc_manager)
:   dbc_manager_(dbc_manager)
,   accessor_   (new dbContent::DBContentAccessor(dbc_manager))
{
    //if the dbc manager is done loading, emit a synthetic finalize event so
    //providers run their dataRefreshed work even when no new content arrived
    //(e.g. empty/cancelled offline load).
    connect(&dbc_manager_, &DBContentManager::loadingDoneSignal,
            this, &DBContentDataStore::finalize, Qt::ConnectionType::QueuedConnection);
}

/**
 */
DBContentDataStore::~DBContentDataStore() = default;

/**
 * Creates and returns a fresh DBContentAccessor populated with the current
 * buffer data. Callers take ownership of the returned accessor.
 */
std::unique_ptr<dbContent::DBContentAccessor> DBContentDataStore::createAccessor() const
{
    auto accessor = std::make_unique<dbContent::DBContentAccessor>(dbc_manager_);
    accessor->add(dbc_manager_.data());

    return accessor;
}

/**
 * Returns the Buffer for the given DBContent type, or nullptr if not present.
 */
std::shared_ptr<Buffer> DBContentDataStore::buffer(unsigned int dbc_id) const
{
    auto it = buffers_.find(dbc_id);
    if (it == buffers_.end())
        return nullptr;

    return it->second;
}

/**
 */
std::shared_ptr<dbContent::TargetReportAccessor> DBContentDataStore::targetReportAccessor(unsigned int dbc_id) const
{
    auto dbc_name = dbc_manager_.dbContentWithId(dbc_id);
    traced_assert(accessor_->has(dbc_name));
    
    return accessor_->createTargetReportAccessor(dbc_name);
}

/**
 * Returns all row indices that belong to the given DBContent type, aggregated
 * across all data sources and line IDs.
 */
DBContentDataStore::BufferIndices DBContentDataStore::indicesForDBContent(unsigned int dbc_id) const
{
    BufferIndices result;

    auto dbc_it = indices_.find(dbc_id);
    if (dbc_it == indices_.end())
        return result;

    for (const auto& [ds_id, line_map] : dbc_it->second)
        for (const auto& [line_id, indices] : line_map)
            result.insert(result.end(), indices->begin(), indices->end());

    return result;
}

/**
 * Returns all row indices that belong to the given DBContent type and data
 * source, aggregated across all line IDs.
 */
DBContentDataStore::BufferIndices DBContentDataStore::indicesForDSID(unsigned int dbc_id, unsigned int ds_id) const
{
    BufferIndices result;

    auto dbc_it = indices_.find(dbc_id);
    if (dbc_it == indices_.end())
        return result;

    auto ds_it = dbc_it->second.find(ds_id);
    if (ds_it == dbc_it->second.end())
        return result;

    for (const auto& [line_id, indices] : ds_it->second)
        result.insert(result.end(), indices->begin(), indices->end());

    return result;
}

/**
 * Returns all row indices that belong to the given data source, aggregated
 * across all DBContent types and line IDs.
 */
DBContentDataStore::BufferIndices DBContentDataStore::indicesForDSID(unsigned int ds_id) const
{
    BufferIndices result;

    for (const auto& [dbc_id, ds_map] : indices_)
    {
        auto ds_it = ds_map.find(ds_id);
        if (ds_it == ds_map.end())
            continue;

        for (const auto& [line_id, indices] : ds_it->second)
            result.insert(result.end(), indices->begin(), indices->end());
    }

    return result;
}

/**
 * Full atomic rebuild from manager.data(). Drops all prior state, repopulates
 * for every content currently in the manager, then emits ONE
 * dataChangedSignal(all_ids, reset=true, last=true). Used by the live tick:
 * the single queued event ensures the listener processes reset + rebuild +
 * finalize in one event-loop turn, with no visible empty intermediate state.
 */
void DBContentDataStore::update()
{
    clearState();

    std::vector<unsigned int> rebuilt_ids;
    rebuilt_ids.reserve(dbc_manager_.data().size());

    for (const auto& data_it : dbc_manager_.data())
    {
        rebuildContent(data_it.first, data_it.second);
        rebuilt_ids.push_back(dbc_manager_.dbContentId(data_it.first));
    }

    emit dataChangedSignal(rebuilt_ids, /*reset=*/true, /*last=*/true);
}

/**
 * Drops all buffers, indices, and accessor entries. Emits
 * dataChangedSignal({}, true, last).
 */
void DBContentDataStore::reset(bool last)
{
    clearState();

    emit dataChangedSignal({}, /*reset=*/true, last);
}

/**
 * Incrementally refreshes a subset of DBContent types from the manager's
 * current data, then emits ONE dataChangedSignal(those_ids, false, last).
 * Caller passes last=true on the final per-content arrival of an offline load
 * (so providers finalize), false otherwise.
 */
void DBContentDataStore::update(const std::vector<std::string>& dbc_names, bool last)
{
    const auto& data = dbc_manager_.data();

    std::vector<unsigned int> rebuilt_ids;
    rebuilt_ids.reserve(dbc_names.size());

    for (const auto& dbc_name : dbc_names)
    {
        traced_assert(data.count(dbc_name));
        rebuildContent(dbc_name, data.at(dbc_name));
        rebuilt_ids.push_back(dbc_manager_.dbContentId(dbc_name));
    }

    emit dataChangedSignal(rebuilt_ids, /*reset=*/false, last);
}

/**
 * Synthetic finalize: emits dataChangedSignal({}, false, true). Wired to the
 * manager's loadingDoneSignal so providers run their finalize work even when
 * an offline load arrived no new content (empty or cancelled).
 */
void DBContentDataStore::finalize()
{
    emit dataChangedSignal({}, /*reset=*/false, /*last=*/true);
}

/**
 * Clears all internal containers. No signal emitted.
 */
void DBContentDataStore::clearState()
{
    buffers_.clear();
    indices_.clear();
    accessor_->clear();
}

/**
 * Replaces the stored data for a single DBContent type: removes any existing
 * buffer, accessor entry, and index entries for dbc_name, then registers the
 * new buffer, rebuilds the accessor lookup, and repopulates the ds_id/line_id
 * index from scratch. Emits no signals; the calling update*() method batches
 * a single dataChangedSignal after rebuilding all requested contents.
 */
void DBContentDataStore::rebuildContent(const std::string& dbc_name,
                                        const std::shared_ptr<Buffer>& buffer)
{
    traced_assert(buffer);

    const auto dbc_id = dbc_manager_.dbContentId(dbc_name);

    //remove old existing content
    buffers_.erase(dbc_id);
    accessor_->removeDBContent(dbc_name);
    indices_.erase(dbc_id);

    //add to accessor
    accessor_->add(dbc_name, buffer, true);

    //add to buffer
    buffers_[ dbc_id ] = buffer;

    //add to index map
    auto& dbc_indices = indices_[ dbc_id ];

    auto tr_acc = accessor_->targetReportAccessor(dbc_name);

    auto n = tr_acc.size();
    for (unsigned int i = 0; i < n; ++i)
    {
        auto ds_id   = tr_acc.dsID(i);
        auto line_id = tr_acc.lineID(i);

        auto& idx_ptr = dbc_indices[ ds_id ][ line_id ];
        if (!idx_ptr)
            idx_ptr.reset(new BufferIndices());

        idx_ptr->push_back(i);
    }
}
