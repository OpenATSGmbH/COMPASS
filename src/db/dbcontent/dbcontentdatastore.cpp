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
    //if the dbc manager is done loading we are done updating too
    connect(&dbc_manager_, &DBContentManager::loadingDoneSignal, this, &DBContentDataStore::dataRefreshedSignal, Qt::ConnectionType::QueuedConnection);
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
 * Full rebuild: resets the store then updates every DBContent type currently
 * held by the manager. Emits dataChanged() once when done.
 */
void DBContentDataStore::update()
{
    reset();

    for (const auto& data_it : dbc_manager_.data())
        update(data_it.first, data_it.second, true);

    emit dataRefreshedSignal();
}

/**
 * Clears all buffers, indices, and the accessor, returning the store to an
 * empty state. Emits dataReset().
 */
void DBContentDataStore::reset()
{
    buffers_.clear();
    indices_.clear();
    accessor_->clear();

    emit dataResetSignal();
}

/**
 * Incrementally refreshes a subset of DBContent types from the manager's
 * current data. Calls the single-dbc update with notify=true for each name,
 * so dataChanged(dbc_id) is emitted per entry.
 */
void DBContentDataStore::update(const std::vector<std::string>& dbc_names)
{
    const auto& data = dbc_manager_.data();

    for (const auto& dbc_name : dbc_names)
    {
        traced_assert(data.count(dbc_name));
        const auto& buffer = data.at(dbc_name);

        update(dbc_name, buffer, true);
    }
}

/**
 * Replaces the stored data for a single DBContent type: removes any existing
 * buffer, accessor entry, and index entries for dbc_name, then registers the
 * new buffer, rebuilds the accessor lookup, and repopulates the ds_id/line_id
 * index from scratch. If notify is true, emits dataChanged(dbc_id) when done.
 */
void DBContentDataStore::update(const std::string& dbc_name,
                                const std::shared_ptr<Buffer>& buffer,
                                bool notify)
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

    if (notify)
        emit dataChangedSignal(dbc_id);
}
