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

#include "dbcontentdataindex.h"

#include "dbcontentmanager.h"
#include "dbcontentaccessor.h"
#include "targetreportaccessor.h"
#include "buffer.h"

/**
 */
DBContentDataIndex::DBContentDataIndex(DBContentManager& dbcont_man)
:   dbcont_man_(dbcont_man)
,   accessor_  (new dbContent::DBContentAccessor(dbcont_man))
{
}

/**
 */
DBContentDataIndex::~DBContentDataIndex() = default;

/**
 * Drops all state and repopulates buffers, accessor and indices from the given
 * name->buffer map.
 */
void DBContentDataIndex::rebuild(const NamedBufferMap& buffers)
{
    clear();

    for (const auto& buf_it : buffers)
    {
        if (!buf_it.second)
            continue;

        rebuildContent(buf_it.first, buf_it.second);
    }
}

/**
 */
void DBContentDataIndex::clear()
{
    buffers_.clear();
    indices_.clear();
    accessor_->clear();
}

/**
 */
std::shared_ptr<Buffer> DBContentDataIndex::buffer(unsigned int dbc_id) const
{
    auto it = buffers_.find(dbc_id);
    if (it == buffers_.end())
        return nullptr;

    return it->second;
}

/**
 */
std::shared_ptr<dbContent::TargetReportAccessor> DBContentDataIndex::targetReportAccessor(unsigned int dbc_id) const
{
    auto dbc_name = dbcont_man_.dbContentWithId(dbc_id);
    traced_assert(accessor_->has(dbc_name));

    return accessor_->createTargetReportAccessor(dbc_name);
}

/**
 * All row indices for the given DBContent type, across all data sources and lines.
 */
DBContentDataIndex::BufferIndices DBContentDataIndex::indicesForDBContent(unsigned int dbc_id) const
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
 * All row indices for the given DBContent type and data source, across all lines.
 */
DBContentDataIndex::BufferIndices DBContentDataIndex::indicesForDSID(unsigned int dbc_id, unsigned int ds_id) const
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
 * All row indices for the given data source, across all DBContent types and lines.
 */
DBContentDataIndex::BufferIndices DBContentDataIndex::indicesForDSID(unsigned int ds_id) const
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
 * Registers one buffer with the accessor, stores it by id, and builds its
 * ds_id/line_id row index.
 */
void DBContentDataIndex::update(const NamedBufferMap& buffers, const std::vector<std::string>& names)
{
    for (const auto& name : names)
    {
        auto it = buffers.find(name);
        if (it == buffers.end() || !it->second)
            continue;

        const auto dbc_id = dbcont_man_.dbContentId(name);
        indices_.erase(dbc_id);            // drop this content's stale row indices
        rebuildContent(name, it->second);  // add/replace on the persistent accessor + rebuild its indices
    }
}

/**
 */
void DBContentDataIndex::rebuildContent(const std::string& dbc_name,
                                        const std::shared_ptr<Buffer>& buffer)
{
    traced_assert(buffer);

    const auto dbc_id = dbcont_man_.dbContentId(dbc_name);

    accessor_->add(dbc_name, buffer, true);
    buffers_[dbc_id] = buffer;

    auto& dbc_indices = indices_[dbc_id];

    auto tr_acc = accessor_->targetReportAccessor(dbc_name);

    auto n = tr_acc.size();
    for (unsigned int i = 0; i < n; ++i)
    {
        auto ds_id   = tr_acc.dsID(i);
        auto line_id = tr_acc.lineID(i);

        auto& idx_ptr = dbc_indices[ds_id][line_id];
        if (!idx_ptr)
            idx_ptr.reset(new BufferIndices());

        idx_ptr->push_back(i);
    }
}
