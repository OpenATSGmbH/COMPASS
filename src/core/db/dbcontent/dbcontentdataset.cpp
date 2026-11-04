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

#include "dbcontentdataset.h"
#include "dbcontentdataindex.h"
#include "dbcontentmanager.h"
#include "buffer.h"

/**
 */
DBContentDataSet::DBContentDataSet(DBContentManager& dbcont_man)
:   dbcont_man_(dbcont_man)
,   index_     (new DBContentDataIndex(dbcont_man))
{
}

/**
 */
DBContentDataSet::~DBContentDataSet() = default;

/**
 */
bool DBContentDataSet::empty() const
{
    for (const auto& buf_it : buffers_)
        if (buf_it.second && buf_it.second->size())
            return false;

    return true;
}

/**
 */
const DBContentDataIndex& DBContentDataSet::index() const
{
    ensureIndex();
    return *index_;
}

/**
 */
const dbContent::DBContentAccessor& DBContentDataSet::accessor() const
{
    ensureIndex();
    return index_->accessor();
}

/**
 * Loaded row counts (ds_id -> dbcontent -> line_id) taken directly from the
 * index bucket sizes - no per-row rescan.
 */
DBContentDataSet::LoadedCounts DBContentDataSet::loadedCounts() const
{
    ensureIndex();

    LoadedCounts result;

    for (const auto& [dbc_id, ds_map] : index_->indices())
    {
        const std::string& name = dbcont_man_.dbContentWithId(dbc_id);
        for (const auto& [ds_id, line_map] : ds_map)
            for (const auto& [line_id, indices] : line_map)
                result[ds_id][name][line_id] = static_cast<unsigned int>(indices->size());
    }

    return result;
}

/**
 * Stores/replaces one content's buffer and marks the index stale; the caller
 * emits the change signal once the batch is complete.
 */
void DBContentDataSet::setBuffer(const std::string& name, std::shared_ptr<Buffer> buffer)
{
    buffers_[name] = std::move(buffer);
    // incremental: only this content needs (re)indexing - a full rebuild would clear()
    // the shared accessor and dangle accessors already handed out for prior contents
    index_dirty_names_.push_back(name);
}

/**
 */
void DBContentDataSet::clearBuffers()
{
    buffers_.clear();
    index_full_dirty_ = true;
    index_dirty_names_.clear();
}

/**
 */
void DBContentDataSet::invalidateIndex()
{
    // in-place buffer mutation (e.g. the live tick) - the whole index must be rebuilt
    index_full_dirty_ = true;
    index_dirty_names_.clear();
}

/**
 */
void DBContentDataSet::emitChanged(const std::vector<std::string>& names, bool reset, bool last)
{
    emit dataChangedSignal(names, reset, last);
}

/**
 * Rebuilds the index from the current buffers on demand; kept consistent before
 * any consumer reads index()/accessor().
 */
void DBContentDataSet::ensureIndex() const
{
    if (index_full_dirty_)
    {
        // full rebuild (after clear / in-place mutation)
        index_->rebuild(buffers_);
        index_full_dirty_ = false;
        index_dirty_names_.clear();
    }
    else if (!index_dirty_names_.empty())
    {
        // incremental: (re)index only the changed contents, keeping prior contents'
        // accessors valid (see setBuffer)
        index_->update(buffers_, index_dirty_names_);
        index_dirty_names_.clear();
    }
}
