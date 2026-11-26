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

#include <map>
#include <vector>
#include <memory>
#include <string>

class DBContentManager;
class Buffer;

namespace dbContent
{
class DBContentAccessor;
class TargetReportAccessor;
}

// Row index + accessor projection over a DBContentDataSet's buffers:
//   DBContent id -> Data Source id -> Line id -> buffer row indices.
// Absorbed from the former DBContentDataStore; here it is a pure projection
// (rebuild() from a name->buffer map), owned/rebuilt by DBContentDataSet.
class DBContentDataIndex
{
public:
    typedef std::vector<unsigned int>                              BufferIndices;
    typedef std::map<unsigned int, std::shared_ptr<BufferIndices>> LineIDMap;
    typedef std::map<unsigned int, LineIDMap>                      DSIDMap;
    typedef std::map<unsigned int, DSIDMap>                        DBContentMap;
    typedef std::map<unsigned int, std::shared_ptr<Buffer>>        BufferMap;
    typedef std::map<std::string, std::shared_ptr<Buffer>>         NamedBufferMap;

    DBContentDataIndex(DBContentManager& dbcont_man);
    virtual ~DBContentDataIndex();

    // Drops all state and repopulates from the given name->buffer map.
    void rebuild(const NamedBufferMap& buffers);
    // Incrementally (re)index only the named contents: add/replace them on the
    // persistent accessor and leave the other contents' index entries - and the
    // accessors already handed out for them - valid. Use for per-content arrivals
    // during an incremental load (a full rebuild() would clear() the shared
    // accessor and dangle those prior accessors).
    void update(const NamedBufferMap& buffers, const std::vector<std::string>& names);
    void clear();

    const DBContentMap& indices() const { return indices_; }
    const dbContent::DBContentAccessor& accessor() const { return *accessor_; }

    std::shared_ptr<Buffer> buffer(unsigned int dbc_id) const;
    std::shared_ptr<dbContent::TargetReportAccessor> targetReportAccessor(unsigned int dbc_id) const;

    BufferIndices indicesForDBContent(unsigned int dbc_id) const;
    BufferIndices indicesForDSID(unsigned int ds_id) const;
    BufferIndices indicesForDSID(unsigned int dbc_id, unsigned int ds_id) const;

private:
    void rebuildContent(const std::string& dbc_name, const std::shared_ptr<Buffer>& buffer);

    DBContentManager& dbcont_man_;

    BufferMap                                     buffers_;  // dbc id -> buffer
    std::unique_ptr<dbContent::DBContentAccessor> accessor_;
    DBContentMap                                  indices_;  // dbc id -> ds id -> line id -> row indices
};
