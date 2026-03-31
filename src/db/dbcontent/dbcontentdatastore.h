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

#include <QObject>

class DBContentManager;
class Buffer;

namespace dbContent
{
class DBContentAccessor;
class TargetReportAccessor;
} 

/**
 * Snapshot/cache of the currently loaded surveillance dataset.
 *
 * Holds the in-memory Buffer objects for every DBContent type present in the
 * current dataset (e.g. CAT048, CAT062), a DBContentAccessor for typed field
 * access, and a pre-built three-level index over all buffer rows:
 *
 *   DBContent ID  ->  Data Source ID  ->  Line ID  ->  buffer row indices
 *
 * The index allows callers to obtain the subset of row indices belonging to a
 * particular DBContent type, data source, or combination thereof without
 * re-scanning the underlying Buffer columns.
 *
 * Call update() after every dataset load/change to rebuild buffers, accessor,
 * and indices from the current DBContentManager state.
 */
class DBContentDataStore : public QObject
{ 
    Q_OBJECT

signals:
    void dataResetSignal();               // emitted when the data store has been reset
    void dataChangedSignal(unsigned int); // emitted when the data for a certain dbcontent has changed
    void dataRefreshedSignal();           // emitted when refreshing the data store finished

public:
    typedef std::vector<unsigned int>                              BufferIndices;
    typedef std::map<unsigned int, std::shared_ptr<BufferIndices>> LineIDMap;
    typedef std::map<unsigned int, LineIDMap>                      DSIDMap;
    typedef std::map<unsigned int, DSIDMap>                        DBContentMap;
    typedef std::map<unsigned int, std::shared_ptr<Buffer>>        BufferMap;

    DBContentDataStore(DBContentManager& dbc_manager);
    virtual ~DBContentDataStore();

    DBContentManager& dbcManager() { return dbc_manager_; }
    const DBContentManager& dbcManager() const { return dbc_manager_; }
    const DBContentMap& indices() const { return indices_; }
    const dbContent::DBContentAccessor& accessor() const { return *accessor_; }
    const BufferMap& buffers() const { return buffers_; }

    std::unique_ptr<dbContent::DBContentAccessor> createAccessor() const;

    BufferIndices indicesForDBContent(unsigned int dbc_id) const;
    BufferIndices indicesForDSID(unsigned int ds_id) const;
    BufferIndices indicesForDSID(unsigned int dbc_id, unsigned int ds_id) const;

    std::shared_ptr<Buffer> buffer(unsigned int dbc_id) const;
    std::shared_ptr<dbContent::TargetReportAccessor> targetReportAccessor(unsigned int dbc_id) const;

    void reset();
    void update(const std::vector<std::string>& dbc_names);
    void update();

private:
    void update(const std::string& dbc_name, 
                const std::shared_ptr<Buffer>& buffer,
                bool notify);
    
    DBContentManager& dbc_manager_;

    BufferMap                                     buffers_;  // current per dbcontent data buffers
    std::unique_ptr<dbContent::DBContentAccessor> accessor_; // dbcontent accessor for internal usage
    DBContentMap                                  indices_;  // current structured indices dbcontent -> ds id -> line id -> buffer indices
};
