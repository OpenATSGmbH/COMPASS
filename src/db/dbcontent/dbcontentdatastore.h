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
    // Emitted when buffers/indices have changed.
    // dbc_ids = ids of contents just rebuilt (may be empty).
    // reset   = if true, the listener must drop ALL prior state (including for
    //           contents NOT in dbc_ids) before processing dbc_ids - full
    //           dataset replacement semantics. The reset + rebuild happen in a
    //           single event-loop turn, so a queued listener sees them atomically
    //           with no visible empty intermediate state.
    // last    = if true, this is the final event of a logical batch (e.g. last
    //           content arrival of an offline load, or the single event of a
    //           live tick). Listeners should run heavy finalize work (e.g.
    //           rebuilding payloads, downstream visual refresh) only on last.
    void dataChangedSignal(const std::vector<unsigned int>& dbc_ids, bool reset, bool last);

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

    // Drops all internal state. Emits dataChangedSignal({}, true, last).
    // Pass last=true for standalone wipes (e.g. live "data went empty"); pass
    // last=false at the start of an offline load that will be followed by
    // per-content arrivals.
    void reset(bool last = true);

    // Incremental rebuild of the listed contents (drops their prior entries and
    // re-adds from manager.data()). Emits dataChangedSignal(those_ids, false, last).
    // The caller passes last=true for the final per-content arrival of an
    // offline load (so providers can finalize), false otherwise.
    void update(const std::vector<std::string>& dbc_names, bool last);

    // Full atomic rebuild from manager.data(). Drops all prior state, repopulates
    // for every content currently in the manager, emits ONE
    // dataChangedSignal(all_ids, true, true). Used by the live tick - the single
    // queued event ensures providers reset+rebuild+finalize in one event-loop
    // turn with no visible empty intermediate state.
    void update();

    // Synthetic finalize: emits dataChangedSignal({}, false, true). Used by the
    // loadingDoneSignal hook so providers finalize even when a load arrived no
    // new content (empty or cancelled offline load).
    void finalize();

private:
    void clearState();
    void rebuildContent(const std::string& dbc_name,
                        const std::shared_ptr<Buffer>& buffer);
    
    DBContentManager& dbc_manager_;

    BufferMap                                     buffers_;  // current per dbcontent data buffers
    std::unique_ptr<dbContent::DBContentAccessor> accessor_; // dbcontent accessor for internal usage
    DBContentMap                                  indices_;  // current structured indices dbcontent -> ds id -> line id -> buffer indices
};
