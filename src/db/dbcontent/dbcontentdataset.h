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

#include <QObject>

#include <map>
#include <memory>
#include <string>
#include <vector>

class DBContentManager;
class Buffer;
class DBContentDataEngine;
class DBContentDataIndex;

namespace dbContent
{
class DBContentAccessor;
}

// Base for a self-describing in-memory dataset: owns the DBContent buffer map
// plus a lazily-built index/accessor, and emits one change signal. Subtypes:
// LoadOperation (one-shot) and LiveDataFeed (continuous). Deliberately thin -
// no query state (spec/state/cancel) lives here.
class DBContentDataSet : public QObject
{
    Q_OBJECT

signals:
    // names = contents whose buffers changed; reset = drop all prior state;
    // last  = final event of a logical batch (listeners run finalize work).
    void dataChangedSignal(const std::vector<std::string>& names, bool reset, bool last);

public:
    typedef std::map<std::string, std::shared_ptr<Buffer>> BufferMap;
    // ds_id -> dbcontent name -> line_id -> loaded row count
    typedef std::map<unsigned int, std::map<std::string, std::map<unsigned int, unsigned int>>> LoadedCounts;

    DBContentDataSet(DBContentManager& dbcont_man);
    virtual ~DBContentDataSet();

    const BufferMap& buffers() const { return buffers_; }
    bool empty() const;

    const DBContentDataIndex& index() const;                 // lazily (re)built
    const dbContent::DBContentAccessor& accessor() const;    // via index

    LoadedCounts loadedCounts() const;                       // row counts from the index

protected:
    friend class DBContentDataEngine;

    DBContentManager& dbcont_man_;
    BufferMap         buffers_;

    void setBuffer(const std::string& name, std::shared_ptr<Buffer> buffer); // invalidates index
    void clearBuffers();
    void invalidateIndex(); // call after mutating buffers_ in place
    void emitChanged(const std::vector<std::string>& names, bool reset, bool last);

private:
    void ensureIndex() const;

    mutable std::unique_ptr<DBContentDataIndex> index_;
    mutable bool                     index_full_dirty_ {true}; // needs a full rebuild (after clear / in-place mutation)
    mutable std::vector<std::string> index_dirty_names_;       // contents to (re)index incrementally
};
