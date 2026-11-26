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

#include "dbcontentdataset.h"
#include "dbcontent/variable/variableset.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

#include <functional>

class DBContentManager;

// Continuous live dataset (DBContentDataSet sibling of LoadOperation). Fed by the
// insert path (addInserted), which reduces each insert to the columns the views need
// and stages it as a batch; each processTick() merges the staged batches, trims the
// live window, applies data-source and view filters in memory, and emits one
// atomic change event. In-memory display only - no DB writes (those live in the
// engine). ViewManager owns it and supplies the read-set provider used to prune
// wide decoded inserts down to the columns the views need.
// Staged batches are private (never distributed), so they may pile up while ticks are
// held (live overload); batches entirely older than the live window are dropped.
class LiveDataFeed : public DBContentDataSet
{
    Q_OBJECT

public:
    LiveDataFeed(DBContentManager& dbcont_man);
    virtual ~LiveDataFeed();

    // live buffers contain what the inserts produced - sparse columns are legitimate
    bool fulfillsReadSet() const override { return false; }

    // Live view read set per content (from ViewManager); the prune target.
    void setReadSetProvider(std::function<dbContent::VariableSet(const std::string&)> provider);

    void addInserted(BufferMap inserted); // reduce + stage newly inserted buffers
    void processTick();                   // merge + trim + filter -> one atomic emit

    void seedFrom(const BufferMap& source); // initialise the live cache (e.g. from the priming load)
    void clear();                           // drop all buffers/staged/latency (live exit)

    // max latency of the inserts staged since the last call (once per tick attempt, also kept as latency())
    boost::optional<boost::posix_time::time_duration> takeInsertLatency();

    // latency taken by the last tick attempt
    bool hasLatency() const { return (bool) latency_; }
    boost::posix_time::time_duration latency() const;

private:
    struct StagedBatch
    {
        std::shared_ptr<Buffer>                   buffer;
        boost::optional<boost::posix_time::ptime> ts_max; // newest timestamp (none = no timestamps)
    };

    void transformInserted(BufferMap& inserted); // prune to read set, rename to variables, add selection flag
    void dropOutdatedBatches();                  // drop staged batches entirely older than the live window

    void addStagedToCache();  // merge/sort staged batches into buffers_
    void cutCachedData();     // drop rows older than the live cache window
    void filterDataSources(); // remove unwanted data sources / lines

    std::function<dbContent::VariableSet(const std::string&)> read_set_provider_;

    std::map<std::string, std::vector<StagedBatch>> staged_; // content -> reduced inserts awaiting merge

    boost::optional<boost::posix_time::time_duration> pending_latency_; // max latency of the inserts since the last take
    boost::optional<boost::posix_time::time_duration> latency_;         // latency taken by the last tick attempt
};
