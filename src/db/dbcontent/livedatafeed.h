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
// insert path (addInserted); each processTick() merges staged buffers, trims the
// live window, applies data-source and view filters in memory, and emits one
// atomic change event. In-memory display only - no DB writes (those live in the
// engine). ViewManager owns it and supplies the read-set provider used to prune
// wide decoded inserts down to the columns the views need.
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

    void addInserted(BufferMap inserted); // stage newly inserted buffers
    void processTick();                   // merge + trim + filter -> one atomic emit

    void seedFrom(const BufferMap& source); // initialise the live cache (e.g. from the priming load)
    void clear();                           // drop all buffers/staged/latency (live exit)

    bool hasLatency() const { return (bool) latency_; }
    boost::posix_time::time_duration latency() const;

private:
    void addStagedToCache();  // prune/transform/select/merge/sort staged into buffers_
    void cutCachedData();     // drop rows older than the live cache window
    void filterDataSources(); // remove unwanted data sources / lines

    boost::optional<boost::posix_time::ptime> minTargetReportTime() const; // over staged_, for latency

    std::function<dbContent::VariableSet(const std::string&)> read_set_provider_;

    BufferMap staged_; // inserted buffers awaiting merge

    boost::optional<boost::posix_time::time_duration> latency_;
};
