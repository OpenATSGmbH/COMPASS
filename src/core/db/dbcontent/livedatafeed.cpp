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

#include "livedatafeed.h"
#include "dbcontentmanager.h"
#include "dbcontent.h"
#include "buffer.h"
#include "buffer_utils.h"
#include "compass.h"
#include "ifilterprovider.h"
#include "db_context_manager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "base/idbvariableresolver.h"
#include "timeconv.h"
#include "tbbhack.h"
#include "logger.h"

#include <algorithm>

using namespace std;
using namespace Utils;
using namespace dbContent;

/**
 */
LiveDataFeed::LiveDataFeed(DBContentManager& dbcont_man)
:   DBContentDataSet(dbcont_man)
{
}

/**
 */
LiveDataFeed::~LiveDataFeed() = default;

/**
 */
void LiveDataFeed::setReadSetProvider(std::function<dbContent::VariableSet(const std::string&)> provider)
{
    read_set_provider_ = std::move(provider);
}

/**
 * Stages newly inserted buffers: each is reduced to the live view read set (see transformInserted()) and
 * kept as a batch with its newest timestamp until the next tick merges it. Records the latency of the
 * insert (now - oldest target report) and drops batches which are entirely older than the live window.
 */
void LiveDataFeed::addInserted(BufferMap inserted)
{
    transformInserted(inserted);

    boost::optional<boost::posix_time::ptime> min_tr_time;

    for (auto& buf_it : inserted)
    {
        const string& name = buf_it.first;

        StagedBatch batch;
        batch.buffer = buf_it.second;

        auto& ts_meta_var = dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name());

        if (ts_meta_var.existsIn(name))
        {
            Variable& ts_var = ts_meta_var.getFor(name);

            // columns carry variable names after the transform
            if (batch.buffer->has<boost::posix_time::ptime>(ts_var.name()))
            {
                bool has_min_max;
                boost::posix_time::ptime ts_min, ts_max;
                tie(has_min_max, ts_min, ts_max) =
                    batch.buffer->get<boost::posix_time::ptime>(ts_var.name()).minMaxValues();

                if (has_min_max)
                {
                    batch.ts_max = ts_max;

                    if (dbcont_man_.dbContent(name).containsTargetReports())
                        min_tr_time = min_tr_time ? std::min(*min_tr_time, ts_min) : ts_min;
                }
            }
        }

        staged_[ name ].push_back(batch);
    }

    if (min_tr_time)
    {
        auto latency = Time::currentUTCTime() - *min_tr_time;

        if (!pending_latency_ || latency > *pending_latency_)
            pending_latency_ = latency;
    }

    dropOutdatedBatches();
}

/**
 * Reduces wide decoded inserts to the live view read set (incl. the assoc column), renames the DB columns
 * to variable names and adds the selection flag. Runs in parallel per content.
 */
void LiveDataFeed::transformInserted(BufferMap& inserted)
{
    traced_assert(read_set_provider_);

    struct InsertedItem
    {
        std::shared_ptr<Buffer>   buffer;
        VariableSet               read_set;
        boost::optional<Property> utn_prop; // assoc column, ensured before the transform
    };

    // resolve everything that reads the view read sets / variable model here, on the calling
    // thread - the parallel phase below must touch buffers only
    std::vector<InsertedItem> items;
    items.reserve(inserted.size());

    for (auto& buf_it : inserted)
    {
        const string& name = buf_it.first;

        items.emplace_back();
        InsertedItem& item = items.back();

        item.buffer = buf_it.second;

        // prune target: the live view read set (incl. standard/CAT063 vars) from ViewManager
        item.read_set = read_set_provider_(name);

        if (dbcont_man_.metaCanGetVariable(name, dbcontent_vars::meta_var_utn_))
        {
            Variable& utn_var = dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_utn_);
            item.utn_prop = Property(utn_var.dbColumnName(), utn_var.dataType());
        }
    }

    // per-content work on disjoint buffers
    tbb::parallel_for(uint(0), (unsigned int) items.size(), [&](unsigned int item_cnt)
    {
        InsertedItem& item   = items[item_cnt];
        Buffer&       buffer = *item.buffer;

        // remove all columns not in the read set
        vector<Property> to_remove;
        for (const auto& prop_it : buffer.properties().properties())
            if (!item.read_set.hasDBColumnName(prop_it.name()))
                to_remove.push_back(prop_it);

        for (auto& prop_it : to_remove)
            buffer.deleteProperty(prop_it);

        // ensure the assoc (utn) column exists so it survives the transform
        if (item.utn_prop && !buffer.hasProperty(*item.utn_prop))
            buffer.addProperty(*item.utn_prop);

        // rename DB columns to variable names
        buffer_utils::transformVariables(buffer, item.read_set, true);

        // add selection flag
        buffer.addProperty(dbcontent_vars::selected_var_);
    });
}

/**
 * Drops staged batches whose newest timestamp is outside the live cache window - the merge would cut
 * all their rows anyway. Batches without timestamps are kept.
 * Notice: Only matters while ticks are held, otherwise a single fresh batch is staged.
 */
void LiveDataFeed::dropOutdatedBatches()
{
    boost::posix_time::ptime min_ts =
        Time::currentUTCTime() - boost::posix_time::minutes(dbcont_man_.compass().maxLiveDataAgeCache());

    size_t num_dropped = 0;

    for (auto& [ name, batches ] : staged_)
    {
        size_t n = batches.size();

        batches.erase(std::remove_if(batches.begin(), batches.end(),
                                     [ & ] (const StagedBatch& b) { return b.ts_max && *b.ts_max <= min_ts; }),
                      batches.end());

        num_dropped += n - batches.size();
    }

    if (num_dropped)
        logdbg << "dropped " << num_dropped << " outdated staged batch(es)";
}

/**
 * Returns the max latency of the inserts staged since the last call and resets it.
 * Notice: Taken per tick attempt, so it does not grow with ticks held during an overload.
 */
boost::optional<boost::posix_time::time_duration> LiveDataFeed::takeInsertLatency()
{
    latency_         = pending_latency_;
    pending_latency_ = boost::none;

    return latency_;
}

/**
 */
boost::posix_time::time_duration LiveDataFeed::latency() const
{
    traced_assert(latency_);
    return *latency_;
}

/**
 * Initialises the live cache from an existing dataset (the primed load) so the
 * first tick accumulates onto the recent history rather than starting empty.
 */
void LiveDataFeed::seedFrom(const BufferMap& source)
{
    buffers_ = source;
    invalidateIndex();
}

/**
 */
void LiveDataFeed::clear()
{
    buffers_.clear();
    staged_.clear();
    pending_latency_ = boost::none;
    latency_         = boost::none;
    invalidateIndex();
}

/**
 * One live tick, in memory only: merge staged buffers, trim the live window, apply
 * data-source and view filters, then emit a single atomic change event so listeners
 * reset + rebuild + finalize in one event-loop turn.
 * Notice: The only place where distributed buffers change, always followed by the synchronous
 * distribution (ticks are held by the LiveController instead of being skipped by views).
 */
void LiveDataFeed::processTick()
{
    bool had_data = !buffers_.empty();

    addStagedToCache();
    cutCachedData();
    filterDataSources();

    if (dbcont_man_.compass().filterProvider().useFilters())
        dbcont_man_.compass().filterProvider().filterBuffers(buffers_);

    invalidateIndex();

    if (!buffers_.empty())
    {
        std::vector<std::string> names;
        names.reserve(buffers_.size());
        for (const auto& buf_it : buffers_)
            names.push_back(buf_it.first);

        emitChanged(names, /*reset=*/true, /*last=*/true);
    }
    else if (had_data)
    {
        emitChanged({}, /*reset=*/true, /*last=*/true);
    }
}

/**
 * Merges the staged batches (already reduced, see transformInserted()) into the cache:
 * seize them into buffers_ and sort by timestamp.
 */
void LiveDataFeed::addStagedToCache()
{
    struct StagedItem
    {
        std::string                          name;
        std::vector<std::shared_ptr<Buffer>> batches;
        boost::optional<Property>            ts_prop; // sort key (variable name)
    };

    // resolve the variable model here, on the calling thread - the parallel phase below must
    // touch buffers only
    std::vector<StagedItem> items;
    items.reserve(staged_.size());

    for (auto& [ name, batches ] : staged_)
    {
        if (batches.empty()) // all dropped as outdated
            continue;

        items.emplace_back();
        StagedItem& item = items.back();

        item.name = name;

        item.batches.reserve(batches.size());
        for (auto& batch : batches)
            item.batches.push_back(batch.buffer);

        traced_assert(dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).existsIn(name));
        Variable& ts_var = dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).getFor(name);
        item.ts_prop = Property(ts_var.name(), ts_var.dataType());
    }

    // make the cache map structurally final before any worker runs: a std::map insert
    // rebalances the tree, so inserting from a worker would race the unlocked lookups the
    // other workers do. A content not yet cached adopts its first staged batch as the cache
    // buffer (mutated in place below).
    for (auto& item : items)
    {
        if (!buffers_.count(item.name))
            buffers_[item.name] = item.batches.front();
    }

    // per-content work on disjoint buffers; buffers_ is only read from here on
    tbb::parallel_for(uint(0), (unsigned int) items.size(), [&](unsigned int item_cnt)
    {
        StagedItem& item   = items[item_cnt];
        auto&       cached = buffers_.at(item.name);

        for (auto& batch : item.batches)
        {
            if (batch != cached) // an adopted batch is the cache buffer itself
                cached->seizeBuffer(*batch);
        }

        // sort by timestamp
        traced_assert(cached->hasProperty(*item.ts_prop));
        cached->sortByProperty(*item.ts_prop);
    });

    staged_.clear();
}

/**
 * Drops rows older than the live cache window from every cached buffer.
 */
void LiveDataFeed::cutCachedData()
{
    boost::posix_time::ptime min_ts =
        Time::currentUTCTime() - boost::posix_time::minutes(dbcont_man_.compass().maxLiveDataAgeCache());

    for (auto& buf_it : buffers_)
    {
        unsigned int buffer_size = buf_it.second->size();

        traced_assert(dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).existsIn(buf_it.first));
        Variable& ts_var = dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).getFor(buf_it.first);
        Property ts_prop {ts_var.name(), ts_var.dataType()};

        if (!buf_it.second->hasProperty(ts_prop))
        {
            logwrn << "buffer " << buf_it.first << " has no timestamp for cutoff";
            continue;
        }

        NullableVector<boost::posix_time::ptime>& ts_vec =
            buf_it.second->get<boost::posix_time::ptime>(ts_var.name());

        unsigned int index = 0;
        for (; index < buffer_size; ++index)
            if (!ts_vec.isNull(index) && ts_vec.get(index) > min_ts)
                break;

        if (index) // cut at the row before the first kept one
        {
            index--;
            traced_assert(index < buffer_size);
            buf_it.second->cutUpToIndex(index);
        }
    }

    // remove empty buffers
    BufferMap tmp = buffers_;
    for (auto& buf_it : tmp)
        if (!buf_it.second->size())
            buffers_.erase(buf_it.first);
}

/**
 * Removes rows belonging to unwanted data sources / lines from every buffer.
 */
void LiveDataFeed::filterDataSources()
{
    auto& ctx_man = dbcont_man_.compass().dbContextManager();

    unsigned int num_buffers = buffers_.size();

    tbb::parallel_for(uint(0), num_buffers, [&](unsigned int buffer_cnt)
    {
        auto buf_it = buffers_.begin();
        std::advance(buf_it, buffer_cnt);

        // the ds/line selection for this content, shared with the offline load path
        // (DBContextManager::loadingSelection). Computed once, tested per record.
        auto selection = ctx_man.loadingSelection(buf_it->first);

        if (!selection) // no constraint -> keep everything
            return;

        traced_assert(dbcont_man_.metaVariable(dbcontent_vars::meta_var_ds_id_.name()).existsIn(buf_it->first));
        traced_assert(dbcont_man_.metaVariable(dbcontent_vars::meta_var_line_id_.name()).existsIn(buf_it->first));

        Variable& ds_id_var   = dbcont_man_.metaVariable(dbcontent_vars::meta_var_ds_id_.name()).getFor(buf_it->first);
        Variable& line_id_var = dbcont_man_.metaVariable(dbcontent_vars::meta_var_line_id_.name()).getFor(buf_it->first);

        Property ds_id_prop {ds_id_var.name(), ds_id_var.dataType()};
        traced_assert(buf_it->second->hasProperty(ds_id_prop));

        NullableVector<unsigned int>& ds_id_vec   = buf_it->second->get<unsigned int>(ds_id_var.name());
        NullableVector<unsigned int>& line_id_vec = buf_it->second->get<unsigned int>(line_id_var.name());

        unsigned int buffer_size = buf_it->second->size();

        vector<unsigned int> indexes_to_remove;

        for (unsigned int index = 0; index < buffer_size; ++index)
        {
            traced_assert(!ds_id_vec.isNull(index));
            traced_assert(!line_id_vec.isNull(index));

            unsigned int ds_id   = ds_id_vec.get(index);
            unsigned int line_id = line_id_vec.get(index);

            // keep a record iff its data source is wanted and its line is among that
            // source's wanted lines (explicit set - same selection the offline load applies)
            auto ds_it = selection->find(ds_id);
            bool keep = ds_it != selection->end() && ds_it->second.count(line_id);

            if (!keep)
                indexes_to_remove.push_back(index);
        }

        if (indexes_to_remove.size())
            buf_it->second->removeIndexes(indexes_to_remove);
    });

    // remove empty buffers
    BufferMap tmp = buffers_;
    for (auto& buf_it : tmp)
        if (!buf_it.second->size())
            buffers_.erase(buf_it.first);
}
