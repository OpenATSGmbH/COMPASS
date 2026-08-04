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
#include "filtermanager.h"
#include "db_context_manager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "base/idbvariableresolver.h"
#include "util/timeconv.h"
#include "util/tbbhack.h"
#include "logger.h"


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
 * Stages newly inserted buffers, accumulating with any not yet merged.
 */
void LiveDataFeed::addInserted(BufferMap inserted)
{
    for (auto& buf_it : inserted)
    {
        if (staged_.count(buf_it.first))
            staged_.at(buf_it.first)->seizeBuffer(*buf_it.second.get());
        else
            staged_[buf_it.first] = buf_it.second;
    }
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
    latency_ = boost::none;
    invalidateIndex();
}

/**
 * One live tick, in memory only: capture latency, merge staged buffers, trim the
 * live window, apply data-source and view filters, then emit a single atomic
 * change event so listeners reset + rebuild + finalize in one event-loop turn.
 */
void LiveDataFeed::processTick()
{
    auto min_tr_time = minTargetReportTime(); // over staged_, before it is consumed

    bool had_data = !buffers_.empty();

    addStagedToCache();
    cutCachedData();
    filterDataSources();

    if (dbcont_man_.compass().filterManager().useFilters())
        dbcont_man_.compass().filterManager().filterBuffers(buffers_);

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

    if (min_tr_time)
        latency_ = Time::currentUTCTime() - *min_tr_time;
    else
        latency_ = boost::none;
}

/**
 * Minimum target-report timestamp across the staged buffers, used to report the
 * live latency (now - oldest freshly-arrived target report).
 */
boost::optional<boost::posix_time::ptime> LiveDataFeed::minTargetReportTime() const
{
    boost::optional<boost::posix_time::ptime> min_time;

    for (const auto& buf_it : staged_)
    {
        const string& dbcont_name = buf_it.first;

        if (!dbcont_man_.dbContent(dbcont_name).containsTargetReports())
            continue;

        traced_assert(dbcont_man_.metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_));
        Variable& var = dbcont_man_.metaGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_);

        if (!buf_it.second->has<boost::posix_time::ptime>(var.dbColumnName()))
            continue;

        NullableVector<boost::posix_time::ptime>& data_vec =
            buf_it.second->get<boost::posix_time::ptime>(var.dbColumnName());

        bool has_min_max;
        boost::posix_time::ptime ts_min, ts_max;
        tie(has_min_max, ts_min, ts_max) = data_vec.minMaxValues();

        if (!has_min_max)
            continue;

        if (!min_time)
            min_time = ts_min;
        else
            min_time = std::min(*min_time, ts_min);
    }

    return min_time;
}

/**
 * Merges the staged buffers into the cache: prune each wide decoded buffer to the
 * live view read set, ensure the utn column, rename DB columns to variable names,
 * add the selection flag, seize into buffers_, and sort by timestamp.
 */
void LiveDataFeed::addStagedToCache()
{
    traced_assert(read_set_provider_);

    struct StagedItem
    {
        std::string               name;
        std::shared_ptr<Buffer>   buffer;
        VariableSet               read_set;
        boost::optional<Property> utn_prop; // assoc column, ensured before the transform
        boost::optional<Property> ts_prop;  // sort key (variable name, i.e. post-transform)
        bool                      merge {false};
    };

    // resolve everything that reads the view read sets / variable model here, on the calling
    // thread - the parallel phase below must touch buffers only
    std::vector<StagedItem> items;
    items.reserve(staged_.size());

    for (auto& buf_it : staged_)
    {
        const string& name = buf_it.first;

        items.emplace_back();
        StagedItem& item = items.back();

        item.name   = name;
        item.buffer = buf_it.second;

        // prune target: the live view read set (incl. standard/CAT063 vars) from ViewManager
        item.read_set = read_set_provider_(name);

        if (dbcont_man_.metaCanGetVariable(name, dbcontent_vars::meta_var_utn_))
        {
            Variable& utn_var = dbcont_man_.metaGetVariable(name, dbcontent_vars::meta_var_utn_);
            item.utn_prop = Property(utn_var.dbColumnName(), utn_var.dataType());
        }

        traced_assert(dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).existsIn(name));
        Variable& ts_var = dbcont_man_.metaVariable(dbcontent_vars::meta_var_timestamp_.name()).getFor(name);
        item.ts_prop = Property(ts_var.name(), ts_var.dataType());
    }

    // make the cache map structurally final before any worker runs: a std::map insert
    // rebalances the tree, so inserting from a worker would race the unlocked lookups the
    // other workers do. A content not yet cached adopts its staged buffer as the cache buffer
    // (mutated in place below, exactly as before).
    for (auto& item : items)
    {
        if (buffers_.count(item.name))
            item.merge = true;
        else
            buffers_[item.name] = item.buffer;
    }

    // per-content work on disjoint buffers; buffers_ is only read from here on
    tbb::parallel_for(uint(0), (unsigned int) items.size(), [&](unsigned int item_cnt)
    {
        StagedItem& item   = items[item_cnt];
        Buffer&     staged = *item.buffer;

        // remove all columns not in the read set
        vector<Property> to_remove;
        for (const auto& prop_it : staged.properties().properties())
            if (!item.read_set.hasDBColumnName(prop_it.name()))
                to_remove.push_back(prop_it);

        for (auto& prop_it : to_remove)
            staged.deleteProperty(prop_it);

        // ensure the assoc (utn) column exists so it survives the transform
        if (item.utn_prop && !staged.hasProperty(*item.utn_prop))
            staged.addProperty(*item.utn_prop);

        // rename DB columns to variable names
        buffer_utils::transformVariables(staged, item.read_set, true);

        // add selection flag
        staged.addProperty(dbcontent_vars::selected_var_);

        auto& cached = buffers_.at(item.name);

        if (item.merge)
            cached->seizeBuffer(staged);

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
