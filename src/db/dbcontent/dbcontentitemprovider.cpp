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

#include "dbcontentitemprovider.h"
#include "dbcontentdatastore.h"
#include "dbcontentaccessor.h"
#include "dbcontentmanager.h"
#include "targetreportaccessor.h"
#include "targetreportdefs.h"

#include "compass.h"
#include "datasourcemanager.h"

#include "traced_assert.h"

const std::string DBContentItemProvider::GroupingStrNone            = "None";
const std::string DBContentItemProvider::GroupingStrAircraftAddress = "Aircraft Address";
const std::string DBContentItemProvider::GroupingStrAircraftID      = "Aircraft Identification";
const std::string DBContentItemProvider::GroupingStrTrackNumber     = "Track Number";
const std::string DBContentItemProvider::GroupingStrMode3ACode      = "Mode 3/A Code";
const std::string DBContentItemProvider::GroupingStrUTN             = "UTN";

/**
 * Constructs the item provider and connects to the given data store's signals,
 * so that item groups are kept in sync with data store changes.
 */
DBContentItemProvider::DBContentItemProvider(DBContentDataStore& data_store, 
                                             Grouping grouping)
:   data_store_(data_store)
,   grouping_  (grouping  )
{
    connect(&data_store_, &DBContentDataStore::dataResetSignal, this, &DBContentItemProvider::reset, Qt::QueuedConnection);
    connect(&data_store_, &DBContentDataStore::dataChangedSignal, this, &DBContentItemProvider::dataChanged, Qt::QueuedConnection);
    connect(&data_store_, &DBContentDataStore::dataRefreshedSignal, this, &DBContentItemProvider::dataRefreshed, Qt::QueuedConnection);
}

/**
 */
DBContentItemProvider::~DBContentItemProvider() = default;

/**
 * Converts a grouping mode to a human-readable string representation.
 * Returns "None" for Grouping::None and unrecognized values.
 */
std::string DBContentItemProvider::groupingToString(Grouping grouping)
{
    switch (grouping)
    {
        case Grouping::AircraftAddress: return GroupingStrAircraftAddress;
        case Grouping::AircraftID:      return GroupingStrAircraftID;
        case Grouping::TrackNumber:     return GroupingStrTrackNumber;
        case Grouping::Mode3ACode:      return GroupingStrMode3ACode;
        case Grouping::UTN:             return GroupingStrUTN;
        case Grouping::None:            return GroupingStrNone;
        default:                        return GroupingStrNone;
    }
}

/**
 * Parses a string representation of a grouping mode and returns the corresponding enum value.
 * Returns Grouping::None for unrecognized strings.
 */
DBContentItemProvider::Grouping DBContentItemProvider::groupingFromString(const std::string& str)
{
    if (str == GroupingStrAircraftAddress) return Grouping::AircraftAddress;
    if (str == GroupingStrAircraftID)      return Grouping::AircraftID;
    if (str == GroupingStrTrackNumber)     return Grouping::TrackNumber;
    if (str == GroupingStrMode3ACode)      return Grouping::Mode3ACode;
    if (str == GroupingStrUTN)             return Grouping::UTN;
    if (str == GroupingStrNone)            return Grouping::None;

    return Grouping::None;
}

/**
 * Returns true if str matches any known grouping display string.
 */
bool DBContentItemProvider::isGroupingString(const std::string& str)
{
    return str == GroupingStrAircraftAddress
        || str == GroupingStrAircraftID
        || str == GroupingStrTrackNumber
        || str == GroupingStrMode3ACode
        || str == GroupingStrUTN
        || str == GroupingStrNone;
}

/**
 */
bool DBContentItemProvider::isTargetSpecific(Grouping grouping)
{
    return grouping == Grouping::AircraftAddress ||
           grouping == Grouping::AircraftID      ||
           grouping == Grouping::TrackNumber     ||
           grouping == Grouping::UTN;
}

/**
 */
bool DBContentItemProvider::isNumeric(Grouping grouping)
{
    return grouping == Grouping::UTN         ||
           grouping == Grouping::TrackNumber ||
           grouping == Grouping::Mode3ACode;
}

/**
 * Sets the grouping mode used to partition buffer rows into items.
 * Triggers a full rebuild of all item groups if the grouping changes.
 */
void DBContentItemProvider::setGrouping(Grouping grouping, bool run_update)
{
    if (grouping_ == grouping)
        return;

    grouping_ = grouping;

    if (run_update)
        update();
}

/**
 *  Converts the current grouping mode to a string representation.
 */
std::string DBContentItemProvider::groupingAsString() const
{
    return DBContentItemProvider::groupingToString(grouping_);
}

/**
 */
bool DBContentItemProvider::groupingIsNumeric() const
{
    return DBContentItemProvider::isNumeric(grouping_);
}

/**
 */
bool DBContentItemProvider::groupingIsTargetSpecific() const
{
    return DBContentItemProvider::isTargetSpecific(grouping_);
}

/**
 * Clears all item groups and notifies subclasses via reset_impl().
 * Called when the data store signals a full data reset.
 */
void DBContentItemProvider::reset()
{
    item_groups_.clear();

    reset_impl();

    emit dataResetSignal();
}

/**
 * Creates a function that extracts a grouping key (as JSON) from a buffer row index,
 * based on the current grouping mode. Returns a null JSON value for rows where the
 * grouping field is not set. These null-keyed rows are collected into a single unnamed group.
 */
std::function<nlohmann::json(unsigned int)> DBContentItemProvider::createGroupFunc(dbContent::TargetReportAccessor& accessor) const
{
    std::function<nlohmann::json(unsigned int)> func;

    if (grouping_ == Grouping::AircraftAddress)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            auto acad = accessor.acad(idx);
            if (!acad.has_value())
                return nlohmann::json();
            return nlohmann::json(acad.value());
        };
    }
    else if (grouping_ == Grouping::AircraftID)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            auto acid = accessor.acid(idx);
            if (!acid.has_value())
                return nlohmann::json();
            return nlohmann::json(acid.value());
        };
    }
    else if (grouping_ == Grouping::TrackNumber)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            auto tn = accessor.trackNumber(idx);
            if (!tn.has_value())
                return nlohmann::json();
            return nlohmann::json(tn.value());
        };
    }
    else if (grouping_ == Grouping::Mode3ACode)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            auto modea = accessor.modeACode(idx);
            if (!modea.has_value())
                return nlohmann::json();
            return nlohmann::json(modea.value().code_);
        };
    }
    else if (grouping_ == Grouping::UTN)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            auto utn = accessor.utn(idx);
            if (!utn.has_value())
                return nlohmann::json();
            return nlohmann::json(utn.value());
        };
    }
    else // Grouping::None — all rows share a single null key (one group per ds/line)
    {
        func = [ &accessor ] (unsigned int idx)
        {
            return nlohmann::json();
        };
    }

    traced_assert(func);

    return func;
}

/**
 * Rebuilds item groups for the given DBContent type after its data has changed.
 * Notifies subclasses before and after the rebuild via dataToBeChanged_impl() and
 * dataChanged_impl(), passing the range of newly appended groups in item_groups_.
 * For each (ds_id, line_id) combination, rows are partitioned into items by the
 * grouping key extracted via createGroupFunc(). The resulting item groups are
 * appended to item_groups_.
 */
void DBContentItemProvider::dataChanged(unsigned int dbc_id)
{
    auto dbc_name = data_store_.dbcManager().dbContentWithId(dbc_id);

    // queued signal may arrive after data store was reset — nothing to do
    if (!data_store_.indices().count(dbc_id))
        return;

    const auto& indices = data_store_.indices().at(dbc_id);

    auto tr_acc = data_store_.targetReportAccessor(dbc_id);
    auto buffer = data_store_.buffer(dbc_id);

    traced_assert(tr_acc);
    traced_assert(buffer);

    dataToBeChanged_impl(dbc_id);

    auto group_func = createGroupFunc(*tr_acc);
    traced_assert(group_func);

    for (const auto& ds_it : indices)
    {
        for (const auto& line_it : ds_it.second)
        {
            traced_assert(line_it.second);

            auto group = std::make_unique<dbContent::ItemGroup>();
            group->dbc_id   = dbc_id;
            group->ds_id    = ds_it.first;
            group->line_id  = line_it.first;

            group->accessor = tr_acc;
            group->buffer   = buffer;

            // partition row indices by their grouping key
            std::map<nlohmann::json, std::vector<unsigned int>> item_indices_map;

            for (auto idx : *line_it.second)
            {
                // collect item id of buffer index
                auto item_id = group_func(idx);
                
                auto& item_indices = item_indices_map[ item_id ];
                item_indices.push_back(idx);
            }

            group->items.reserve(item_indices_map.size());

            for (const auto& item_indices_it : item_indices_map)
            {
                size_t idx0 = group->indices.size();

                //insert item indices into group indices
                group->indices.insert(group->indices.end(), 
                                      item_indices_it.second.begin(), 
                                      item_indices_it.second.end());

                size_t idx1 = group->indices.size();

                //collect item
                dbContent::Item item;
                item.item_id   = item_indices_it.first;
                item.idx_begin = idx0;
                item.idx_end   = idx1;

                group->items.push_back(item);
            }

            setGroupIDNames(*group);

            size_t gidx = item_groups_.size();
            group->group_index = gidx;

            item_groups_.push_back(std::move(group));

            dataChanged_impl(dbc_id, gidx);

            emit dataChangedSignal(dbc_id, gidx);
        }
    }
}

/**
 */
void DBContentItemProvider::setGroupIDNames(dbContent::ItemGroup& group) const
{
    // dbcontent name
    group.dbc_name = data_store_.dbcManager().dbContentWithId(group.dbc_id);

    // ds name
    const auto& ds_manager = data_store_.dbcManager().compass().dataSourceManager();

    if (ds_manager.hasDBDataSource(group.ds_id))
    {
        const auto& ds = ds_manager.dbDataSource(group.ds_id);
        group.ds_name = ds.hasShortName() ? ds.shortName() : ds.name();
    }
    else if (ds_manager.hasConfigDataSource(group.ds_id))
    {
        const auto& ds = ds_manager.configDataSource(group.ds_id);
        group.ds_name = ds.hasShortName() ? ds.shortName() : ds.name();
    }
    else
    {
        group.ds_name = std::to_string(group.ds_id);
    }

    // line name
    group.line_name = Utils::String::lineStrFrom(group.line_id);
}

/**
 * Called when the data store signals that data has finished refreshing.
 * Delegates to dataRefreshed_impl() so subclasses can react to the refresh completion.
 */
void DBContentItemProvider::dataRefreshed()
{
    // react on data finishing refreshing in the data store

    dataRefreshed_impl();

    loginf << toString();

    emit dataRefreshedSignal();
}

/**
 * Rebuilds all item groups from scratch by resetting state and re-processing
 * every DBContent type currently held in the data store.
 */
void DBContentItemProvider::update()
{
    //reset
    reset();

    //update all dbcontents
    for (const auto& it : data_store_.buffers())
        dataChanged(it.first);

    //invoke final refreshed hook
    dataRefreshed();
}

/**
 * Converts the current grouping mode to a string representation.
 */
std::string DBContentItemProvider::toString() const
{
    std::stringstream ss;

    ss << "Grouping: " << groupingAsString() << "\n\n";

    for (const auto& group : item_groups_)
    {
        const auto dbc_name   = data_store_.dbcManager().dbContentWithId(group->dbc_id);

        const auto& ds_manager = data_store_.dbcManager().compass().dataSourceManager();

        std::string ds_name = "unknown";
        if (ds_manager.hasDBDataSource(group->ds_id))
            ds_name = ds_manager.dbDataSource(group->ds_id).name();
        else if (ds_manager.hasConfigDataSource(group->ds_id))
            ds_name = ds_manager.configDataSource(group->ds_id).name();

        size_t n_items   = group->items.size();
        size_t n_indices = group->indices.size();

        std::string item_info;

        if (n_items > 2)
        {
            item_info += group->items.front().item_id.is_null() ? "null," : "";
            item_info += group->items.front().item_id.is_null() ? group->items[ 1 ].item_id.dump(0) : group->items[ 0 ].item_id.dump(0);
            item_info += "-";
            item_info += group->items.back().item_id.dump(0);
        }
        else if (n_items == 2)
        {
            item_info += group->items.front().item_id.is_null() ? "null," : "";
            item_info += group->items.front().item_id.is_null() ? "" : group->items[ 0 ].item_id.dump(0);
            item_info += group->items.front().item_id.is_null() ? "" : "-";
            item_info += group->items[ 1 ].item_id.dump(0);
        }
        else if (n_items == 1)
        {
            item_info = group->items.front().item_id.is_null() ? "null" : group->items[ 0 ].item_id.dump(0);
        }
        else
        {
            item_info = "empty";
        }
        
        ss << dbc_name << " " << ds_name << " " << "Line " << group->line_id << ": " 
           << n_indices << " indices, " << n_items << " item(s) " << item_info << "\n";
    }

    return ss.str();
}
