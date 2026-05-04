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

#include "json.hpp"

class Buffer;

namespace dbContent
{

class TargetReportAccessor;

/**
 * Item grouping mode.
 */
enum class Grouping
{
    None = 0,
    AircraftAddress,
    AircraftID,
    TrackNumber,
    DSIDTrackNumber,
    Mode3ACode,
    UTN
};

/**
 * Item grouping flags.
 */
enum GroupingFlags
{
    GroupingNone            = 1 << 0,
    GroupingAircraftAddress = 1 << 1,
    GroupingAircraftID      = 1 << 2,
    GroupingTrackNumber     = 1 << 3,
    GroupingDSIDTrackNumber = 1 << 4,
    GroupingMode3ACode      = 1 << 5,
    GroupingUTN             = 1 << 6
};

/**
 * DBContent item containing buffer data which is determined by a certain grouping (e.g. an UTN).
 */
struct Item
{
    std::string itemIDString() const
    {
        return item_id.is_null() ? "None" : item_id.dump(0);
    }

    nlohmann::json            item_id;   // item id (content determined by grouping)
    unsigned int              idx_begin; // range index into group buffer indices begin
    unsigned int              idx_end;   // range index into group buffer indices end
};

/**
 * Holds grouped items for a certain (dbcontent, ds, line) combination.
 */
struct ItemGroup
{
    unsigned int group_index;

    unsigned int dbc_id;
    unsigned int ds_id;
    unsigned int line_id;

    std::string dbc_name;
    std::string ds_name;
    std::string ds_type;
    std::string line_name;
    
    std::vector<unsigned int>  indices; // buffer indices belonging to this group,
                                        // containing contiguous item indices
    std::vector<Item>          items;   // group items

    std::shared_ptr<Buffer>               buffer;
    std::shared_ptr<TargetReportAccessor> accessor;
};

} // namespace dbContent
