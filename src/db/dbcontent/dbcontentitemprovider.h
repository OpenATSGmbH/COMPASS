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

#include "dbcontentitem.h"

#include <tuple>

#include "json.hpp"

#include <QObject>

class DBContentDataStore;

namespace dbContent
{
    class TargetReportAccessor;
}

/**
 * Provides DBContent buffer items grouped by certain attributes (e.g. UTN),
 * given a DBContentDataStore. 
 */
class DBContentItemProvider : public QObject
{
    Q_OBJECT

public:
    /**
     * Item grouping mode.
     */
    enum class Grouping
    {
        None = 0,
        AircraftAddress,
        AircraftID,
        TrackNumber,
        Mode3ACode,
        UTN
    };

    DBContentItemProvider(DBContentDataStore& data_store, Grouping grouping = Grouping::None);
    virtual ~DBContentItemProvider();

    void setGrouping(Grouping grouping, bool run_update = true);
    Grouping grouping() const { return grouping_; }
    std::string groupingAsString() const;

    const std::vector<std::unique_ptr<dbContent::ItemGroup>>& itemGroups() const { return item_groups_; }

    static std::string groupingToString(Grouping grouping);
    static Grouping groupingFromString(const std::string& str);
    static bool isGroupingString(const std::string& str);

    static const std::string GroupingStrNone;
    static const std::string GroupingStrAircraftAddress;
    static const std::string GroupingStrAircraftID;
    static const std::string GroupingStrTrackNumber;
    static const std::string GroupingStrMode3ACode;
    static const std::string GroupingStrUTN;

protected:
    void update();
    std::string toString() const;

    virtual void reset_impl() {}
    virtual void dataToBeChanged_impl(unsigned int dbc_id) {}
    virtual void dataChanged_impl(unsigned int dbc_id, size_t group_idx) {}
    virtual void dataRefreshed_impl() {}

    DBContentDataStore& dataStore() { return data_store_; }
    std::vector<std::unique_ptr<dbContent::ItemGroup>>& itemGroups() { return item_groups_; }

private:
    void reset();
    void dataChanged(unsigned int dbc_id);
    void dataRefreshed();

    void setGroupIDNames(dbContent::ItemGroup& group) const;

    std::function<nlohmann::json(unsigned int)> createGroupFunc(dbContent::TargetReportAccessor& accessor) const;

    DBContentDataStore&                                data_store_;                // data store providing the data
    Grouping                                           grouping_ = Grouping::None; // item grouping mode
    std::vector<std::unique_ptr<dbContent::ItemGroup>> item_groups_;               // per (dbcontent, ds, line) item groups
};
