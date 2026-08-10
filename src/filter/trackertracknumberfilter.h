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

#include "dbfilter.h"
#include "json_fwd.hpp"

#include <QObject>

class TrackerTrackNumberFilter : public QObject, public DBFilter
{
    Q_OBJECT

public slots:
    void updateDataSourcesSlot();


public:
    TrackerTrackNumberFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver);
    virtual ~TrackerTrackNumberFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name,
      dbContent::VariableSet& read_set, bool& first) override;

    virtual FilterClause getClause(const std::string& dbcontent_name) override;

    virtual bool filters(const std::string& dbcontent_name) override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    void setTrackerTrackNum(unsigned int ds_id, unsigned int line_id, const std::string& value);
    std::map<unsigned int, std::map<unsigned int, std::string>> getActiveTrackerTrackNums ();
    std::map<std::string, std::map<std::string, std::string>> getActiveTrackerTrackNumsStr ();
    // ds_id -> line -> track nums

    // pushed by FilterManager when data sources change
    // ds_id -> set of line_ids (only Tracker sources with inserted data)
    // ds_names: ds_id -> human-readable name
    void updateTrackerDataSources(
        const std::map<unsigned int, std::map<unsigned int, unsigned int>>& tracker_lines,
        const std::map<unsigned int, std::string>& ds_names);

    bool hasDataSourceName(unsigned int ds_id) const;
    std::string dataSourceName(unsigned int ds_id) const;

protected:
    nlohmann::json tracker_track_nums_; // ds_id -> line -> track nums

    // cached tracker data source info: ds_id -> line_id -> count
    std::map<unsigned int, std::map<unsigned int, unsigned int>> tracker_lines_;
    // cached data source names: ds_id -> name
    std::map<unsigned int, std::string> ds_names_;

    virtual DBFilterWidget* createWidget() override;
};
