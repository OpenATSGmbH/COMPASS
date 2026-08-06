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
#include "util/timewindow.h"

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

class ExcludedTimeWindowsFilter : public DBFilter
{
public:
    ExcludedTimeWindowsFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver);
    virtual ~ExcludedTimeWindowsFilter();

    virtual std::string getConditionString(const std::string& dbcontent_name,
      dbContent::VariableSet& read_set, bool& first) override;

    virtual FilterClause getClause(const std::string& dbcontent_name) override;

    virtual bool filters(const std::string& dbcontent_name) override;
    virtual void reset() override;

    virtual void saveViewPointConditions (nlohmann::json& filters) override;
    virtual void loadViewPointConditions (const nlohmann::json& filters) override;

    Utils::TimeWindowCollection& timeWindows();

    // pushed by FilterManager when database is opened
    void updateMinMaxTimestamp(const boost::posix_time::ptime& min_ts,
                               const boost::posix_time::ptime& max_ts);

    bool hasMinMaxTimestamp() const;
    std::pair<boost::posix_time::ptime, boost::posix_time::ptime> minMaxTimestamp() const;

protected:
    nlohmann::json time_windows_json_;
    Utils::TimeWindowCollection time_windows_;

    boost::optional<boost::posix_time::ptime> min_timestamp_;
    boost::optional<boost::posix_time::ptime> max_timestamp_;

    virtual DBFilterWidget* createWidget() override;
};

