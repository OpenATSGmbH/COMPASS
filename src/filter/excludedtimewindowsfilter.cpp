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

#include "excludedtimewindowsfilter.h"
#include "excludedtimewindowsfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

ExcludedTimeWindowsFilter::ExcludedTimeWindowsFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("time_windows_json", &time_windows_json_, json::array());

    time_windows_.setFrom(time_windows_json_);

    name_ = "Excluded Time Windows";

    createSubConfigurables();
}

ExcludedTimeWindowsFilter::~ExcludedTimeWindowsFilter() {}

bool ExcludedTimeWindowsFilter::filters(const std::string& dbcont_name)
{
    return variableResolver().metaCanGetVariable(dbcont_name, DBContent::meta_var_timestamp_);
}

std::string ExcludedTimeWindowsFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, DBContent::meta_var_timestamp_))
        return "";

    stringstream ss;

    if (active_ && time_windows_.size())
    {
        string col_name = variableResolver().metaGetVariableDBColumn(dbcontent_name, DBContent::meta_var_timestamp_);

        if (!first)
        {
            ss << " AND";
        }

        ss << " NOT (";

        for (unsigned int cnt=0; cnt < time_windows_.size(); cnt++)
        {
            const Utils::TimeWindow& tw = time_windows_.get(cnt);

            if (cnt != 0)
                ss << " OR";

            ss << " " << col_name << " BETWEEN " << Time::toLong(tw.begin())
               << " AND " << Time::toLong(tw.end());
        }

        ss <<")";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

DBFilterWidget* ExcludedTimeWindowsFilter::createWidget()
{
    return new ExcludedTimeWindowsFilterWidget(*this);
}


void ExcludedTimeWindowsFilter::reset()
{
    time_windows_.clear();

    widget_->update();
}

void ExcludedTimeWindowsFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Windows"] = time_windows_json_;
}

void ExcludedTimeWindowsFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    logdbg << "filter '" << filters.dump(4) << "'";

    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Windows"));
    time_windows_json_ = filter.at("Windows");

    time_windows_.setFrom(time_windows_json_);

    if (widget())
        widget()->update();
}

Utils::TimeWindowCollection& ExcludedTimeWindowsFilter::timeWindows()
{
    return time_windows_;
}

void ExcludedTimeWindowsFilter::updateMinMaxTimestamp(const boost::posix_time::ptime& min_ts,
                                                       const boost::posix_time::ptime& max_ts)
{
    min_timestamp_ = min_ts;
    max_timestamp_ = max_ts;
}

bool ExcludedTimeWindowsFilter::hasMinMaxTimestamp() const
{
    return min_timestamp_.has_value() && max_timestamp_.has_value();
}

std::pair<boost::posix_time::ptime, boost::posix_time::ptime> ExcludedTimeWindowsFilter::minMaxTimestamp() const
{
    return {min_timestamp_.value(), max_timestamp_.value()};
}
