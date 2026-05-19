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

#include "timestampfilter.h"
#include "timestampfilterwidget.h"
#include "filtermanager.h"
#include "dbcontent/dbcontentmanager.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

TimestampFilter::TimestampFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("min_value", &min_value_str_, std::string());
    registerParameter("max_value", &max_value_str_, std::string());

    if (min_value_str_.size())
        min_value_ = Time::fromString(min_value_str_);

    if (max_value_str_.size())
        max_value_ = Time::fromString(max_value_str_);

    name_ = "Timestamp";

    createSubConfigurables();
}

TimestampFilter::~TimestampFilter() {}

bool TimestampFilter::filters(const std::string& dbcont_name)
{
    return variableResolver().metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_);
}

std::string TimestampFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_))
        return "";

    stringstream ss;

    if (active_)
    {
        string col_name = variableResolver().metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_timestamp_);

        if (!first)
        {
            ss << " AND";
        }

        ss << " (" << col_name << " >= " << Time::toLong(min_value_)
           << " AND " << col_name << " <= " << Time::toLong(max_value_) << ")";

        loginf << "dbcont " << dbcontent_name << " active " << active_
               << " min " << Time::toString(min_value_) << " max " << Time::toString(max_value_);

        first = false;
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}

DBFilterWidget* TimestampFilter::createWidget()
{
    return new TimestampFilterWidget(*this);
}


void TimestampFilter::reset()
{
    if (widget_)
        widget_->update();
}

void TimestampFilter::reset(boost::posix_time::ptime min, boost::posix_time::ptime max)
{
    minValue(min);
    maxValue(max);

    if (widget_)
        widget_->update();
}

void TimestampFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Timestamp Minimum"] = min_value_str_;
    filter["Timestamp Maximum"] = max_value_str_;
}

void TimestampFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    logdbg << "filter '" << filters.dump(4) << "'";

    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Timestamp Minimum"));
    min_value_str_ = filter.at("Timestamp Minimum");
    traced_assert(min_value_str_.size());
    min_value_ = Time::fromString(min_value_str_);

    traced_assert(filter.contains("Timestamp Maximum"));
    max_value_str_ = filter.at("Timestamp Maximum");
    traced_assert(max_value_str_.size());
    max_value_ = Time::fromString(max_value_str_);

    if (widget_)
        widget_->update();
}

boost::posix_time::ptime TimestampFilter::minValue() const
{
    return min_value_;
}

void TimestampFilter::minValue(boost::posix_time::ptime min_value, bool update_widget)
{
    min_value_ = min_value;
    min_value_str_ = Time::toString(min_value_);

    loginf << "start" << min_value_str_;

    if (widget_ && update_widget)
        widget_->update();
}

boost::posix_time::ptime TimestampFilter::maxValue() const
{
    return max_value_;
}

void TimestampFilter::maxValue(boost::posix_time::ptime max_value, bool update_widget)
{
    max_value_ = max_value;
    max_value_str_ = Time::toString(max_value_);

    loginf << "start" << max_value_str_;

    if (widget_ && update_widget)
        widget_->update();
}

void TimestampFilter::shiftWindow(int minutes)
{
    boost::posix_time::time_duration delta = boost::posix_time::minutes(minutes);

    boost::posix_time::ptime new_min = min_value_ + delta;
    boost::posix_time::ptime new_max = max_value_ + delta;

    auto& dbcont_man = filter_manager_->dbContentManager();
    if (dbcont_man.hasMinMaxTimestamp())
    {
        auto minmax = dbcont_man.minMaxTimestamp();

        if (new_min < minmax.first)
            new_min = minmax.first;
        if (new_max > minmax.second)
            new_max = minmax.second;
    }

    min_value_ = new_min;
    max_value_ = new_max;

    min_value_str_ = Time::toString(min_value_);
    max_value_str_ = Time::toString(max_value_);

    loginf << "shifted by " << minutes << " min, new min " << min_value_str_
           << " max " << max_value_str_;

    if (widget_)
        widget_->update();
}

bool TimestampFilter::canShiftWindow(int minutes) const
{
    auto& dbcont_man = filter_manager_->dbContentManager();

    if (!dbcont_man.hasMinMaxTimestamp())
        return true;

    auto minmax = dbcont_man.minMaxTimestamp();

    if (minutes > 0)
        return max_value_ < minmax.second;
    else
        return min_value_ > minmax.first;
}
