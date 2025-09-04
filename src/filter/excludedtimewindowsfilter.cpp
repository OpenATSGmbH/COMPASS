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
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

ExcludedTimeWindowsFilter::ExcludedTimeWindowsFilter(const std::string& class_id, const std::string& instance_id,
                                                     Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("time_windows_json", &time_windows_json_, json::array());

    time_windows_.setFrom(time_windows_json_);

    name_ = "Excluded Time Windows";

    createSubConfigurables();
}

ExcludedTimeWindowsFilter::~ExcludedTimeWindowsFilter() {}

bool ExcludedTimeWindowsFilter::filters(const std::string& dbcont_name)
{
    return COMPASS::instance().dbContentManager().metaVariable(
                                                     DBContent::meta_var_timestamp_.name()).existsIn(dbcont_name);
}

std::string ExcludedTimeWindowsFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_ && time_windows_.size())
    {
        dbContent::Variable& var = dbcont_man.metaVariable(
                                                 DBContent::meta_var_timestamp_.name()).getFor(dbcontent_name);

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

            ss << " " << var.dbColumnName() << " BETWEEN " << Time::toLong(tw.begin())
               << " AND " << Time::toLong(tw.end());
        }

        ss <<")";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}


void ExcludedTimeWindowsFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("ExcludedTimeWindowsFilter: generateSubConfigurable: unknown class_id " + class_id);
}

DBFilterWidget* ExcludedTimeWindowsFilter::createWidget()
{
    return new ExcludedTimeWindowsFilterWidget(*this);
}


void ExcludedTimeWindowsFilter::checkSubConfigurables()
{
    logdbg;
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
