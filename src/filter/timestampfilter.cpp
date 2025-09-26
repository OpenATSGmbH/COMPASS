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
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "util/timeconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

TimestampFilter::TimestampFilter(const std::string& class_id, const std::string& instance_id,
                                 Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
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
    return COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_timestamp_.name()).existsIn(dbcont_name);
}

std::string TimestampFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.metaVariable(
                DBContent::meta_var_timestamp_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_)
    {
        dbContent::Variable& var = dbcont_man.metaVariable(
                    DBContent::meta_var_timestamp_.name()).getFor(dbcontent_name);

        if (!first)
        {
            ss << " AND";
        }

        ss << " (" << var.dbColumnName() << " >= " << Time::toLong(min_value_)
           << " AND " << var.dbColumnName() << " <= " << Time::toLong(max_value_) << ")";

        loginf << "dbcont " << dbcontent_name << " active " << active_
               << " min " << Time::toString(min_value_) << " max " << Time::toString(max_value_);

        first = false;
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}


void TimestampFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("TimestampFilter: generateSubConfigurable: unknown class_id " + class_id);
}

DBFilterWidget* TimestampFilter::createWidget()
{
    return new TimestampFilterWidget(*this);
}


void TimestampFilter::checkSubConfigurables()
{
    logdbg;
}


void TimestampFilter::reset()
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    loginf << "has db min/max " << dbcont_man.hasMinMaxTimestamp();

    if (dbcont_man.hasMinMaxTimestamp())
    {
        std::pair<boost::posix_time::ptime , boost::posix_time::ptime> minmax_ts =  dbcont_man.minMaxTimestamp();
        minValue(get<0>(minmax_ts));
        maxValue(get<1>(minmax_ts));
    }

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

    if (widget())
        widget()->update();
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
