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

#include "trackertracknumberfilter.h"
#include "trackertracknumberfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
//#include "util/timeconv.h"

#include <sstream>

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

TrackerTrackNumberFilter::TrackerTrackNumberFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("tracker_track_nums", &tracker_track_nums_, json::object());

    name_ = "Tracker Track Number";

    createSubConfigurables();
}

TrackerTrackNumberFilter::~TrackerTrackNumberFilter() {}

bool TrackerTrackNumberFilter::filters(const std::string& dbcontent_name)
{
    return dbcontent_name == "CAT062";
}

std::string TrackerTrackNumberFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    if (dbcontent_name != "CAT062")
        return "";

    stringstream ss;

    auto& resolver = variableResolver();

    traced_assert(resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_));
    traced_assert(resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_num_));

    // ds_id -> line_id -> values
    std::map<unsigned int, std::map<unsigned int, std::string>> active_tns = getActiveTrackerTrackNums();

    if (active_ && active_tns.size())
    {
        string ds_id_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_ds_id_);
        string line_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_line_id_);
        string tn_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_track_num_);

        if (!first)
        {
            ss << " AND ";
        }

        ss << "(";

        bool first_inside = true;

        for (auto& ds_it : active_tns)
        {
            for (auto& line_it : ds_it.second)
            {
                if (!first_inside)
                {
                    ss << " OR ";
                }

                ss << " (" + ds_id_col << " = " << ds_it.first;
                ss << " AND " + line_col << " = " << line_it.first;
                ss << " AND " << tn_col << " IN (" << line_it.second << "))";

                first_inside = false;
            }
        }

        ss << ")";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

DBFilterWidget* TrackerTrackNumberFilter::createWidget()
{
    return new TrackerTrackNumberFilterWidget(*this);
}


void TrackerTrackNumberFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();

    json& filter = filters.at(name_);

    traced_assert(!filter.contains("Values"));
    filter["Values"] = getActiveTrackerTrackNumsStr();
}

void TrackerTrackNumberFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    logdbg << "filter '" << filters.dump(4) << "'";

    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Values"));

    std::map<std::string, std::map<std::string, std::string>> vp_values =
            filter.at("Values").get<std::map<std::string, std::map<std::string, std::string>>>();

    for (auto& ds_it : vp_values)
    {
        for (auto& line_it : ds_it.second)
        {
            if (!tracker_track_nums_.contains(ds_it.first))
                tracker_track_nums_[ds_it.first] = json::object();

            tracker_track_nums_[ds_it.first][line_it.first] = line_it.second;
        }
    }

    if (widget_)
        widget_->update();
}

void TrackerTrackNumberFilter::setTrackerTrackNum(unsigned int ds_id, unsigned int line_id, const std::string& value)
{
    loginf << "ds_id " << ds_id
           << " line_id " << line_id << " value '" << value << "'";

    if (!tracker_track_nums_.contains(to_string(ds_id)))
        tracker_track_nums_[to_string(ds_id)] = json::object();

    tracker_track_nums_[to_string(ds_id)][to_string(line_id)] = value;
}

std::map<unsigned int, std::map<unsigned int, std::string>> TrackerTrackNumberFilter::getActiveTrackerTrackNums ()
{
    // ds_id -> line_id -> values
    std::map<std::string, std::map<std::string, std::string>> saved_values =
            tracker_track_nums_.get<std::map<std::string, std::map<std::string, std::string>>>();

    std::map<unsigned int, std::map<unsigned int, std::string>> active_values;

    for (auto& ds_it : tracker_lines_)
    {
        string ds_id_str = to_string(ds_it.first);

        for (auto& line_cnt_it : ds_it.second)
        {
            if (line_cnt_it.second == 0)
                continue;

            string line_id_str = to_string(line_cnt_it.first);

            if (saved_values.count(ds_id_str) && saved_values.at(ds_id_str).count(line_id_str))
                active_values[ds_it.first][line_cnt_it.first] = saved_values.at(ds_id_str).at(line_id_str);
            else
                active_values[ds_it.first][line_cnt_it.first] = "";
        }
    }

    return active_values;
}

std::map<std::string, std::map<std::string, std::string>> TrackerTrackNumberFilter::getActiveTrackerTrackNumsStr ()
{
    // ds_id -> line_id -> values
    std::map<std::string, std::map<std::string, std::string>> saved_values =
            tracker_track_nums_.get<std::map<std::string, std::map<std::string, std::string>>>();

    std::map<std::string, std::map<std::string, std::string>> active_values;

    for (auto& ds_it : tracker_lines_)
    {
        string ds_id_str = to_string(ds_it.first);

        for (auto& line_cnt_it : ds_it.second)
        {
            if (line_cnt_it.second == 0)
                continue;

            string line_id_str = to_string(line_cnt_it.first);

            if (saved_values.count(ds_id_str) && saved_values.at(ds_id_str).count(line_id_str))
                active_values[ds_id_str][line_id_str] = saved_values.at(ds_id_str).at(line_id_str);
            else
                active_values[ds_id_str][line_id_str] = "";
        }
    }

    return active_values;
}

void TrackerTrackNumberFilter::updateTrackerDataSources(
    const std::map<unsigned int, std::map<unsigned int, unsigned int>>& tracker_lines,
    const std::map<unsigned int, std::string>& ds_names)
{
    tracker_lines_ = tracker_lines;
    ds_names_ = ds_names;

    if (widget_)
        widget_->update();
}

bool TrackerTrackNumberFilter::hasDataSourceName(unsigned int ds_id) const
{
    return ds_names_.count(ds_id) > 0;
}

std::string TrackerTrackNumberFilter::dataSourceName(unsigned int ds_id) const
{
    auto it = ds_names_.find(ds_id);
    if (it != ds_names_.end())
        return it->second;
    return std::to_string(ds_id);
}


void TrackerTrackNumberFilter::updateDataSourcesSlot()
{
    loginf;

    if (widget_)
        widget_->update();
}

