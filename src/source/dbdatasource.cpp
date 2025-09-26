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

#include "source/dbdatasource.h"
#include "source/dbdatasourcewidget.h"
#include "source/configurationdatasource.h"
#include "util/timeconv.h"
#include "util/number.h"
#include "logger.h"

using namespace nlohmann;
using namespace Utils;
using namespace std;

namespace dbContent
{


const std::string DBDataSource::table_name_{"data_sources"};

const Property DBDataSource::id_column_{"ds_id", PropertyDataType::UINT};
const Property DBDataSource::ds_type_column_{"ds_type", PropertyDataType::STRING};
const Property DBDataSource::sac_column_{"sac", PropertyDataType::UINT};
const Property DBDataSource::sic_column_{"sic", PropertyDataType::UINT};
const Property DBDataSource::name_column_{"name", PropertyDataType::STRING};
const Property DBDataSource::short_name_{"short_name", PropertyDataType::STRING};
const Property DBDataSource::info_column_{"info", PropertyDataType::STRING};
const Property DBDataSource::counts_column_{"counts", PropertyDataType::STRING};

DBDataSource::DBDataSource()
{
    counts_ = json::object(); // init
}

DBDataSource::~DBDataSource()
{

}

void DBDataSource::counts (const std::string& counts)
{
    counts_ = json::parse(counts);

    num_inserted_ = counts_.get<std::map<std::string, std::map<unsigned int, unsigned int>>>();
}

std::string DBDataSource::countsStr()
{
    return counts_.dump();
}

bool DBDataSource::hasNumInserted() const
{
    return num_inserted_.size();
}

bool DBDataSource::hasNumInserted(const std::string& db_content) const
{
    if (!num_inserted_.count(db_content))
        return false;

    return num_inserted_.at(db_content).size();
}

bool DBDataSource::hasNumInserted(unsigned int line_id) const
{
    for (auto& dbcont_cnt_it : num_inserted_)
    {
        if (dbcont_cnt_it.second.count(line_id))
            return true;
    }

    return false;
}

bool DBDataSource::hasNumInserted(const std::string& db_content, unsigned int line_id) const
{
    if (!num_inserted_.count(db_content))
        return false;

    return num_inserted_.at(db_content).count(line_id);
}

const std::map<std::string, std::map<unsigned int, unsigned int>>& DBDataSource::numInsertedMap() const
{
    return num_inserted_;
}

std::map<unsigned int, unsigned int> DBDataSource::numInsertedLinesMap() const
{
    std::map<unsigned int, unsigned int> line_cnts;

    for (auto& db_cont_it : num_inserted_)
    {
        for (auto& cnt_it : db_cont_it.second)
        {
            line_cnts[cnt_it.first] += cnt_it.second;
        }
    }

    return line_cnts;
}

std::map<std::string, unsigned int> DBDataSource::numInsertedSummedLinesMap() const
{
    std::map<std::string, unsigned int> line_sum_map;

    for (auto& db_cont_it : num_inserted_)
    {
        for (auto& cnt_it : db_cont_it.second)
        {
            line_sum_map[db_cont_it.first] += cnt_it.second;
        }
    }

    return line_sum_map;
}

void DBDataSource::addNumInserted(const std::string& db_content, unsigned int line_id, unsigned int num)
{
    num_inserted_[db_content][line_id] += num;

    counts_ = num_inserted_;
}

void DBDataSource::clearNumInserted(const std::string& db_content)
{
    if (num_inserted_.count(db_content))
        num_inserted_.erase(db_content);
}

void DBDataSource::clearNumInserted(const std::string& db_content, unsigned int line_id)
{
    if (!num_inserted_.count(db_content))
        return;

    if (!num_inserted_.at(db_content).count(line_id))
        return;

    num_inserted_.at(db_content).erase(line_id);
}

void DBDataSource::addNumLoaded(const std::string& db_content, unsigned int line_id, unsigned int num)
{
    num_loaded_[db_content][line_id] += num;
}

unsigned int DBDataSource::numLoaded (unsigned int line_id) const
{
    unsigned int num_loaded = 0;

    for (auto& dbcont_cnt_it : num_loaded_)
    {
        if (dbcont_cnt_it.second.count(line_id))
            num_loaded += dbcont_cnt_it.second.at(line_id);
    }

    return num_loaded;
}

unsigned int DBDataSource::numLoaded (const std::string& db_content) const
{
    unsigned int num_loaded = 0;

    if (num_loaded_.count(db_content))
    {
        for (auto& cnt_it : num_loaded_.at(db_content))
            num_loaded += cnt_it.second;
    }

    return num_loaded;
}

unsigned int DBDataSource::numLoaded (const std::string& db_content, unsigned int line_id) const
{
    if (num_loaded_.count(db_content) && num_loaded_.at(db_content).count(line_id))
        return num_loaded_.at(db_content).at(line_id);

    return 0;
}

bool DBDataSource::hasNumLoaded (unsigned int line_id) const // for any DBContent
{
    for (auto& loaded_it : num_loaded_)
        if (loaded_it.second.count(line_id))
            return true;

    return false;
}

bool DBDataSource::hasAnyNumLoaded () const // for any DBContent, line
{
    for (auto& loaded_it : num_loaded_)
    {
        for (auto& loaded_line_it : loaded_it.second)
        {
            if (loaded_line_it.second)
                return true;
        }
    }

    return false;
}

unsigned int DBDataSource::getFirstLoadedLine() // for any DBContent
{
    traced_assert(hasAnyNumLoaded());

    for (auto& loaded_it : num_loaded_)
    {
        for (auto& loaded_line_it : loaded_it.second)
        {
            if (loaded_line_it.second)
                return loaded_line_it.first;
        }
    }

    traced_assert(false); // should never happen
}

void DBDataSource::clearNumLoaded()
{
    num_loaded_.clear();
}

unsigned int DBDataSource::id() const
{
    return id_;
}

void DBDataSource::id(unsigned int id)
{
    id_ = id;
}

bool DBDataSource::loadingWanted() const
{
    return loading_wanted_;
}

void DBDataSource::loadingWanted(bool loading_wanted)
{
    logdbg << "ds " << name_ << " wanted " << loading_wanted;

    loading_wanted_ = loading_wanted;
}

bool DBDataSource::lineSpecificLoadingWanted() const
{
    for (auto& line_it : line_loading_wanted_)
    {
        if (!line_it.second)
            return true;
    }

    return false;
}

bool DBDataSource::anyLinesLoadingWanted() const
{
    bool any_wanted = false;

    for (unsigned int cnt=0; cnt < 4; ++cnt)
    {
        if (lineLoadingWanted(cnt))
        {
            any_wanted = true;
        }
    }

    return any_wanted;
}

void DBDataSource::disableAllLines()
{
    for (unsigned int cnt=0; cnt < 4; ++cnt)
        line_loading_wanted_[cnt] = false;
}

void DBDataSource::enableAllLines()
{
    line_loading_wanted_.clear();
}

void DBDataSource::lineLoadingWanted(unsigned int line_id, bool wanted)
{
    traced_assert(line_id <= 4);
    line_loading_wanted_[line_id] = wanted;
}

bool DBDataSource::lineLoadingWanted(unsigned int line_id) const
{
    traced_assert(line_id <= 4);

    if (line_loading_wanted_.count(line_id))
        return line_loading_wanted_.at(line_id);
    else
        return true;
}

std::set<unsigned int> DBDataSource::getLoadingWantedLines() const
{
    std::set<unsigned int> lines;

    for (unsigned int cnt=0; cnt < 4; ++cnt)
        if (lineLoadingWanted(cnt))
            lines.insert(cnt);

    return lines;
}

//DBDataSourceWidget* DBDataSource::widget()
//{
//    if (!widget_)
//        widget_.reset(new DBDataSourceWidget(*this));

//    traced_assert(widget_);
//    return widget_.get();
//}

bool DBDataSource::hasMaxTimestamp(unsigned int line) const
{
    return max_line_tods_.count(line);
}

void DBDataSource::maxTimestamp(unsigned int line, boost::posix_time::ptime value)
{
    max_line_tods_[line] = Time::toLong(value);
}

boost::posix_time::ptime DBDataSource::maxTimestamp(unsigned int line) const
{
    traced_assert(hasMaxTimestamp(line));
    return Time::fromLong(max_line_tods_.at(line));
}

bool DBDataSource::hasLiveData(unsigned int line, boost::posix_time::ptime current_ts) const
{
    bool ret;

    if (hasMaxTimestamp(line) && hasUpdateInterval())
        ret = (current_ts - maxTimestamp(line)).total_milliseconds()/1000.0 < updateInterval() + 2; // 2s max latency
    else
        ret = false;

//    if (hasMaxTimestamp(line))
//        loginf << "name " << name_ << " current_ts " << Time::toString(current_ts)
//               << " hasMax " << hasMaxTimestamp(line) << " hasUI " << hasUpdateInterval()
//               << " maxTS " << Time::toString(maxTimestamp(line))
//               << " diff " << Time::toString(current_ts - maxTimestamp(line)) << " ret " << ret;
//    else
//        loginf << "name " << name_ << " no maxTS";

    return ret;
}

nlohmann::json DBDataSource::getAsJSON() const
{
    auto j = DataSourceBase::getAsJSON();

    //add counts
    if (!counts_.is_null())
        j[ "counts" ] = counts_;

    return j;
}

}
