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

#include "mlatrufilter.h"
#include "compass.h"
#include "datasourcemanager.h"
#include "mlatrufilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

MLATRUFilter::MLATRUFilter(const std::string& class_id, const std::string& instance_id,
                     Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("rus_str", &rus_str_, std::string());

    name_ = "MLAT RUs";

    createSubConfigurables();
}

MLATRUFilter::~MLATRUFilter() {}

bool MLATRUFilter::filters(const std::string& dbcontent_name)
{
    return dbcontent_name == "CAT020";
}

std::string MLATRUFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    if (!active_)
        return "";

    traced_assert(dbcontent_name == "CAT020");

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    traced_assert(
        dbcontent_man.canGetVariable(dbcontent_name, DBContent::var_cat020_crontrib_recv_));

    std::string contrib_dbcol_name =
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_cat020_crontrib_recv_).dbColumnName();

    traced_assert(
        dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));

    std::string dsid_dbcol_name =
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_).dbColumnName();        

    vector<string> split_str = String::split(rus_str_, ',');
    vector<unsigned int> numbers;

    // check if null wanted, and keep only rest
    bool null_wanted = false;
    bool ok;

    for (auto it = split_str.begin(); it != split_str.end();)
    {
        // check if null
        if (String::trim(*it) == "NULL" || String::trim(*it) == "null")
        {
            null_wanted = true;
            it = split_str.erase(it);
            continue;
        }

        // check if number
        unsigned int num_tmp = QString(it->c_str()).toInt(&ok);

        if (ok)
        {
            numbers.push_back(num_tmp);
            it = split_str.erase(it);
            continue;
        }

        ++it;
    }

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    std::map<unsigned int, std::multimap<std::string, unsigned int>> ru_lookup; // ds id -> ru name -> ru index

    for (auto& db_src_it : ds_man.dbDataSources())
    {
        if (db_src_it && db_src_it->dsType() == "MLAT" && db_src_it->hasRemoteUnits())
            ru_lookup[db_src_it->id()] = db_src_it->mlatRUNames();
    }

    stringstream ss;

    if (!first)
        ss << " AND";

    ss << " (";

    // add null wanted
    if (null_wanted)
    {
        ss << contrib_dbcol_name << " IS NULL";
        first = false;
    }

    for (auto& ds_it : ru_lookup)
    {
        // already entered numbers
        vector<unsigned int> ds_numbers = numbers;

        // add all ru names as numbers
        for (auto& ru_name : split_str)
        {
            auto range = ds_it.second.equal_range(String::trim(ru_name));
            for (auto it = range.first; it != range.second; ++it)
            {
                ds_numbers.push_back(it->second);
            }
        }

        if (ds_numbers.size())
        {
            if (!first)
                ss << " OR";
                
            ss << " (";

            ss << dsid_dbcol_name << " = " << ds_it.first;
            ss << " AND";

            // SELECT * FROM data WHERE json_contains(json_list, '3') OR json_contains(json_list, '7');

            bool first_value{true};

            for (auto& value : ds_numbers)
            {
                if (!first_value)
                    ss << " OR";

                ss << " json_contains(" << contrib_dbcol_name << ", '" << value << "')";

                first_value = false;
            }
            ss << ")";

            first = false;
        }
    }

    ss << ")";

    first = false;

    loginf << "here '" << ss.str() << "'";
    
    return ss.str();
}

void MLATRUFilter::generateSubConfigurable(const std::string& class_id,
                                           const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("MLATRUFilter: generateSubConfigurable: unknown class_id " + class_id);
}

void MLATRUFilter::checkSubConfigurables()
{
    logdbg;

}

DBFilterWidget* MLATRUFilter::createWidget()
{
    return new MLATRUFilterWidget(*this);
}


void MLATRUFilter::reset()
{
    if (widget_)
        widget_->update();
}

void MLATRUFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["rus"] = rus_str_;
}

void MLATRUFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("rus"));
    rus_str_ = filter.at("rus");

    if (widget())
        widget()->update();
}

std::string MLATRUFilter::rus() const
{
    return rus_str_;
}

void MLATRUFilter::rus(const std::string& rus_str)
{
    rus_str_ = rus_str;
}


