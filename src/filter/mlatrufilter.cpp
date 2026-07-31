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
#include "mlatrufilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"
#include "stringconv.h"

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

MLATRUFilter::MLATRUFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("rus_str", &rus_str_, std::string());
    registerParameter("match_all", &match_all_, false);

    name_ = "MLAT RUs";

    createSubConfigurables();
}

MLATRUFilter::~MLATRUFilter() {}

bool MLATRUFilter::filters(const std::string& dbcontent_name)
{
    return dbcontent_name == "CAT020";
}

std::string MLATRUFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    loginf << "dbcont_name " << dbcontent_name << " active " << active_ << " rus_str '" << rus_str_
           << "' match_all " << match_all_;

    if (!active_)
        return "";

    traced_assert(dbcontent_name == "CAT020");

    auto& resolver = variableResolver();

    traced_assert(resolver.canGetVariable(dbcontent_name, dbcontent_vars::var_cat020_contrib_recv_));
    std::string contrib_dbcol_name = resolver.getVariableDBColumn(dbcontent_name, dbcontent_vars::var_cat020_contrib_recv_);

    traced_assert(resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_));
    std::string dsid_dbcol_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_ds_id_);        

    vector<string> split_str = String::split(rus_str_, ',');
    
    vector<vector<unsigned int>> numbers; // raw numbers that need no converion

    // check if null wanted, and keep only rest
    bool null_wanted = false;
    bool ok;

    // add only numbers which do not need conversion, and remove them
    for (auto it = split_str.begin(); it != split_str.end();)
    {
        string tmp = *it;

        boost::algorithm::trim(tmp);
        boost::algorithm::to_lower(tmp);

        if (!tmp.size())
        {
            it = split_str.erase(it);
            continue;
        }

        if (tmp == "null")
        {
            null_wanted = true;
            it = split_str.erase(it);
            continue;
        }

        // check if number
        unsigned int num_tmp = QString(it->c_str()).toInt(&ok);

        if (ok)
        {
            numbers.push_back({num_tmp});
            it = split_str.erase(it);
            continue;
        }

        ++it;
    }

    loginf << "numbers " << numbers.size() << " null " << null_wanted;

    const auto& ru_lookup = mlat_ru_lookup_; // ds id -> ru name -> {ru indexes}

    stringstream ss;

    if (!first)
        ss << " AND";

    ss << " (";

    bool first_in_filter = true;

    // add null wanted
    if (null_wanted)
    {
        ss << contrib_dbcol_name << " IS NULL";
        first_in_filter = false;
    }

    for (auto& ds_it : ru_lookup)
    {
        // already entered numbers
        vector<vector<unsigned int>> ds_numbers = numbers;

        // add all ru names as numbers
        for (string ru_name : split_str)
        {
            boost::algorithm::trim(ru_name);
            boost::algorithm::to_lower(ru_name);

            if (ds_it.second.count(ru_name))
                ds_numbers.push_back(ds_it.second.at(ru_name));
        }

        if (ds_numbers.size())
        {
            if (!first_in_filter)
                ss << " OR";
                
            ss << " (";

            ss << dsid_dbcol_name << " = " << ds_it.first;
            ss << " AND";

            // SELECT * FROM data WHERE json_contains(json_list, '3') OR json_contains(json_list, '7');

            bool first_value{true};

            for (auto& values : ds_numbers)
            {
                if (!first_value)
                    ss << (match_all_ ? " AND" : " OR");

                assert (values.size());

                if (values.size() == 1)
                    ss << " json_contains(" << contrib_dbcol_name << ", '" << values.at(0) << "')";
                else
                {
                    ss << " (";

                    for (unsigned int cnt=0; cnt < values.size(); cnt++) // any of these indexes corresond to a matching name
                    {
                        if (cnt != 0)
                            ss << " OR";

                        ss << " json_contains(" << contrib_dbcol_name << ", '" << values.at(cnt) << "')";
                    }

                    ss << ")";
                }

                first_value = false;
            }
            ss << ")";

            first_in_filter = false;
        }
    }

    ss << ")";

    if (first_in_filter)
        return "";

    first = false;

    loginf << "'" << ss.str() << "'";
    
    return ss.str();
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
    filter["match_all"] = match_all_;
}

void MLATRUFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("rus"));
    rus_str_ = filter.at("rus");

    if (filter.contains("match_all"))
        match_all_ = filter.at("match_all");
    else
        match_all_ = false;

    if (widget_)
        widget_->update();
}

std::string MLATRUFilter::rus() const
{
    return rus_str_;
}

bool MLATRUFilter::checkRUs(const std::string& rus_str)
{
    vector<string> split_str = String::split(rus_str, ',');

    bool ok;
    
    // check if null wanted, and keep only rest
    for (string str : split_str)
    {
        boost::algorithm::trim(str);
        boost::algorithm::to_lower(str);

        if (!str.size())
            continue;

        // check if null
        if (str == "null")
            continue;

        // check if number
        unsigned int num_tmp = QString(str.c_str()).toInt(&ok);

        if (ok)
            continue;

        // else a name

        if (!known_ru_names_.count(str))
        {
            loginf << "ru '" << str << "' not in '" << String::compress(known_ru_names_,',') << "'";
            return false;
        }
    }

    return true;
}

void MLATRUFilter::rus(const std::string& rus_str)
{
    loginf << "'" << rus_str << "'";

    rus_str_ = rus_str;
}

bool MLATRUFilter::matchAll() const
{
    return match_all_;
}

void MLATRUFilter::matchAll(bool match_all)
{
    match_all_ = match_all;
}

void MLATRUFilter::updateMLATDataSources(
    const std::map<unsigned int, std::map<std::string, std::vector<unsigned int>>>& mlat_ru_lookup)
{
    // normalize RU names to lower case, matching the query normalization in
    // getConditionString() / checkRUs()
    mlat_ru_lookup_.clear();

    for (const auto& ds_it : mlat_ru_lookup)
    {
        auto& names = mlat_ru_lookup_[ds_it.first];

        for (const auto& name_it : ds_it.second)
        {
            std::string name = name_it.first;
            boost::algorithm::to_lower(name);

            auto& indexes = names[name];
            indexes.insert(indexes.end(), name_it.second.begin(), name_it.second.end());
        }
    }
}

void MLATRUFilter::updateMLATKnownRUNames(const std::set<std::string>& known_ru_names)
{
    known_ru_names_.clear();

    for (std::string name : known_ru_names)
    {
        boost::algorithm::to_lower(name);
        known_ru_names_.insert(name);
    }
}


