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

#include "acidfilter.h"
#include "acidfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "buffer/buffer.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

ACIDFilter::ACIDFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("values_str", &values_str_, std::string());
    updateValuesFromStr(values_str_);

    name_ = "Aircraft Identification";

    createSubConfigurables();
}

ACIDFilter::~ACIDFilter() {}

bool ACIDFilter::filters(const std::string& dbcont_name)
{
    if (dbcont_name == "CAT062")
        return true; // acid and callsign fpl
    else
        return variableResolver().metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_acid_);
}

std::string ACIDFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
        return "";

    stringstream ss;

    if (active_  && (values_.size() || null_wanted_))
    {
        string acid_col = variableResolver().metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_acid_);

        string cs_fpl_col; // only set in cat062
        bool has_cs_fpl = false;

        if (dbcontent_name == "CAT062")
        {
            traced_assert(variableResolver().canGetVariable(
                        dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_));

            cs_fpl_col = variableResolver().getVariableDBColumn(
                        dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_);
            has_cs_fpl = true;
        }

        if (!first)
            ss << " AND";

        ss << " (";

        bool first_val = true;

        for (auto val_it : values_)
        {
            if (!first_val)
                ss << " OR";

             ss << " (" << acid_col  << " LIKE '%" << val_it << "%'";

             if (has_cs_fpl)
                ss << " OR " << cs_fpl_col  << " LIKE '%" << val_it << "%'";

             ss << ")";

            first_val = false;
        }

        if (null_wanted_)
        {
            if (!first_val)
                ss << " OR";

            ss << " (" << acid_col  << " IS NULL";

            if (has_cs_fpl)
               ss << " OR " << cs_fpl_col  << " IS NULL";

            ss << ")";
        }

        ss << ")";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

FilterClause ACIDFilter::getClause(const std::string& dbcontent_name)
{
    if (!active_)
        return FilterClause{};

    return sqlFor(variableResolver(), values_, null_wanted_, dbcontent_name);
}

FilterClause ACIDFilter::sqlFor(IDBVariableResolver& resolver,
                                const std::set<std::string>& values, bool null_wanted,
                                const std::string& dbcontent_name)
{
    FilterClause clause;

    if (!resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
        return clause;

    if (!(values.size() || null_wanted))
        return clause;

    string acid_col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_acid_);

    string cs_fpl_col; // only set in cat062
    bool has_cs_fpl = false;

    if (dbcontent_name == "CAT062")
    {
        traced_assert(resolver.canGetVariable(
                    dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_));

        cs_fpl_col = resolver.getVariableDBColumn(
                    dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_);
        has_cs_fpl = true;
    }

    stringstream ss;

    ss << "(";

    bool first_val = true;

    for (auto val_it : values)
    {
        if (!first_val)
            ss << " OR";

        ss << " (" << acid_col << " LIKE '%" << val_it << "%'";

        if (has_cs_fpl)
            ss << " OR " << cs_fpl_col << " LIKE '%" << val_it << "%'";

        ss << ")";

        first_val = false;
    }

    if (null_wanted)
    {
        if (!first_val)
            ss << " OR";

        ss << " (" << acid_col << " IS NULL";

        if (has_cs_fpl)
            ss << " OR " << cs_fpl_col << " IS NULL";

        ss << ")";
    }

    ss << ")";

    clause.sql = ss.str();
    return clause;
}


DBFilterWidget* ACIDFilter::createWidget()
{
    return new ACIDFilterWidget(*this);
}

void ACIDFilter::reset()
{
    if (widget_)
        widget_->update();
}

void ACIDFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Aircraft Identification Values"] = values_str_;
}

void ACIDFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Aircraft Identification Values"));
    values_str_ = filter.at("Aircraft Identification Values");

    updateValuesFromStr(values_str_);

    if (widget_)
        widget_->update();
}

std::string ACIDFilter::valuesString() const
{
    return values_str_;
}

void ACIDFilter::valuesString(const std::string& values_str)
{
    if (!updateValuesFromStr(values_str)) // false on failure
        return;

    values_str_ = values_str;
}

bool ACIDFilter::activeInLiveMode()
{
    return true;
}

std::vector<unsigned int> ACIDFilter::filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
        return to_be_removed;

    string acid_var_name = variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_acid_);

    traced_assert(buffer->has<string> (acid_var_name));

    NullableVector<string>& acid_vec = buffer->get<string> (acid_var_name);

    NullableVector<string>* cs_fpl_vec {nullptr}; // only set in cat062

    if (dbcontent_name == "CAT062")
    {
        traced_assert(variableResolver().canGetVariable(
                    dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_));

        string cs_fpl_var_name = variableResolver().getVariableName(
                    dbcontent_name, dbcontent_vars::var_cat062_callsign_fpl_);

        traced_assert(buffer->has<string> (cs_fpl_var_name));

        cs_fpl_vec = &buffer->get<string> (cs_fpl_var_name);
    }

    bool found;

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        if (acid_vec.isNull(cnt)
                && (cs_fpl_vec != nullptr ? cs_fpl_vec->isNull(cnt) : true)) // null or not found
        {
            if (!null_wanted_)
                to_be_removed.push_back(cnt);

            continue;
        }
        else
        {
            found = false;

            if (!acid_vec.isNull(cnt))
            {
                for (auto& val_it : values_)
                {
                    if (acid_vec.get(cnt).find(val_it) != std::string::npos)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (cs_fpl_vec && !cs_fpl_vec->isNull(cnt))
            {
                for (auto& val_it : values_)
                {
                    if (cs_fpl_vec->get(cnt).find(val_it) != std::string::npos)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (!found)
                to_be_removed.push_back(cnt);
        }
    }

    loginf << "content " << dbcontent_name << " erase '" << values_str_ << "' num "
           << to_be_removed.size() << " total " << buffer->size();

    return to_be_removed;
}

bool ACIDFilter::updateValuesFromStr(const std::string& values_str)
{
    set<string> values_tmp;
    vector<string> split_str = String::split(values_str, ',');

    null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        if (String::trim(tmp_str) == "NULL" || String::trim(tmp_str) == "null")
        {
            null_wanted_ = true;
            continue;
        }

        values_tmp.insert(boost::algorithm::to_upper_copy(String::trim(tmp_str)));
    }

    values_ = values_tmp;

    return true;
}
