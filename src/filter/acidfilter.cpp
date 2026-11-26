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
    return variableResolver().metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_acid_);
}

std::vector<std::string> ACIDFilter::searchedCandidates(IDBVariableResolver& resolver,
                                                        const std::string& dbcontent_name)
{
    // a selection, CAT062 "Aircraft Identification*", is searched in every candidate: a filter
    // is a search, the chosen value alone would hide a match in another candidate

    return resolver.variableSelectionCandidateVariables(
                dbcontent_name, dbcontent_vars::meta_var_acid_.name(), META_OBJECT_NAME);
}

std::string ACIDFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    if (!active_)
        return "";

    FilterClause clause = sqlFor(variableResolver(), values_, null_wanted_, dbcontent_name);

    if (clause.sql.empty())
        return "";

    stringstream ss;

    if (!first)
        ss << " AND";

    ss << " " << clause.sql;

    first = false;

    logdbg << "here '" << ss.str() << "'";

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

    std::vector<std::string> columns;

    for (const auto& var_name : searchedCandidates(resolver, dbcontent_name))
        columns.push_back(resolver.variableDBColumnName(dbcontent_name, var_name, dbcontent_name));

    if (columns.empty()) // no selection, the variable of the Meta Variable
        columns.push_back(resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_acid_));

    stringstream ss;

    ss << "(";

    bool first_val = true;

    for (const auto& val_it : values)
    {
        if (!first_val)
            ss << " OR";

        ss << " (";

        for (size_t cnt = 0; cnt < columns.size(); ++cnt)
            ss << (cnt ? " OR " : "") << columns.at(cnt) << " LIKE '%" << val_it << "%'";

        ss << ")";

        first_val = false;
    }

    if (null_wanted) // no identification in any searched variable
    {
        if (!first_val)
            ss << " OR";

        ss << " (";

        for (size_t cnt = 0; cnt < columns.size(); ++cnt)
            ss << (cnt ? " AND " : "") << columns.at(cnt) << " IS NULL";

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

    // the searched variables present in the buffer, a string selection has no column of its own
    std::vector<std::string> var_names = searchedCandidates(variableResolver(), dbcontent_name);

    if (var_names.empty()) // no selection, the variable of the Meta Variable
        var_names.push_back(variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_acid_));

    std::vector<NullableVector<string>*> vectors;

    for (const auto& var_name : var_names)
    {
        if (buffer->has<string>(var_name))
            vectors.push_back(&buffer->get<string>(var_name));
    }

    if (vectors.empty())
    {
        logerr << "content " << dbcontent_name << " has no identification variable in the buffer";
        return to_be_removed;
    }

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        bool any_value = false;
        bool found = false;

        for (NullableVector<string>* vec : vectors)
        {
            if (vec->isNull(cnt))
                continue;

            any_value = true;

            for (const auto& val_it : values_)
            {
                if (vec->get(cnt).find(val_it) != std::string::npos)
                {
                    found = true;
                    break;
                }
            }

            if (found)
                break;
        }

        if (!any_value) // null, kept only when null is wanted
        {
            if (!null_wanted_)
                to_be_removed.push_back(cnt);
        }
        else if (!found)
            to_be_removed.push_back(cnt);
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
