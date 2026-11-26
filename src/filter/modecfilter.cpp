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

#include "modecfilter.h"
#include "modecfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "buffer/buffer.h"
#include "logger.h"

using namespace std;
using namespace nlohmann;
using namespace dbContent;

ModeCFilter::ModeCFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{
    registerParameter("min_value", &min_value_, -1000.0f);
    registerParameter("max_value", &max_value_, 10000.0f);
    registerParameter("null_wanted", &null_wanted_, false);

    name_ = "Mode C Codes";

    createSubConfigurables();
}

ModeCFilter::~ModeCFilter() {}

bool ModeCFilter::filters(const std::string& dbcont_name)
{
    return variableResolver().metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_mc_);
}

std::vector<std::string> ModeCFilter::searchedCandidates(IDBVariableResolver& resolver,
                                                         const std::string& dbcontent_name)
{
    // a selection, CAT062 "Mode C Code", is searched in every candidate: a filter is a search,
    // the chosen value alone would hide a match in another candidate

    return resolver.variableSelectionCandidateVariables(
                dbcontent_name, dbcontent_vars::meta_var_mc_.name(), META_OBJECT_NAME);
}

std::string ModeCFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    if (!active_)
        return "";

    FilterClause clause = sqlFor(variableResolver(), min_value_, max_value_, null_wanted_, dbcontent_name);

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

FilterClause ModeCFilter::getClause(const std::string& dbcontent_name)
{
    if (!active_)
        return FilterClause{};

    return sqlFor(variableResolver(), min_value_, max_value_, null_wanted_, dbcontent_name);
}

FilterClause ModeCFilter::sqlFor(IDBVariableResolver& resolver,
                                 float min, float max, bool null_wanted,
                                 const std::string& dbcontent_name)
{
    FilterClause clause;

    if (!resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
        return clause;

    std::vector<std::string> columns;

    for (const auto& var_name : searchedCandidates(resolver, dbcontent_name))
        columns.push_back(resolver.variableDBColumnName(dbcontent_name, var_name, dbcontent_name));

    if (columns.empty()) // no selection, the variable of the Meta Variable
        columns.push_back(resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_mc_));

    // stream-format the bounds to match the legacy default float formatting
    stringstream ss;

    if (columns.size() == 1) // the legacy form of a single column
    {
        ss << "(" << columns.front() << " BETWEEN " << min << " AND " << max;

        if (null_wanted)
            ss << " OR " << columns.front() << " IS NULL";

        ss << ")";
    }
    else // any candidate in range, null when no candidate has a value
    {
        ss << "(";

        for (size_t cnt = 0; cnt < columns.size(); ++cnt)
            ss << (cnt ? " OR " : "") << "(" << columns.at(cnt) << " BETWEEN " << min << " AND " << max << ")";

        if (null_wanted)
        {
            ss << " OR (";

            for (size_t cnt = 0; cnt < columns.size(); ++cnt)
                ss << (cnt ? " AND " : "") << columns.at(cnt) << " IS NULL";

            ss << ")";
        }

        ss << ")";
    }

    clause.sql = ss.str();
    return clause;
}

DBFilterWidget* ModeCFilter::createWidget()
{
    return new ModeCFilterWidget(*this);
}


void ModeCFilter::reset()
{
    widget_->update();
}

void ModeCFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Barometric Altitude Minimum"] = min_value_;
    filter["Barometric Altitude Maximum"] = max_value_;
    filter["Barometric Altitude NULL"] = null_wanted_;
}

void ModeCFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Barometric Altitude Minimum"));
    min_value_ = filter.at("Barometric Altitude Minimum");

    traced_assert(filter.contains("Barometric Altitude Maximum"));
    max_value_ = filter.at("Barometric Altitude Maximum");

    if (filter.contains("Barometric Altitude NULL"))
        null_wanted_ = filter.at("Barometric Altitude NULL");
    else
        null_wanted_ = false;


    if (widget_)
        widget_->update();
}

bool ModeCFilter::activeInLiveMode()
{
    return true;
}

std::vector<unsigned int> ModeCFilter::filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
        return to_be_removed;

    // the searched variables present in the buffer
    std::vector<std::string> var_names = searchedCandidates(variableResolver(), dbcontent_name);

    if (var_names.empty()) // no selection, the variable of the Meta Variable
        var_names.push_back(variableResolver().metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_mc_));

    std::vector<NullableVector<float>*> vectors;

    for (const auto& var_name : var_names)
    {
        if (buffer->has<float>(var_name))
            vectors.push_back(&buffer->get<float>(var_name));
    }

    if (vectors.empty())
    {
        logerr << "content " << dbcontent_name << " has no Mode C variable in the buffer";
        return to_be_removed;
    }

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        bool any_value = false;
        bool in_range = false;

        for (NullableVector<float>* vec : vectors)
        {
            if (vec->isNull(cnt))
                continue;

            any_value = true;

            float value = vec->get(cnt);

            if (value >= min_value_ && value <= max_value_)
            {
                in_range = true;
                break;
            }
        }

        if (!any_value) // null, kept only when null is wanted
        {
            if (!null_wanted_)
                to_be_removed.push_back(cnt);
        }
        else if (!in_range)
            to_be_removed.push_back(cnt);
    }

    return to_be_removed;
}

float ModeCFilter::minValue() const
{
    return min_value_;
}

void ModeCFilter::minValue(float min_value)
{
    min_value_ = min_value;

    loginf << "min_value " << min_value_;
}

float ModeCFilter::maxValue() const
{
    return max_value_;
}

void ModeCFilter::maxValue(float max_value)
{
    max_value_ = max_value;

    loginf << "max_value " << max_value_;
}

bool ModeCFilter::nullWanted() const
{
    return null_wanted_;
}

void ModeCFilter::nullWanted(bool null_wanted)
{
    null_wanted_ = null_wanted;
}
