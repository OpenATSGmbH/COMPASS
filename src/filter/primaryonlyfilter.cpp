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

#include "primaryonlyfilter.h"
#include "primaryonlyfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "buffer/buffer.h"

using namespace std;
using namespace nlohmann;

PrimaryOnlyFilter::PrimaryOnlyFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
{

    createSubConfigurables();
}

PrimaryOnlyFilter::~PrimaryOnlyFilter()
{

}

bool PrimaryOnlyFilter::filters(const std::string& dbcontent_name)
{
    auto& resolver = variableResolver();

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_))
        return true;

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
        return true;

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
        return true;

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
        return true;

    return false;
}

std::string PrimaryOnlyFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    stringstream ss;

    auto& resolver = variableResolver();

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_))
    {
        string col_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_m3a_);

        if (!first)
            ss << " AND";

        ss << " " + col_name << " IS NULL";

        first = false;
    }

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
    {
        string col_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_mc_);

        if (!first)
            ss << " AND";

        ss << " " + col_name << " IS NULL";

        first = false;
    }

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
    {
        string col_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_acad_);

        if (!first)
            ss << " AND";

        ss << " " + col_name << " IS NULL";

        first = false;
    }

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
    {
        string col_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_acid_);

        if (!first)
            ss << " AND";

        ss << " " + col_name << " IS NULL";

        first = false;
    }

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_detection_type_))
    {
        string col_name = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_detection_type_);

        if (!first)
            ss << " AND";

        ss << " (" + col_name << " IN (1,3,6,7) OR " << col_name << " IS NULL)";

        first = false;
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}

FilterClause PrimaryOnlyFilter::getClause(const std::string& dbcontent_name)
{
    auto& resolver = variableResolver();

    std::vector<FilterClause> parts;

    // "primary only" = the identifying meta vars are all NULL (no secondary/ADS-B info)
    auto add_null = [&](const auto& metavar) {
        if (resolver.metaCanGetVariable(dbcontent_name, metavar))
        {
            FilterClause c;
            c.sql = resolver.metaGetVariableDBColumn(dbcontent_name, metavar) + " IS NULL";
            parts.push_back(c);
        }
    };

    add_null(dbcontent_vars::meta_var_m3a_);
    add_null(dbcontent_vars::meta_var_mc_);
    add_null(dbcontent_vars::meta_var_acad_);
    add_null(dbcontent_vars::meta_var_acid_);

    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_detection_type_))
    {
        string col = resolver.metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_detection_type_);
        FilterClause c;
        c.sql = "(" + col + " IN (1,3,6,7) OR " + col + " IS NULL)";
        parts.push_back(c);
    }

    return combineAnd(parts);
}


DBFilterWidget* PrimaryOnlyFilter::createWidget()
{
    return new PrimaryOnlyFilterWidget(*this);
}


void PrimaryOnlyFilter::reset()
{
    if (widget_)
        widget_->update();
}

void PrimaryOnlyFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
//    json& filter = filters.at(name_);

//    filter["Aircraft Address Values"] = values_str_;
}

void PrimaryOnlyFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
//    const json& filter = filters.at(name_);

//    traced_assert(filter.contains("Aircraft Address Values"));
//    values_str_ = filter.at("Aircraft Address Values");

//    updateValuesFromStr(values_str_);

//    if (widget())
//        widget()->update();
}

bool PrimaryOnlyFilter::activeInLiveMode()
{
    return true;
}

std::vector<unsigned int> PrimaryOnlyFilter::filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    auto& resolver = variableResolver();

    NullableVector<unsigned int>* m3a_vec {nullptr};
    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_))
    {
        string vn = resolver.metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_m3a_);
        traced_assert(buffer->has<unsigned int> (vn));
        m3a_vec = &buffer->get<unsigned int> (vn);
    }

    NullableVector<float>* mc_vec {nullptr};
    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
    {
        string vn = resolver.metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_mc_);
        traced_assert(buffer->has<float> (vn));
        mc_vec = &buffer->get<float> (vn);
    }

    NullableVector<unsigned int>* ta_vec {nullptr};
    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
    {
        string vn = resolver.metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_acad_);
        traced_assert(buffer->has<unsigned int> (vn));
        ta_vec = &buffer->get<unsigned int> (vn);
    }

    NullableVector<string>* ti_vec {nullptr};
    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
    {
        string vn = resolver.metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_acid_);
        traced_assert(buffer->has<string> (vn));
        ti_vec = &buffer->get<string> (vn);
    }

    NullableVector<unsigned char>* type_vec {nullptr};
    if (resolver.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_detection_type_))
    {
        string vn = resolver.metaGetVariableName(dbcontent_name, dbcontent_vars::meta_var_detection_type_);
        traced_assert(buffer->has<unsigned char> (vn));
        type_vec = &buffer->get<unsigned char> (vn);
    }

    std::set<unsigned char> psr_detection {1,3,6,7};

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        if (m3a_vec && !m3a_vec->isNull(cnt))
            to_be_removed.push_back(cnt);
        else if (mc_vec && !mc_vec->isNull(cnt))
            to_be_removed.push_back(cnt);
        else if (ta_vec && !ta_vec->isNull(cnt))
            to_be_removed.push_back(cnt);
        else if (ti_vec && !ti_vec->isNull(cnt))
            to_be_removed.push_back(cnt);
        else if (type_vec && !type_vec->isNull(cnt) && !psr_detection.count(type_vec->get(cnt)))
            to_be_removed.push_back(cnt);
    }

    return to_be_removed;
}
