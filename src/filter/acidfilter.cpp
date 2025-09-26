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
#include "compass.h"
#include "acidfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

ACIDFilter::ACIDFilter(const std::string& class_id, const std::string& instance_id,
                       Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
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
        return COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_acid_.name()).existsIn(dbcont_name);
}

std::string ACIDFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_acid_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_  && (values_.size() || null_wanted_))
    {
        dbContent::Variable& acid_var = COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_acid_.name()).getFor(dbcontent_name);

        dbContent::Variable* cs_fpl_var {nullptr}; // only set in cat062

        if (dbcontent_name == "CAT062")
        {
            traced_assert(COMPASS::instance().dbContentManager().canGetVariable(
                        dbcontent_name, DBContent::var_cat062_callsign_fpl_));

            cs_fpl_var = &COMPASS::instance().dbContentManager().getVariable(
                        dbcontent_name, DBContent::var_cat062_callsign_fpl_);
        }

        if (!first)
            ss << " AND";

        ss << " (";

        bool first_val = true;

        for (auto val_it : values_)
        {
            if (!first_val)
                ss << " OR";

             ss << " (" << acid_var.dbColumnName()  << " LIKE '%" << val_it << "%'";

             if (cs_fpl_var)
                ss << " OR " << cs_fpl_var->dbColumnName()  << " LIKE '%" << val_it << "%'";

             ss << ")";

            first_val = false;
        }

        if (null_wanted_)
        {
            if (!first_val)
                ss << " OR";

            ss << " (" << acid_var.dbColumnName()  << " IS NULL";

            if (cs_fpl_var)
               ss << " OR " << cs_fpl_var->dbColumnName()  << " IS NULL";

            ss << ")";
        }

        ss << ")";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

void ACIDFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("ACIDFilter: generateSubConfigurable: unknown class_id " + class_id);
}

void ACIDFilter::checkSubConfigurables()
{
    logdbg;
}

DBFilterWidget* ACIDFilter::createWidget()
{
    return new ACIDFilterWidget(*this);
}

void ACIDFilter::reset()
{
    if (widget())
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

    if (widget())
        widget()->update();
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

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_acid_.name()).existsIn(dbcontent_name))
        return to_be_removed;

    dbContent::Variable& acid_var = COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_acid_.name()).getFor(dbcontent_name);

    traced_assert(buffer->has<string> (acid_var.name()));

    NullableVector<string>& acid_vec = buffer->get<string> (acid_var.name());

    dbContent::Variable* cs_fpl_var {nullptr}; // only set in cat062
    NullableVector<string>* cs_fpl_vec {nullptr}; // only set in cat062

    if (dbcontent_name == "CAT062")
    {
        traced_assert(COMPASS::instance().dbContentManager().canGetVariable(
                    dbcontent_name, DBContent::var_cat062_callsign_fpl_));

        cs_fpl_var = &COMPASS::instance().dbContentManager().getVariable(
                    dbcontent_name, DBContent::var_cat062_callsign_fpl_);

        traced_assert(buffer->has<string> (cs_fpl_var->name()));

        cs_fpl_vec = &buffer->get<string> (cs_fpl_var->name());
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
