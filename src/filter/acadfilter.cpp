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

#include "acadfilter.h"
#include "compass.h"
#include "acadfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

ACADFilter::ACADFilter(const std::string& class_id, const std::string& instance_id,
                       Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("values_str", &values_str_, std::string());
    updateValuesFromStr(values_str_);

    name_ = "Aircraft Address";

    createSubConfigurables();
}

ACADFilter::~ACADFilter() {}

bool ACADFilter::filters(const std::string& dbcont_name)
{
    return COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_acad_.name()).existsIn(dbcont_name);
}

std::string ACADFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_acad_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_ && (values_.size() || null_wanted_))
    {
        dbContent::Variable& var = COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_acad_.name()).getFor(dbcontent_name);

        if (!first)
            ss << " AND";

        ss << " ";

        if (null_wanted_)
            ss << "(";

        if (values_.size())
        {
            ss << var.dbColumnName() << " IN (" << String::compress(values_, ',') << ")";
        }

        if (null_wanted_)
        {
            if (values_.size())
                ss << " OR";

            ss << " " << var.dbColumnName() << " IS NULL)";
        }

        first = false;
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}

void ACADFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("ACADFilter: generateSubConfigurable: unknown class_id " + class_id);
}

void ACADFilter::checkSubConfigurables()
{
    logdbg;
}

DBFilterWidget* ACADFilter::createWidget()
{
    return new ACADFilterWidget(*this);
}


void ACADFilter::reset()
{
    if (widget_)
        widget_->update();
}

void ACADFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Aircraft Address Values"] = values_str_;
}

void ACADFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Aircraft Address Values"));
    values_str_ = filter.at("Aircraft Address Values");

    updateValuesFromStr(values_str_);

    if (widget())
        widget()->update();
}

std::string ACADFilter::valuesString() const
{
    return values_str_;
}

void ACADFilter::valuesString(const std::string& values_str)
{
    if (!updateValuesFromStr(values_str)) // false on failure
        return;

    values_str_ = values_str;
}

bool ACADFilter::activeInLiveMode()
{
    return true;
}

std::vector<unsigned int> ACADFilter::filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_acad_.name()).existsIn(dbcontent_name))
        return to_be_removed;

    dbContent::Variable& var = COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_acad_.name()).getFor(dbcontent_name);

    traced_assert(buffer->has<unsigned int> (var.name()));

    NullableVector<unsigned int>& data_vec = buffer->get<unsigned int> (var.name());

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        if (data_vec.isNull(cnt))
        {
            if (!null_wanted_)
                to_be_removed.push_back(cnt);

            continue;
        }
        else if (!values_.count(data_vec.get(cnt)))
            to_be_removed.push_back(cnt);
    }

    return to_be_removed;
}

bool ACADFilter::updateValuesFromStr(const std::string& values_str)
{
    set<unsigned int> values_tmp;
    vector<string> split_str = String::split(values_str, ',');

    bool ok = true;

    null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        if (String::trim(tmp_str) == "NULL" || String::trim(tmp_str) == "null")
        {
            null_wanted_ = true;
            continue;
        }

        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok, 16);

        if (!ok)
        {
            logerr << "utn '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.insert(utn_tmp);
    }

    if (!ok)
        return false;

    values_ = values_tmp;

    return true;
}
