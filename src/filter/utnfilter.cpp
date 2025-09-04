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

#include "utnfilter.h"
#include "compass.h"
#include "utnfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
#include "stringconv.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

UTNFilter::UTNFilter(const std::string& class_id, const std::string& instance_id,
                     Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("utns_str", &utns_str_, std::string());
    updateUTNSFromStr(utns_str_);

    name_ = "UTNs";

    createSubConfigurables();
}

UTNFilter::~UTNFilter() {}

bool UTNFilter::filters(const std::string& dbcont_name)
{
    if (!COMPASS::instance().dbContentManager().hasAssociations())
        return false;

    return true; // condition string for non-associated dbcontent as well
}

std::string UTNFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcontent " << dbcontent_name << " active " << active_
           << " null_wanted " << null_wanted_;

    if (!active_ || !COMPASS::instance().dbContentManager().hasAssociations())
        return "";

    stringstream ss;

    // check if filter non-associated content
    if (!COMPASS::instance().dbContentManager().metaCanGetVariable(dbcontent_name, DBContent::meta_var_utn_))
    {
        if (!null_wanted_)
        {
            if (!first)
                ss << " AND";

            ss << " false";

            first = false;

            return ss.str();
        }
        // else no condition

        logdbg << "condition '" << ss.str() << "'";
        return ss.str();
    }

    if (values_.size() || null_wanted_)
    {
        dbContent::Variable& var = COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_utn_.name()).getFor(dbcontent_name);

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

    logdbg << "condition '" << ss.str() << "'";

    return ss.str();
}

void UTNFilter::generateSubConfigurable(const std::string& class_id,
                                        const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("UTNFilter: generateSubConfigurable: unknown class_id " + class_id);
}

void UTNFilter::checkSubConfigurables()
{
    logdbg;

}

DBFilterWidget* UTNFilter::createWidget()
{
    return new UTNFilterWidget(*this);
}


void UTNFilter::reset()
{
    if (widget_)
        widget_->update();
}

void UTNFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["utns"] = utns_str_;
}

void UTNFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("utns"));
    utns_str_ = filter.at("utns");

    updateUTNSFromStr(utns_str_);

    if (widget())
        widget()->update();
}

std::string UTNFilter::utns() const
{
    return utns_str_;
}

void UTNFilter::utns(const std::string& utns)
{
    if (!updateUTNSFromStr(utns)) // false on failure
    {
//        if (widget_)
//            widget_->update();

        return;
    }

    utns_str_ = utns;
}

const string null_str_1 = "NULL";
const string null_str_2 = "null";

bool UTNFilter::updateUTNSFromStr(const std::string& values_str)
{
    values_.clear();

    vector<unsigned int> values_tmp;
    vector<string> split_str = String::split(values_str, ',');

    bool ok = true;
    null_wanted_ = false;

    for (auto& tmp_str : split_str)
    {
        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok);

        if (!ok)
        {
            string tmp_str_trim = String::trim(tmp_str);

            if (tmp_str_trim == null_str_1 || tmp_str_trim == null_str_2)
            {
                null_wanted_ = true;
                continue;
            }
            else if (null_str_1.find(tmp_str_trim) != std::string::npos
                     || null_str_2.find(tmp_str_trim) != std::string::npos) // part null string
            {
                continue;
            }
            else
            {
                logerr << "utn '" << tmp_str << "' not valid";
                break;
            }
        }

        values_tmp.push_back(utn_tmp);
    }

    if (!ok)
        return false;

    values_ = values_tmp;

    return true;
}
