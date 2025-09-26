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
    updateRUsFromStr(rus_str_);

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

    stringstream ss;

    if (active_ && (values_.size() || null_wanted_))
    {
        traced_assert(dbcontent_name == "CAT020");

        DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

        traced_assert(dbcontent_man.canGetVariable(dbcontent_name, DBContent::var_cat020_crontrib_recv_));
        std::string dbcol_name =
            dbcontent_man.getVariable(dbcontent_name, DBContent::var_cat020_crontrib_recv_).dbColumnName();

        if (!first)
            ss << " AND";

        // SELECT * FROM data WHERE json_contains(json_list, '3') OR json_contains(json_list, '7');
        // SELECT * FROM data WHERE json_contains_any(json_list, '[3, 7]');

        ss << " (";

        bool first_value=true;

        if (values_.size() == 1)
        {
            ss << "json_contains(" << dbcol_name << ", '" << rus_str_ << "')";

            first_value = false;
        }
        else if (values_.size() > 1)
        {
            for (auto& value : values_)
            {
                if (!first_value)
                    ss << " OR ";

                ss << "json_contains(" << dbcol_name << ", '" << value << "')";

                first_value = false;
            }
        }

        if (null_wanted_)
        {
            if (!first_value)
                ss << " OR ";

            ss << dbcol_name << " IS NULL";
        }

        ss << ")";

        first = false;
    }

    logerr << "here '" << ss.str() << "'";

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

    updateRUsFromStr(rus_str_);

    if (widget())
        widget()->update();
}

std::string MLATRUFilter::rus() const
{
    return rus_str_;
}

void MLATRUFilter::rus(const std::string& rus_str)
{
    if (!updateRUsFromStr(rus_str)) // false on failure
    {
//        if (widget_)
//            widget_->update();

        return;
    }

    rus_str_ = rus_str;
}

bool MLATRUFilter::updateRUsFromStr(const std::string& values_str)
{
    values_.clear();

    vector<unsigned int> values_tmp;
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

        unsigned int utn_tmp = QString(tmp_str.c_str()).toInt(&ok);

        if (!ok)
        {
            logerr << "utn '" << tmp_str << "' not valid";
            break;
        }

        values_tmp.push_back(utn_tmp);
    }

    if (!ok)
        return false;

    values_ = values_tmp;

    return true;
}
