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

#include "reftrajaccuracyfilter.h"
#include "compass.h"
#include "reftrajaccuracyfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

RefTrajAccuracyFilter::RefTrajAccuracyFilter(const std::string& class_id, const std::string& instance_id,
                       Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{
    registerParameter("min_value", &min_value_, 30.0f);

    name_ = "RefTraj Accuracy";

    createSubConfigurables();
}

RefTrajAccuracyFilter::~RefTrajAccuracyFilter() {}

bool RefTrajAccuracyFilter::filters(const std::string& dbcontent_name)
{
    return dbcontent_name == "RefTraj";
}

std::string RefTrajAccuracyFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "start" << dbcontent_name << " active " << active_;

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_mc_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_)
    {
        dbContent::Variable& x_stddev_var = COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_x_stddev_.name()).getFor(dbcontent_name);

        dbContent::Variable& y_stddev_var = COMPASS::instance().dbContentManager().metaVariable(
                    DBContent::meta_var_y_stddev_.name()).getFor(dbcontent_name);

        if (!first)
        {
            ss << " AND";
        }

            ss << " sqrt(pow(" << x_stddev_var.dbColumnName() << ",2) + (pow("
               << y_stddev_var.dbColumnName() << ",2))) <= " << min_value_ << "";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

void RefTrajAccuracyFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("RefTrajAccuracyFilter: generateSubConfigurable: unknown class_id " + class_id);
}

DBFilterWidget* RefTrajAccuracyFilter::createWidget()
{
    return new RefTrajAccuracyFilterWidget(*this);
}


void RefTrajAccuracyFilter::checkSubConfigurables()
{
    logdbg;
}


void RefTrajAccuracyFilter::reset()
{
    widget_->update();
}

void RefTrajAccuracyFilter::saveViewPointConditions (nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(!filters.contains(name_));
    filters[name_] = json::object();
    json& filter = filters.at(name_);

    filter["Accuracy Minimum"] = min_value_;
}

void RefTrajAccuracyFilter::loadViewPointConditions (const nlohmann::json& filters)
{
    traced_assert(conditions_.size() == 0);

    traced_assert(filters.contains(name_));
    const json& filter = filters.at(name_);

    traced_assert(filter.contains("Accuracy Minimum"));
    string value = filter.at("Accuracy Minimum");
    min_value_ = std::stod(value);

    if (widget())
        widget()->update();
}

float RefTrajAccuracyFilter::minValue() const
{
    return min_value_;
}

void RefTrajAccuracyFilter::minValue(float min_value)
{
    min_value_ = min_value;
}


