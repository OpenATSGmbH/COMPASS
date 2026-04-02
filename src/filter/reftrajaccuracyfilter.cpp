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
#include "reftrajaccuracyfilterwidget.h"
#include "idbvariableresolver.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

RefTrajAccuracyFilter::RefTrajAccuracyFilter(nlohmann::json& config, FilterManager* parent, IDBVariableResolver& var_resolver)
    : DBFilter(config, false, parent, var_resolver)
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

std::string RefTrajAccuracyFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "start" << dbcontent_name << " active " << active_;

    if (!variableResolver().metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
        return "";

    stringstream ss;

    if (active_)
    {
        string x_col = variableResolver().metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_x_stddev_);
        string y_col = variableResolver().metaGetVariableDBColumn(dbcontent_name, dbcontent_vars::meta_var_y_stddev_);

        if (!first)
        {
            ss << " AND";
        }

            ss << " sqrt(pow(" << x_col << ",2) + (pow("
               << y_col << ",2))) <= " << min_value_ << "";

        first = false;
    }

    loginf << "here '" << ss.str() << "'";

    return ss.str();
}

DBFilterWidget* RefTrajAccuracyFilter::createWidget()
{
    return new RefTrajAccuracyFilterWidget(*this);
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
    const auto& val = filter.at("Accuracy Minimum");

    if (val.is_number())
        min_value_ = val.get<float>();
    else
        min_value_ = std::stod(val.get<string>());

    if (widget_)
        widget_->update();
}

float RefTrajAccuracyFilter::minValue() const
{
    return min_value_;
}

void RefTrajAccuracyFilter::minValue(float min_value)
{
    min_value_ = min_value;
}


