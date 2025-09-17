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
#include "compass.h"
#include "modecfilterwidget.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/metavariable.h"
#include "logger.h"
//#include "stringconv.h"

using namespace std;
//using namespace Utils;
using namespace nlohmann;
using namespace dbContent;

ModeCFilter::ModeCFilter(const std::string& class_id, const std::string& instance_id,
                         Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
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
    return COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_mc_.name()).existsIn(dbcont_name);
}

std::string ModeCFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    if (!dbcont_man.metaVariable(DBContent::meta_var_mc_.name()).existsIn(dbcontent_name))
        return "";

    stringstream ss;

    if (active_)
    {
        // if (!first)
        // {
        //     ss << " AND";
        // }

                // if (null_wanted_)
                // {
                //     ss << " (" << var.dbColumnName() << " IS NULL OR";
                //     ss << " (" << var.dbColumnName() << " >= " << min_value_
                //        << " AND " << var.dbColumnName() << " <= " << max_value_ << "))";

                // }
                // else
                // {
                //     ss << " (" << var.dbColumnName() << " >= " << min_value_
                //        << " AND " << var.dbColumnName() << " <= " << max_value_ << ")";
                // }

        { // first check always to enforce if null_wanted_
            dbContent::Variable& var = dbcont_man.metaGetVariable(
                dbcontent_name, DBContent::meta_var_mc_);

            if (!first)
                ss << " AND";

            ss << " (" << var.dbColumnName() << " BETWEEN " << min_value_ << " AND " << max_value_;

            if (null_wanted_)
                ss << " OR " << var.dbColumnName() << " IS NULL";

            ss << ")";

            first = false;
        }

        if (dbcontent_name == "CAT062")
        {
            {
                dbContent::Variable& var = dbcont_man.getVariable(
                    dbcontent_name, DBContent::var_cat062_baro_alt_);

                if (var.hasDBContent())
                {
                    if (!first)
                        ss << " AND";

                    ss << " (" << var.dbColumnName() << " BETWEEN " << min_value_ << " AND " << max_value_;

                    if (null_wanted_)
                        ss << " OR " << var.dbColumnName() << " IS NULL";

                    ss << ")";

                    first = false;
                }
            }

            {
                dbContent::Variable& var = dbcont_man.getVariable(
                    dbcontent_name, DBContent::var_cat062_fl_measured_);

                if (var.hasDBContent())
                {
                    if (!first)
                        ss << " AND";

                    ss << " (" << var.dbColumnName() << " BETWEEN " << min_value_ << " AND " << max_value_;

                    if (null_wanted_)
                        ss << " OR " << var.dbColumnName() << " IS NULL";

                    ss << ")";

                    first = false;
                }
            }
        }
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}

void ModeCFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("ModeCFilter: generateSubConfigurable: unknown class_id " + class_id);
}

DBFilterWidget* ModeCFilter::createWidget()
{
    return new ModeCFilterWidget(*this);
}


void ModeCFilter::checkSubConfigurables()
{
    logdbg;
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


    if (widget())
        widget()->update();
}

bool ModeCFilter::activeInLiveMode()
{
    return true;
}

std::vector<unsigned int> ModeCFilter::filterBuffer(const std::string& dbcontent_name, std::shared_ptr<Buffer> buffer)
{
    std::vector<unsigned int> to_be_removed;

    if (!COMPASS::instance().dbContentManager().metaVariable(DBContent::meta_var_mc_.name()).existsIn(dbcontent_name))
        return to_be_removed;

    dbContent::Variable& var = COMPASS::instance().dbContentManager().metaVariable(
                                                                         DBContent::meta_var_mc_.name()).getFor(dbcontent_name);

    traced_assert(buffer->has<float> (var.name()));

    NullableVector<float>& data_vec = buffer->get<float> (var.name());

    float value;

    for (unsigned int cnt=0; cnt < buffer->size(); ++cnt)
    {
        if (data_vec.isNull(cnt))
        {
            if (!null_wanted_)
                to_be_removed.push_back(cnt);

            continue;
        }

        value = data_vec.get(cnt);

        if (value < min_value_ || value > max_value_)
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

