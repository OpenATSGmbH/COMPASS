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
//#include "stringconv.h"

using namespace std;
//using namespace Utils;
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
    return variableResolver().metaCanGetVariable(dbcont_name, DBContent::meta_var_mc_);
}

std::string ModeCFilter::getConditionString(const std::string& dbcontent_name, dbContent::VariableSet& read_set, bool& first)
{
    logdbg << "dbcont " << dbcontent_name << " active " << active_;

    auto& resolver = variableResolver();

    if (!resolver.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
        return "";

    stringstream ss;

    if (active_)
    {
        { // first check always to enforce if null_wanted_
            string col_name = resolver.metaGetVariableDBColumn(
                dbcontent_name, DBContent::meta_var_mc_);

            if (!first)
                ss << " AND";

            ss << " (" << col_name << " BETWEEN " << min_value_ << " AND " << max_value_;

            if (null_wanted_)
                ss << " OR " << col_name << " IS NULL";

            ss << ")";

            first = false;
        }

        if (dbcontent_name == "CAT062")
        {
            {
                if (resolver.variableHasDBContent(dbcontent_name, DBContent::var_cat062_baro_alt_))
                {
                    string col_name = resolver.getVariableDBColumn(
                        dbcontent_name, DBContent::var_cat062_baro_alt_);

                    if (!first)
                        ss << " AND";

                    ss << " (" << col_name << " BETWEEN " << min_value_ << " AND " << max_value_;

                    if (null_wanted_)
                        ss << " OR " << col_name << " IS NULL";

                    ss << ")";

                    first = false;
                }
            }

            {
                if (resolver.variableHasDBContent(dbcontent_name, DBContent::var_cat062_fl_measured_))
                {
                    string col_name = resolver.getVariableDBColumn(
                        dbcontent_name, DBContent::var_cat062_fl_measured_);

                    if (!first)
                        ss << " AND";

                    ss << " (" << col_name << " BETWEEN " << min_value_ << " AND " << max_value_;

                    if (null_wanted_)
                        ss << " OR " << col_name << " IS NULL";

                    ss << ")";

                    first = false;
                }
            }
        }
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
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

    if (!variableResolver().metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
        return to_be_removed;

    string var_name = variableResolver().metaGetVariableName(dbcontent_name, DBContent::meta_var_mc_);

    traced_assert(buffer->has<float> (var_name));

    NullableVector<float>& data_vec = buffer->get<float> (var_name);

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

