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
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/variable/metavariable.h"
#include "compass.h"

using namespace std;
using namespace nlohmann;

PrimaryOnlyFilter::PrimaryOnlyFilter(const std::string& class_id, const std::string& instance_id,
                                     Configurable* parent)
    : DBFilter(class_id, instance_id, parent, false)
{

    createSubConfigurables();
}

PrimaryOnlyFilter::~PrimaryOnlyFilter()
{

}

bool PrimaryOnlyFilter::filters(const std::string& dbcontent_name)
{
    DBContentManager& cont_man = COMPASS::instance().dbContentManager();

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_))
        return true;

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
        return true;

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
        return true;

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_))
        return true;

    return false;
}

std::string PrimaryOnlyFilter::getConditionString(const std::string& dbcontent_name, bool& first)
{
    logdbg << "dbcont_name " << dbcontent_name << " active " << active_;

    stringstream ss;

    DBContentManager& cont_man = COMPASS::instance().dbContentManager();

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_);

        if (!first)
            ss << " AND";

        ss << " " + var.dbColumnName() << " IS NULL";

        first = false;
    }

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_);

        if (!first)
            ss << " AND";

        ss << " " + var.dbColumnName() << " IS NULL";

        first = false;
    }

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_);

        if (!first)
            ss << " AND";

        ss << " " + var.dbColumnName() << " IS NULL";

        first = false;
    }

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_);

        if (!first)
            ss << " AND";

        ss << " " + var.dbColumnName() << " IS NULL";

        first = false;
    }

    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_detection_type_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_detection_type_);

        if (!first)
            ss << " AND";

        ss << " (" + var.dbColumnName() << " IN (1,3,6,7) OR " << var.dbColumnName() << " IS NULL)";

        first = false;
    }

    logdbg << "here '" << ss.str() << "'";

    return ss.str();
}

void PrimaryOnlyFilter::generateSubConfigurable(const std::string& class_id, const std::string& instance_id)
{
    logdbg << "class_id " << class_id;

    throw std::runtime_error("PrimaryOnlyFilter: generateSubConfigurable: unknown class_id " + class_id);
}

void PrimaryOnlyFilter::checkSubConfigurables()
{
    logdbg;
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

    DBContentManager& cont_man = COMPASS::instance().dbContentManager();

    NullableVector<unsigned int>* m3a_vec {nullptr};
    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_);
        traced_assert(buffer->has<unsigned int> (var.name()));
        m3a_vec = &buffer->get<unsigned int> (var.name());
    }

    NullableVector<float>* mc_vec {nullptr};
    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_);
        traced_assert(buffer->has<float> (var.name()));
        mc_vec = &buffer->get<float> (var.name());
    }

    NullableVector<unsigned int>* ta_vec {nullptr};
    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_);
        traced_assert(buffer->has<unsigned int> (var.name()));
        ta_vec = &buffer->get<unsigned int> (var.name());
    }

    NullableVector<string>* ti_vec {nullptr};
    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_);
        traced_assert(buffer->has<string> (var.name()));
        ti_vec = &buffer->get<string> (var.name());
    }

    NullableVector<unsigned char>* type_vec {nullptr};
    if (cont_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_detection_type_))
    {
        dbContent::Variable& var = cont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_detection_type_);
        traced_assert(buffer->has<unsigned char> (var.name()));
        type_vec = &buffer->get<unsigned char> (var.name());
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
