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

#include "tableviewdatasource.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "logger.h"
#include "global.h"

#include <QMessageBox>

#include <algorithm>
#include "traced_assert.h"

#include "json.hpp"

using namespace std;
using namespace nlohmann;
using namespace dbContent;

const string DEFAULT_SET_NAME {"Default"};

TableViewDataSource::TableViewDataSource(const std::string& class_id,
                                             const std::string& instance_id, 
                                             Configurable* parent)
:   QObject()
,   Configurable(class_id, instance_id, parent)
{
    createSubConfigurables();
}

TableViewDataSource::~TableViewDataSource()
{
    unshowViewPoint(nullptr); // removes tmps TODO not done yet
}

void TableViewDataSource::generateSubConfigurable(const std::string& class_id,
                                                    const std::string& instance_id)
{
    logdbg << "class_id " << class_id
           << " instance_id " << instance_id;

    if (class_id.compare("VariableOrderedSet") == 0)
    {
        traced_assert(!set_);

        set_.reset(new VariableOrderedSet(class_id, instance_id, this));

        connect(set_.get(), &VariableOrderedSet::setChangedSignal, this,
                &TableViewDataSource::setChangedSignal, Qt::UniqueConnection);

        //some variable modifications result in a reload
        connect(set_.get(), &VariableOrderedSet::variableAddedChangedSignal, this,
                &TableViewDataSource::reloadNeeded, Qt::UniqueConnection);
    }
    else
    {
        throw std::runtime_error(
            "TableViewDataSource: generateSubConfigurable: unknown class_id " + class_id);
    }
}

void TableViewDataSource::checkSubConfigurables()
{
    //DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (!set_)
    {
        generateSubConfigurable("VariableOrderedSet", DEFAULT_SET_NAME);
        traced_assert(set_);
        addDefaultVariables(*set_.get());
    }

//    if (!hasSet("ADS-B Quality"))
//    {
//        DBContent& cat021_cont = dbcont_man.dbContent("CAT021");

//        generateSubConfigurable("VariableOrderedSet", "ADS-B Quality");
//        traced_assert(hasSet("ADS-B Quality"));

//        std::unique_ptr<dbContent::VariableOrderedSet>& set = sets_.at("ADS-B Quality");

//        addDefaultVariables(*set.get());

//        traced_assert(cat021_cont.hasVariable(DBContent::var_cat021_mops_version_.name()));
//        set->add(cat021_cont.variable(DBContent::var_cat021_mops_version_.name()));

//        traced_assert(cat021_cont.hasVariable(DBContent::var_cat021_nacp_.name()));
//        set->add(cat021_cont.variable(DBContent::var_cat021_nacp_.name()));

//        traced_assert(cat021_cont.hasVariable(DBContent::var_cat021_sil_.name()));
//        set->add(cat021_cont.variable(DBContent::var_cat021_sil_.name()));

//        traced_assert(cat021_cont.hasVariable(DBContent::var_cat021_nucp_nic_.name()));
//        set->add(cat021_cont.variable(DBContent::var_cat021_nucp_nic_.name()));
//    }

//    if (!hasSet("Horizontal Movement"))
//    {
//        generateSubConfigurable("VariableOrderedSet", "Horizontal Movement");
//        traced_assert(hasSet("Horizontal Movement"));

//        std::unique_ptr<dbContent::VariableOrderedSet>& set = sets_.at("Horizontal Movement");

//        addDefaultVariables(*set.get());

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_angle_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_track_angle_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_ground_speed_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_ground_speed_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_horizontal_man_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_horizontal_man_.name()));
//    }

//    if (!hasSet("Mode A/C Info"))
//    {
//        generateSubConfigurable("VariableOrderedSet", "Mode A/C Info");
//        traced_assert(hasSet("Mode A/C Info"));

//        std::unique_ptr<dbContent::VariableOrderedSet>& set = sets_.at("Mode A/C Info");

//        addDefaultVariables(*set.get());

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_m3a_g_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_m3a_g_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_m3a_v_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_m3a_v_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_m3a_smoothed_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_m3a_smoothed_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_mc_g_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_mc_g_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_mc_v_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_mc_v_.name()));
//    }

//    if (!hasSet("Track Lifetime"))
//    {
//        generateSubConfigurable("VariableOrderedSet", "Track Lifetime");
//        traced_assert(hasSet("Track Lifetime"));

//        std::unique_ptr<dbContent::VariableOrderedSet>& set = sets_.at("Track Lifetime");

//        addDefaultVariables(*set.get());

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_begin_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_track_begin_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_confirmed_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_track_confirmed_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_coasting_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_track_coasting_.name()));

//        if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_end_.name()))
//            set->add(dbcont_man.metaVariable(DBContent::meta_var_track_end_.name()));
//    }
}

VariableOrderedSet* TableViewDataSource::getSet()
{
    traced_assert(set_);
    return set_.get();
}

void TableViewDataSource::unshowViewPoint (const ViewableDataConfig* vp)
{
    for (auto& var_it : temporary_added_variables_)
    {
        loginf << "removing var " << var_it.first << ", " << var_it.second;

        removeTemporaryVariable(var_it.first, var_it.second);
    }

    temporary_added_variables_.clear();
}

void TableViewDataSource::showViewPoint (const ViewableDataConfig* vp)
{
    traced_assert(vp);
    const json& data = vp->data();

//    "context_variables": {
//        "Tracker": [
//            "mof_long",
//            "mof_trans",
//            "mof_vert"
//        ]
//    }

    if (data.contains("context_variables"))
    {
        const json& context_variables = data.at("context_variables");
        traced_assert(context_variables.is_object());

        for (auto& obj_it : context_variables.get<json::object_t>())
        {
            string dbcontent_name = obj_it.first;
            json& variable_names = obj_it.second;

            traced_assert(variable_names.is_array());

            for (auto& var_it : variable_names.get<json::array_t>())
            {
                string var_name = var_it;

                if (addTemporaryVariable(dbcontent_name, var_name))
                {
                    loginf << "added var " << dbcontent_name << ", " << var_name;
                    temporary_added_variables_.push_back({dbcontent_name, var_name});
                }
            }
        }
    }
}

bool TableViewDataSource::addTemporaryVariable (const std::string& dbcontent_name, const std::string& var_name)
{
    loginf << "dbcontent_name '" << dbcontent_name
           << "' var_name '" << var_name << "'";

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();
    
    traced_assert(set_);
    if (dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(dbcont_man.existsMetaVariable(var_name));
        MetaVariable& meta_var = dbcont_man.metaVariable(var_name);
        if (!getSet()->hasMetaVariable(meta_var))
        {
            getSet()->add(meta_var);
            return true;
        }
        else
            return false;
    }
    else
    {
        traced_assert(dbcont_man.existsDBContent(dbcontent_name));
        DBContent& obj = dbcont_man.dbContent(dbcontent_name);

        traced_assert(obj.hasVariable(var_name));
        Variable& var = obj.variable(var_name);

        if (!getSet()->hasVariable(var))
        {
            getSet()->add(var);
            return true;
        }
        else
            return false;
    }
}

void TableViewDataSource::removeTemporaryVariable (const std::string& dbcontent_name, const std::string& var_name)
{
//    auto el = find(temporary_added_variables_.begin(), temporary_added_variables_.end(),
//                   pair<string, string>{dbcontent_name, var_name});
//    traced_assert(el != temporary_added_variables_.end());
//    temporary_added_variables_.erase(el);

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(dbcont_man.existsMetaVariable(var_name));
        MetaVariable& meta_var = dbcont_man.metaVariable(var_name);
        traced_assert(getSet()->hasMetaVariable(meta_var));
        getSet()->removeMetaVariable(meta_var);
    }
    else
    {
        traced_assert(dbcont_man.existsDBContent(dbcontent_name));
        DBContent& obj = dbcont_man.dbContent(dbcontent_name);

        traced_assert(obj.hasVariable(var_name));
        Variable& var = obj.variable(var_name);
        traced_assert(getSet()->hasVariable(var));
        getSet()->removeVariable(var);
    }
}

void TableViewDataSource::addDefaultVariables (VariableOrderedSet& set)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    // Timestamp
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_timestamp_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_timestamp_.name()));

    // Datasource
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_ds_id_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_ds_id_.name()));

    // Mode 3/A code
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_m3a_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_m3a_.name()));

    // Mode S TA
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_acad_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_acad_.name()));

    // Mode S Callsign
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_acid_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_acid_.name()));

    // Mode C
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_mc_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_mc_.name()));

    // Track Number
    if (dbcont_man.existsMetaVariable(DBContent::meta_var_track_num_.name()))
        set.add(dbcont_man.metaVariable(DBContent::meta_var_track_num_.name()));
}
