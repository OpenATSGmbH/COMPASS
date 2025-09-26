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

#include "scatterplotviewdatasource.h"

#include <QMessageBox>

#include "compass.h"
//#include "configuration.h"
//#include "configurationmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
//#include "job.h"
#include "logger.h"
//#include "viewpoint.h"
#include "global.h"

#include <algorithm>
#include "traced_assert.h"

#include "json.hpp"

using namespace std;
using namespace nlohmann;

ScatterPlotViewDataSource::ScatterPlotViewDataSource(const std::string& class_id,
                                             const std::string& instance_id, Configurable* parent)
    : QObject(),
      Configurable(class_id, instance_id, parent)
{
    createSubConfigurables();
}

ScatterPlotViewDataSource::~ScatterPlotViewDataSource()
{
    unshowViewPoint(nullptr); // removes tmps TODO not done yet

    if (set_)
    {
        delete set_;
        set_ = nullptr;
    }
}

void ScatterPlotViewDataSource::generateSubConfigurable(const std::string& class_id,
                                                    const std::string& instance_id)
{
    logdbg << "class_id " << class_id
           << " instance_id " << instance_id;

    if (class_id.compare("VariableOrderedSet") == 0)
    {
        traced_assert(set_ == 0);
        set_ = new dbContent::VariableOrderedSet(class_id, instance_id, this);
    }
    else
        throw std::runtime_error(
            "ScatterPlotViewDataSource: generateSubConfigurable: unknown class_id " + class_id);
}

void ScatterPlotViewDataSource::checkSubConfigurables()
{
    if (set_ == nullptr)
    {
        generateSubConfigurable("VariableOrderedSet", "VariableOrderedSet0");
        traced_assert(set_);

        DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

        if (dbcont_man.existsMetaVariable("rec_num"))
            set_->add(dbcont_man.metaVariable("rec_num"));
    }
}

void ScatterPlotViewDataSource::unshowViewPoint (const ViewableDataConfig* vp)
{
    for (auto& var_it : temporary_added_variables_)
    {
        loginf << "removing var " << var_it.first << ", " << var_it.second;

        removeTemporaryVariable(var_it.first, var_it.second);
    }

    temporary_added_variables_.clear();
}

void ScatterPlotViewDataSource::showViewPoint (const ViewableDataConfig* vp)
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

bool ScatterPlotViewDataSource::addTemporaryVariable (const std::string& dbcontent_name, const std::string& var_name)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(dbcont_man.existsMetaVariable(var_name));
        dbContent::MetaVariable& meta_var = dbcont_man.metaVariable(var_name);
        if (!set_->hasMetaVariable(meta_var))
        {
            set_->add(meta_var);
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
        dbContent::Variable& var = obj.variable(var_name);

        if (!set_->hasVariable(var))
        {
            set_->add(var);
            return true;
        }
        else
            return false;
    }
}

void ScatterPlotViewDataSource::removeTemporaryVariable (const std::string& dbcontent_name, const std::string& var_name)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcontent_name == META_OBJECT_NAME)
    {
        traced_assert(dbcont_man.existsMetaVariable(var_name));
        dbContent::MetaVariable& meta_var = dbcont_man.metaVariable(var_name);
        traced_assert(set_->hasMetaVariable(meta_var));
        set_->removeMetaVariable(meta_var);
    }
    else
    {
        traced_assert(dbcont_man.existsDBContent(dbcontent_name));
        DBContent& obj = dbcont_man.dbContent(dbcontent_name);

        traced_assert(obj.hasVariable(var_name));
        dbContent::Variable& var = obj.variable(var_name);
        traced_assert(set_->hasVariable(var));
        set_->removeVariable(var);
    }
}


