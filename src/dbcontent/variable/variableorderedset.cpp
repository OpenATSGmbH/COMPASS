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

#include "dbcontent/variable/variableorderedset.h"
#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableorderedsetwidget.h"
#include "dbcontent/variable/variableset.h"
#include "dbcontent/variable/metavariable.h"
#include "global.h"

#include <algorithm>

using namespace std;

namespace dbContent
{

VariableOrderedSet::VariableOrderedSet(const std::string& class_id,
                                       const std::string& instance_id, 
                                       Configurable* parent)
    : Configurable(class_id, instance_id, parent)
{
    registerParameter("variable_definitions", &variable_definitions_, nlohmann::json::array());

    createSubConfigurables();

    // check set contents

    loginf << "checking";

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    for (auto& def_it : tmp_vec)
    {
        if (def_it.first == META_OBJECT_NAME)
        {
            if (!dbcont_man.existsMetaVariable(def_it.second))
            {
                logwrn << "outdated meta variable " << def_it.second;
                removeVariableAt(getIndexFor(def_it.first, def_it.second), false);
            }
        }
        else if (!dbcont_man.existsDBContent(def_it.first) ||
                 !dbcont_man.dbContent(def_it.first).hasVariable(def_it.second))
        {
            logwrn << "outdated dbcont name "
                   << def_it.first << " variable " << def_it.second;
            removeVariableAt(getIndexFor(def_it.first, def_it.second), false);
        }
    }

    loginf << "checking done";
}

VariableOrderedSet::~VariableOrderedSet() = default;

void VariableOrderedSet::generateSubConfigurable(const std::string& class_id,
                                                 const std::string& instance_id)
{
    logdbg << "class_id " << class_id
           << " instance_id " << instance_id;

    throw std::runtime_error("VariableOrderedSet: generateSubConfigurable: unknown class_id " + class_id);
}

void VariableOrderedSet::checkSubConfigurables() {}

void VariableOrderedSet::add(Variable& var)
{
    if (!hasVariable(var))
    {
        variable_definitions_.push_back({var.dbContentName(), var.name()});
        notifyModifications();

        emit setChangedSignal();
        emit variableAddedChangedSignal();
    }
}

void VariableOrderedSet::add(MetaVariable& var)
{
    if (!hasMetaVariable(var))
    {
        variable_definitions_.push_back({META_OBJECT_NAME, var.name()});
        notifyModifications();

        emit setChangedSignal();
        emit variableAddedChangedSignal();
    }
}

void VariableOrderedSet::add (const std::string& dbcontent_name, const std::string var_name)
{
    if (!hasVariable(dbcontent_name, var_name))
    {
        variable_definitions_.push_back({dbcontent_name, var_name});
        notifyModifications();

        emit setChangedSignal();
        emit variableAddedChangedSignal();
    }
}

void VariableOrderedSet::set(const std::vector<std::pair<std::string,std::string>>& vars)
{
    variable_definitions_.clear();

    for (const auto& var : vars)
    {
        if (!hasVariable(var.first, var.second))
            variable_definitions_.push_back({ var.first, var.second });
    }

    notifyModifications();

    emit setChangedSignal();
    emit variableAddedChangedSignal();
}

void VariableOrderedSet::removeVariableAt(unsigned int index)
{
    removeVariableAt(index, true);
}

void VariableOrderedSet::removeVariableAt(unsigned int index, bool signal_changes)
{
    loginf << "index " << index;

    traced_assert(index < variable_definitions_.size());

    variable_definitions_.erase(index);
    notifyModifications();

    if (signal_changes)
    {
        emit variableRemovedSignal();
        emit setChangedSignal();
    }
}

void VariableOrderedSet::removeVariable(const Variable& variable)
{
    traced_assert(hasVariable(variable));

    unsigned int index =  getIndexFor(variable.dbContentName(), variable.name());

    removeVariableAt(index);
}

void VariableOrderedSet::removeMetaVariable(const MetaVariable& variable)
{
    traced_assert(hasMetaVariable(variable));

    unsigned int index =  getIndexFor(META_OBJECT_NAME, variable.name());

    removeVariableAt(index);
}

template <typename t> void vec_move(std::vector<t>& v, size_t old_index, size_t new_index)
{
    if (old_index > new_index)
        std::rotate(v.rend() - old_index - 1, v.rend() - old_index, v.rend() - new_index);
    else
        std::rotate(v.begin() + old_index, v.begin() + old_index + 1, v.begin() + new_index + 1);
}

void VariableOrderedSet::moveVariableUp(unsigned int index)
{
    logdbg << "index " << index;
    traced_assert(index < variable_definitions_.size());

    if (index == 0)
    {
        logwrn << "tried to move up first variable";
        return;
    }

    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    vec_move(tmp_vec, index, index-1);

    variable_definitions_ = tmp_vec;
    notifyModifications();

    emit variableMovedSignal();
    emit setChangedSignal();
}

void VariableOrderedSet::moveVariableDown(unsigned int index)
{
    logdbg << "index " << index;
    traced_assert(index < variable_definitions_.size());

    if (index == variable_definitions_.size() - 1)
    {
        logerr << "tried to down up last variable";
        return;
    }

    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    vec_move(tmp_vec, index, index+1);

    variable_definitions_ = tmp_vec;
    notifyModifications();

    emit variableMovedSignal();
    emit setChangedSignal();
}

VariableSet VariableOrderedSet::getFor(const std::string& dbcontent_name)
{
    logdbg << "dbcontent_name " << dbcontent_name;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    VariableSet per_dbcont_set;

    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    for (auto& def_it : tmp_vec)
    {
        if (def_it.first == META_OBJECT_NAME)
        {
            traced_assert(dbcont_man.existsMetaVariable(def_it.second));
            if (dbcont_man.metaVariable(def_it.second).existsIn(dbcontent_name))
                per_dbcont_set.add(dbcont_man.metaVariable(def_it.second).getFor(dbcontent_name));
        }
        else if (def_it.first == dbcontent_name)
        {
            traced_assert(dbcont_man.existsDBContent(dbcontent_name));
            traced_assert(dbcont_man.dbContent(dbcontent_name).hasVariable(def_it.second));
            per_dbcont_set.add(dbcont_man.dbContent(dbcontent_name).variable(def_it.second));
        }
    }

    return per_dbcont_set;
}

std::vector<std::pair<std::string,std::string>> VariableOrderedSet::definitions() const
{
    return variable_definitions_.get<std::vector<std::pair<std::string,std::string>>>();
}

std::pair<std::string,std::string> VariableOrderedSet::variableDefinition(unsigned int index) const
{
    traced_assert(index < variable_definitions_.size());
    return definitions().at(index);
}

unsigned int VariableOrderedSet::getSize() const
{
    return variable_definitions_.size();
}

bool VariableOrderedSet::hasVariable(const Variable& variable) const
{
    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    return std::find(tmp_vec.begin(), tmp_vec.end(),
                     std::pair<std::string,std::string>{variable.dbContent().name(), variable.name()}) != tmp_vec.end();
}

bool VariableOrderedSet::hasMetaVariable(const MetaVariable& variable) const
{
    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    return std::find(tmp_vec.begin(), tmp_vec.end(),
                     std::pair<std::string,std::string>{META_OBJECT_NAME, variable.name()}) != tmp_vec.end();
}

bool VariableOrderedSet::hasVariable(const std::string& dbcontent_name, const std::string& name) const
{
    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    return std::find(tmp_vec.begin(), tmp_vec.end(),
                     std::pair<std::string,std::string>{dbcontent_name, name}) != tmp_vec.end();
}

unsigned int VariableOrderedSet::getIndexFor(const std::string& dbcontent_name, const std::string var_name)
{
    std::vector<std::pair<std::string,std::string>> tmp_vec = definitions();

    auto it = std::find(tmp_vec.begin(), tmp_vec.end(), std::pair<std::string,std::string>{dbcontent_name,var_name});

    // If element was found
    traced_assert(it != tmp_vec.end());

    // calculating the index
    int index = it - tmp_vec.begin();
    traced_assert(index >= 0);

    return index;
}

VariableOrderedSetWidget* VariableOrderedSet::createWidget()
{
    return new VariableOrderedSetWidget(*this);
}

void VariableOrderedSet::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    //set has definitely changed if the variable set has been overwritten
    emit setChangedSignal();
    emit variableAddedChangedSignal();
}

}
