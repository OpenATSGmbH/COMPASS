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

//#include <boost/algorithm/string.hpp>

#include "dbcontent/variable/metavariable.h"

#include "compass.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariablewidget.h"

using namespace std;

namespace dbContent
{

MetaVariable::MetaVariable(const std::string& class_id, const std::string& instance_id,
                                 DBContentManager* object_manager)
    : Configurable(class_id, instance_id, object_manager),
      object_manager_(*object_manager),
      widget_(nullptr)
{
    registerParameter("name", &name_, std::string());

    registerParameter("dbcont_variables", &dbcont_variables_, nlohmann::json::object());

    // DBContVAR LOWERCASE HACK
    // boost::algorithm::to_lower(name_);

    traced_assert(name_.size() > 0);

    createSubConfigurables();

    loginf << "name " << name_;
    for (auto it = dbcont_variables_.begin(); it != dbcont_variables_.end(); ++it)
    {
        const std::string& dbcontent_name = it.key();
        const std::string& dbcontvar_name = it.value();

        loginf << "dbcontent " << dbcontent_name << ": variable name '" << dbcontvar_name << "'";

        if (!object_manager_.existsDBContent(dbcontent_name))
        {
            logerr << "name " << name_ << " dbcontvariable definition " << instance_id
                   << " has unknown dbcont, ignoring";
            // delete definition;
            return;
        }

        if (!object_manager_.dbContent(dbcontent_name).hasVariable(dbcontvar_name))
        {
            logerr << "name " << name_ << " dbcontvariable definition " << instance_id
                   << " has unknown dbcont variable, ignoring";
            // delete definition;
            return;
        }

        if (variables_.find(dbcontent_name) != variables_.end())
        {
            logerr << "name " << name_ << " dbcontvariable definition " << instance_id
                   << " has already defined dbcont, ignoring";
            // delete definition;
            return;
        }

        traced_assert(object_manager_.existsDBContent(dbcontent_name));
        traced_assert(object_manager_.dbContent(dbcontent_name).hasVariable(dbcontvar_name));
        traced_assert(variables_.find(dbcontent_name) == variables_.end());

        variables_.insert(std::pair<std::string, Variable&>(
            dbcontent_name, object_manager_.dbContent(dbcontent_name).variable(dbcontvar_name)));
    }

    checkSubVariables();
    updateDescription();
}

MetaVariable::~MetaVariable()
{
    if (widget_)
    {
        delete widget_;
        widget_ = nullptr;
    }

    variables_.clear();
}

void MetaVariable::checkSubConfigurables()
{
    // nothing to do here
}

void MetaVariable::generateSubConfigurable(const std::string& class_id,
                                              const std::string& instance_id)
{
    throw std::runtime_error("MetaVariable: generateSubConfigurable: unknown class_id " + class_id);
}

bool MetaVariable::existsIn(const std::string& dbcontent_name)
{
    return variables_.count(dbcontent_name) > 0;
}

Variable& MetaVariable::getFor(const std::string& dbcontent_name)
{
    traced_assert(existsIn(dbcontent_name));
    return variables_.at(dbcontent_name);
}

std::string MetaVariable::getNameFor(const std::string& dbcontent_name)
{
    traced_assert(existsIn(dbcontent_name));
    return variables_.at(dbcontent_name).name();
}

void MetaVariable::set(Variable& var)
{
    string dbcontent_name = var.dbObject().name();

    loginf << name_ << ": dbcont " << dbcontent_name << " name " << var.name();

    if (existsIn(dbcontent_name))
        removeVariable(dbcontent_name);

    addVariable(dbcontent_name, var.name());
}

void MetaVariable::removeVariable(const std::string& dbcontent_name)
{
    loginf << name_ << ": dbcont " << dbcontent_name;
    traced_assert(existsIn(dbcontent_name));
    
    dbcont_variables_.erase(dbcontent_name);
    variables_.erase(dbcontent_name);

    updateDescription();
}

void MetaVariable::addVariable(const std::string& dbcontent_name, const std::string& dbcontvariable_name)
{
    loginf << name_ << ": dbcont " << dbcontent_name << " varname "
           << dbcontvariable_name;

    traced_assert(!existsIn(dbcontent_name));

    dbcont_variables_[dbcontent_name] = dbcontvariable_name;
    updateDescription();
}

bool MetaVariable::uses(const Variable& variable)
{
    for (auto& var_it : variables_)
        if (var_it.second == variable)
            return true;

    return false;
}

MetaVariableWidget* MetaVariable::widget()
{
    if (!widget_)
    {
        widget_ = new MetaVariableWidget(*this);

        if (locked_)
            widget_->lock();
    }
    traced_assert(widget_);
    return widget_;
}

void MetaVariable::unlock()
{
    if (!locked_)
        return;

    locked_ = false;

    if (widget_)
        widget_->unlock();
}

void MetaVariable::lock()
{
    if (locked_)
        return;

    locked_ = true;

    if (widget_)
        widget_->lock();
}

std::string MetaVariable::name() const { return name_; }

void MetaVariable::name(const std::string& name) { name_ = name; }

std::string MetaVariable::description() const { return description_; }

std::string MetaVariable::info() const
{
    std::ostringstream ss;

    ss << "Name: '" << name_ << "'" << endl;
    ss << "Data Type: " << dataTypeString() << endl;
    ss << "Description: " << description() << endl;

    return ss.str();
}

PropertyDataType MetaVariable::dataType() const
{
    traced_assert(hasVariables());

    //checked in checkSubVariables

    return variables_.begin()->second.dataType();
}

const std::string& MetaVariable::dataTypeString() const
{
    traced_assert(hasVariables());
    return Property::asString(dataType());
}

Variable::Representation MetaVariable::representation()
{
    traced_assert(hasVariables());

    //checked in checkSubVariables

    return variables_.begin()->second.representation();
}

//std::string MetaVariable::getMinString() const
//{
//    std::string value_string;

//    for (auto variable_it : variables_)
//    {
//        if (value_string.size() == 0)
//            value_string = variable_it.second.getMinString();
//        else
//            value_string = variable_it.second.getSmallerValueString(
//                value_string, variable_it.second.getMinString());
//    }
//    // traced_assert(value_string.size());
//    return value_string;
//}

//std::string MetaVariable::getMaxString() const
//{
//    std::string value_string;

//    for (auto variable_it : variables_)
//    {
//        if (value_string.size() == 0)
//            value_string = variable_it.second.getMaxString();
//        else
//            value_string = variable_it.second.getLargerValueString(
//                value_string, variable_it.second.getMaxString());
//    }
//    // traced_assert(value_string.size());
//    return value_string;
//}

//std::string MetaVariable::getMinStringRepresentation() const
//{
//    traced_assert(variables_.size());
//    return variables_.begin()->second.getRepresentationStringFromValue(getMinString());
//}

//std::string MetaVariable::getMaxStringRepresentation() const
//{
//    traced_assert(variables_.size());
//    return variables_.begin()->second.getRepresentationStringFromValue(getMaxString());
//}

bool MetaVariable::hasDBContent() const
{
    for (auto& variable_it : variables_)
    {
        if (variable_it.second.hasDBContent())
            return true;
    }

    return false;
}


void MetaVariable::updateDescription()
{
    description_ = "";

    for (auto& variable_it : variables_)
    {
        description_ += "For " + variable_it.first +" ("+ variable_it.second.dataTypeString()+"):\n";
        description_ += variable_it.second.info() + "\n\n";
    }
}

void MetaVariable::checkSubVariables()
{
    if (hasVariables())
    {
        PropertyDataType data_type = variables_.begin()->second.dataType();

        for (auto& variable_it : variables_)
        {
            if (variable_it.second.dataType() != data_type)
            {
                logerr << "meta var " << name_
                       << " has different data types in sub variables ("
                       << Property::asString(data_type) << ", " << variable_it.second.dataTypeString() << ")";
                throw std::runtime_error("Conflicting data types in metavariable '" + name_ + "' (" +
                                         Property::asString(data_type) + " =/= " + variable_it.second.dataTypeString() + ")");
            }
        }

        string rep_str = variables_.begin()->second.representationString();

        for (auto& variable_it : variables_)
        {
            if (variable_it.second.representationString() != rep_str)
            {
                logerr << "meta var " << name_
                       << " has different representations in sub variables ("
                       << rep_str << ", " << variable_it.second.representationString() << ")";
                throw std::runtime_error("Conflicting representations in metavariable '" + name_ + "' (" +
                                         rep_str + " =/= " + variable_it.second.representationString() + ")");
            }
        }
    }
}

}
