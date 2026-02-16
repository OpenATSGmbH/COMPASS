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

#pragma once

#include "configurable.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"

#include "json.hpp"

namespace dbContent
{

class MetaVariableWidget;

class MetaVariable : public Configurable
{
public:
    MetaVariable(const std::string& class_id, const std::string& instance_id,
                 DBContentManager* object_manager);
    virtual ~MetaVariable();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

    bool hasVariables() const { return variables_.size() > 0; }
    PropertyDataType dataType() const;
    const std::string& dataTypeString() const;
    Variable::Representation representation();

    bool existsIn(const std::string& dbcontent_name);
    Variable& getFor(const std::string& dbcontent_name);
    std::string getNameFor(const std::string& dbcontent_name);
    void set(Variable& var);

    void removeVariable(const std::string& dbcontent_name);
    /// @brief Sets sub-variable name for DBContent
    void addVariable(const std::string& dbcontent_name, const std::string& dbcontvariable_name);

    const std::map<std::string, Variable&>& variables() { return variables_; }
    bool uses(const Variable& variable);

    std::string name() const;
    void name(const std::string& name);

    std::string description() const;
    std::string info() const;

    //    std::string getMinString() const;
    //    std::string getMaxString() const;
    //    std::string getMinStringRepresentation() const;
    //    std::string getMaxStringRepresentation() const;

    MetaVariableWidget* widget();

    void unlock();
    void lock();

    bool hasDBContent() const;

protected:
    std::string name_;
    std::string description_;

    nlohmann::json dbcont_variables_; // dbcont name -> var name

    DBContentManager& object_manager_;

    MetaVariableWidget* widget_;

    bool locked_{false};

    std::map<std::string, Variable&> variables_; // dbcont name -> var

    virtual void checkSubConfigurables();
    void updateDescription();

    void checkSubVariables();
};

}
