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

//#include "propertylist.h"

#include <vector>
#include <string>

namespace dbContent
{

class Variable;


class VariableSet
{
  public:
    /// @brief Constructor
    VariableSet();
    /// @brief Destructor
    virtual ~VariableSet();

    /// @brief Returns flag indicating if a change occurred
    bool getChanged() { return changed_; }
    /// @brief Sets the change occurred flag
    void setChanged(bool changed) { changed_ = changed; }

    /// @brief Adds a DBContVariable
    bool add(Variable& var);
    bool add(const Variable& var);
    /// @brief Adds another set of variables
    bool add(const VariableSet& set);
    /// @brief Adds variables for a given type from a given set of variables
    // bool addOnly (DBContVariableSet &set, const std::string &dbcont_name);
    /// @brief Removes variable at a given index
    void removeVariableAt(unsigned int index);
    void removeVariable(const Variable& var);
    /// @brief Intersects with another set of variables
    bool intersect(VariableSet& set);
    /// @brief Removes all variables
    void clear();
    /// @brief Returns if given variable is in the set
    bool hasVariable(const Variable& variable);
    bool hasDBColumnName(const std::string& db_column_name);
    unsigned int getVariableWithDBColumnName(const std::string& db_column_name);

    VariableSet& operator=(const VariableSet& source);
    // DBContVariableSet *clone ();

    /// @brief Returns container with all variables
    const std::vector<Variable*>& getSet() const { return set_; }
    /// @brief Returns variable at a given index
    Variable& getVariable(unsigned int index) const;

    std::string str() const;
    void print() const;

    /// @brief Returns number of variables in the set
    unsigned int getSize() const { return set_.size(); }

  protected:
    /// Container with all variables in the set
    std::vector<Variable*> set_;

    /// Change occurred flag
    bool changed_;
};

}
