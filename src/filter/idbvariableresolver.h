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

#include <string>
#include <vector>

#include "property.h"

class Property;

namespace dbContent {
class VariableSet;
}

class IDBVariableResolver
{
public:
    virtual ~IDBVariableResolver() = default;

    // --- Property-based resolution (for specific named filters) ---

    // meta variable resolution (e.g. meta_var_m3a_, meta_var_timestamp_)
    virtual bool metaCanGetVariable(const std::string& dbcontent_name,
                                    const Property& meta_var) const = 0;
    virtual std::string metaGetVariableDBColumn(const std::string& dbcontent_name,
                                                const Property& meta_var) const = 0;
    virtual std::string metaGetVariableName(const std::string& dbcontent_name,
                                            const Property& meta_var) const = 0;

    // direct variable resolution (e.g. var_cat062_baro_alt_)
    virtual bool canGetVariable(const std::string& dbcontent_name,
                                const Property& var) const = 0;
    virtual std::string getVariableDBColumn(const std::string& dbcontent_name,
                                            const Property& var) const = 0;
    virtual std::string getVariableName(const std::string& dbcontent_name,
                                        const Property& var) const = 0;
    virtual bool variableHasDBContent(const std::string& dbcontent_name,
                                      const Property& var) const = 0;

    // --- Name-based resolution (for DBFilterCondition / generic filters) ---

    // existence checks (mirrors DBContentManager)
    virtual bool existsMetaVariable(const std::string& var_name) const = 0;
    virtual bool metaVariableExistsIn(const std::string& var_name,
                                      const std::string& dbcontent_name) const = 0;
    virtual bool existsDBContent(const std::string& dbcontent_name) const = 0;
    virtual bool dbContentHasVariable(const std::string& dbcontent_name,
                                      const std::string& var_name) const = 0;

    // returns dbcontent names that a meta variable has concrete variables for
    virtual std::vector<std::string> metaVariableDBContentNames(
        const std::string& var_name) const = 0;

    // variable attribute queries (mirrors Variable methods)
    // var_dbcontent_name == META_OBJECT_NAME => resolve via meta variable
    // var_dbcontent_name == concrete name    => resolve via direct variable
    virtual std::string variableDBColumnName(const std::string& dbcontent_name,
                                             const std::string& var_name,
                                             const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableDBTableName(const std::string& dbcontent_name,
                                            const std::string& var_name,
                                            const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableDBExpression(const std::string& dbcontent_name,
                                             const std::string& var_name,
                                             const std::string& var_dbcontent_name) const = 0;
    virtual PropertyDataType variableDataType(const std::string& dbcontent_name,
                                              const std::string& var_name,
                                              const std::string& var_dbcontent_name) const = 0;
    virtual bool variableHasNonStandardRepresentation(const std::string& dbcontent_name,
                                                      const std::string& var_name,
                                                      const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableValueFromRepresentation(const std::string& dbcontent_name,
                                                        const std::string& var_name,
                                                        const std::string& var_dbcontent_name,
                                                        const std::string& value_str) const = 0;

    // VariableSet interaction (wraps Variable& lookup + VariableSet operations)
    virtual bool readSetHasVariable(const std::string& dbcontent_name,
                                    const std::string& var_name,
                                    const std::string& var_dbcontent_name,
                                    const dbContent::VariableSet& read_set) const = 0;
    virtual void addVariableToReadSet(const std::string& dbcontent_name,
                                      const std::string& var_name,
                                      const std::string& var_dbcontent_name,
                                      dbContent::VariableSet& read_set) const = 0;
};
