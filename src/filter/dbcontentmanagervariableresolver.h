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

#include "idbvariableresolver.h"

class DBContentManager;

namespace dbContent {
class Variable;
}

class DBContentManagerVariableResolver : public IDBVariableResolver
{
public:
    DBContentManagerVariableResolver(DBContentManager& dbcontent_man);

    // --- Property-based resolution ---

    bool metaCanGetVariable(const std::string& dbcontent_name,
                            const Property& meta_var) const override;
    std::string metaGetVariableDBColumn(const std::string& dbcontent_name,
                                        const Property& meta_var) const override;
    std::string metaGetVariableName(const std::string& dbcontent_name,
                                    const Property& meta_var) const override;

    bool canGetVariable(const std::string& dbcontent_name,
                        const Property& var) const override;
    std::string getVariableDBColumn(const std::string& dbcontent_name,
                                    const Property& var) const override;
    std::string getVariableName(const std::string& dbcontent_name,
                                const Property& var) const override;
    bool variableHasDBContent(const std::string& dbcontent_name,
                              const Property& var) const override;

    // --- Name-based resolution ---

    bool existsMetaVariable(const std::string& var_name) const override;
    bool metaVariableExistsIn(const std::string& var_name,
                              const std::string& dbcontent_name) const override;
    bool existsDBContent(const std::string& dbcontent_name) const override;
    bool dbContentHasVariable(const std::string& dbcontent_name,
                              const std::string& var_name) const override;

    std::vector<std::string> metaVariableDBContentNames(
        const std::string& var_name) const override;

    std::string variableDBColumnName(const std::string& dbcontent_name,
                                     const std::string& var_name,
                                     const std::string& var_dbcontent_name) const override;
    std::string variableDBTableName(const std::string& dbcontent_name,
                                    const std::string& var_name,
                                    const std::string& var_dbcontent_name) const override;
    std::string variableDBExpression(const std::string& dbcontent_name,
                                     const std::string& var_name,
                                     const std::string& var_dbcontent_name) const override;
    PropertyDataType variableDataType(const std::string& dbcontent_name,
                                      const std::string& var_name,
                                      const std::string& var_dbcontent_name) const override;
    bool variableHasNonStandardRepresentation(const std::string& dbcontent_name,
                                              const std::string& var_name,
                                              const std::string& var_dbcontent_name) const override;
    std::string variableValueFromRepresentation(const std::string& dbcontent_name,
                                                const std::string& var_name,
                                                const std::string& var_dbcontent_name,
                                                const std::string& value_str) const override;

    bool readSetHasVariable(const std::string& dbcontent_name,
                            const std::string& var_name,
                            const std::string& var_dbcontent_name,
                            const dbContent::VariableSet& read_set) const override;
    void addVariableToReadSet(const std::string& dbcontent_name,
                              const std::string& var_name,
                              const std::string& var_dbcontent_name,
                              dbContent::VariableSet& read_set) const override;

private:
    DBContentManager& dbcontent_man_;

    // resolves a variable by name, handling both meta and direct paths
    dbContent::Variable& resolveVariable(const std::string& dbcontent_name,
                                         const std::string& var_name,
                                         const std::string& var_dbcontent_name) const;
};
