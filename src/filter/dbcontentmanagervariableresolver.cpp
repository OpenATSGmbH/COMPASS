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

#include "dbcontentmanagervariableresolver.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "global.h"

DBContentManagerVariableResolver::DBContentManagerVariableResolver(DBContentManager& dbcontent_man)
    : dbcontent_man_(dbcontent_man)
{
}

// --- Property-based resolution ---

bool DBContentManagerVariableResolver::metaCanGetVariable(
    const std::string& dbcontent_name, const Property& meta_var) const
{
    return dbcontent_man_.metaCanGetVariable(dbcontent_name, meta_var);
}

std::string DBContentManagerVariableResolver::metaGetVariableDBColumn(
    const std::string& dbcontent_name, const Property& meta_var) const
{
    return dbcontent_man_.metaGetVariable(dbcontent_name, meta_var).dbColumnName();
}

std::string DBContentManagerVariableResolver::metaGetVariableName(
    const std::string& dbcontent_name, const Property& meta_var) const
{
    return dbcontent_man_.metaGetVariable(dbcontent_name, meta_var).name();
}

bool DBContentManagerVariableResolver::canGetVariable(
    const std::string& dbcontent_name, const Property& var) const
{
    return dbcontent_man_.canGetVariable(dbcontent_name, var);
}

std::string DBContentManagerVariableResolver::getVariableDBColumn(
    const std::string& dbcontent_name, const Property& var) const
{
    return dbcontent_man_.getVariable(dbcontent_name, var).dbColumnName();
}

std::string DBContentManagerVariableResolver::getVariableName(
    const std::string& dbcontent_name, const Property& var) const
{
    return dbcontent_man_.getVariable(dbcontent_name, var).name();
}

bool DBContentManagerVariableResolver::variableHasDBContent(
    const std::string& dbcontent_name, const Property& var) const
{
    return dbcontent_man_.getVariable(dbcontent_name, var).hasDBContent();
}

// --- Name-based resolution ---

bool DBContentManagerVariableResolver::existsMetaVariable(const std::string& var_name) const
{
    return dbcontent_man_.existsMetaVariable(var_name);
}

bool DBContentManagerVariableResolver::metaVariableExistsIn(
    const std::string& var_name, const std::string& dbcontent_name) const
{
    return dbcontent_man_.metaVariable(var_name).existsIn(dbcontent_name);
}

bool DBContentManagerVariableResolver::existsDBContent(const std::string& dbcontent_name) const
{
    return dbcontent_man_.existsDBContent(dbcontent_name);
}

bool DBContentManagerVariableResolver::dbContentHasVariable(
    const std::string& dbcontent_name, const std::string& var_name) const
{
    return dbcontent_man_.dbContent(dbcontent_name).hasVariable(var_name);
}

std::vector<std::string> DBContentManagerVariableResolver::metaVariableDBContentNames(
    const std::string& var_name) const
{
    std::vector<std::string> names;
    for (auto& it : dbcontent_man_.metaVariable(var_name).variables())
        names.push_back(it.first);
    return names;
}

dbContent::Variable& DBContentManagerVariableResolver::resolveVariable(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    if (var_dbcontent_name == META_OBJECT_NAME)
        return dbcontent_man_.metaVariable(var_name).getFor(dbcontent_name);
    else
        return dbcontent_man_.dbContent(var_dbcontent_name).variable(var_name);
}

std::string DBContentManagerVariableResolver::variableDBColumnName(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name).dbColumnName();
}

std::string DBContentManagerVariableResolver::variableDBTableName(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name).dbTableName();
}

std::string DBContentManagerVariableResolver::variableDBExpression(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name).dbExpression();
}

PropertyDataType DBContentManagerVariableResolver::variableDataType(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name).dataType();
}

bool DBContentManagerVariableResolver::variableHasNonStandardRepresentation(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name).representation()
           != dbContent::Variable::Representation::STANDARD;
}

std::string DBContentManagerVariableResolver::variableValueFromRepresentation(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name, const std::string& value_str) const
{
    return resolveVariable(dbcontent_name, var_name, var_dbcontent_name)
        .getValueStringFromRepresentation(value_str);
}

bool DBContentManagerVariableResolver::readSetHasVariable(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name, const dbContent::VariableSet& read_set) const
{
    dbContent::Variable& var = resolveVariable(dbcontent_name, var_name, var_dbcontent_name);
    return const_cast<dbContent::VariableSet&>(read_set).hasVariable(var);
}

void DBContentManagerVariableResolver::addVariableToReadSet(
    const std::string& dbcontent_name, const std::string& var_name,
    const std::string& var_dbcontent_name, dbContent::VariableSet& read_set) const
{
    dbContent::Variable& var = resolveVariable(dbcontent_name, var_name, var_dbcontent_name);
    read_set.add(var);
}
