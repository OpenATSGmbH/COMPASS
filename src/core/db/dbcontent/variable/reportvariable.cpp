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

#include "dbcontent/variable/reportvariable.h"
#include "dbcontent/variable/variable.h"

#include "logger.h"
#include "traced_assert.h"

#include <algorithm>

namespace dbContent
{

/************************************************************************************************
 * ReportVariable
 ************************************************************************************************/

/**
 */
ReportVariable::ReportVariable(const std::string& name,
                               unsigned int result_id,
                               const ReportTableInfo& table,
                               const ReportTableColumn& column)
:   name_           (name)
,   group_          (table.display_name)
,   table_key_      (table.key)
,   table_name_     (table.tableName(result_id))
,   column_         (column)
,   host_dbcontents_(table.host_dbcontents)
{
    //the aliases must be unique unquoted identifiers in the SELECT list of a load. The join
    //alias names the subquery, the column alias the value inside it.
    join_alias_   = "rt" + std::to_string(result_id) + "_" + ReportTableDefinition::identifierFrom(table.key);
    column_alias_ = join_alias_ + "__" + ReportTableDefinition::identifierFrom(column.name);
}

/**
 */
ReportVariable::~ReportVariable() = default;

/**
 */
std::string ReportVariable::dataTypeString() const
{
    return Property::asString(column_.data_type);
}

/**
 */
std::string ReportVariable::info() const
{
    std::string info = name_;

    if (!column_.description.empty())
        info += "\n" + column_.description;

    std::string hosts;
    for (const auto& host : host_dbcontents_)
        hosts += (hosts.empty() ? "" : ", ") + host;

    info += "\nReport Table '" + table_key_ + "' on " + hosts;

    return info;
}

/**
 */
bool ReportVariable::existsIn(const std::string& dbcontent_name) const
{
    return std::find(host_dbcontents_.begin(), host_dbcontents_.end(), dbcontent_name)
           != host_dbcontents_.end();
}

/**
 */
Variable& ReportVariable::getFor(const std::string& dbcontent_name)
{
    traced_assert(existsIn(dbcontent_name));

    auto it = variables_.find(dbcontent_name);
    if (it != variables_.end())
        return *it->second;

    auto variable = Variable::createReportVariable(name_,
                                                   dbcontent_name,
                                                   column_alias_,
                                                   join_alias_,
                                                   table_name_,
                                                   column_.name,
                                                   column_.data_type,
                                                   column_.description,
                                                   column_.dimension,
                                                   column_.unit,
                                                   column_.viewRepresentation());

    auto& ref = *variable;

    variables_[ dbcontent_name ] = std::move(variable);

    return ref;
}

/************************************************************************************************
 * ReportContent
 ************************************************************************************************/

/**
 */
ReportContent::ReportContent(const std::string& report_name,
                             unsigned int result_id,
                             const std::vector<ReportTableInfo>& tables)
:   name_     (report_name)
,   result_id_(result_id)
{
    for (const auto& table : tables)
    {
        //a cell table has no record number, so it cannot be joined onto a data content
        if (table.kind != ReportTableKind::Record || table.host_dbcontents.empty())
            continue;

        bool group_used = false;

        for (const auto& column : table.columns)
        {
            //the key columns carry no information for a View, they are the join
            if (column.name == ReportTableDefinition::KeyColumnName ||
                column.name == ReportTableDefinition::UTNColumnName)
                continue;

            //a column display name repeats across the tables of a report, the table display
            //name keeps the entries apart. The Buffer column name must be unique.
            std::string name = report_name + ": " + column.display_name;

            if (variables_.count(name))
                name = report_name + ": " + table.display_name + " " + column.display_name;

            if (variables_.count(name))
            {
                logwrn << "report '" << report_name << "' duplicate variable '" << name << "' skipped";
                continue;
            }

            variables_[ name ].reset(new ReportVariable(name, result_id, table, column));
            group_used = true;
        }

        if (group_used &&
            std::find(groups_.begin(), groups_.end(), table.display_name) == groups_.end())
            groups_.push_back(table.display_name);
    }

    logdbg << "report '" << name_ << "' id " << result_id_
           << " variables " << variables_.size() << " groups " << groups_.size();
}

/**
 */
ReportContent::~ReportContent() = default;

/**
 */
bool ReportContent::hasVariable(const std::string& name) const
{
    return variables_.count(name) > 0;
}

/**
 */
ReportVariable& ReportContent::variable(const std::string& name)
{
    traced_assert(hasVariable(name));
    return *variables_.at(name);
}

}
