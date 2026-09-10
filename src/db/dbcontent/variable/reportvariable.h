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

#include "task/result/reporttable.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace dbContent
{

class Variable;

/**
 * One column of a Report Table, offered in the variable selection like a DBContent variable.
 *
 * It owns one runtime Variable per host data content of its table. getFor() resolves like the
 * Meta content does, see readme_dynamic_dbcontent.md Section 4.3.
 */
class ReportVariable
{
public:
    ReportVariable(const std::string& name,
                   unsigned int result_id,
                   const ReportTableInfo& table,
                   const ReportTableColumn& column);
    virtual ~ReportVariable();

    /// name shown in the Views and used as Buffer column name, "<report name>: <display name>"
    const std::string& name() const { return name_; }
    /// name shown in the tree under the Report entry
    const std::string& displayName() const { return column_.display_name; }
    const std::string& description() const { return column_.description; }
    const std::string& tableKey() const { return table_key_; }
    /// the DuckDB table holding the column
    const std::string& tableName() const { return table_name_; }
    /// the alias of the join subquery of this table, shared by all its columns
    const std::string& joinAlias() const { return join_alias_; }
    /// the alias of this column inside the join subquery
    const std::string& columnAlias() const { return column_alias_; }
    /// the column name inside the Report Table
    const std::string& columnName() const { return column_.name; }
    /// group shown in the tree, the display name of the table
    const std::string& group() const { return group_; }

    PropertyDataType dataType() const { return column_.data_type; }
    std::string dataTypeString() const;
    std::string info() const;

    const std::vector<std::string>& hostDBContents() const { return host_dbcontents_; }
    bool existsIn(const std::string& dbcontent_name) const;
    /// runtime variable for one host data content, created on first use
    Variable& getFor(const std::string& dbcontent_name);

private:
    std::string              name_;
    std::string              group_;
    std::string              table_key_;
    std::string              table_name_;
    std::string              join_alias_;
    std::string              column_alias_;
    ReportTableColumn        column_;
    std::vector<std::string> host_dbcontents_;

    std::map<std::string, std::unique_ptr<Variable>> variables_; // dbcontent name -> variable
};

/**
 * The Report Variables of one stored Report, built from its catalog. Only record tables are
 * offered, a cell table has no record number to join on.
 */
class ReportContent
{
public:
    ReportContent(const std::string& report_name,
                  unsigned int result_id,
                  const std::vector<ReportTableInfo>& tables);
    virtual ~ReportContent();

    const std::string& name() const { return name_; }
    unsigned int resultID() const { return result_id_; }

    bool hasVariables() const { return !variables_.empty(); }
    bool hasVariable(const std::string& name) const;
    ReportVariable& variable(const std::string& name);

    const std::map<std::string, std::unique_ptr<ReportVariable>>& variables() const { return variables_; }

    /// display names of the tables, in catalog order, used as tree groups
    const std::vector<std::string>& groups() const { return groups_; }

private:
    std::string                                            name_;
    unsigned int                                           result_id_;
    std::vector<std::string>                               groups_;
    std::map<std::string, std::unique_ptr<ReportVariable>> variables_; // variable name -> variable
};

}
