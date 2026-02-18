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

#include "dbcontent/variable/variableset.h"
#include "dbtableinfo.h"
#include "dbdefs.h"

#include "boost/date_time/posix_time/posix_time.hpp"

#include <memory>
#include <string>

#include "json_fwd.hpp"

class Buffer;
class DBCommand;
class DBCommandList;
class DBContent;

/**
 */
class SQLGenerator
{
public:
    SQLGenerator(const db::SQLConfig& config);
    virtual ~SQLGenerator();

    std::string getCreateTableStatement(const DBContent& object);
    std::string getCreateTableStatement(const std::string& table_name,
                                        const std::vector<DBTableColumnInfo>& column_infos, 
                                        const std::vector<db::Index>& indices = std::vector<db::Index>());
    std::string getInsertDBUpdateStringBind(std::shared_ptr<Buffer> buffer, 
                                            std::string table_name);
    std::string getCreateDBUpdateStringBind(std::shared_ptr<Buffer> buffer,
                                            const std::string& key_col_name, 
                                            std::string table_name);
    std::string getUpdateTableFromTableStatement(const std::string& table_name_src,
                                                 const std::string& table_name_dst,
                                                 const std::vector<std::string>& col_names,
                                                 const std::string& key_col);
    std::string getUpdateCellStatement(const std::string& table_name,
                                       const std::string& col_name,
                                       const nlohmann::json& col_value,
                                       const std::string& key_col_name,
                                       const nlohmann::json& key_col_value);

    std::shared_ptr<DBCommand> getSelectCommand(const DBContent& object, 
                                                dbContent::VariableSet read_list, 
                                                const std::string& filter,
                                                bool use_order = false, 
                                                dbContent::Variable* order_variable = nullptr);
    std::shared_ptr<DBCommand> getSelectCommand(const std::string& table_name, 
                                                const PropertyList& properties, 
                                                const std::string& filter,
                                                bool use_order = false, 
                                                const std::string& order_variable = "");

    std::shared_ptr<DBCommand> getDataSourcesSelectCommand();
    std::shared_ptr<DBCommand> getFFTSelectCommand();

    std::shared_ptr<DBCommand> getDeleteCommand(const DBContent& dbcontent, boost::posix_time::ptime before_timestamp);
    std::shared_ptr<DBCommand> getDeleteCommand(const DBContent& dbcontent);
    std::shared_ptr<DBCommand> getDeleteCommand(const DBContent& dbcontent, unsigned int sac, unsigned int sic);
    std::shared_ptr<DBCommand> getDeleteCommand(const DBContent& dbcontent, unsigned int sac, unsigned int sic, unsigned int line_id);
    std::shared_ptr<DBCommand> getDeleteCommand(const std::string& table_name, const std::string& filter);

    //std::shared_ptr<DBCommand> getDistinctDataSourcesSelectCommand(DBContent& object);
    std::shared_ptr<DBCommand> getMaxUIntValueCommand(const std::string& table_name,
                                                      const std::string& col_name);
    std::shared_ptr<DBCommand> getMaxULongIntValueCommand(const std::string& table_name,
                                                          const std::string& col_name);
    std::shared_ptr<DBCommand> getADSBInfoCommand(DBContent& adsb_obj);

//    std::string getCreateAssociationTableStatement(const std::string& table_name);
//    std::shared_ptr<DBCommand> getSelectAssociationsCommand(const std::string& table_name);

    std::string getCountStatement(const std::string& table);

    //std::string getTableMinMaxCreateStatement();
    std::string getTablePropertiesCreateStatement();
    std::string getTableDataSourcesCreateStatement();
    std::string getTableFFTsCreateStatement();
    std::string getTableSectorsCreateStatement();
    std::string getTableViewPointsCreateStatement();
    std::string getTableTargetsCreateStatement();
    std::string getTableTaskLogCreateStatement();
    std::string getTableTaskResultsCreateStatement();
    std::string getTableReportContentsCreateStatement();

    std::string getDeleteStatement (const std::string& table, const std::string& filter);

    std::string getInsertTargetStatement(unsigned int utn, const std::string& info);

    std::string getInsertPropertyStatement(const std::string& id, const std::string& value);
    std::string getSelectPropertyStatement(const std::string& id);
    std::string getSelectAllPropertiesStatement();

    std::string getSetNullStatement (const std::string& table_name, const std::string& col_name);

    //    std::string getInsertMinMaxStatement(const std::string& variable_name,
    //                                         const std::string& object_name, const std::string& min,
    //                                         const std::string& max);
    //    std::string getSelectMinMaxStatement(const std::string& variable_name,
    //                                         const std::string& object_name);
    //    std::string getSelectMinMaxStatement();

    std::string getSelectNullCount (const std::string& table_name, const std::vector<std::string> columns);

    std::string getInsertViewPointStatement(const unsigned int id, const std::string& json);
    std::string getSelectAllViewPointsStatement();

    std::string getReplaceSectorStatement(const unsigned int id, const std::string& name,
                                          const std::string& layer_name, const std::string& json);
    std::string getSelectAllSectorsStatement();
    std::string getSelectAllTargetsStatement();
    std::string getSelectAllTaslLogMessagesStatement();

    std::shared_ptr<DBCommand> getTableSelectMinMaxNormalStatement(const DBContent& object);

    std::string configurePragma(const db::SQLPragma& pragma);

private:
    std::string placeholder(int index = 1) const;

    std::string replaceStatement(const std::string& table, 
                                 const std::vector<std::string>& values) const;

    std::string getCreateTableStatement(const std::string& table_name,
                                        const PropertyList& properties,
                                        int primary_key = -1) const;

    db::SQLConfig config_;

    //    std::string subTablesWhereClause(const DBTable& table,
    //                                     const std::vector<std::string>& used_tables);
    //    std::string subTableKeyClause(const DBTable& table, const std::string& sub_table_name);
};
