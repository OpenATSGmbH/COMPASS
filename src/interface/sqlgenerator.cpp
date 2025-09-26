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

#include "sqlgenerator.h"
#include "compass.h"
#include "buffer.h"
#include "dbcommand.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "logger.h"
#include "property.h"
#include "propertylist.h"
#include "source/dbdatasource.h"
#include "fft/dbfft.h"
#include "util/timeconv.h"
#include "dbinterface.h"

#include "task/result/taskresult.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"

#include <algorithm>
#include <iomanip>
#include <string>

using namespace Utils;
using namespace std;
using namespace dbContent;

/**
 */
SQLGenerator::SQLGenerator(const db::SQLConfig& config) 
:   config_(config)
{
}

/**
 */
SQLGenerator::~SQLGenerator() = default;

/**
 */
std::string SQLGenerator::placeholder(int index) const
{
    return (config_.placeholder == db::SQLPlaceholder::AtVar ? "@VAR" + std::to_string(index) : "?");
}

/**
 */
string SQLGenerator::getCreateTableStatement(const DBContent& object)
{
    //collect needed columns
    std::vector<DBTableColumnInfo> column_infos;
    for (auto& var_it : object.variables())
    {
        const auto& v = var_it.second;

        if (!v->dbColumnName().size()) // if only db expression
            continue;

        column_infos.push_back(DBTableColumnInfo(v->dbColumnName(), v->dataType(), v->isKey()));
    }

    //enable indexing on some metavars?
    std::vector<db::Index> indices;
    if (config_.indexing)
    {
        auto& dbcont_man = COMPASS::instance().dbContentManager();

        indices.emplace_back("TIMESTAMP_INDEX_" + object.name(), 
                             dbcont_man.metaGetVariable(object.name(), DBContent::meta_var_timestamp_).dbColumnName());
        indices.emplace_back("DS_ID_INDEX_" + object.name(), 
                             dbcont_man.metaGetVariable(object.name(), DBContent::meta_var_ds_id_).dbColumnName());
        indices.emplace_back("LINE_ID_INDEX_" + object.name(), 
                             dbcont_man.metaGetVariable(object.name(), DBContent::meta_var_line_id_).dbColumnName());
        if (dbcont_man.metaCanGetVariable(object.name(), DBContent::meta_var_utn_))
        {
            indices.emplace_back("UTN_INDEX_" + object.name(), 
                                 dbcont_man.metaGetVariable(object.name(), DBContent::meta_var_utn_).dbColumnName());
        }
    }

    return getCreateTableStatement(object.dbTableName(), column_infos, indices);
}

/**
 */
std::string SQLGenerator::getCreateTableStatement(const std::string& table_name,
                                                  const std::vector<DBTableColumnInfo>& column_infos,
                                                  const std::vector<db::Index>& indices)
{
    stringstream ss;

    //    CREATE TABLE contacts (
    //     contact_id integer PRIMARY KEY,
    //     first_name text NOT NULL,
    //     last_name text NOT NULL,
    //     email text NOT NULL UNIQUE,
    //     phone text NOT NULL UNIQUE
    //    );

    // sqlite
    //    if (data_type == "BOOLEAN")
    //        data_type = "BOOL";
    //    else if (data_type == "TEXT")
    //        data_type = "STRING";
    //    else if (data_type == "REAL")
    //        data_type = "DOUBLE";
    //    else if (data_type == "INTEGER")
    //        data_type = "INT";
    // mysql same

    ss << "CREATE TABLE " << table_name << "(";

    string data_type;

    std::string pkey_type_str = Property::asDBString(config_.precise_types ? PropertyDataType::ULONGINT : PropertyDataType::INT, true);

    unsigned int cnt = 0;
    for (const auto& cinfo : column_infos)
    {
        ss << "\"" << cinfo.name() << "\"";

        //should always be true (as the column info was most likely feed with a property data type)
        traced_assert(cinfo.hasDBType());

        data_type = cinfo.dbType();

        if (cinfo.key())
            ss << " " << pkey_type_str << " PRIMARY KEY NOT NULL"; // AUTOINCREMENT
        else
            ss << " " << data_type;

        if (cnt != column_infos.size() - 1)
            ss << ",";

        cnt++;
    }

    ss << ");";

    if (config_.indexing)
    {
        // CREATE [UNIQUE] INDEX index_name ON table_name(column_list);
        for (const auto& index : indices)
            ss << "\nCREATE INDEX " << index.indexName() << " ON " << table_name << "(" << index.columnName() << ");";
    }

    if (config_.verbose)
        loginf << "sql '" << ss.str() << "'";

    return ss.str();
}

/**
 */
std::string SQLGenerator::getCreateTableStatement(const std::string& table_name,
                                                  const PropertyList& properties,
                                                  int primary_key) const
{
    std::stringstream ss;

    traced_assert(properties.size());

    ss << "CREATE TABLE " << table_name << "(";

    size_t n = properties.size();
    for (size_t i = 0; i < n; ++i)
        ss << properties.properties().at(i).name() << " "  << properties.properties().at(i).dbDataTypeString(config_.precise_types) <<  (i < n - 1 ? ", " : "");

    if (primary_key >= 0)
        ss << ", " << "PRIMARY KEY (" << properties.properties().at(primary_key).name() << ")";
    
    ss << ");";

    if (config_.verbose)
        loginf << "sql '" << ss.str() << "'";

    return ss.str();
}

/**
 */
shared_ptr<DBCommand> SQLGenerator::getDataSourcesSelectCommand()
{
    using namespace dbContent;

    PropertyList list;
    list.addProperty(DBDataSource::id_column_);
    list.addProperty(DBDataSource::ds_type_column_);
    list.addProperty(DBDataSource::sac_column_);
    list.addProperty(DBDataSource::sic_column_);
    list.addProperty(DBDataSource::name_column_);
    list.addProperty(DBDataSource::short_name_);
    list.addProperty(DBDataSource::info_column_);
    list.addProperty(DBDataSource::counts_column_);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT ";

    bool first = true;

    for (const auto& prop_it : list.properties())
    {
        if (!first)
            ss << ",";

        ss << " " << prop_it.name();

        first = false;
    }

    ss << " FROM ";

    ss << DBDataSource::table_name_;

    ss << ";";

    command->set(ss.str());
    command->list(list);

    return command;
}

/**
 */
shared_ptr<DBCommand> SQLGenerator::getFFTSelectCommand()
{
    using namespace dbContent;

    PropertyList list;
    list.addProperty(DBDataSource::name_column_);
    list.addProperty(DBDataSource::info_column_);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT ";

    bool first = true;

    for (const auto& prop_it : list.properties())
    {
        if (!first)
            ss << ",";

        ss << " " << prop_it.name();

        first = false;
    }

    ss << " FROM ";

    ss << DBFFT::table_name_;

    ss << ";";

    command->set(ss.str());
    command->list(list);

    return command;
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getDeleteCommand(const std::string& table_name, const std::string& filter)
{
    //DELETE FROM table WHERE search_condition;

    stringstream ss;

    ss << "DELETE FROM " << table_name;

    if (!filter.empty())
        ss << " WHERE " << filter;

    ss << ";";

    logdbg << "sql '" << ss.str() << "'";

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());
    command->set(ss.str());
    return command;
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getDeleteCommand(
        const DBContent& dbcontent, boost::posix_time::ptime before_timestamp)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    stringstream ss;
    ss << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_timestamp_).dbColumnName();
    ss << " < " << Time::toLong(before_timestamp);

    return getDeleteCommand(dbcontent.dbTableName(), ss.str());
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getDeleteCommand(const DBContent& dbcontent)
{
    return getDeleteCommand(dbcontent.dbTableName(), "");
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getDeleteCommand(const DBContent& dbcontent, 
                                                          unsigned int sac, 
                                                          unsigned int sic)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    stringstream ss;
    ss << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_sac_id_).dbColumnName() << " = " << sac
       << " AND "
       << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_sic_id_).dbColumnName() << " = " << sic;

    return getDeleteCommand(dbcontent.dbTableName(), ss.str());
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getDeleteCommand(const DBContent& dbcontent, 
                                                          unsigned int sac, 
                                                          unsigned int sic, 
                                                          unsigned int line_id)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    stringstream ss;
    ss << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_sac_id_).dbColumnName() << " = " << sac
       << " AND "
       << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_sic_id_).dbColumnName() << " = " << sic
       << " AND "
       << dbcont_man.metaGetVariable(dbcontent.name(), DBContent::meta_var_line_id_).dbColumnName() << " = " << line_id;

       return getDeleteCommand(dbcontent.dbTableName(), ss.str());
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getMaxUIntValueCommand(const std::string& table_name,
                                                                const std::string& col_name)
{
    PropertyList list;
    list.addProperty(col_name, PropertyDataType::UINT);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT MAX(" << col_name;

    ss << ") FROM ";

    ss << table_name << ";";

    command->set(ss.str());
    command->list(list);

    return command;
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getMaxULongIntValueCommand(const std::string& table_name,
                                                                    const std::string& col_name)
{
    PropertyList list;
    list.addProperty(col_name, PropertyDataType::ULONGINT);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT MAX(" << col_name;

    ss << ") FROM ";

    ss << table_name << ";";

    command->set(ss.str());
    command->list(list);

    return command;
}

/**
 */
shared_ptr<DBCommand> SQLGenerator::getADSBInfoCommand(DBContent& adsb_obj)
{

    PropertyList list;
    list.addProperty("TARGET_ADDR", PropertyDataType::INT);
    list.addProperty("MOPS_VERSION", PropertyDataType::INT);
    list.addProperty("MIN_NUCP_NIC", PropertyDataType::CHAR);
    list.addProperty("MAX_NUCP_NIC", PropertyDataType::CHAR);
    list.addProperty("MIN_NACP", PropertyDataType::CHAR);
    list.addProperty("MAX_NACP", PropertyDataType::CHAR);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT TARGET_ADDR, MOPS_VERSION, MIN(NUCP_NIC), MAX(NUCP_NIC), MIN(NAC_P), MAX(NAC_P)";

    ss << " FROM ";

    ss << adsb_obj.dbTableName();

    ss << " group by TARGET_ADDR;";

    command->set(ss.str());
    command->list(list);

    return command;
}

//string SQLGenerator::getCreateAssociationTableStatement(const string& table_name)
//{
//    stringstream ss;

//    ss << "CREATE TABLE " << table_name
//       << " (assoc_id INTEGER PRIMARY KEY AUTOINCREMENT, rec_num INTEGER, utn INTEGER, src_rec_num "
//          "INTEGER, ds_id INTEGER);";

//    return ss.str();
//}

//shared_ptr<DBCommand> SQLGenerator::getSelectAssociationsCommand(const string& table_name)
//{
//    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

//    stringstream ss;

//    ss << "SELECT assoc_id, rec_num, utn, src_rec_num FROM " << table_name;

//    PropertyList property_list;
//    property_list.addProperty("assoc_id", PropertyDataType::INT);
//    property_list.addProperty("rec_num", PropertyDataType::INT);
//    property_list.addProperty("utn", PropertyDataType::INT);
//    property_list.addProperty("src_rec_num", PropertyDataType::INT);

//    command->set(ss.str());
//    command->list(property_list);

//    return command;
//}

/**
 */
string SQLGenerator::getCountStatement(const string& table)
{
    return "SELECT COUNT(*) FROM " + table + ";";
}

/**
 */
shared_ptr<DBCommand> SQLGenerator::getTableSelectMinMaxNormalStatement(const DBContent& object)
{
    logdbg << "start for table " << object.dbTableName();

    stringstream ss;

    shared_ptr<DBCommand> command(new DBCommand());
    traced_assert(command);

    PropertyList command_list;

    ss << "SELECT ";

    bool first = true;

    for (auto& var_it : object.variables())
    {
        logdbg << "current name "
               << var_it.first;

        if (!first)
            ss << ",";

        ss << "MIN(" << var_it.second->dbColumnName() << "),MAX(" << var_it.second->dbColumnName() << ")";

        command_list.addProperty(var_it.second->dbColumnName() + "MIN", PropertyDataType::STRING);
        command_list.addProperty(var_it.second->dbColumnName() + "MAX", PropertyDataType::STRING);

        first = false;
    }

    ss << " FROM " << object.dbTableName() << ";";

    command->set(ss.str());
    command->list(command_list);

    logdbg << "sql '" << ss.str() << "'";

    return command;
}

/**
 */
std::string SQLGenerator::replaceStatement(const std::string& table, 
                                           const std::vector<std::string>& values) const
{
    traced_assert(values.size() >= 2);

    size_t n = values.size();

    std::string s;
    if (config_.use_conflict_resolution)
    {
        s += "INSERT OR REPLACE INTO " + table + " VALUES (";
        for (size_t i = 0; i < n; ++i)
            s += "'" + values[ i ] + "'" + (i < n - 1 ? ", " : "");
        s += ");";
    }
    else
    {
        s += "REPLACE INTO " + table + " VALUES (";
        for (size_t i = 0; i < n; ++i)
            s += "'" + values[ i ] + "'" + (i < n - 1 ? ", " : "");
        s += ");";
    }

    return s;
}

/**
 */
std::string SQLGenerator::getInsertTargetStatement(unsigned int utn, const std::string& info)
{
    return replaceStatement(TABLE_NAME_TARGETS, { to_string(utn), info });
}

/**
 */
string SQLGenerator::getInsertPropertyStatement(const string& id,
                                                const string& value)
{
    traced_assert(id.size() < 255);

    // REPLACE into table (id, name, age) values(1, "A", 19)
    return replaceStatement(TABLE_NAME_PROPERTIES, { id, value });
}

/**
 */
string SQLGenerator::getSelectPropertyStatement(const string& id)
{
    stringstream ss;
    ss << "SELECT value FROM " << TABLE_NAME_PROPERTIES << " WHERE id = '" << id << "';";
    return ss.str();
}

/**
 */
string SQLGenerator::getSelectAllPropertiesStatement()
{
    stringstream ss;
    ss << "SELECT id, value FROM " << TABLE_NAME_PROPERTIES << ";";
    return ss.str();
}

/**
 */
std::string SQLGenerator::getSetNullStatement (const std::string& table_name, const std::string& col_name)
{
    return "UPDATE "+ table_name +" SET " + col_name + " = NULL";
}

/**
 */
string SQLGenerator::getInsertViewPointStatement(const unsigned int id, const string& json)
{
    // REPLACE into table (id, name, age) values(1, "A", 19)
    return replaceStatement(TABLE_NAME_VIEWPOINTS, { std::to_string(id), json });
}

/**
 */
string SQLGenerator::getSelectAllViewPointsStatement()
{
    stringstream ss;
    ss << "SELECT id, json FROM " << TABLE_NAME_VIEWPOINTS << ";";
    return ss.str();
}

/**
 */
string SQLGenerator::getReplaceSectorStatement(const unsigned int id, 
                                               const string& name,
                                               const string& layer_name, 
                                               const string& json)
{
    // REPLACE into table (id, name, age) values(1, "A", 19)
    return replaceStatement(TABLE_NAME_SECTORS, { std::to_string(id), name, layer_name, json });
}

/**
 */
string SQLGenerator::getSelectAllSectorsStatement()
{
    stringstream ss;
    ss << "SELECT id, name, layer_name, json FROM " << TABLE_NAME_SECTORS << ";";
    return ss.str();
}

/**
 */
string SQLGenerator::getSelectAllTargetsStatement()
{
    stringstream ss;
    ss << "SELECT utn, json FROM " << TABLE_NAME_TARGETS << ";";
    return ss.str();
}

std::string SQLGenerator::getSelectAllTaslLogMessagesStatement()
{
    stringstream ss;
    ss << "SELECT msg_id, json FROM " << TABLE_NAME_TASK_LOG << ";";
    return ss.str();
}

//@TODO: !rewrite using replaceStatement() if again using these!
//string SQLGenerator::getInsertMinMaxStatement(const string& variable_name,
//                                              const string& object_name,
//                                              const string& min, const string& max)
//{
//    stringstream ss;
//    ss << "REPLACE INTO " << TABLE_NAME_MINMAX << " VALUES ('" << variable_name << "', '"
//       << object_name << "', '" << min << "', '" << max << "');";
//    return ss.str();
//}
//string SQLGenerator::getSelectMinMaxStatement(const string& variable_name,
//                                              const string& object_name)
//{
//    stringstream ss;
//    ss << "SELECT min,max FROM " << TABLE_NAME_MINMAX << " WHERE variable_name = '" << variable_name
//       << "' AND object_name = '" << object_name << "';";
//    return ss.str();
//}

//string SQLGenerator::getSelectMinMaxStatement()
//{
//    stringstream ss;
//    ss << "SELECT variable_name,object_name,min,max FROM " << TABLE_NAME_MINMAX << ";";
//    return ss.str();
//}

/**
 */
string SQLGenerator::getSelectNullCount (const string& table_name, 
                                         const vector<string> columns)
{
    stringstream ss;
    ss << "SELECT count(*) FROM " << table_name << " WHERE";

    bool first = true;
    for (auto& col_it : columns)
    {
        if (!first)
            ss << " OR";

        ss << " " << col_it << " IS NULL";

        first = false;
    }
    ss << ";";

    return ss.str();
}

//string SQLGenerator::getTableMinMaxCreateStatement() { return table_minmax_create_statement_; }

/**
 */
string SQLGenerator::getTablePropertiesCreateStatement()
{
    stringstream ss;

    ss << "CREATE TABLE " << TABLE_NAME_PROPERTIES
       << "(id VARCHAR(255), value TEXT, PRIMARY KEY (id));";
    
    return ss.str();
}

/**
 */
std::string SQLGenerator::getTableDataSourcesCreateStatement()
{
    stringstream ss;

    ss << "CREATE TABLE " << DBDataSource::table_name_ << "("
        << DBDataSource::id_column_.name() << " "  << DBDataSource::id_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::ds_type_column_.name() << " "  << DBDataSource::ds_type_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::sac_column_.name() << " "  << DBDataSource::sac_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::sic_column_.name() << " "  << DBDataSource::sic_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::name_column_.name() << " "  << DBDataSource::name_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::short_name_.name() << " "  << DBDataSource::short_name_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::info_column_.name() << " "  << DBDataSource::info_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBDataSource::counts_column_.name() << " "  << DBDataSource::counts_column_.dbDataTypeString(config_.precise_types) << ", " 
        << "PRIMARY KEY (" << DBDataSource::id_column_.name() << ")"
        << ");";
    
    return ss.str();
}

/**
 */
std::string SQLGenerator::getTableFFTsCreateStatement()
{
    stringstream ss;

    ss << "CREATE TABLE " << DBFFT::table_name_ << "("
        << DBFFT::name_column_.name() << " "  << DBFFT::name_column_.dbDataTypeString(config_.precise_types) << ", "
        << DBFFT::info_column_.name() << " "  << DBFFT::info_column_.dbDataTypeString(config_.precise_types) << ", "
        << "PRIMARY KEY (" << DBFFT::name_column_.name() << ")"
        << ");";
    
    return ss.str();
}

/**
 */
string SQLGenerator::getTableSectorsCreateStatement()
{
    stringstream ss;

    ss << "CREATE TABLE " << TABLE_NAME_SECTORS
       << "(id INT, name VARCHAR(255), layer_name VARCHAR(255), json TEXT, PRIMARY KEY (id));";

    return ss.str();
}

/**
 */
string SQLGenerator::getTableViewPointsCreateStatement()
{
    stringstream ss;

    ss << "CREATE TABLE " << TABLE_NAME_VIEWPOINTS
       << "(id INT, json TEXT, PRIMARY KEY (id));";
    
    return ss.str();
}

/**
 */
std::string SQLGenerator::getTableTargetsCreateStatement()
{
    return getCreateTableStatement(TABLE_NAME_TARGETS, dbContent::Target::DBPropertyList, 0);
}

std::string SQLGenerator::getTableTaskLogCreateStatement()
{
    return getCreateTableStatement(TABLE_NAME_TASK_LOG, LogStore::LogEntry::DBPropertyList, 0);
}

/**
 */
std::string SQLGenerator::getTableTaskResultsCreateStatement()
{
    return getCreateTableStatement(TaskResult::DBTableName, TaskResult::DBPropertyList, 0);
}

/**
 */
std::string SQLGenerator::getTableReportContentsCreateStatement()
{
    return getCreateTableStatement(ResultReport::SectionContent::DBTableName, ResultReport::SectionContent::DBPropertyList, 0);
}

/**
 */
string SQLGenerator::getInsertDBUpdateStringBind(shared_ptr<Buffer> buffer,
                                                 string tablename)
{
    traced_assert(buffer);
    traced_assert(tablename.size() > 0);

    const vector<Property>& properties = buffer->properties().properties();

    // INSERT INTO table_name (column1, column2, column3, ...) VALUES (value1, value2, value3, ...);

    unsigned int size = properties.size();
    logdbg << "creating db string";
    stringstream ss;  // create a stringstream

    ss << "INSERT INTO " << tablename << " (";

    stringstream values_ss;
    values_ss << "VALUES (";

    for (unsigned int cnt = 0; cnt < size; cnt++)
    {
        ss << properties.at(cnt).name();

        values_ss << placeholder(cnt + 1);

        if (cnt != size - 1)
        {
            ss << ", ";
            values_ss << ", ";
        }
    }

    ss << ") " << values_ss.str() << ");";

    logdbg << "var insert string '" << ss.str() << "'";

    return ss.str();
}

/**
 */
string SQLGenerator::getCreateDBUpdateStringBind(shared_ptr<Buffer> buffer,
                                                 const string& key_col_name,
                                                 string table_name)
{
    traced_assert(buffer);
    traced_assert(table_name.size() > 0);

    const vector<Property>& properties = buffer->properties().properties();

    // UPDATE table_name SET col1=val1,col2=value2 WHERE somecol=someval;

    unsigned int size = properties.size();
    traced_assert(size);

    logdbg << "creating db string";
    stringstream ss;  // create a stringstream

    logdbg << "idvar name " << key_col_name;

    ss << "UPDATE " << table_name << " SET ";

    if (key_col_name != properties.at(size - 1).name())
    {
        logerr << "SQLGenerator::createDBUpdateStringBind: key_col_name '" << key_col_name
               << "' not at last position, but '" << properties.at(size - 1).name() << "'";

        throw runtime_error(
                    "SQLGenerator: createDBUpdateStringBind: key_col_name not at last position");
    }

    for (unsigned int cnt = 0; cnt < size; cnt++)
    {
        if (key_col_name == properties.at(cnt).name())
        {
            if (cnt == size - 1)
                continue;
            else
                throw runtime_error(
                        "SQLGenerator: createDBUpdateStringBind: id var at other than last position " +
                        to_string(cnt));
        }
        ss << properties.at(cnt).name() << "=";

        ss << placeholder(cnt + 1);

        if (cnt != size - 2)
        {
            ss << ", ";
        }
    }

    ss << " WHERE " << key_col_name << "=";

    ss << placeholder(size);

    ss << ";";

    logdbg << "var update string '" << ss.str() << "'";

    return ss.str();
}

/**
 */
std::string SQLGenerator::getUpdateTableFromTableStatement(const std::string& table_name_src,
                                                           const std::string& table_name_dst,
                                                           const std::vector<std::string>& col_names,
                                                           const std::string& key_col)
{
    //@TODO: maybe allow more variants of this command in the future and assert a little less
    traced_assert(!table_name_src.empty());
    traced_assert(!table_name_dst.empty());
    traced_assert(table_name_src != table_name_dst);
    traced_assert(!col_names.empty());
    traced_assert(!key_col.empty());

    stringstream ss;
    ss << "UPDATE " + table_name_dst + " SET ";

    size_t n = col_names.size();
    for (size_t i = 0; i < n; ++i)
        ss << col_names[ i ] << "=" << table_name_src << "." << col_names[ i ] << (i < n - 1 ? ", " : "");

    ss << " FROM " << table_name_src;
    ss << " WHERE " << table_name_dst << "." << key_col << "=" << table_name_src << "." << key_col;
    ss << ";";

    logdbg << "sql '" << ss.str() << "'";

    return ss.str();
}

/**
 */
std::string SQLGenerator::getUpdateCellStatement(const std::string& table_name,
                                                 const std::string& col_name,
                                                 const nlohmann::json& col_value,
                                                 const std::string& key_col_name,
                                                 const nlohmann::json& key_col_value)
{
    auto jsonString = [ & ] (const nlohmann::json& j)
    {
        std::string jstr = j.dump();
        bool value_quoted = j.is_string() || !j.is_primitive();

        return value_quoted ? "'" + jstr + "'" : jstr;
    };

    std::string col_value_str     = col_value.is_string()     ? "'" + col_value.dump()     + "'" : col_value.dump();
    std::string key_col_value_str = key_col_value.is_string() ? "'" + key_col_value.dump() + "'" : key_col_value.dump();

    stringstream ss;
    ss << "UPDATE " + table_name + " "
       << "SET "   << col_name     << " = " << jsonString(col_value    ) << " "
       << "WHERE " << key_col_name << " = " << jsonString(key_col_value) << ";";

    return ss.str();
}

/**
 */
shared_ptr<DBCommand> SQLGenerator::getSelectCommand(const DBContent& object, 
                                                     VariableSet read_list, 
                                                     const string& filter,
                                                     bool use_order, 
                                                     Variable* order_variable)
{
    logdbg << "dbcont " << object.name() << " read list size " << read_list.getSize();
    traced_assert(read_list.getSize() != 0);

    //collect needed properties
    PropertyList property_list;
    for (auto var_it : read_list.getSet())
    {
        Variable* variable = var_it;
        property_list.addProperty(variable->dbColumnOrExpression(), variable->dataType());
    }

    return getSelectCommand(object.dbTableName(), 
                            property_list, 
                            filter, 
                            use_order, 
                            order_variable ? order_variable->dbColumnName() : "");
}

/**
 */
std::shared_ptr<DBCommand> SQLGenerator::getSelectCommand(const std::string& table_name, 
                                                          const PropertyList& properties, 
                                                          const std::string& filter,
                                                          bool use_order, 
                                                          const std::string& order_variable)
{
    logdbg << "table " << table_name << " num properties " << properties.size();

    traced_assert(properties.size() != 0);

    shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

    stringstream ss;

    ss << "SELECT ";

    bool first = true;
    for (const auto& p : properties.properties())
        // look what tables are needed for loaded variables and add variables to sql query
    {
        if (!first)
            ss << ", ";

        ss << p.name();

        first = false;
    }
    
    ss << " FROM " << table_name;  // << table->getAllTableNames();

    // add extra from parts
//    for (auto& from_part : extra_from_parts)
//        ss << ", " << from_part;

    logdbg << "filtering statement";

    // add filter statement
    if (filter.size() > 0)
        ss << " WHERE "  << filter;

    if (use_order)
    {
        traced_assert(!order_variable.empty());

        ss << " ORDER BY " << order_variable;

        ss << " ASC";
    }

    ss << ";";

    command->set(ss.str());
    command->list(properties);

    logdbg << "command sql '" << ss.str() << "'";

    return command;
}

//shared_ptr<DBCommand> SQLGenerator::getSelectCommand(const DBContent& object,
//                                                     const vector<string>& columns,
//                                                     bool distinct)
//{
//        logdbg << "meta table " << meta_table.name()
//               << " db columns size " << columns.size();
//        traced_assert(columns.size() != 0);

//        shared_ptr<DBCommand> command = make_shared<DBCommand>(DBCommand());

//        stringstream ss;

//        ss << "SELECT ";

//        if (distinct)
//            ss << "DISTINCT ";

//        vector<string> used_tables;

//        logdbg << "collecting required variables";

//        PropertyList property_list;

//        bool first = true;
//        for (auto col_it : columns)
//            // look what tables are needed for loaded variables and add variables to sql query
//        {
//            if (property_list.hasProperty(col_it->name())) // for already added ones
//                continue;

//            if (!first)
//                ss << ", ";

//            string table_db_name = col_it->table().name();

//            if (find(used_tables.begin(), used_tables.end(), table_db_name) == used_tables.end())
//                used_tables.push_back(table_db_name);

//            ss << table_db_name << "." << col_it->name();

//            property_list.addProperty(col_it->name(), col_it->propertyType());

//            first = false;
//        }

//        ss << " FROM ";

//        string main_table_name = meta_table.mainTableName();

//        logdbg << "left join query";
//        //    SELECT news.id, users.username, news.title, news.date, news.body, COUNT(comments.id)
//        //    FROM news
//        //    LEFT JOIN users
//        //    ON news.user_id = users.id
//        //    LEFT JOIN comments
//        //    ON comments.news_id = news.id
//        //    GROUP BY news.id
//        ss << main_table_name;

//        traced_assert(used_tables.size() > 0);
//        vector<string>::iterator it;
//        for (it = used_tables.begin(); it != used_tables.end(); it++)
//        {
//            if (it->compare(main_table_name) != 0)  // not main table
//            {
//                ss << " LEFT JOIN " << *it;
//                ss << " ON " << subTableKeyClause(meta_table, *it);
//            }
//        }

//        ss << ";";

//        command->set(ss.str());
//        command->list(property_list);

//        logdbg << "command sql '" << ss.str() << "'";

//        return command;
//}

//string SQLGenerator::subTablesWhereClause(const DBTable& table,
//                                               const vector<string>& used_tables)
//{
//    traced_assert(false); // TODO
//    stringstream ss;

//    bool first = true;

//    for (auto it : meta_table.subTableDefinitions())
//    {
//        if (find(used_tables.begin(), used_tables.end(), it.second->subTableName()) ==
//                used_tables.end())
//            continue;

//        if (!first)
//            ss << " AND ";

//        ss << meta_table.mainTableName() << "." << it.second->mainTableKey() << "="
//           << it.second->subTableName() << "." << it.second->subTableKey();
//        first = false;
//    }

//    return ss.str();
//}

//string SQLGenerator::subTableKeyClause(const DBTable& table,
//                                            const string& sub_table_name)
//{
//    traced_assert(false); // TODO
//    if (meta_table.subTableDefinitions().count(sub_table_name) == 0)
//        throw range_error("SQLGenerator: getSubTableKeyClause: sub_table_name " +
//                               sub_table_name + " not found");

//    auto subtable = meta_table.subTableDefinitions().at(sub_table_name);

//    // found subtable
//    stringstream ss;

//    ss << meta_table.mainTableName() << "." << subtable->mainTableKey() << "="
//       << subtable->subTableName() << "." << subtable->subTableKey();

//    return ss.str();
//}

/**
 */
string SQLGenerator::getDeleteStatement (const string& table, const string& filter)
{
    // DELETE FROM table_name [WHERE Clause]
    return "DELETE FROM "+table+" WHERE "+filter+";";
}

/**
 */
std::string SQLGenerator::configurePragma(const db::SQLPragma& pragma)
{
    return "PRAGMA " + pragma.name() + " = " + pragma.value() + ";";
}
