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

#include "duckdbconnection.h"
#include "duckdbinstance.h"
#include "duckdbappender.h"
#include "duckdbprepare.h"
#include "duckdbexecresult.h"
#include "duckdbreader.h"

#include "dbtemporarytable.h"
#include "sqlgenerator.h"

#include "buffer.h"
#include "dbcommand.h"
#include "dbcommandlist.h"
#include "dbinterface.h"
#include "dbresult.h"
#include "dbtableinfo.h"
#include "logger.h"
#include "property.h"
#include "dbcontent.h"
#include "property_templates.h"
#include "compass.h"
#include "dbcontentmanager.h"

#include "util/timeconv.h"

#include <cstring>

#include <boost/filesystem.hpp>

#include <QFile>

/**
 */
DuckDBConnection::DuckDBConnection(DuckDBInstance* instance, bool verbose)
:   DBConnection(instance, verbose)
{
    //loginf;
}

/**
 */
DuckDBConnection::~DuckDBConnection()
{
    //loginf;

    if (connected())
        disconnect();
}

/**
 */
DuckDBInstance* DuckDBConnection::duckDBInstance()
{
    return dynamic_cast<DuckDBInstance*>(instance());
}

/**
 */
Result DuckDBConnection::connect_impl()
{
    auto duck_db = duckDBInstance();

    traced_assert(duck_db->dbOpen());
    traced_assert(duck_db->db_);
    

    //connect to db
    auto state = duckdb_connect(duck_db->db_, &connection_);
    if (state != DuckDBSuccess)
        return Result::failed("Could not connect to database");

    return Result::succeeded();
}

/**
 */
void DuckDBConnection::disconnect_impl()
{
    duckdb_disconnect(&connection_);

    connection_ = nullptr;
}

/**
 */
std::shared_ptr<DuckDBScopedAppender> DuckDBConnection::createAppender(const std::string& table)
{
    return std::shared_ptr<DuckDBScopedAppender>(new DuckDBScopedAppender(connection_, table));
}

/**
 */
std::shared_ptr<DBScopedPrepare> DuckDBConnection::prepareStatement(const std::string& statement,
                                                                    bool begin_transaction)
{
    return std::shared_ptr<DBScopedPrepare>(new DuckDBScopedPrepare(connection_, statement, begin_transaction));
}

/**
 */
std::shared_ptr<DBScopedReader> DuckDBConnection::createReader(const std::shared_ptr<DBCommand>& select_cmd, 
                                                               size_t offset, 
                                                               size_t chunk_size)
{
    return std::shared_ptr<DBScopedReader>(new DuckDBScopedReader(connection_, select_cmd, offset, chunk_size));
}

/**
 */
Result DuckDBConnection::executeSQL_impl(const std::string& sql, DBResult* result, bool fetch_result_buffer)
{
    DuckDBExecResult dbresult;

    auto state = duckdb_query(connection_, sql.c_str(), dbresult.result());
    dbresult.result_valid_ = true;
    dbresult.result_error_ = state == DuckDBError;

    if (dbresult.hasError()) 
    {
        auto err = dbresult.errorString();
        if (result)
            result->setError(err);
        return Result::failed(err);
    }

    if (fetch_result_buffer && result)
    {
        DBExecResult& result_base = dbresult;

        auto b = result_base.toBuffer();
        loginf << "buffer has size " << (b ? b->size() : 0);

        result->buffer(b);
    }

    return Result::succeeded();
}

/**
 */
bool DuckDBConnection::executeCmd_impl(const std::string& command, 
                                       const PropertyList* properties, 
                                       DBResult* result)
{
    DuckDBScopedPrepare prepare(connection_, command);

    if (!prepare.valid())
    {
        logerr << "preparing command '" << command << "' failed";
        if (result)
            result->setError("could not prepare command '" + command + "': " + prepare.lastError());
        return false;
    }

    DBPrepare::ExecOptions options;
    if (result && properties)
        options.bufferProperties(*properties);

    auto dbresult = prepare.execute(options);
    traced_assert(dbresult);

    if (dbresult->hasError())
        logerr << "executing command '" << command << "' failed";

    if (result)
        *result = *dbresult;

    return !dbresult->hasError();
}

/**
 */
Result DuckDBConnection::insertBuffer_impl(const std::string& table_name, 
                                           const std::shared_ptr<Buffer>& buffer,
                                           const boost::optional<size_t>& idx_from, 
                                           const boost::optional<size_t>& idx_to,
                                           PropertyList* table_properties)
{
    //loginf << "inserting into table '" << table_name << "'";

    if (!buffer || buffer->properties().size() == 0 || buffer->size() == 0)
        return Result::failed("Input buffer invalid");

    //loginf << "creating appender...";

    auto appender = createAppender(table_name);
    if (!appender->valid())
        return Result::failed("Creating duckdb appender failed: " + appender->lastError());

    size_t appender_column_count = appender->columnCount();

    //loginf << "configuring table props...";

    const auto& buffer_properties = buffer->properties();
    const auto& created_tables    = instance()->tableInfo();

    PropertyList table_props;
    const PropertyList* properties = nullptr;

    if (table_properties)
    {
        //loginf << "using externally passed properties";

        //use externally passed properties
        properties = table_properties;
    }
    else if (created_tables.count(table_name))
    {
        //loginf << "using stored table properties";

        //use stored table properties
        //@TODO: might fail!
        auto it = created_tables.find(table_name);
        auto tp = it->second.tableProperties();

        traced_assert(tp.has_value());
        table_props = tp.value();

        properties = &table_props;
    }
    else
    {
        //loginf << "using buffer properties";

        //use buffers properties
        //note: !in this case the buffer properties need to reflect the table properties 1:1!
        properties = &buffer_properties;
    }
    traced_assert(properties);

    //loginf << "update func...";

    #define UpdateFunc(PDType, DType, Suffix)                                                                               \
        NullableVector<DType>* vec_ptr = &buffer->get<DType>(pname);                                                        \
                                                                                                                            \
        auto cb = [ appender_ptr, vec_ptr, pname ] (size_t row)                                                             \
        {                                                                                                                   \
            bool ok;                                                                                                        \
            if (vec_ptr->isNull(row))                                                                                       \
                ok = appender_ptr->appendNull();                                                                            \
            else                                                                                                            \
                ok = appender_ptr->append<DType>(vec_ptr->get(row));                                                        \
                                                                                                                            \
            return ok;                                                                                                      \
        };                                                                                                                  \
                                                                                                                            \
        importers[ c ] = cb;

    #define NotFoundFunc                                                                                      \
        logerr << "unknown property type " << Property::asString(dtype); \
        traced_assert(false);

    auto n = buffer->size();

    //loginf << "buffer properties: " << buffer_properties.size() << ", used properties: " << properties->size();

    //loginf << "inserting " << n << "row(s)...";

    unsigned int np = properties->size();
    traced_assert(np == appender_column_count);

    std::vector<char> has_property(np, 0);
    for (unsigned int c = 0; c < properties->size(); ++c)
        has_property[ c ] = buffer->hasAnyPropertyNamed(properties->at(c).name()) ? 1 : 0;

    unsigned int r0 = idx_from.has_value() ? idx_from.value()   : 0;
    unsigned int r1 = idx_to.has_value()   ? idx_to.value() + 1 : n;
    traced_assert(r0 <= r1);

    auto appender_ptr = appender.get();

    std::vector<std::function<bool(size_t)>> importers(np);

    for (unsigned int c = 0; c < np; ++c)
    {
        if (!has_property[ c ])
        {
            //property not available => add null appender
            importers[ c ] = [ appender_ptr ] (size_t row) { return appender_ptr->appendNull(); };
            continue;
        }

        const auto& p     = properties->at(c);
        const auto& pname = p.name();
        const auto& pb    = buffer->properties().get(p.name());      

        auto dtype = pb.dataType();

        SwitchPropertyDataType(dtype, UpdateFunc, NotFoundFunc);
    }

    for (unsigned int r = r0; r < r1; ++r)
    {
        //loginf << "appending row " << std::to_string(r + 1) << "/" << std::to_string(n);

        for (unsigned int c = 0; c < np; ++c)
        {
            bool ok = importers[ c ](r);

            if (!ok)
                logerr << "appending column " << c << " failed: " << appender->lastError();
            traced_assert(ok);
        }
        traced_assert(appender->currentColumnCount() == np);
        appender->endRow();

        //if (r % 2000 == 0)
        //    appender->flush();
    }

    appender->flush();

    //cleanup appender
    appender.reset();

    return Result::succeeded();
}

/**
 * In DuckDB we update via a temporary table which seems to be much faster.
 */
Result DuckDBConnection::updateBuffer_impl(const std::string& table_name, 
                                           const std::shared_ptr<Buffer>& buffer,
                                           const std::string& key_column,
                                           const boost::optional<size_t>& idx_from, 
                                           const boost::optional<size_t>& idx_to)
{
    if (!buffer || buffer->properties().size() == 0 || buffer->size() == 0)
        return Result::failed("Input buffer invalid");

    //create temporary table for buffer data
    std::vector<DBTableColumnInfo> table_columns;
    for (const auto& p : buffer->properties().properties())
        table_columns.push_back(DBTableColumnInfo(p.name(), p.dataType(), false));

    DBScopedTemporaryTable temp_table(this, table_columns);
    if (!temp_table.valid())
        return Result::failed("Could not create temporary table for transaction: " + temp_table.result().error());

    //insert buffer data into temp table
    auto res_insert = insertBuffer(temp_table.name(), buffer, idx_from, idx_to);
    if (!res_insert.ok())
        return res_insert;

    //assign temp table data to target table
    std::vector<std::string> update_columns;
    bool key_col_found = false;
    for (const auto& p : buffer->properties().properties())
    {
        if (p.name() == key_column)
            key_col_found = true;
        else
            update_columns.push_back(p.name());
    }

    if (!key_col_found)
        return Result::failed("Key column '" + key_column + "' not found in buffer");
    
    auto sql = sqlGenerator().getUpdateTableFromTableStatement(temp_table.name(),
                                                               table_name,
                                                               update_columns,
                                                               key_column);
    auto res_sql = execute(sql);
    if (!res_sql.ok())
        return res_sql;

    return Result::succeeded();
}

/**
 */
ResultT<std::vector<std::string>> DuckDBConnection::getTableList_impl()
{
    std::vector<std::string> tables;

    DBCommand command;
    command.set("SELECT table_name FROM duckdb_tables()");

    PropertyList list;
    list.addProperty("table_name", PropertyDataType::STRING);
    command.list(list);

    std::shared_ptr<DBResult> result = execute(command);
    traced_assert(result);

    if (result->hasError())
        return ResultT<std::vector<std::string>>::failed(result->error());
    if (!result->buffer() || !result->containsData())
        return ResultT<std::vector<std::string>>::failed("Table list could not be retrieved");

    std::shared_ptr<Buffer> buffer = result->buffer();

    unsigned int size = buffer->size();
    std::string table_name;

    for (unsigned int cnt = 0; cnt < size; cnt++)
    {
        table_name = buffer->get<std::string>("table_name").get(cnt);

        //@TODO: any tables to skip?

        tables.push_back(table_name);
    }

    return ResultT<std::vector<std::string>>::succeeded(tables);
}

/**
 */
std::string DuckDBConnection::dbInfo()
{
    const std::string& fn = instance()->dbFilename();

    std::stringstream ss;

    ss << "**********************************************************************************" << std::endl;
    ss << "* DBInfo | DB: " << fn << std::endl;
    ss << "**********************************************************************************" << std::endl;
    ss << std::endl;

    auto tables = getTableList();
    if (!tables.ok())
        return "";

    for (const auto& t : tables.result())
    {
        ss << "[Table '" << t << "']" << std::endl;
        ss << std::endl;
        ss << tableInfo(t) << std::endl;
        ss << std::endl;
    }

    return ss.str();
}

/**
 */
std::string DuckDBConnection::tableInfo(const std::string& table_name)
{
    std::stringstream ss;

    {
        // std::string sql = std::string("") 
        //                 + "SELECT * EXCLUDE (column_path, segment_id, start, stats, persistent, block_id, block_offset, has_updates) "
        //                 + "FROM pragma_storage_info('" + table_name + "') "
        //                 + "USING SAMPLE 10 ROWS "
        //                 + "ORDER BY row_group_id;";
        // auto result = execute(sql, true);

        // if (result->hasError())
        //     loginf << "retrieving compression info failed: " << result->error();

        // bool has_result = !result->hasError() && result->buffer();

        // ss << "Compression: " << (has_result ? "\n" + result->buffer()->asJSON().dump(-4) : "-") << std::endl << std::endl;
    }

    std::string ret = ss.str();
    if (ret.empty())
        ret = "no info available";

    return ret;
}
