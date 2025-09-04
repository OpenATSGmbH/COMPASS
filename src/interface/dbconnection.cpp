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

#include "dbconnection.h"
#include "duckdbexecresult.h"
#include "dbprepare.h"
#include "dbreader.h"

#include "sqlgenerator.h"

#include "buffer.h"
#include "dbcommand.h"
#include "dbcommandlist.h"
#include "dbinstance.h"
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

#include <QElapsedTimer>

/**
 */
DBConnection::DBConnection(DBInstance* instance, bool verbose)
:   instance_ (instance  )
,   connected_(false     )
,   verbose_  (verbose   )
{
    if (verbose_)
        loginf;

    traced_assert(instance_);
}

/**
 */
DBConnection::~DBConnection()
{
    if (verbose_)
        logdbg << "start";

    traced_assert(!connected());
}

/**
 */
db::SQLConfig DBConnection::sqlConfiguration(bool verbose) const
{
    return instance()->sqlConfiguration(verbose);
}

/**
 * Returns the current connection status.
 */
std::string DBConnection::status() const
{
    if (connected_)
        return "Ready";
    else
        return "Not connected";
}

/**
 * Connects to the given database file. Will disconnect from the current db if connected.
 */
Result DBConnection::connect()
{
    traced_assert(instance_->dbOpen());

    if (verbose_)
        loginf << "connecting to instance '" << instance_->dbFilename() << "'...";

    //close any opened connection
    if (connected())
        disconnect();

    auto connect_result = connect_impl();
    if (!connect_result.ok())
        return connect_result;

    connected_ = true;

    if (verbose_)
        loginf << "done";

    return Result::succeeded();
}

/**
 * Disconnects from the currently connected database.
 */
void DBConnection::disconnect()
{
    if (verbose_)
        logdbg << "connected " << connected_;

    if (!connected_)
        return;

    if (verbose_)
        logdbg << "";

    //close any active reader
    stopRead();

    //stop active performance metrics
    stopPerformanceMetrics();

    disconnect_impl();

    connected_ = false;
}

/**
 * Executes the given sql query without returning any data or specific error messages.
 */
Result DBConnection::execute(const std::string& sql)
{
    if (verbose_)
        logdbg << "sql statement execute: '" << sql << "'";

    traced_assert(connected());

    return executeSQL_impl(sql, nullptr, false);
}

/**
 * Executes the given sql query and optionally fetches a result buffer (if implemented by derived connection).
 */
std::shared_ptr<DBResult> DBConnection::execute(const std::string& sql, bool fetch_result_buffer)
{
    if (verbose_)
        logdbg << "sql statement execute: '" << sql << "'";

    traced_assert(connected());

    std::shared_ptr<DBResult> result(new DBResult());

    auto res = executeSQL_impl(sql, result.get(), fetch_result_buffer);

    if (res.ok() && fetch_result_buffer && !result->buffer())
    {
        //logerr << "query '" << sql << "': buffer creation failed";
        result->setError("buffer creation failed");
    }

    return result;
}

/**
 * Executes the given DBCommand.
 */
std::shared_ptr<DBResult> DBConnection::execute(const DBCommand& command)
{
    if (verbose_)
        logdbg << "executing single command";

    traced_assert(connected());

    bool fetch_buffer = command.expectsResult();

    std::shared_ptr<DBResult> result(new DBResult());
    bool ok = executeCmd_impl(command.get(), fetch_buffer ? &command.resultList() : nullptr, result.get());

    if (ok && fetch_buffer && !result->buffer())
    {
        //logerr << "command '" << command.get() << "': buffer creation failed";
        result->setError("buffer creation failed");
    }
    
    return result;
}

/**
 * Executes the given DBCommandList.
 */
std::shared_ptr<DBResult> DBConnection::execute(const DBCommandList& command_list)
{
    if (verbose_)
        logdbg << "executing " << command_list.getNumCommands() << " command(s)";

    traced_assert(connected());

    std::shared_ptr<DBResult> dbresult(new DBResult());

    unsigned int num_commands = command_list.getNumCommands();
    bool fetch_buffers = command_list.getExpectDataResult();

    std::shared_ptr<Buffer> buffer;
    if (fetch_buffers)
        buffer.reset(new Buffer(command_list.getResultList()));

    for (unsigned int cnt = 0; cnt < num_commands; cnt++)
    {
        const auto& cmd = command_list.getCommands().at(cnt);

        std::shared_ptr<DBResult> result(new DBResult());
        auto ok = executeCmd_impl(cmd, fetch_buffers ? &command_list.getResultList() : nullptr, result.get());
        if (!ok)
        {
            //logerr << "command '" << cmd << "' failed: " << result->error();
            dbresult->setError(result->error());
            break;
        }
        else if (fetch_buffers)
        {
            if (!result->buffer())
            {
                //logerr << "command '" << cmd << "': buffer creation failed";
                dbresult->setError("buffer creation failed");
                break;
            }

            std::shared_ptr<Buffer> b = result->buffer();
            buffer->seizeBuffer(*b);
        }
    }

    if (fetch_buffers && !dbresult->hasError())
        dbresult->buffer(buffer);

    if (verbose_)
        logdbg << "end";

    return dbresult;
}

/**
 */
Result DBConnection::executePragma(const db::SQLPragma& pragma)
{
    auto sql = sqlGenerator().configurePragma(pragma);
    return execute(sql);
}

/**
 * Creates a table of the given name and column properties.
 */
Result DBConnection::createTable(const std::string& table_name, 
                                 const std::vector<DBTableColumnInfo>& column_infos,
                                 const std::vector<db::Index>& indices,
                                 bool table_must_not_exist)
{
    traced_assert(connected());
    traced_assert(instance_);

    const auto& table_info = instance_->tableInfo();

    auto it = table_info.find(table_name);
    if (it != table_info.end())
    {
        //@TODO: check properties of existing table against requested one?
        return table_must_not_exist ? Result::failed("table '" + table_name + "' already exists") : Result::succeeded();
    }

    auto res = createTableInternal(table_name, column_infos, indices, true);
    if (!res.ok())
        return res;

    return Result::succeeded();
}

/**
 */
Result DBConnection::createTableInternal(const std::string& table_name, 
                                         const std::vector<DBTableColumnInfo>& column_infos,
                                         const std::vector<db::Index>& indices,
                                         bool verbose)
{
    traced_assert(connected());

    //get sql statement
    std::string sql = SQLGenerator(sqlConfiguration(verbose)).getCreateTableStatement(table_name, column_infos, indices);

    auto result = execute(sql, false);
    traced_assert(result);

    if (result->hasError())
    {
        //logerr << "creating table '" << table_name << "' failed: " << result->error();
        return Result::failed(result->error());
    }

    return Result::succeeded();
}

/**
 */
Result DBConnection::deleteTable(const std::string& table_name)
{
    auto res = execute("DROP TABLE " + table_name + ";");
    if (!res.ok())
        return res;

    return Result::succeeded();
}

/**
 */
Result DBConnection::deleteTableContents(const std::string& table_name)
{
    return execute("DELETE FROM " + table_name + ";");
}

/**
 * Inserts the given buffer into the given table.
 */
Result DBConnection::insertBuffer(const std::string& table_name, 
                                  const std::shared_ptr<Buffer>& buffer,
                                  const boost::optional<size_t>& idx_from, 
                                  const boost::optional<size_t>& idx_to,
                                  PropertyList* table_properties)
{
    traced_assert(connected());
    traced_assert(buffer);

    if (buffer->size() == 0)
        return Result::succeeded();

    QElapsedTimer t;
    if (perf_metrics_.has_value())
    {
        t.start();
    }

    auto res = insertBuffer_impl(table_name, buffer, idx_from, idx_to, table_properties);

    if (perf_metrics_.has_value())
    {
        perf_metrics_->insert_time       += t.elapsed();
        perf_metrics_->insert_num_chunks += 1;
        perf_metrics_->insert_num_rows   += buffer->size();
    }

    //if (!res.first)
    //    logerr << "inserting into table '" << table_name << "' failed: " << res.second;

    return res;
}

/**
 * Updates the given table using values of the given buffer.
 */
Result DBConnection::updateBuffer(const std::string& table_name, 
                                  const std::shared_ptr<Buffer>& buffer,
                                  const std::string& key_column,
                                  const boost::optional<size_t>& idx_from, 
                                  const boost::optional<size_t>& idx_to)
{
    traced_assert(connected());
    traced_assert(buffer);

    if (buffer->size() == 0)
        return Result::succeeded();

    QElapsedTimer t;
    if (perf_metrics_.has_value())
    {
        t.start();
    }

    size_t idx0 = idx_from.has_value() ? idx_from.value()   : 0;
    size_t idx1 = idx_to.has_value()   ? idx_to.value() + 1 : buffer->size();
    traced_assert(idx1 >= idx0);

    auto res = updateBuffer_impl(table_name, buffer, key_column, idx_from, idx_to);

    if (perf_metrics_.has_value())
    {
        perf_metrics_->update_time       += t.elapsed();
        perf_metrics_->update_num_chunks += 1;
        perf_metrics_->update_num_rows   += idx1 - idx0;
    }

    //if (!res.first)
    //    logerr << "updating table '" << table_name 
    //           << "' at key column '" << key_column << "' failed: " << res.second;

    return res;
}

/**
 * Default implementation: use prepared INSERT statement.
 */
Result DBConnection::insertBuffer_impl(const std::string& table_name, 
                                       const std::shared_ptr<Buffer>& buffer,
                                       const boost::optional<size_t>& idx_from, 
                                       const boost::optional<size_t>& idx_to,
                                       PropertyList* table_properties)
{
    auto sql = SQLGenerator(sqlConfiguration()).getInsertDBUpdateStringBind(buffer, table_name);

    //loginf << "executing statement:\n" << sql;

    auto stmnt = prepareStatement(sql, true);
    traced_assert(stmnt);

    if (!stmnt->valid())
        return Result::failed("Could not prepare insert statement: " + stmnt->lastError());

    auto res = stmnt->executeBuffer(buffer, idx_from, idx_to);
    if (!res.ok())
        return Result::failed("Could not execute insert statement on buffer: " + res.error());

    //cleanup prepared statement
    stmnt.reset();

    return Result::succeeded();
}

/**
 * Default implementation: use prepared UPDATE statement.
 */
Result DBConnection::updateBuffer_impl(const std::string& table_name, 
                                       const std::shared_ptr<Buffer>& buffer,
                                       const std::string& key_column,
                                       const boost::optional<size_t>& idx_from, 
                                       const boost::optional<size_t>& idx_to)
{
    auto sql = SQLGenerator(sqlConfiguration()).getCreateDBUpdateStringBind(buffer, key_column, table_name);

    auto stmnt = prepareStatement(sql, true);
    traced_assert(stmnt);

    if (!stmnt->valid())
        return Result::failed("Could not prepare update statement" + stmnt->lastError());

    auto res = stmnt->executeBuffer(buffer, idx_from, idx_to);
    if (!res.ok())
        return Result::failed("Could not execute update statement on buffer: " + res.error());

    //cleanup prepared statement
    stmnt.reset();

    return Result::succeeded();
}

/**
 * Gets a list of tables in the database.
 */
ResultT<std::vector<std::string>> DBConnection::getTableList()
{
    traced_assert(connected());

    auto res = getTableList_impl();

    //if (!res.first)
    //    logerr << "retrieving table list failed"; 

    return res;
}

/**
 * Gets info about a specific table's columns.
 */
ResultT<DBTableInfo> DBConnection::getColumnList(const std::string& table)
{
    traced_assert(connected());

    auto res = getColumnList_impl(table);

    //if (!res.first)
    //    logerr << "retrieving column list failed"; 

    return res;
}

/**
 * Basic implementation which should work for sql like interfaces.
 */
ResultT<DBTableInfo> DBConnection::getColumnList_impl(const std::string& table)
{
    DBTableInfo table_info(table);

    DBCommand command;
    command.set("PRAGMA table_info(" + table + ")");

    // int cid, string name, string type,int notnull, string dflt_value, int pk

    PropertyList list;
    list.addProperty("cid", PropertyDataType::INT);
    list.addProperty("name", PropertyDataType::STRING);
    list.addProperty("type", PropertyDataType::STRING);
    list.addProperty("notnull", PropertyDataType::BOOL);
    list.addProperty("dfltvalue", PropertyDataType::STRING);
    list.addProperty("pk", PropertyDataType::BOOL);

    command.list(list);

    std::shared_ptr<DBResult> result = execute(command);
    traced_assert(result);

    if (result->hasError())
        return ResultT<DBTableInfo>::failed(result->error());

    if (!result->containsData())
        return ResultT<DBTableInfo>::failed("Table information could not be retrieved");

    std::shared_ptr<Buffer> buffer = result->buffer();

    for (unsigned int cnt = 0; cnt < buffer->size(); cnt++)
    {
        traced_assert(buffer->has<std::string>("type"));

        std::string data_type = buffer->get<std::string>("type").get(cnt);

        traced_assert(buffer->has<std::string>("name"));
        traced_assert(buffer->has<bool>("pk"));
        traced_assert(buffer->has<bool>("notnull"));

        table_info.addColumn(buffer->get<std::string>("name").get(cnt), 
                             data_type,
                             buffer->get<bool>("pk").get(cnt),
                             !buffer->get<bool>("notnull").get(cnt), 
                             "");
    }

    return ResultT<DBTableInfo>::succeeded(table_info);
}

/**
 */
Result DBConnection::startRead(const std::shared_ptr<DBCommand>& select_cmd, 
                               size_t offset, 
                               size_t chunk_size)
{
    traced_assert(connected());
    traced_assert(select_cmd && QString::fromStdString(select_cmd->get()).startsWith("SELECT ") && select_cmd->expectsResult());

    active_reader_ = createReader(select_cmd, offset, chunk_size);
    return Result(active_reader_->isReady(), active_reader_->lastError());
}

/**
 */
std::shared_ptr<DBResult> DBConnection::readChunk()
{
    traced_assert(connected());

    std::shared_ptr<DBResult> result;

    if (!active_reader_)
    {
        result.reset(new DBResult);
        result->setError("no reader active");
        return result;
    }
    if (!active_reader_->isReady())
    {
        result.reset(new DBResult);
        result->setError("active reader not ready");
        return result;
    }

    QElapsedTimer t;
    if (perf_metrics_.has_value())
    {
        t.start();
    }

    result = active_reader_->readChunk();
    traced_assert(result);

    if (perf_metrics_.has_value())
    {
        perf_metrics_->read_time       += t.elapsed();
        perf_metrics_->read_num_chunks += (result && result->containsData() ? 1 : 0);
        perf_metrics_->read_num_rows   += (result && result->containsData() ? result->buffer()->size() : 0);
    }

    //error during read?
    if (result->hasError())
        return result;

    traced_assert(result->buffer() && result->containsData());

    if (verbose_)
        logdbg << "read " << result->buffer()->size()
               << " left " << active_reader_->numLeft() << " hasmore " << result->hasMore();

    // if (!result->hasMore())
    // {
    //     //reader has finished => reset
    //     stopRead();
    // }

    return result;
}

/**
 */
void DBConnection::stopRead()
{
    active_reader_.reset();
}

/**
 */
SQLGenerator DBConnection::sqlGenerator() const
{
    return SQLGenerator(sqlConfiguration());
}

/**
 */
void DBConnection::startPerformanceMetrics() const
{
    perf_metrics_ = db::PerformanceMetrics();
    perf_metrics_->valid = true;
}

/**
 */
db::PerformanceMetrics DBConnection::stopPerformanceMetrics() const
{
    if(!perf_metrics_.has_value())
        return db::PerformanceMetrics();

    db::PerformanceMetrics pm = perf_metrics_.value();
    perf_metrics_.reset();

    return pm;
}

/**
 */
bool DBConnection::hasActivePerformanceMetrics() const
{
    return perf_metrics_.has_value();
}

/**
 */
ResultT<DBConnection::TableInfo> DBConnection::createTableInfo()
{
    traced_assert(connected());

    TableInfo table_info;

    auto list_res = getTableList();
    if (!list_res.ok())
        return list_res;
    
    traced_assert(list_res.hasResult());

    for (const auto& tname : list_res.result())
    {
        auto table_res = getColumnList(tname);
        if (!table_res.ok())
            return table_res;

        traced_assert(table_res.hasResult());

        table_info[ tname ] = table_res.result();
    }

    return ResultT<TableInfo>::succeeded(table_info);
}
