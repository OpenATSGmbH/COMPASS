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

#include "sqliteconnection.h"
#include "sqliteprepare.h"
#include "sqlitereader.h"
#include "sqliteinstance.h"

#include "buffer.h"
#include "dbcommand.h"
#include "dbcommandlist.h"
#include "dbinterface.h"
#include "dbresult.h"
#include "dbtableinfo.h"
#include "logger.h"
#include "property.h"
#include "util/timeconv.h"

#include <QApplication>

#include <cstring>

using namespace std;
using namespace Utils;

/**
 */
SQLiteConnection::SQLiteConnection(SQLiteInstance* instance,
                                   sqlite3* db_handle, 
                                   bool verbose)
:   DBConnection(instance, verbose)
,   db_handle_  (db_handle)
{
    traced_assert(db_handle);
}

/**
 */
SQLiteConnection::~SQLiteConnection()
{
    if (connected())
        disconnect();
}

/**
 */
std::shared_ptr<DBScopedPrepare> SQLiteConnection::prepareStatement(const std::string& statement, 
                                                                    bool begin_transaction)
{
    return std::shared_ptr<DBScopedPrepare>(new SQLiteScopedPrepare(db_handle_, statement, begin_transaction));
}

/**
 */
std::shared_ptr<DBScopedReader> SQLiteConnection::createReader(const std::shared_ptr<DBCommand>& select_cmd, 
                                                               size_t offset, 
                                                               size_t chunk_size)
{
    return std::shared_ptr<DBScopedReader>(new SQLiteScopedReader(db_handle_, select_cmd, offset, chunk_size));
}

/**
 */
Result SQLiteConnection::connect_impl()
{
    //nothing to do, instance handles connect => just pass open status
    bool is_open = instance()->dbOpen();
    if (!is_open)
        return Result::failed("Could not connect to closed database");
    
    return Result::succeeded();
}

/**
 */
void SQLiteConnection::disconnect_impl()
{
    //nothing to do, instance handles disconnect
}

/**
 */
Result SQLiteConnection::executeSQL_impl(const std::string& sql, 
                                         DBResult* result, 
                                         bool fetch_result_buffer)
{
    //@TODO
    return Result::failed("Not yet implemented");
}

/**
 */
bool SQLiteConnection::executeCmd_impl(const std::string& command, 
                                       const PropertyList* properties, 
                                       DBResult* result)
{
    //@TODO
    return false;
}

/**
 */
ResultT<std::vector<std::string>> SQLiteConnection::getTableList_impl()
{
    std::vector<std::string> tables;

    DBCommand command;
    command.set("SELECT name FROM sqlite_master WHERE type='table' ORDER BY name DESC;");
    // command.set ("SELECT name FROM sqlite_master WHERE name != 'sqlite_sequence' ORDER BY name  DESC;");

    PropertyList list;
    list.addProperty("name", PropertyDataType::STRING);
    command.list(list);

    std::shared_ptr<DBResult> result = execute(command);
    if (result->hasError())
        return ResultT<std::vector<std::string>>::failed(result->error());
    if (!result->buffer() || !result->containsData())
        return ResultT<std::vector<std::string>>::failed("Table list could not be retrieved");

    traced_assert(result->containsData());
    std::shared_ptr<Buffer> buffer = result->buffer();

    unsigned int size = buffer->size();

    loginf << "buffer size " << size;

    std::string table_name;

    for (unsigned int cnt = 0; cnt < size; cnt++)
    {
        table_name = buffer->get<std::string>("name").get(cnt);

        if (table_name.find("sqlite") != std::string::npos)  // ignore sqlite system tables
            continue;

        tables.push_back(buffer->get<std::string>("name").get(cnt));
    }
    return ResultT<std::vector<std::string>>::succeeded(tables);
}
