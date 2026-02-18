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

#include "sqliteprepare.h"
#include "dbexecresult.h"

#include "logger.h"

/**
 */
SQLitePrepare::SQLitePrepare(sqlite3* connection) 
:   connection_(connection) 
{
    traced_assert(connection_);
}

/**
 */
SQLitePrepare::~SQLitePrepare() = default;

/**
 */
bool SQLitePrepare::init_impl(const std::string& sql_statement)
{
    int result;
    const char* remaining_sql;

    // Prepare the select statement
    remaining_sql = NULL;
    result = sqlite3_prepare_v2(connection_, sql_statement.c_str(), sql_statement.size(), &statement_, &remaining_sql);
    if (result != SQLITE_OK)
    {
        setError(sqlite3_errmsg(connection_));
        return false;
    }

    if (remaining_sql && *remaining_sql != '\0')
    {
        setError("there was unparsed sql text: " + std::string(remaining_sql));
        return false;
    }
    
    return true;
}

/**
 */
void SQLitePrepare::cleanup_impl()
{
    sqlite3_finalize(statement_);
}

/**
 */
void SQLitePrepare::cleanupBinds_impl()
{
    sqlite3_clear_bindings(statement_);
    sqlite3_reset(statement_);
}

/**
 */
bool SQLitePrepare::beginTransaction_impl()
{
    char* sErrMsg = 0;
    auto res = sqlite3_exec(connection_, "BEGIN TRANSACTION", NULL, NULL, &sErrMsg);
    return res == SQLITE_OK;
}

/**
 */
bool SQLitePrepare::commitTransaction_impl()
{
    char* sErrMsg = 0;
    auto res = sqlite3_exec(connection_, "END TRANSACTION", NULL, NULL, &sErrMsg);
    return res == SQLITE_OK;
}

/**
 */
bool SQLitePrepare::rollbackTransaction_impl()
{
    return false;
}

/**
 */
bool SQLitePrepare::executeBinds_impl()
{
    //@TODO
    return false;
}

/**
 */
bool SQLitePrepare::execute_impl(const ExecOptions* options, DBResult* result)
{
    //@TODO
    return false;
}

/**
 */
bool SQLitePrepare::bind_null_impl(size_t idx) 
{ 
    return sqlite3_bind_null(statement_, idx) == SQLITE_OK; 
}

/**
 */
bool SQLitePrepare::bind_bool_impl(size_t idx, bool v) 
{ 
    return bind<int>(idx, static_cast<int>(v)); 
}

/**
 */
bool SQLitePrepare::bind_char_impl(size_t idx, char v) 
{ 
    return bind<int>(idx, static_cast<int>(v)); 
}

/**
 */
bool SQLitePrepare::bind_uchar_impl(size_t idx, unsigned char v) 
{ 
    return bind<int>(idx, static_cast<int>(v)); 
}

/**
 */
bool SQLitePrepare::bind_int_impl(size_t idx, int v) 
{ 
    return bind<int>(idx, v); 
}

/**
 */
bool SQLitePrepare::bind_uint_impl(size_t idx, unsigned int v) 
{ 
    return bind<int>(idx, static_cast<int>(v)); 
}

/**
 */
bool SQLitePrepare::bind_long_impl(size_t idx, long v) 
{ 
    return bind<long>(idx, v); 
}

/**
 */
bool SQLitePrepare::bind_ulong_impl(size_t idx, unsigned long v) 
{ 
    return bind<long>(idx, static_cast<long>(v)); 
}

/**
 */
bool SQLitePrepare::bind_float_impl(size_t idx, float v) 
{ 
    return bind<double>(idx, static_cast<double>(v)); 
}

/**
 */
bool SQLitePrepare::bind_double_impl(size_t idx, double v) 
{ 
    return bind<double>(idx, v); 
}

/**
 */
bool SQLitePrepare::bind_string_impl(size_t idx, const std::string& v) 
{ 
    return bind<std::string>(idx, v); 
}

/**
 */
bool SQLitePrepare::bind_json_impl(size_t idx, const nlohmann::json& v) 
{ 
    return bind<nlohmann::json>(idx, v); 
}

/**
 */
bool SQLitePrepare::bind_timestamp_impl(size_t idx, const boost::posix_time::ptime& v) 
{ 
    return bind<boost::posix_time::ptime>(idx, v); 
}
