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

#include "duckdbprepare.h"
#include "duckdbexecresult.h"
#include "dbresult.h"

#include "logger.h"

/**
 */
DuckDBPrepare::DuckDBPrepare(duckdb_connection connection) 
:   connection_(connection) 
{
    traced_assert(connection_);
}

/**
 */
DuckDBPrepare::~DuckDBPrepare() = default;

/**
 */
bool DuckDBPrepare::init_impl(const std::string& sql_statement)
{
    auto result = duckdb_prepare(connection_, sql_statement.c_str(), &statement_);
    if (result != DuckDBSuccess)
    {
        auto err = duckdb_prepare_error(statement_);
        setError("could not prepare statement: " + std::string(err));
        return false;
    }
    return true;
}

/**
 */
void DuckDBPrepare::cleanup_impl()
{
    duckdb_destroy_prepare(&statement_);
}

/**
 */
bool DuckDBPrepare::beginTransaction_impl()
{
    auto res = duckdb_query(connection_, "BEGIN TRANSACTION", nullptr);
    return res == DuckDBSuccess;
}

/**
 */
bool DuckDBPrepare::commitTransaction_impl()
{
    auto res = duckdb_query(connection_, "COMMIT", nullptr);
    return res == DuckDBSuccess;
}

/**
 */
bool DuckDBPrepare::rollbackTransaction_impl()
{
    auto res = duckdb_query(connection_, "ROLLBACK", nullptr);
    return res == DuckDBSuccess;
}

/**
 */
bool DuckDBPrepare::executeBinds_impl()
{
    auto result = duckdb_execute_prepared(statement_, NULL);
    return result == DuckDBSuccess;
}

/**
 */
bool DuckDBPrepare::execute_impl(const ExecOptions* options, DBResult* result)
{
    traced_assert(result);

    bool fetch_buffer = result && result->buffer() != nullptr;

    auto exec_result = executeDuckDB();
    traced_assert(exec_result);

    if (exec_result->hasError())
    {
        if (result) result->setError(exec_result->errorString());
        return false;
    }

    if (fetch_buffer && !exec_result->toBuffer(*result->buffer()))
    {
        if (result) result->setError("reading query result failed");
        return false;
    }

    return true;
}

/**
 */
std::shared_ptr<DuckDBExecResult> DuckDBPrepare::executeDuckDB()
{
    std::shared_ptr<DuckDBExecResult> result(new DuckDBExecResult);

    auto state = duckdb_execute_prepared(statement_, result->result());

    result->result_valid_ = true;
    result->result_error_ = state != DuckDBSuccess;

    return result;
}

/**
 */
bool DuckDBPrepare::bind_null_impl(size_t idx) 
{ 
    return duckdb_bind_null(statement_, idx) == DuckDBSuccess; 
}

/**
 */
bool DuckDBPrepare::bind_bool_impl(size_t idx, bool v) 
{ 
    return bind<bool>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_char_impl(size_t idx, char v) 
{ 
    return bind<char>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_uchar_impl(size_t idx, unsigned char v) 
{ 
    return bind<unsigned char>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_int_impl(size_t idx, int v) 
{ 
    return bind<int>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_uint_impl(size_t idx, unsigned int v) 
{ 
    return bind<unsigned int>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_long_impl(size_t idx, long v) 
{ 
    return bind<long>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_ulong_impl(size_t idx, unsigned long v) 
{ 
    return bind<unsigned long>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_float_impl(size_t idx, float v) 
{ 
    return bind<float>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_double_impl(size_t idx, double v) 
{ 
    return bind<double>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_string_impl(size_t idx, const std::string& v) 
{ 
    return bind<std::string>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_json_impl(size_t idx, const nlohmann::json& v) 
{ 
    return bind<nlohmann::json>(idx, v); 
}

/**
 */
bool DuckDBPrepare::bind_timestamp_impl(size_t idx, const boost::posix_time::ptime& v) 
{ 
    return bind<boost::posix_time::ptime>(idx, v); 
}

/**
 */
std::shared_ptr<DuckDBExecResult> DuckDBScopedPrepare::executeDuckDB()
{
    auto prepare = dynamic_cast<DuckDBPrepare*>(db_prepare_.get());
    traced_assert(prepare);

    return prepare->executeDuckDB();
}
