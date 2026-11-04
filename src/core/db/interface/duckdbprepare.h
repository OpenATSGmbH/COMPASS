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

#include <duckdb.h>

#include "dbprepare.h"
#include "timeconv.h"

#include <string>
#include <memory>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <json.hpp>

class DuckDBExecResult;

/**
 */
class DuckDBPrepare : public DBPrepare
{
public:
    DuckDBPrepare(duckdb_connection connection);
    virtual ~DuckDBPrepare();

    std::shared_ptr<DuckDBExecResult> executeDuckDB();

protected:
    bool init_impl(const std::string& sql_statement) override final;
    void cleanup_impl() override final;

    bool beginTransaction_impl() override final;
    bool commitTransaction_impl() override final;
    bool rollbackTransaction_impl() override final;

    bool bind_null_impl(size_t idx) override final;
    bool bind_bool_impl(size_t idx, bool v) override final;
    bool bind_char_impl(size_t idx, char v) override final;
    bool bind_uchar_impl(size_t idx, unsigned char v) override final;
    bool bind_int_impl(size_t idx, int v) override final;
    bool bind_uint_impl(size_t idx, unsigned int v) override final;
    bool bind_long_impl(size_t idx, long v) override final;
    bool bind_ulong_impl(size_t idx, unsigned long v) override final;
    bool bind_float_impl(size_t idx, float v) override final;
    bool bind_double_impl(size_t idx, double v) override final;
    bool bind_string_impl(size_t idx, const std::string& v) override final;
    bool bind_json_impl(size_t idx, const nlohmann::json& v) override final;
    bool bind_timestamp_impl(size_t idx, const boost::posix_time::ptime& v) override final;

    bool executeBinds_impl() override final;
    bool execute_impl(const ExecOptions* options, DBResult* result) override final;

    template<typename T>
    bool bind(size_t idx, const T& value)
    {
        throw std::runtime_error("DuckDBScopedPrepare: bind: not implemented for type");
        return false;
    }

private:
    duckdb_connection         connection_;
    duckdb_prepared_statement statement_;
};

#define StandardBind(DType, DuckDBDTypeName)                                         \
template<>                                                                           \
inline bool DuckDBPrepare::bind(size_t idx, const DType& value)                      \
{                                                                                    \
    return duckdb_bind_ ## DuckDBDTypeName(statement_, idx, value) == DuckDBSuccess; \
}

StandardBind(bool, boolean)
StandardBind(char, int8)
StandardBind(unsigned char, uint8)
StandardBind(int, int32)
StandardBind(unsigned int, uint32)
StandardBind(long, int64)
StandardBind(unsigned long, uint64)
StandardBind(float, float)
StandardBind(double, double)

template<>
inline bool DuckDBPrepare::bind(size_t idx, const std::string& value)
{
    return duckdb_bind_varchar_length(statement_, idx, value.data(), value.size()) == DuckDBSuccess;
}

template<>
inline bool DuckDBPrepare::bind(size_t idx, const nlohmann::json& value)
{
    auto str = value.dump();
    return bind<std::string>(idx, str);
}

template<>
inline bool DuckDBPrepare::bind(size_t idx, const boost::posix_time::ptime& value)
{
    long ts = Utils::Time::toLong(value);
    return bind<long>(idx, ts);
}

/**
 */
class DuckDBScopedPrepare : public DBScopedPrepare
{
public:
    DuckDBScopedPrepare(duckdb_connection connection,
                        const std::string& sql_statement,
                        bool begin_transaction = false) 
    :   DBScopedPrepare(std::shared_ptr<DBPrepare>(new DuckDBPrepare(connection)), sql_statement, begin_transaction) {}

    virtual ~DuckDBScopedPrepare() = default;

    std::shared_ptr<DuckDBExecResult> executeDuckDB();
};
