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

#include "timeconv.h"

#include <string>

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include <json.hpp>

/**
 * Handles scoped appending of data to a specific duckdb table.
 * Will flush and destroy the appender on destruction.
 */
class DuckDBScopedAppender
{
public:
    DuckDBScopedAppender(duckdb_connection connection, 
                         const std::string& table_name);
    virtual ~DuckDBScopedAppender();

    bool valid() const;

    template<typename T>
    bool append(const T& value)
    {
        throw std::runtime_error("DuckDBScopedAppender: append: not implemented for type");
        return false;
    }

    bool appendNull()
    {
        if (!ok_ || duckdb_append_null(appender_) != DuckDBSuccess)
            return false;
        ++appended_;
        return true;
    }

    size_t endRow();
    bool flush();

    std::string lastAppenderError() const;
    size_t columnCount() const;
    size_t currentColumnCount() const { return appended_; }

    bool hasError() const { return error_.has_value(); }
    std::string lastError() const { return hasError() ? error_.value() : ""; }
    
private:
    void setError(const std::string& err) { error_ = err; }

    duckdb_appender appender_;
    bool ok_ = false;
    size_t appended_ = 0;

    boost::optional<std::string> error_; 
};

#define StandardApppender(DType, DuckDBDTypeName)                                     \
template<>                                                                            \
inline bool DuckDBScopedAppender::append(const DType& value)                          \
{                                                                                     \
    if (!ok_ || duckdb_append_ ## DuckDBDTypeName(appender_, value) != DuckDBSuccess) \
        return false;                                                                 \
    ++appended_;                                                                      \
    return true;                                                                      \
}

StandardApppender(bool, bool)
StandardApppender(char, int8)
StandardApppender(unsigned char, uint8)
StandardApppender(int, int32)
StandardApppender(unsigned int, uint32)
StandardApppender(long, int64)
StandardApppender(unsigned long, uint64)
StandardApppender(float, float)
StandardApppender(double, double)

template<>
inline bool DuckDBScopedAppender::append(const std::string& value)
{
    if (!ok_ || duckdb_append_varchar(appender_, value.c_str()) != DuckDBSuccess)
        return false;
    ++appended_;
    return true;
}

template<>
inline bool DuckDBScopedAppender::append(const nlohmann::json& value)
{
    auto str = value.dump();
    return append<std::string>(str);
}

template<>
inline bool DuckDBScopedAppender::append(const boost::posix_time::ptime& value)
{
    long ts = Utils::Time::toLong(value);
    return append<long>(ts);
}
