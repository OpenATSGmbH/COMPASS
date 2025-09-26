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

#include "dbexecresult.h"

#include "property.h"
#include "timeconv.h"

#include <memory>
#include <string>
#include "traced_assert.h"
#include <stdexcept>

#include <json.hpp>

#include <boost/date_time/posix_time/ptime.hpp>

class Buffer;
class PropertyList;

/**
 */
class DuckDBExecResult : public DBExecResult
{
public:
    DuckDBExecResult();
    virtual ~DuckDBExecResult();

    bool hasError() const override final;
    std::string errorString() const override final;
    boost::optional<PropertyList> propertyList() const override final;
    boost::optional<size_t> numColumns() const override final;
    boost::optional<size_t> numRows() const override final;

    bool toBuffer(Buffer& buffer,
                  const boost::optional<size_t>& offset = boost::optional<size_t>(),
                  const boost::optional<size_t>& max_entries = boost::optional<size_t>()) override final;
    ResultT<bool> readNextChunk(Buffer& buffer,
                                size_t max_entries) override final;
    
    duckdb_result* result();

    bool hasChunk() const; 

    static PropertyDataType dataTypeFromDuckDB(duckdb_type type);

private:
    friend class DuckDBPrepare;
    friend class DuckDBConnection;

    void nextChunk(std::vector<void*>& data_vectors,
                   std::vector<uint64_t*>& valid_vectors,
                   size_t num_cols);
    void fetchVectors(std::vector<void*>& data_vectors,
                      std::vector<uint64_t*>& valid_vectors,
                      size_t num_cols);

    template <typename T>
    T read(idx_t col, idx_t row)
    {
        traced_assert(result_valid_);
        throw std::runtime_error("DuckDBResult: read: not implemented for type");
    }

    template <typename T>
    T readVector(void* v, idx_t row)
    {
        traced_assert(result_valid_);
        throw std::runtime_error("DuckDBResult: readVector: not implemented for type");
    }

    mutable duckdb_result                      result_;
    mutable boost::optional<duckdb_data_chunk> chunk_;
    size_t                                     chunk_num_rows_ = 0;
    size_t                                     chunk_idx_      = 0;
    bool                                       result_valid_   = false;
    bool                                       result_error_   = false;
};

#define StandardRead(DType, DuckDBDTypeName)                         \
template<>                                                           \
inline DType DuckDBExecResult::read(idx_t col, idx_t row)            \
{                                                                    \
    traced_assert(result_valid_);                                           \
    return duckdb_value_ ## DuckDBDTypeName(&result_, col, row);     \
}

StandardRead(bool, boolean)
StandardRead(char, int8)
StandardRead(unsigned char, uint8)
StandardRead(int, int32)
StandardRead(unsigned int, uint32)
StandardRead(long, int64)
StandardRead(unsigned long, uint64)
StandardRead(float, float)
StandardRead(double, double)

template<>
inline std::string DuckDBExecResult::read(idx_t col, idx_t row)
{
    traced_assert(result_valid_);
    return std::string(duckdb_value_varchar(&result_, col, row));
}

template<>
inline nlohmann::json DuckDBExecResult::read(idx_t col, idx_t row)
{
    traced_assert(result_valid_);
    auto str = read<std::string>(col, row);
    return nlohmann::json::parse(str);
}

template<>
inline boost::posix_time::ptime DuckDBExecResult::read(idx_t col, idx_t row)
{
    traced_assert(result_valid_);
    long ts = read<long>(col, row);
    return Utils::Time::fromLong(ts);
}

#define StandardReadVector(DType)                                    \
template<>                                                           \
inline DType DuckDBExecResult::readVector(void* v, idx_t row)        \
{                                                                    \
    traced_assert(result_valid_);                                           \
    return ((DType*)v)[ row ];                                       \
}

StandardReadVector(bool)
StandardReadVector(char)
StandardReadVector(unsigned char)
StandardReadVector(int)
StandardReadVector(unsigned int)
StandardReadVector(long)
StandardReadVector(unsigned long)
StandardReadVector(float)
StandardReadVector(double)

template<>
inline std::string DuckDBExecResult::readVector(void* v, idx_t row)
{
    traced_assert(result_valid_);
    duckdb_string_t str = ((duckdb_string_t*)v)[ row ];
    if (duckdb_string_is_inlined(str)) 
        return std::string((char*)str.value.inlined.inlined, str.value.inlined.length);
    else 
        return std::string((char*)str.value.pointer.ptr, str.value.pointer.length);
}

template<>
inline nlohmann::json DuckDBExecResult::readVector(void* v, idx_t row)
{
    traced_assert(result_valid_);
    auto str = readVector<std::string>(v, row);
    return nlohmann::json::parse(str);
}

template<>
inline boost::posix_time::ptime DuckDBExecResult::readVector(void* v, idx_t row)
{
    traced_assert(result_valid_);
    long ts = readVector<long>(v, row);
    return Utils::Time::fromLong(ts);
}
