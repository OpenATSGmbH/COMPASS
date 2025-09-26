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

#include "duckdbexecresult.h"
#include "buffer.h"
#include "property_templates.h"
#include "propertylist.h"

#include "traced_assert.h"

/**
 */
PropertyDataType DuckDBExecResult::dataTypeFromDuckDB(duckdb_type type)
{
    if (type == duckdb_type::DUCKDB_TYPE_BOOLEAN)
        return PropertyDataType::BOOL;
    else if (type == duckdb_type::DUCKDB_TYPE_TINYINT)
        return PropertyDataType::CHAR;
    else if (type == duckdb_type::DUCKDB_TYPE_UTINYINT)
        return PropertyDataType::UCHAR;
    else if (type == duckdb_type::DUCKDB_TYPE_INTEGER)
        return PropertyDataType::INT;
    else if (type == duckdb_type::DUCKDB_TYPE_UINTEGER)
        return PropertyDataType::UINT;
    else if (type == duckdb_type::DUCKDB_TYPE_BIGINT)
        return PropertyDataType::LONGINT;
    else if (type == duckdb_type::DUCKDB_TYPE_UBIGINT)
        return PropertyDataType::ULONGINT;
    else if (type == duckdb_type::DUCKDB_TYPE_FLOAT)
        return PropertyDataType::FLOAT;
    else if (type == duckdb_type::DUCKDB_TYPE_DOUBLE)
        return PropertyDataType::DOUBLE;
    else if (type == duckdb_type::DUCKDB_TYPE_VARCHAR)
        return PropertyDataType::STRING;
    
    //@TODO: more types needed (how to handle types like 'list'?)

    logerr << "data type not implemented: " << type;
    traced_assert(false);

    return PropertyDataType::BOOL;
}

/**
 */
DuckDBExecResult::DuckDBExecResult() = default;

/**
 */
DuckDBExecResult::~DuckDBExecResult()
{
    if (result_valid_)
    {
        //@TODO: any extra freeing needed?
        duckdb_destroy_result(&result_);
        result_valid_ = false;
    }
}

/**
 */
bool DuckDBExecResult::hasError() const
{
    traced_assert(result_valid_);
    return result_error_;
}

/**
 */
std::string DuckDBExecResult::errorString() const
{
    if (!hasError())
        return "";

    std::string errstr = (duckdb_result_error(&result_));
    if (errstr.empty())
        errstr = "unknown error";

    return errstr;
}

/**
 */
boost::optional<PropertyList> DuckDBExecResult::propertyList() const
{
    traced_assert(result_valid_);

    if (hasError())
        return boost::optional<PropertyList>();

    auto num_cols = numColumns();
    if (!num_cols.has_value())
        return boost::optional<PropertyList>();

    size_t nc = num_cols.value();

    //collect props
    PropertyList properties;
    for (idx_t c = 0; c < nc; ++c)
    {
        std::string name(duckdb_column_name(&result_, c));
        auto dtype = DuckDBExecResult::dataTypeFromDuckDB(duckdb_column_type(&result_, c));

        properties.addProperty(name, dtype);
    }

    return properties;
}

/**
 */
boost::optional<size_t> DuckDBExecResult::numColumns() const
{
    traced_assert(result_valid_);

    if (hasError())
        return boost::optional<size_t>();

    size_t n = duckdb_column_count(&result_);
    return n;
}

/**
 */
boost::optional<size_t> DuckDBExecResult::numRows() const
{
    traced_assert(result_valid_);

    if (hasError())
        return boost::optional<size_t>();

    size_t n = duckdb_row_count(&result_);
    return n;
}

/**
 */
duckdb_result* DuckDBExecResult::result()
{
    return &result_;
}

/**
 * Fill a given buffer with the current result.
 * In this version the scheme is specified by the buffer.
 */
bool DuckDBExecResult::toBuffer(Buffer& buffer,
                                const boost::optional<size_t>& offset,
                                const boost::optional<size_t>& max_entries)
{
    traced_assert(result_valid_);

    const auto& properties = buffer.properties();

    auto nc = numColumns();
    auto nr = numRows();
    traced_assert(nc.has_value());
    traced_assert(nr.has_value());

    idx_t col_count = nc.value();
    idx_t row_count = nr.value();
    traced_assert(col_count == properties.size()); // result column count must match provided buffer

    #define UpdateFuncToBuffer(PDType, DType, Suffix)                      \
        bool is_null = duckdb_value_is_null(&result_, c, r);               \
        if (!is_null)                                                      \
        {                                                                  \
            DType v = read<DType>(c, r);                                   \
            buffer.get<DType>(pname).set(buf_idx, v);                      \
        }

    #define NotFoundFuncToBuffer                                                                     \
        logerr << "unknown property type " << Property::asString(dtype); \
        traced_assert(false);

    size_t r0 = offset.has_value() ? offset.value() : 0;
    size_t r1 = std::min(row_count, max_entries.has_value() ? r0 + max_entries.value() : row_count);

    //nothing to read?
    if (r0 >= r1 || r0 >= row_count)
        return true;

    //read rows into buffer
    for (idx_t r = r0, buf_idx = 0; r < r1; ++r, ++buf_idx)
    {
        for (idx_t c = 0; c < col_count; ++c)
        {
            const auto& p = properties.at(c);
            auto dtype = p.dataType();
            const auto& pname = p.name();

            SwitchPropertyDataType(dtype, UpdateFuncToBuffer, NotFoundFuncToBuffer)
        }
    }

    return true;
}

/**
 */
void DuckDBExecResult::nextChunk(std::vector<void*>& data_vectors,
                                 std::vector<uint64_t*>& valid_vectors, 
                                 size_t num_cols)
{
    chunk_          = duckdb_fetch_chunk(result_);
    chunk_idx_      = 0;
    chunk_num_rows_ = duckdb_data_chunk_get_size(chunk_.value());

    fetchVectors(data_vectors, valid_vectors, num_cols);
}

/**
 */
void DuckDBExecResult::fetchVectors(std::vector<void*>& data_vectors,
                                    std::vector<uint64_t*>& valid_vectors,
                                    size_t num_cols)
{
    data_vectors.assign(num_cols, nullptr);
    valid_vectors.assign(num_cols, nullptr);

    if (!hasChunk())
        return;

    for (size_t i = 0; i < num_cols; ++i)
    {
        duckdb_vector vec      = duckdb_data_chunk_get_vector(chunk_.value(), i);
        auto          data     = duckdb_vector_get_data(vec);
        auto          validity = duckdb_vector_get_validity(vec);

        data_vectors [ i ] = data;
        valid_vectors[ i ] = validity;
    }
}

/**
 */
bool DuckDBExecResult::hasChunk() const
{
    return (chunk_.has_value() && chunk_.value() != nullptr);
}

/**
 */
ResultT<bool> DuckDBExecResult::readNextChunk(Buffer& buffer,
                                              size_t max_entries)
{
    traced_assert(result_valid_);

    const auto& properties = buffer.properties();
    size_t np = properties.size();

    // loginf << "reading...";
    // loginf << "   chunk idx:   " << chunk_idx_;
    // loginf << "   chunk rows:  " << chunk_num_rows_;
    // loginf << "   max entries: " << max_entries;
    // loginf << "   num props:   " << np;

    std::vector<void*>     data_vectors;
    std::vector<uint64_t*> valid_vectors;

    //init chunk?
    if (!chunk_.has_value())
        nextChunk(data_vectors, valid_vectors, np);
    else
        fetchVectors(data_vectors, valid_vectors, np);

    traced_assert(chunk_.has_value());

    //already at end? => no more data
    if (!hasChunk())
        return ResultT<bool>::succeeded(false);

    #define UpdateFuncNextChunk(PDType, DType, Suffix)                                                \
        auto& vec = buffer.get<DType>(pname);                                                         \
        NullableVector<DType>* vec_ptr = &vec;                                                        \
        auto data_vec  = data_vectors[ c ];                                                           \
        auto valid_vec = valid_vectors[ c ];                                                          \
                                                                                                      \
        auto cb = [ vec_ptr, data_vec, valid_vec, this ] (size_t row, size_t buf_idx)                 \
        {                                                                                             \
            bool is_null = !duckdb_validity_row_is_valid(valid_vec, row);                             \
            if (!is_null) vec_ptr->set(buf_idx, this->readVector<DType>(data_vec, row));              \
        };                                                                                            \
                                                                                                      \
        readers[ c ] = cb;

    #define NotFoundFuncNextChunk                                                                         \
        logerr << "unknown property type " << Property::asString(dtype); \
        traced_assert(false);

    std::vector<std::function<void(size_t, size_t)>> readers(np);

    auto updateReaders = [ & ] ()
    {
        for (idx_t c = 0; c < np; ++c)
        {
            const auto& p = properties.at(c);
            auto dtype = p.dataType();
            const auto& pname = p.name();

            SwitchPropertyDataType(dtype, UpdateFuncNextChunk, NotFoundFuncNextChunk)
        }
    };

    updateReaders();

    //read data until we reach end of result or max entries
    size_t buf_idx = 0;
    while (buf_idx < max_entries && chunk_.value() != nullptr)
    {
        //read until chunk's end
        for (size_t r = chunk_idx_; r < chunk_num_rows_; ++r, ++buf_idx, ++chunk_idx_)
        {
            //reached max entries? => break
            if (buf_idx == max_entries)
                break;

            //fetch row data
            for (idx_t c = 0; c < np; ++c)
                readers[ c ] (r, buf_idx);
        }

        //fetch next chunk?
        if (chunk_idx_ >= chunk_num_rows_)
        {
            nextChunk(data_vectors, valid_vectors, np);
            updateReaders(); //new chunk - new data vectors - new readers
        }
    }

    traced_assert(chunk_idx_ <= chunk_num_rows_);
    traced_assert(buf_idx <= max_entries);

    bool has_more = hasChunk();

    return ResultT<bool>::succeeded(has_more);
}
