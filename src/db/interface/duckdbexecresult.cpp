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
 * Check if a row is valid in a DuckDB validity bitmap.
 * Returns true if validity is nullptr (all valid) or the bit is set.
 */
static inline bool isDuckDBValid(const uint64_t* validity, size_t row)
{
    return !validity || (validity[row / 64] & (1ULL << (row % 64)));
}

/**
 * Bulk load a column of fixed-width type T using memcpy.
 * Works for: char, uchar, int, uint, long, ulong, float, double (and bool via specialization).
 */
template <typename T>
static void bulkLoadColumn(Buffer& buffer, const std::string& name,
                           void* data, uint64_t* validity,
                           size_t dst_offset, size_t src_offset, size_t count)
{
    buffer.get<T>(name).bulkSet(
        static_cast<unsigned int>(dst_offset),
        static_cast<const T*>(data) + src_offset,
        validity, src_offset, count);
}

/**
 * Bulk load a string column from DuckDB's duckdb_string_t format.
 */
static void bulkLoadStringColumn(Buffer& buffer, const std::string& name,
                                 void* data, uint64_t* validity,
                                 size_t dst_offset, size_t src_offset, size_t count)
{
    auto& vec = buffer.get<std::string>(name);
    vec.ensureMinSize(static_cast<unsigned int>(dst_offset + count));

    const duckdb_string_t* strs = static_cast<const duckdb_string_t*>(data);

    for (size_t i = 0; i < count; i++)
    {
        size_t src_row = src_offset + i;
        unsigned int dst_idx = static_cast<unsigned int>(dst_offset + i);

        if (isDuckDBValid(validity, src_row))
        {
            duckdb_string_t str = strs[src_row];
            if (duckdb_string_is_inlined(str))
                vec.set(dst_idx, std::string(str.value.inlined.inlined, str.value.inlined.length));
            else
                vec.set(dst_idx, std::string(str.value.pointer.ptr, str.value.pointer.length));
        }
        else
        {
            vec.setNull(dst_idx);
        }
    }
}

/**
 * Bulk load a JSON column from DuckDB's duckdb_string_t format.
 */
static void bulkLoadJSONColumn(Buffer& buffer, const std::string& name,
                               void* data, uint64_t* validity,
                               size_t dst_offset, size_t src_offset, size_t count)
{
    auto& vec = buffer.get<nlohmann::json>(name);
    vec.ensureMinSize(static_cast<unsigned int>(dst_offset + count));

    const duckdb_string_t* strs = static_cast<const duckdb_string_t*>(data);

    for (size_t i = 0; i < count; i++)
    {
        size_t src_row = src_offset + i;
        unsigned int dst_idx = static_cast<unsigned int>(dst_offset + i);

        if (isDuckDBValid(validity, src_row))
        {
            duckdb_string_t str = strs[src_row];
            std::string s;
            if (duckdb_string_is_inlined(str))
                s = std::string(str.value.inlined.inlined, str.value.inlined.length);
            else
                s = std::string(str.value.pointer.ptr, str.value.pointer.length);
            vec.set(dst_idx, nlohmann::json::parse(s));
        }
        else
        {
            vec.setNull(dst_idx);
        }
    }
}

/**
 * Bulk load a timestamp column. DuckDB stores as int64_t microseconds.
 */
static void bulkLoadTimestampColumn(Buffer& buffer, const std::string& name,
                                    void* data, uint64_t* validity,
                                    size_t dst_offset, size_t src_offset, size_t count)
{
    auto& vec = buffer.get<boost::posix_time::ptime>(name);
    vec.ensureMinSize(static_cast<unsigned int>(dst_offset + count));

    const long* timestamps = static_cast<const long*>(data);

    for (size_t i = 0; i < count; i++)
    {
        size_t src_row = src_offset + i;
        unsigned int dst_idx = static_cast<unsigned int>(dst_offset + i);

        if (isDuckDBValid(validity, src_row))
            vec.set(dst_idx, Utils::Time::fromLong(timestamps[src_row]));
        else
            vec.setNull(dst_idx);
    }
}

/**
 */
ResultT<bool> DuckDBExecResult::readNextChunk(Buffer& buffer,
                                              size_t max_entries)
{
    traced_assert(result_valid_);

    const auto& properties = buffer.properties();
    size_t np = properties.size();

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

    //read data until we reach end of result or max entries
    size_t buf_idx = 0;
    while (buf_idx < max_entries && chunk_.value() != nullptr)
    {
        size_t rows_avail  = chunk_num_rows_ - chunk_idx_;
        size_t rows_needed = max_entries - buf_idx;
        size_t rows_to_copy = std::min(rows_avail, rows_needed);

        //bulk load each column
        for (idx_t c = 0; c < np; ++c)
        {
            const auto& p = properties.at(c);
            auto dtype = p.dataType();
            const auto& pname = p.name();

            switch (dtype)
            {
                case PropertyDataType::BOOL:
                    bulkLoadColumn<bool>(buffer, pname, data_vectors[c], valid_vectors[c],
                                         buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::CHAR:
                    bulkLoadColumn<char>(buffer, pname, data_vectors[c], valid_vectors[c],
                                         buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::UCHAR:
                    bulkLoadColumn<unsigned char>(buffer, pname, data_vectors[c], valid_vectors[c],
                                                  buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::INT:
                    bulkLoadColumn<int>(buffer, pname, data_vectors[c], valid_vectors[c],
                                        buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::UINT:
                    bulkLoadColumn<unsigned int>(buffer, pname, data_vectors[c], valid_vectors[c],
                                                 buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::LONGINT:
                    bulkLoadColumn<long int>(buffer, pname, data_vectors[c], valid_vectors[c],
                                             buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::ULONGINT:
                    bulkLoadColumn<unsigned long int>(buffer, pname, data_vectors[c], valid_vectors[c],
                                                      buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::FLOAT:
                    bulkLoadColumn<float>(buffer, pname, data_vectors[c], valid_vectors[c],
                                          buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::DOUBLE:
                    bulkLoadColumn<double>(buffer, pname, data_vectors[c], valid_vectors[c],
                                           buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::STRING:
                    bulkLoadStringColumn(buffer, pname, data_vectors[c], valid_vectors[c],
                                         buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::JSON:
                    bulkLoadJSONColumn(buffer, pname, data_vectors[c], valid_vectors[c],
                                       buf_idx, chunk_idx_, rows_to_copy);
                    break;
                case PropertyDataType::TIMESTAMP:
                    bulkLoadTimestampColumn(buffer, pname, data_vectors[c], valid_vectors[c],
                                            buf_idx, chunk_idx_, rows_to_copy);
                    break;
                default:
                    logerr << "unknown property type " << Property::asString(dtype);
                    traced_assert(false);
            }
        }

        buf_idx    += rows_to_copy;
        chunk_idx_ += rows_to_copy;

        //fetch next chunk?
        if (chunk_idx_ >= chunk_num_rows_)
        {
            nextChunk(data_vectors, valid_vectors, np);
        }
    }

    traced_assert(chunk_idx_ <= chunk_num_rows_);
    traced_assert(buf_idx <= max_entries);

    bool has_more = hasChunk();

    return ResultT<bool>::succeeded(has_more);
}
