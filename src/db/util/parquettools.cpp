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

#include "parquettools.h"

#ifdef WITH_PARQUET_TOOLS

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/filesystem/api.h>
#include <arrow/type.h>

#include <parquet/arrow/writer.h>
#include <parquet/arrow/schema.h>

#include "buffer.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "propertylist.h"

#include "stringconv.h"
#include "files.h"
#include "timeconv.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/filesystem.hpp"

namespace parquettools
{

/**
 */
ParquetWriter::ParquetWriter(DBInterface& db_interface,
                             bool append_mode) 
:   db_interface_(db_interface)
,   append_mode_ (append_mode )
{
}

/**
 */
ParquetWriter::~ParquetWriter()
{
}

/**
 */
void ParquetWriter::commit()
{
    //finish writing
    for (auto& w : table_writers_)
        w.second.writer->Close();

    table_writers_.clear();
}

/**
 */
std::shared_ptr<::arrow::DataType> ParquetWriter::arrowDataType(PropertyDataType dtype)
{
    if (dtype == PropertyDataType::BOOL)
        return ::arrow::boolean();
    else if (dtype == PropertyDataType::CHAR)
        return ::arrow::int8();
    else if (dtype == PropertyDataType::UCHAR)
        return ::arrow::uint8();
    else if (dtype == PropertyDataType::INT)
        return ::arrow::int32();
    else if (dtype == PropertyDataType::UINT)
        return ::arrow::uint32();
    else if (dtype == PropertyDataType::LONGINT)
        return ::arrow::int64();
    else if (dtype == PropertyDataType::ULONGINT)
        return ::arrow::uint64();
    else if (dtype == PropertyDataType::FLOAT)
        return ::arrow::float32();
    else if (dtype == PropertyDataType::DOUBLE)
        return ::arrow::float64();
    else if (dtype == PropertyDataType::STRING)
        return ::arrow::utf8();
    else if (dtype == PropertyDataType::JSON)
        return ::arrow::utf8();
    else if (dtype == PropertyDataType::TIMESTAMP)
        return ::arrow::timestamp(::arrow::TimeUnit::MILLI);
    
    return ::arrow::utf8();
}

/**
 */
bool ParquetWriter::createArrowArray(std::shared_ptr<arrow::Array>& array,
                                     PropertyDataType dtype,
                                     Buffer& buffer,
                                     const boost::optional<size_t>& property_index)
{
    const Property* property = property_index.has_value() ? &buffer.properties().properties().at(property_index.value()) : nullptr;
    auto property_dtype = property ? property->dataType() : dtype;

    traced_assert(property_dtype == dtype);

    size_t n = buffer.size();

    try
    {
        if (dtype == PropertyDataType::BOOL)
        {
            ::arrow::BooleanBuilder builder;

            if (property)
            {
                const auto& values = buffer.get<bool>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::CHAR)
        {
            ::arrow::Int8Builder builder;

            if (property)
            {
                const auto& values = buffer.get<char>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::UCHAR)
        {
            ::arrow::UInt8Builder builder;

            if (property)
            {
                const auto& values = buffer.get<unsigned char>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::INT)
        {
            ::arrow::Int32Builder builder;

            if (property)
            {
                const auto& values = buffer.get<int>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::UINT)
        {
            ::arrow::UInt32Builder builder;

            if (property)
            {
                const auto& values = buffer.get<unsigned int>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::LONGINT)
        {
            ::arrow::Int64Builder builder;

            if (property)
            {
                const auto& values = buffer.get<long>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::ULONGINT)
        {
            ::arrow::UInt64Builder builder;

            if (property)
            {
                const auto& values = buffer.get<unsigned long>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::FLOAT)
        {
            ::arrow::FloatBuilder builder;

            if (property)
            {
                const auto& values = buffer.get<float>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::DOUBLE)
        {
            ::arrow::DoubleBuilder builder;

            if (property)
            {
                const auto& values = buffer.get<double>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::STRING)
        {
            ::arrow::StringBuilder builder;

            if (property)
            {
                const auto& values = buffer.get<std::string>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(values.get(i).data(), values.get(i).size());
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::JSON)
        {
            ::arrow::StringBuilder builder;

            if (property)
            {
                const auto& values = buffer.get<nlohmann::json>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                    {
                        builder.AppendNull();
                    }
                    else
                    {
                        auto str = values.get(i).dump();
                        builder.Append(str.data(), str.size());
                    }
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else if (dtype == PropertyDataType::TIMESTAMP)
        {
            ::arrow::TimestampBuilder builder(::arrow::timestamp(::arrow::TimeUnit::MILLI), ::arrow::default_memory_pool());

            if (property)
            {
                const auto& values = buffer.get<boost::posix_time::ptime>(property->name());
                for (size_t i = 0; i < n; ++i)
                {
                    if (values.isNull(i))
                        builder.AppendNull();
                    else
                        builder.Append(Utils::Time::toLong(values.get(i)));
                }
            }
            else
            {
                builder.AppendNulls(n);
            }

            builder.Finish(&array);
        }
        else
        {
            logerr << "writing buffer property failed: unknown property";
            return false;
        }
    }
    catch (const std::exception& ex)
    {
        logerr << "writing buffer property failed: " << ex.what();
        return false;
    }
    catch(...)
    {
        logerr << "writing buffer property failed: unknown error";
        return false;
    }

    return true;
}

/**
 */
std::string ParquetWriter::currentParquetPath() const
{
    if (!db_interface_.dbOpen() || !Utils::Files::fileExists(db_interface_.dbFilename()))
        return "";

    std::string db_path      = Utils::Files::getDirectoryFromPath(db_interface_.dbFilename());
    std::string db_basename  = boost::filesystem::path(db_interface_.dbFilename()).stem().string();
    std::string parquet_path = db_path + "/" + db_basename;

    return parquet_path;
}

/**
 */
std::string ParquetWriter::parquetPartitionPath(const std::string& table_name) const
{
    if (table_name.empty())
        return "";

    auto pbase = currentParquetPath();
    if (pbase.empty())
        return "";

    return pbase + "/" + table_name;
}

/**
 */
std::string ParquetWriter::tableFile(const std::string& table_name, 
                                     bool unique) const
{
    auto path = parquetPartitionPath(table_name);
    if (path.empty())
        return "";

    std::string table_fn_base = path + "/" + "table_"  + table_name;
    if (!unique)
        return table_fn_base + ".parquet";

    std::string table_fn;
    size_t cnt = 0;
    do
    {
        table_fn = table_fn_base + "_part" + std::to_string(cnt++) + ".parquet";
    } 
    while (Utils::Files::fileExists(table_fn));

    return table_fn;
}

/**
 */
ParquetWriter::Schema ParquetWriter::createSchema(const DBContent& dbcontent)
{
    Schema schema;

    std::vector<std::shared_ptr<arrow::Field>> fields;
    for (const auto& v : dbcontent.variables())
    {
        schema.property_map[ v.second->dbColumnName() ] = fields.size();

        auto dtype = arrowDataType(v.second->dataType());
        fields.push_back(::arrow::field(v.second->dbColumnName(), dtype));
    }

    schema.schema = std::make_shared<::arrow::Schema>(fields);

    return schema;
}

/**
 */
bool ParquetWriter::writeBufferToParquet(const DBContent& dbcontent,
                                         Buffer& buffer)
{
    if (!db_interface_.dbOpen())
        return false;

    //create dir for parquet files
    auto parquet_path = parquetPartitionPath(dbcontent.name());

    if (!Utils::Files::directoryExists(parquet_path) &&
        !Utils::Files::createMissingDirectories(parquet_path))
    {
        logerr << "could not create parquet path at " << parquet_path;
        return false;
    }

    //create schema
    auto schema = createSchema(dbcontent);

    //loginf << "Adding buffer for " << dbcontent.name() << ": " << dbcontent.variables().size() << " variable(s), " << schema.schema->num_fields() << " field(s)";

    //get table filename
    auto table_fn = tableFile(dbcontent.name(), !append_mode_);

    //create writer
    auto& writer = getOrCreateTableWriter(table_fn, schema.schema, false);

    std::vector<std::shared_ptr<::arrow::Array>> arrays(schema.schema->num_fields());

    const auto& properties = buffer.properties().properties();
    std::map<std::string, size_t> available_props;
    for (size_t i = 0; i < properties.size(); i++)
        available_props[ properties[ i ].name() ] = i;

    for (const auto& v : dbcontent.variables())
    {
        size_t array_idx = schema.property_map.at(v.second->dbColumnName());

        boost::optional<size_t> property_idx;
        auto it = available_props.find(v.second->dbColumnName());
        if (it != available_props.end())
            property_idx = it->second;

        if (!createArrowArray(arrays[ array_idx ], v.second->dataType(), buffer, property_idx))
            return false;
    }

    auto table = ::arrow::Table::Make(writer.schema, arrays);

    auto result = writer.writer->WriteTable(*table, 64 * 1024 * 1024);
    if (!result.ok())
    {
        logerr << "could not write table " << dbcontent.name() << ": " << result.message();
        return false;
    }

    if (!append_mode_)
        commit();

    return true;
}

/**
 */
std::shared_ptr<::arrow::io::FileOutputStream> ParquetWriter::createFile(const std::string& fn,
                                                                         bool append)
{
    auto file_result = ::arrow::io::FileOutputStream::Open(fn, append);
    if (!file_result.ok())
        return std::shared_ptr<::arrow::io::FileOutputStream>();

    return file_result.ValueUnsafe();
}

/**
 */
std::unique_ptr<parquet::arrow::FileWriter> ParquetWriter::createWriter(const std::string& fn,
                                                                        const std::shared_ptr<arrow::Schema>& schema,
                                                                        const std::shared_ptr<parquet::WriterProperties>& writer_props,
                                                                        bool append)
{
    if (fn.empty() || !schema || !writer_props)
        return std::unique_ptr<parquet::arrow::FileWriter>();

    std::unique_ptr<parquet::arrow::FileWriter> file_writer;
    
    auto file = ParquetWriter::createFile(fn, append);
    auto result = parquet::arrow::FileWriter::Open(*schema, arrow::default_memory_pool(), file, writer_props);

    if (!result.ok())
        return std::unique_ptr<parquet::arrow::FileWriter>();

    return std::move(result.ValueUnsafe());
}

/**
 */
std::shared_ptr<parquet::WriterProperties> ParquetWriter::createDefaultWriterProps()
{
    parquet::WriterProperties::Builder props_builder;
    std::shared_ptr<parquet::WriterProperties> writer_props = 
        props_builder.compression(::arrow::Compression::ZSTD)->encoding(parquet::Encoding::RLE)->build();

    return writer_props;
}

/**
 */
ParquetWriter::TableWriter& ParquetWriter::getOrCreateTableWriter(const std::string& fn,
                                                                  const std::shared_ptr<arrow::Schema>& schema,
                                                                  bool reset)
{
    if (reset)
        destroyTableWriter(fn);

    auto it = table_writers_.find(fn);
    if (it != table_writers_.end())
        return it->second;
    
    auto writer_props = createDefaultWriterProps();

    auto writer = createWriter(fn, schema, writer_props, false);
    traced_assert(writer);

    auto& tw = table_writers_[ fn ];
    tw.schema       = schema;
    tw.writer_props = writer_props;
    tw.writer       = std::move(writer);

    return tw;
}

/**
 */
void ParquetWriter::destroyTableWriter(const std::string& fn)
{
    auto it = table_writers_.find(fn);
    if (it == table_writers_.end())
        return;

    it->second.writer->Close();

    table_writers_.erase(fn);
}

} // namespace parquettools

#endif
