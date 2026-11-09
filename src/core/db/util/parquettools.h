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

//#define WITH_PARQUET_TOOLS

#ifdef WITH_PARQUET_TOOLS

#include "property.h"

#include <memory>

#include <boost/optional.hpp>

namespace arrow
{
    class DataType;
    class Array;
    class Schema;

    namespace io
    {
        class FileOutputStream;
    }
}

namespace parquet
{
    class WriterProperties;

    namespace arrow
    {
        class FileWriter;
    }
}

class Buffer;
class DBInterface;
class DBContent;

namespace parquettools
{

/**
 */
class ParquetWriter
{
public:
    ParquetWriter(DBInterface& db_interface,
                  bool append_mode);
    virtual ~ParquetWriter();

    std::string currentParquetPath() const;
    std::string parquetPartitionPath(const std::string& table_name) const;

    bool writeBufferToParquet(const DBContent& dbcontent,
                              Buffer& buffer);
    void commit();

    static std::shared_ptr<arrow::DataType> arrowDataType(PropertyDataType dtype);
    static std::shared_ptr<arrow::io::FileOutputStream> createFile(const std::string& fn,
                                                                   bool append = false);
    static std::unique_ptr<parquet::arrow::FileWriter> createWriter(const std::string& fn,
                                                                    const std::shared_ptr<arrow::Schema>& schema,
                                                                    const std::shared_ptr<parquet::WriterProperties>& writer_props,
                                                                    bool append = false);
    static bool createArrowArray(std::shared_ptr<arrow::Array>& array,
                                 PropertyDataType dtype,
                                 Buffer& buffer,
                                 const boost::optional<size_t>& property_index);
private:
    struct TableWriter
    {
        std::shared_ptr<arrow::Schema>              schema;
        std::shared_ptr<parquet::WriterProperties>  writer_props;
        std::unique_ptr<parquet::arrow::FileWriter> writer;
    };

    struct Schema
    {
        std::shared_ptr<arrow::Schema> schema;
        std::map<std::string, size_t>  property_map;
    };

    std::string tableFile(const std::string& table_name, 
                          bool unique) const;

    TableWriter& getOrCreateTableWriter(const std::string& fn,
                                        const std::shared_ptr<arrow::Schema>& schema,
                                        bool reset);
    void destroyTableWriter(const std::string& fn);

    static std::shared_ptr<parquet::WriterProperties> createDefaultWriterProps();
    static Schema createSchema(const DBContent& dbcontent);
 
    DBInterface& db_interface_;
    bool append_mode_ = false;

    std::map<std::string, TableWriter> table_writers_;
};

} // namespace parquettools

#endif
