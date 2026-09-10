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

#include "property.h"
#include "propertylist.h"
#include "buffer.h"

#include "json_fwd.hpp"

#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/date_time/posix_time/ptime.hpp>

class DBInterface;

/**
 * Kind of a Report Table.
 * Record: rows reference target reports through the record number key, joinable onto the host data content.
 * Cell:   rows are grid cells, persistence only.
 */
enum class ReportTableKind
{
    Record = 0,
    Cell
};

/**
 * One column of a Report Table, with the display information the DuckDB schema does not carry.
 */
struct ReportTableColumn
{
    ReportTableColumn() = default;
    ReportTableColumn(const std::string& name,
                      PropertyDataType data_type,
                      const std::string& display_name,
                      const std::string& description = "",
                      const std::string& dimension = "",
                      const std::string& unit = "",
                      const std::string& representation = "");

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    std::string      name;                                   // db column name
    PropertyDataType data_type = PropertyDataType::DOUBLE;
    std::string      display_name;                           // shown in the variable selection
    std::string      description;
    std::string      dimension;                              // e.g. "Length"
    std::string      unit;                                   // e.g. "m"
    std::string      representation;                         // dbContent::Variable representation string, empty for standard
};

/**
 * Schema of a Report Table, defined by the producer at run time.
 * Record tables start with the key column and the common columns, see record().
 */
struct ReportTableDefinition
{
    ReportTableDefinition() = default;
    virtual ~ReportTableDefinition() = default;

    /// definition of a record table with the key column and the common columns utn and timestamp
    static ReportTableDefinition record(const std::string& key,
                                        const std::string& display_name,
                                        const std::vector<std::string>& host_dbcontents);
    /// definition of a cell table without a key column
    static ReportTableDefinition cell(const std::string& key,
                                      const std::string& display_name);

    ReportTableDefinition& addColumn(const ReportTableColumn& column);
    ReportTableDefinition& addColumn(const std::string& name,
                                     PropertyDataType data_type,
                                     const std::string& display_name,
                                     const std::string& description = "",
                                     const std::string& dimension = "",
                                     const std::string& unit = "",
                                     const std::string& representation = "");

    bool hasColumn(const std::string& name) const;
    const ReportTableColumn& column(const std::string& name) const;

    /// the key column name, empty for tables without a key
    std::string keyColumn() const;
    PropertyList propertyList() const;
    std::string tableName(unsigned int result_id) const;

    /// checks key, columns and hosts, returns the first problem or an empty string
    std::string validate() const;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    static std::string tableName(unsigned int result_id, const std::string& key);
    static std::string tableNamePrefix(unsigned int result_id);
    static bool isReportTableName(const std::string& table_name);
    static bool validIdentifier(const std::string& name);
    static bool isSQLKeyword(const std::string& name);
    static std::string identifierFrom(const std::string& text);

    static const std::string TableNamePrefix;
    static const std::string KeyColumnName;
    static const std::string UTNColumnName;
    static const std::string TimestampColumnName;

    std::string                    key;                        // lower case identifier, unique per report
    ReportTableKind                kind = ReportTableKind::Record;
    std::string                    display_name;               // group shown in the variable selection
    std::vector<std::string>       host_dbcontents;            // data contents the record numbers refer to
    std::vector<ReportTableColumn> columns;
};

/**
 * Catalog entry of a written Report Table, stored in the report header.
 */
struct ReportTableInfo : public ReportTableDefinition
{
    ReportTableInfo() = default;
    ReportTableInfo(const ReportTableDefinition& def) : ReportTableDefinition(def) {}

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& j);

    size_t num_rows = 0;
};

/**
 * Writes the Report Tables of one report during the run.
 * define() creates the table (replacing a table of the same name), append() inserts rows in chunks,
 * the catalog is taken over by the report at the end of the run, discard() drops everything on abort.
 */
class ReportTableWriter
{
public:
    ReportTableWriter(unsigned int result_id, DBInterface& db_interface);
    virtual ~ReportTableWriter() = default;

    unsigned int define(const ReportTableDefinition& definition);
    void append(unsigned int table_idx, const std::shared_ptr<Buffer>& buffer);
    void discard();

    bool hasTable(const std::string& key) const;
    unsigned int tableIndex(const std::string& key) const;
    const ReportTableInfo& table(unsigned int table_idx) const;
    const std::vector<ReportTableInfo>& tables() const { return tables_; }

    unsigned int resultID() const { return result_id_; }

private:
    unsigned int                 result_id_;
    DBInterface&                 db_interface_;
    std::vector<ReportTableInfo> tables_;
};

/**
 * Chunked row sink for one report table. Fill the columns of the current row with set(), call
 * nextRow(), and flush() at the end of the run. Columns not set stay null.
 */
class ReportTableRows
{
public:
    ReportTableRows(ReportTableWriter& writer,
                    unsigned int table_idx,
                    size_t chunk_size = 200000);
    virtual ~ReportTableRows();

    template <typename T>
    void set(const std::string& column, const T& value)
    {
        buffer_->get<T>(column).set(row_, value);
    }

    void setRecordNumber(unsigned long rec_num);
    void setUTN(unsigned int utn);
    void setTimestamp(const boost::posix_time::ptime& timestamp);

    /// true if a row with this record number was written already
    bool hasRecordNumber(unsigned long rec_num) const;

    /// closes the row, returns false if it was dropped because its key is taken
    bool nextRow();
    void flush();

    size_t numRows() const { return num_rows_; }
    size_t numDroppedRows() const { return num_dropped_; }
    unsigned int tableIndex() const { return table_idx_; }
    const ReportTableInfo& table() const { return writer_.table(table_idx_); }

private:
    void newBuffer();
    void clearRow();

    ReportTableWriter&      writer_;
    unsigned int            table_idx_;
    size_t                  chunk_size_;
    std::shared_ptr<Buffer> buffer_;
    size_t                  row_        = 0;
    size_t                  num_rows_   = 0;
    size_t                  num_dropped_ = 0;

    //the key of a record table is its primary key, a duplicate would fail the whole insert
    bool                              keyed_ = false;
    std::unordered_set<unsigned long> keys_;
};
