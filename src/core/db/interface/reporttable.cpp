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

#include "reporttable.h"
#include "dbinterface.h"
#include "buffer.h"
#include "logger.h"
#include "traced_assert.h"

#include "property_templates.h"

#include "json.hpp"

#include <regex>

const std::string ReportTableDefinition::TableNamePrefix     = "result_";
const std::string ReportTableDefinition::KeyColumnName       = "rec_num";
const std::string ReportTableDefinition::UTNColumnName       = "utn";
const std::string ReportTableDefinition::TimestampColumnName = "timestamp";

namespace
{
    const std::string FieldName           = "name";
    const std::string FieldDataType       = "data_type";
    const std::string FieldDisplayName    = "display_name";
    const std::string FieldDescription    = "description";
    const std::string FieldDimension      = "dimension";
    const std::string FieldUnit           = "unit";
    const std::string FieldRepresentation = "representation";
    const std::string FieldKey            = "key";
    const std::string FieldKind           = "kind";
    const std::string FieldHosts          = "host_dbcontents";
    const std::string FieldColumns        = "columns";
    const std::string FieldNumRows        = "num_rows";

    // words that break an unquoted SQL position, rejected as column names
    const std::set<std::string> SQLKeywords =
    {
        "all", "alter", "and", "any", "as", "asc", "between", "by", "case", "cast", "check", "column",
        "create", "cross", "current", "default", "delete", "desc", "distinct", "drop", "else", "end",
        "except", "exists", "false", "filter", "from", "full", "group", "having", "in", "index", "inner",
        "insert", "intersect", "into", "is", "join", "key", "left", "like", "limit", "natural", "not",
        "null", "offset", "on", "or", "order", "outer", "over", "partition", "primary", "references",
        "right", "row", "rows", "select", "session", "set", "some", "table", "then", "true", "union",
        "update", "user", "using", "value", "values", "when", "where", "window", "with"
    };
}

/************************************************************************************************
 * ReportTableColumn
 ************************************************************************************************/

/**
 */
std::string ReportTableColumn::viewRepresentation() const
{
    if (!representation.empty())
        return representation;

    // lengths, speeds, times and ratios read well with two decimals, an angle column keeps
    // the full value since the coordinate columns need it
    bool floating = data_type == PropertyDataType::DOUBLE || data_type == PropertyDataType::FLOAT;

    if (floating && dimension != "Angle")
        return "FLOAT_PREC2";

    return "";
}

ReportTableColumn::ReportTableColumn(const std::string& name,
                                     PropertyDataType data_type,
                                     const std::string& display_name,
                                     const std::string& description,
                                     const std::string& dimension,
                                     const std::string& unit,
                                     const std::string& representation)
:   name          (name)
,   data_type     (data_type)
,   display_name  (display_name)
,   description   (description)
,   dimension     (dimension)
,   unit          (unit)
,   representation(representation)
{
}

/**
 */
nlohmann::json ReportTableColumn::toJSON() const
{
    nlohmann::json j;

    j[ FieldName           ] = name;
    j[ FieldDataType       ] = Property::asString(data_type);
    j[ FieldDisplayName    ] = display_name;
    j[ FieldDescription    ] = description;
    j[ FieldDimension      ] = dimension;
    j[ FieldUnit           ] = unit;
    j[ FieldRepresentation ] = representation;

    return j;
}

/**
 */
bool ReportTableColumn::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object() ||
        !j.contains(FieldName) ||
        !j.contains(FieldDataType) ||
        !j.contains(FieldDisplayName))
        return false;

    name         = j[ FieldName ];
    data_type    = Property::asDataType(j[ FieldDataType ]);
    display_name = j[ FieldDisplayName ];

    description    = j.value(FieldDescription   , std::string());
    dimension      = j.value(FieldDimension     , std::string());
    unit           = j.value(FieldUnit          , std::string());
    representation = j.value(FieldRepresentation, std::string());

    return true;
}

/************************************************************************************************
 * ReportTableDefinition
 ************************************************************************************************/

/**
 */
ReportTableDefinition ReportTableDefinition::record(const std::string& key,
                                                    const std::string& display_name,
                                                    const std::vector<std::string>& host_dbcontents)
{
    ReportTableDefinition def;
    def.key             = key;
    def.kind            = ReportTableKind::Record;
    def.display_name    = display_name;
    def.host_dbcontents = host_dbcontents;

    def.addColumn(KeyColumnName, PropertyDataType::ULONGINT, "Record Number",
                  "Record number of the target report the row describes");
    def.addColumn(UTNColumnName, PropertyDataType::UINT, "UTN",
                  "Unique target number, empty for unassociated reports");
    def.addColumn(TimestampColumnName, PropertyDataType::TIMESTAMP, "Timestamp",
                  "Timestamp of the target report the row describes");

    return def;
}

/**
 */
ReportTableDefinition ReportTableDefinition::cell(const std::string& key,
                                                  const std::string& display_name)
{
    ReportTableDefinition def;
    def.key          = key;
    def.kind         = ReportTableKind::Cell;
    def.display_name = display_name;

    return def;
}

/**
 */
ReportTableDefinition& ReportTableDefinition::addColumn(const ReportTableColumn& column)
{
    columns.push_back(column);
    return *this;
}

/**
 */
ReportTableDefinition& ReportTableDefinition::addColumn(const std::string& name,
                                                        PropertyDataType data_type,
                                                        const std::string& display_name,
                                                        const std::string& description,
                                                        const std::string& dimension,
                                                        const std::string& unit,
                                                        const std::string& representation)
{
    return addColumn(ReportTableColumn(name, data_type, display_name, description, dimension, unit, representation));
}

/**
 */
bool ReportTableDefinition::hasColumn(const std::string& name) const
{
    for (const auto& c : columns)
        if (c.name == name)
            return true;
    return false;
}

/**
 */
const ReportTableColumn& ReportTableDefinition::column(const std::string& name) const
{
    for (const auto& c : columns)
        if (c.name == name)
            return c;

    throw std::runtime_error("ReportTableDefinition: column: unknown column '" + name + "'");
}

/**
 */
std::string ReportTableDefinition::keyColumn() const
{
    return kind == ReportTableKind::Record ? KeyColumnName : std::string();
}

/**
 */
PropertyList ReportTableDefinition::propertyList() const
{
    PropertyList properties;
    for (const auto& c : columns)
        properties.addProperty(c.name, c.data_type);

    return properties;
}

/**
 */
std::string ReportTableDefinition::tableName(unsigned int result_id) const
{
    return tableName(result_id, key);
}

/**
 */
std::string ReportTableDefinition::validate() const
{
    if (!validIdentifier(key))
        return "invalid table key '" + key + "'";

    if (columns.empty())
        return "table '" + key + "' has no columns";

    std::set<std::string> names;

    for (const auto& c : columns)
    {
        if (!validIdentifier(c.name))
            return "table '" + key + "' has invalid column name '" + c.name + "'";
        if (isSQLKeyword(c.name))
            return "table '" + key + "' column name '" + c.name + "' is a SQL keyword";
        if (names.count(c.name))
            return "table '" + key + "' has duplicate column '" + c.name + "'";
        if (c.display_name.empty())
            return "table '" + key + "' column '" + c.name + "' has no display name";

        names.insert(c.name);
    }

    if (kind == ReportTableKind::Record)
    {
        if (columns.front().name != KeyColumnName || columns.front().data_type != PropertyDataType::ULONGINT)
            return "record table '" + key + "' does not start with the key column";
        if (host_dbcontents.empty())
            return "record table '" + key + "' has no host data content";
    }

    return "";
}

/**
 */
nlohmann::json ReportTableDefinition::toJSON() const
{
    nlohmann::json j;

    j[ FieldKey         ] = key;
    j[ FieldKind        ] = (int)kind;
    j[ FieldDisplayName ] = display_name;
    j[ FieldHosts       ] = host_dbcontents;

    nlohmann::json j_columns = nlohmann::json::array();
    for (const auto& c : columns)
        j_columns.push_back(c.toJSON());

    j[ FieldColumns ] = j_columns;

    return j;
}

/**
 */
bool ReportTableDefinition::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object() ||
        !j.contains(FieldKey) ||
        !j.contains(FieldKind) ||
        !j.contains(FieldDisplayName) ||
        !j.contains(FieldColumns) ||
        !j[ FieldColumns ].is_array())
        return false;

    key          = j[ FieldKey ];
    kind         = (ReportTableKind)j[ FieldKind ].get<int>();
    display_name = j[ FieldDisplayName ];

    host_dbcontents.clear();
    if (j.contains(FieldHosts) && j[ FieldHosts ].is_array())
        host_dbcontents = j[ FieldHosts ].get<std::vector<std::string>>();

    columns.clear();
    for (const auto& j_column : j[ FieldColumns ])
    {
        ReportTableColumn c;
        if (!c.fromJSON(j_column))
            return false;
        columns.push_back(c);
    }

    return true;
}

/**
 */
std::string ReportTableDefinition::tableName(unsigned int result_id, const std::string& key)
{
    return tableNamePrefix(result_id) + key;
}

/**
 */
std::string ReportTableDefinition::tableNamePrefix(unsigned int result_id)
{
    return TableNamePrefix + std::to_string(result_id) + "_";
}

/**
 */
bool ReportTableDefinition::isReportTableName(const std::string& table_name)
{
    static const std::regex re("^" + TableNamePrefix + "[0-9]+_[a-z][a-z0-9_]*$");
    return std::regex_match(table_name, re);
}

/**
 */
bool ReportTableDefinition::validIdentifier(const std::string& name)
{
    static const std::regex re("^[a-z][a-z0-9_]*$");
    return std::regex_match(name, re);
}

/**
 */
bool ReportTableDefinition::isSQLKeyword(const std::string& name)
{
    return SQLKeywords.count(name) > 0;
}

/************************************************************************************************
 * ReportTableInfo
 ************************************************************************************************/

/**
 */
nlohmann::json ReportTableInfo::toJSON() const
{
    nlohmann::json j = ReportTableDefinition::toJSON();
    j[ FieldNumRows ] = num_rows;

    return j;
}

/**
 */
bool ReportTableInfo::fromJSON(const nlohmann::json& j)
{
    if (!ReportTableDefinition::fromJSON(j))
        return false;

    num_rows = j.value(FieldNumRows, (size_t)0);

    return true;
}

/************************************************************************************************
 * ReportTableWriter
 ************************************************************************************************/

/**
 */
ReportTableWriter::ReportTableWriter(unsigned int result_id, DBInterface& db_interface)
:   result_id_   (result_id)
,   db_interface_(db_interface)
{
}

/**
 * Creates the table of the definition, replacing a table of the same name.
 */
unsigned int ReportTableWriter::define(const ReportTableDefinition& definition)
{
    auto problem = definition.validate();
    if (!problem.empty())
    {
        logerr << "result " << result_id_ << ": " << problem;
        throw std::runtime_error("ReportTableWriter: define: " + problem);
    }

    if (hasTable(definition.key))
    {
        logerr << "result " << result_id_ << ": table '" << definition.key << "' defined twice";
        throw std::runtime_error("ReportTableWriter: define: table '" + definition.key + "' defined twice");
    }

    auto table_name = definition.tableName(result_id_);

    loginf << "result " << result_id_ << " table '" << table_name << "' columns " << definition.columns.size();

    db_interface_.createReportTable(table_name, definition.propertyList(), definition.keyColumn());

    tables_.emplace_back(definition);

    return tables_.size() - 1;
}

/**
 */
void ReportTableWriter::append(unsigned int table_idx, const std::shared_ptr<Buffer>& buffer)
{
    traced_assert(table_idx < tables_.size());
    traced_assert(buffer);

    auto& info = tables_[ table_idx ];

    //buffer columns must match the definition
    const auto& expected = info.columns;
    const auto& props    = buffer->properties().properties();

    if (props.size() != expected.size())
    {
        logerr << "result " << result_id_ << " table '" << info.key << "': buffer has " << props.size()
               << " columns, definition " << expected.size();
        throw std::runtime_error("ReportTableWriter: append: column count mismatch in table '" + info.key + "'");
    }

    for (size_t i = 0; i < props.size(); ++i)
    {
        if (props[ i ].name() != expected[ i ].name || props[ i ].dataType() != expected[ i ].data_type)
        {
            logerr << "result " << result_id_ << " table '" << info.key << "': buffer column " << i
                   << " '" << props[ i ].name() << "' does not match definition column '" << expected[ i ].name << "'";
            throw std::runtime_error("ReportTableWriter: append: column mismatch in table '" + info.key + "'");
        }
    }

    if (buffer->size() == 0)
        return;

    db_interface_.insertBuffer(info.tableName(result_id_), buffer);

    info.num_rows += buffer->size();
}

/**
 * Drops all tables defined so far.
 */
void ReportTableWriter::discard()
{
    loginf << "result " << result_id_ << " dropping " << tables_.size() << " table(s)";

    for (const auto& info : tables_)
        db_interface_.removeReportTable(info.tableName(result_id_));

    tables_.clear();
}

/**
 */
bool ReportTableWriter::hasTable(const std::string& key) const
{
    for (const auto& info : tables_)
        if (info.key == key)
            return true;
    return false;
}

/**
 */
unsigned int ReportTableWriter::tableIndex(const std::string& key) const
{
    for (size_t i = 0; i < tables_.size(); ++i)
        if (tables_[ i ].key == key)
            return i;

    throw std::runtime_error("ReportTableWriter: tableIndex: unknown table '" + key + "'");
}

/**
 */
const ReportTableInfo& ReportTableWriter::table(unsigned int table_idx) const
{
    traced_assert(table_idx < tables_.size());
    return tables_[ table_idx ];
}

/************************************************************************************************
 * ReportTableRows
 ************************************************************************************************/

/**
 */
ReportTableRows::ReportTableRows(ReportTableWriter& writer,
                                 unsigned int table_idx,
                                 size_t chunk_size)
:   writer_    (writer)
,   table_idx_ (table_idx)
,   chunk_size_(chunk_size)
{
    traced_assert(chunk_size_ > 0);

    keyed_ = writer_.table(table_idx_).kind == ReportTableKind::Record;

    newBuffer();
}

/**
 */
ReportTableRows::~ReportTableRows()
{
    if (row_ > 0)
        logwrn << "table '" << table().key << "': " << row_ << " unflushed row(s) discarded";
}

/**
 */
void ReportTableRows::setRecordNumber(unsigned long rec_num)
{
    set<unsigned long>(ReportTableDefinition::KeyColumnName, rec_num);
}

/**
 */
void ReportTableRows::setUTN(unsigned int utn)
{
    set<unsigned int>(ReportTableDefinition::UTNColumnName, utn);
}

/**
 */
void ReportTableRows::setTimestamp(const boost::posix_time::ptime& timestamp)
{
    set<boost::posix_time::ptime>(ReportTableDefinition::TimestampColumnName, timestamp);
}

/**
 */
bool ReportTableRows::hasRecordNumber(unsigned long rec_num) const
{
    return keys_.count(rec_num) > 0;
}

/**
 * Closes the current row, writes the chunk when full. The record number is the primary key of a
 * record table, a duplicate would fail the insert of the whole chunk, so such a row is dropped.
 */
bool ReportTableRows::nextRow()
{
    if (keyed_)
    {
        const std::string& key_column = ReportTableDefinition::KeyColumnName;

        if (!buffer_->has<unsigned long>(key_column) || buffer_->get<unsigned long>(key_column).isNull(row_))
        {
            ++num_dropped_;
            clearRow();
            return false;
        }

        auto rec_num = buffer_->get<unsigned long>(key_column).get(row_);

        if (!keys_.insert(rec_num).second)
        {
            ++num_dropped_;
            clearRow();
            return false;
        }
    }

    ++row_;
    ++num_rows_;

    if (row_ >= chunk_size_)
        flush();

    return true;
}

/**
 * Resets the current row, so the next one starts from null values.
 */
void ReportTableRows::clearRow()
{
    #define SetNullFunc(PDType, DType, Suffix)                     \
        if (buffer_->has<DType>(p.name()))                          \
            buffer_->get<DType>(p.name()).setNull(row_);

    #define SetNullNotFound ;

    for (const auto& p : buffer_->properties().properties())
    {
        auto dtype = p.dataType();
        SwitchPropertyDataType(dtype, SetNullFunc, SetNullNotFound);
    }

    #undef SetNullFunc
    #undef SetNullNotFound
}

/**
 * Writes the pending rows.
 */
void ReportTableRows::flush()
{
    if (num_dropped_ > 0)
        logwrn << "table '" << table().key << "': " << num_dropped_
               << " row(s) dropped, record number missing or already written";

    if (row_ == 0)
        return;

    writer_.append(table_idx_, buffer_);

    newBuffer();
}

/**
 */
void ReportTableRows::newBuffer()
{
    buffer_.reset(new Buffer(table().propertyList()));
    row_ = 0;
}

/**
 * Lower case identifier from free text: letters and digits kept, everything else one underscore.
 */
std::string ReportTableDefinition::identifierFrom(const std::string& text)
{
    std::string id;

    for (char c : text)
    {
        const unsigned char uc = static_cast<unsigned char>(c);

        if (std::isalnum(uc))
            id += static_cast<char>(std::tolower(uc));
        else if (!id.empty() && id.back() != '_')
            id += '_';
    }

    while (!id.empty() && id.back() == '_')
        id.pop_back();

    if (id.empty() || !std::isalpha(static_cast<unsigned char>(id.front())))
        id = "t_" + id;

    return id;
}
