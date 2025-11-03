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

#include "dbdefs.h"
#include "result.h"

#include <memory>
#include <string>
#include <map>
#include <vector>

#include <boost/optional.hpp>

class Buffer;
class DBInstance;
class DBCommand;
class DBCommandList;
class DBResult;
class DBTableInfo;
class DBTableColumnInfo;
class PropertyList;
class DBScopedPrepare;
class DBScopedReader;
class SQLGenerator;

/**
 */
class DBConnection
{
public:
    typedef std::map<std::string, DBTableInfo> TableInfo;

    DBConnection(DBInstance* instance, bool verbose);
    virtual ~DBConnection();

    db::SQLConfig sqlConfiguration(bool verbose = false) const;

    std::string status() const;
    bool connected() const { return connected_; }

    Result connect();
    void disconnect();

    Result execute(const std::string& sql);
    std::shared_ptr<DBResult> execute(const std::string& sql, bool fetch_buffer);
    std::shared_ptr<DBResult> execute(const DBCommand& command);
    std::shared_ptr<DBResult> execute(const DBCommandList& command_list);

    virtual Result executePragma(const db::SQLPragma& pragma); 

    Result createTable(const std::string& table_name, 
                       const std::vector<DBTableColumnInfo>& column_infos,
                       const std::vector<db::Index>& indices = std::vector<db::Index>(),
                       bool table_must_not_exist = true);
    Result deleteTable(const std::string& table_name);
    Result deleteTableContents(const std::string& table_name);

    ResultT<TableInfo> createTableInfo();
    
    Result insertBuffer(const std::string& table_name, 
                        const std::shared_ptr<Buffer>& buffer,
                        const boost::optional<size_t>& idx_from = boost::optional<size_t>(), 
                        const boost::optional<size_t>& idx_to = boost::optional<size_t>(),
                        PropertyList* table_properties = nullptr);
    Result updateBuffer(const std::string& table_name, 
                        const std::shared_ptr<Buffer>& buffer,
                        const std::string& key_column,
                        const boost::optional<size_t>& idx_from = boost::optional<size_t>(), 
                        const boost::optional<size_t>& idx_to = boost::optional<size_t>());
    Result startRead(const std::shared_ptr<DBCommand>& select_cmd, 
                     size_t offset, 
                     size_t chunk_size);
    std::shared_ptr<DBResult> readChunk();
    void stopRead();

    SQLGenerator sqlGenerator() const;

    void startPerformanceMetrics() const;
    db::PerformanceMetrics stopPerformanceMetrics() const;
    bool hasActivePerformanceMetrics() const;

    virtual std::shared_ptr<DBScopedPrepare> prepareStatement(const std::string& statement, 
                                                              bool begin_transaction) = 0;
    virtual std::shared_ptr<DBScopedReader> createReader(const std::shared_ptr<DBCommand>& select_cmd, 
                                                         size_t offset, 
                                                         size_t chunk_size) = 0;
    virtual std::string dbInfo() { return ""; }

    const DBInstance* instance() const { return instance_; }
    
protected:
    friend class DBTemporaryTable;

    virtual Result connect_impl() = 0;
    virtual void disconnect_impl() = 0;

    virtual Result executeSQL_impl(const std::string& sql, DBResult* result, bool fetch_result_buffer) = 0; 
    virtual bool executeCmd_impl(const std::string& command, const PropertyList* properties, DBResult* result) = 0;

    virtual Result insertBuffer_impl(const std::string& table_name, 
                                     const std::shared_ptr<Buffer>& buffer,
                                     const boost::optional<size_t>& idx_from, 
                                     const boost::optional<size_t>& idx_to,
                                     PropertyList* table_properties);
    virtual Result updateBuffer_impl(const std::string& table_name, 
                                     const std::shared_ptr<Buffer>& buffer,
                                     const std::string& key_column,
                                     const boost::optional<size_t>& idx_from, 
                                     const boost::optional<size_t>& idx_to);

    virtual ResultT<DBTableInfo> getColumnList_impl(const std::string& table);
    virtual ResultT<std::vector<std::string>> getTableList_impl() = 0;

    Result createTableInternal(const std::string& table_name, 
                               const std::vector<DBTableColumnInfo>& column_infos,
                               const std::vector<db::Index>& indices,
                               bool verbose = false);

    ResultT<std::vector<std::string>> getTableList();

    DBInstance* instance() { return instance_; }
    
private:
    ResultT<DBTableInfo> getColumnList(const std::string& table);

    DBInstance* instance_;
    bool        connected_ = false;

    std::shared_ptr<DBScopedReader> active_reader_;

    mutable boost::optional<db::PerformanceMetrics> perf_metrics_;

    bool verbose_ = false;
};

/**
 */
class DBConnectionWrapper
{
public:
    typedef std::function<void(DBConnection*)> Destroyer;

    DBConnectionWrapper() {}

    DBConnectionWrapper(DBInstance* instance,
                        DBConnection* connection,
                        const Destroyer& destroyer)
    :   instance_  (instance  )
    ,   connection_(connection)
    ,   destroyer_ (destroyer )
    {
        traced_assert(instance_  );
        traced_assert(connection_);
    }

    DBConnectionWrapper(const std::string& error)
    :   error_(error) {}

    virtual ~DBConnectionWrapper()
    {
        if (connection_ && destroyer_)
            destroyer_(connection_);
    }

    bool isEmpty() const { return instance_ == nullptr; }
    bool hasError() const { return error_.has_value(); }
    const std::string& error() { return error_.value(); }

    DBConnection& connection()
    { 
        traced_assert(!hasError());
        traced_assert(!isEmpty() );
        traced_assert(connection_);

        return *connection_; 
    }

    void detach()
    {
        destroyer_ = Destroyer();
    }

private:
    friend class DBInstance;

    DBInstance*   instance_   = nullptr;
    DBConnection* connection_ = nullptr;
    Destroyer     destroyer_;

    boost::optional<std::string> error_;
};
