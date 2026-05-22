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
#include "dbtableinfo.h"

#include "result.h"

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>

class DBContentManager;
class DBInterface;
class DBConnection;
class DBConnectionWrapper;

/**
 * A DBInstance is governing a db file/project. 
 * It is separated from a DBConnection, as in many db frameworks
 * there can be mutliple connections operating on a single db instance.
 * The instance can spawn new connections on demand.
 */
class DBInstance
{
public:
    typedef std::unique_ptr<DBConnectionWrapper> ConnectionWrapperPtr;
    typedef std::unique_ptr<DBConnection>        ConnectionPtr;

    DBInstance(DBInterface* interface); 
    virtual ~DBInstance();

    bool beVerbose() const; // to log or not to log that is the question
    std::string status() const;

    bool dbOpen() const;
    bool dbConnected() const;
    bool dbReady() const;
    bool dbInMem() const { return db_in_mem_; }

    Result open(const std::string& file_name);
    Result openInMemory();
    void close();
    Result reconnect(bool cleanup_db = false, Result* cleanup_result = nullptr);
    Result exportToFile(const std::string& file_name);
    Result cleanupDB();
    Result cleanupDBInMem();

    const std::string& dbFilename() const { return db_filename_; }

    DBConnection& defaultConnection();
    DBConnectionWrapper concurrentConnection(size_t tIdx);
    ConnectionWrapperPtr newCustomConnection();
    void destroyCustomConnections();
    void destroyConcurrentConnections();

    size_t numCustomConnections() const;
    size_t numConcurrentConnections() const;

    db::SQLConfig sqlConfiguration(bool verbose = false) const;

    Result updateTableInfo();
    void printTableInfo() const;
    const std::map<std::string, DBTableInfo>& tableInfo() const { return table_info_; }

    std::string dbInfo();

    DBContentManager& dbContentManager();

    static const std::string InMemFilename;

protected:
    DBInterface& interface() { return interface_; }
    const DBInterface& interface() const { return interface_; }

    /// implements opening of a db file
    virtual Result open_impl(const std::string& file_name) = 0;

    /// implements closing of a db file
    virtual void close_impl() = 0;

    /// implements connection creation
    virtual ResultT<DBConnection*> createConnection_impl(bool verbose) = 0;

    /// implements db file cleanup (if supported)
    virtual Result cleanupDB_impl(const std::string& db_fn);
    virtual Result cleanupDB_impl();

    /// implements db file cleanup in mem (if supported)
    virtual Result cleanupDBInMem_impl();

    /// implements db export to a specified file
    virtual Result exportToFile_impl(const std::string& filename) = 0;

    /// db backend specific sql settings
    virtual db::SQLConfig sqlConfiguration_impl() const = 0;

    /// sql pragmas used for initial db configuration
    virtual std::vector<db::SQLPragma> sqlPragmas() const { return {}; }

private:
    Result openInternal(const std::string& file_name);
    Result cleanupDB(const std::string& db_fn);
    Result configureDBUsingPragmas();

    ResultT<DBConnection*> createConnection(bool verbose);
    DBConnectionWrapper createConnectionWrapper(DBConnection* conn, 
                                                bool verbose, 
                                                const std::function<void(DBConnection*)>& destroyer);
    ConnectionWrapperPtr createConnectionWrapperPtr(DBConnection* conn, 
                                                    bool verbose, 
                                                    const std::function<void(DBConnection*)>& destroyer);
    void destroyCustomConnection(DBConnection* conn);

    DBInterface& interface_;
    std::string  db_filename_;
    bool         db_open_      = false;
    bool         db_connected_ = false;
    bool         db_in_mem_    = false;

    std::vector<std::unique_ptr<DBConnection>>   custom_connections_;
    std::map<int, std::unique_ptr<DBConnection>> concurrent_connections_;
    std::unique_ptr<DBConnection>                default_connection_;
    boost::mutex                                 connection_mutex_;

    std::map<std::string, DBTableInfo>           table_info_;
};
