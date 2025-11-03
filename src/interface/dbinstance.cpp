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

#include "dbinstance.h"
#include "dbinterface.h"
#include "dbconnection.h"

#include "property.h"

#include "logger.h"
#include "compass.h"

#define PROTECT_CONNECTION

const std::string DBInstance::InMemFilename = "In-Memory";

/**
 */
DBInstance::DBInstance(DBInterface* interface)
:   interface_(*interface)
{
    loginf;
}

/**
 */
db::SQLConfig DBInstance::sqlConfiguration(bool verbose) const
{
    auto config = sqlConfiguration_impl();

    config.verbose = verbose;

    return config;
}

/**
 */
DBInstance::~DBInstance()
{
    logdbg;

    close();
}

/**
 * Returns the current instance status.
 */
std::string DBInstance::status() const
{
    if (!db_open_)
        return "Not connected";
    else if (!db_connected_)
        return "Not connected";

    return "Ready";
}

/**
 */
bool DBInstance::dbOpen() const
{
    return db_open_;
}

/**
 */
bool DBInstance::dbConnected() const
{
    return db_connected_;
}

/**
 */
bool DBInstance::dbReady() const
{
    return dbOpen() && dbConnected();
}

/**
 */
size_t DBInstance::numCustomConnections() const
{
    return custom_connections_.size();
}

/**
 */
size_t DBInstance::numConcurrentConnections() const
{
    return concurrent_connections_.size();
}

/**
 */
Result DBInstance::open(const std::string& file_name)
{
    traced_assert(!file_name.empty());
    return openInternal(file_name);
}

/**
 */
Result DBInstance::openInMemory()
{
    return openInternal("");
}

/**
 * Opens the given database file and creates a default connection. 
 * Will close the currently opened database and destroy all open connections.
 * If an empty filename is passed, an in-memory db is created.
 */
Result DBInstance::openInternal(const std::string& file_name)
{
    std::string fn     = file_name;
    bool        in_mem = file_name.empty(); //empty filename means in-mem

    if (in_mem)
        fn = InMemFilename;

    loginf << "'" << fn << "'";

    //close first
    if (dbOpen())
        close();

    //in-mem => check if supported
    if (in_mem && !sqlConfiguration().supports_in_mem)
        return Result::failed("Database backend does not support in-memory mode");

    auto open_result = open_impl(file_name);
    if (!open_result.ok())
        return open_result;

    db_open_     = true;
    db_filename_ = fn;
    db_in_mem_   = in_mem;

    //open default connection
    auto conn_result = createConnection(true);
    if (!conn_result.ok())
        return conn_result;

    default_connection_.reset(conn_result.result());
    traced_assert(default_connection_);

    db_connected_ = true;

    //update table info
    auto ti_result = updateTableInfo();
    if (!ti_result.ok())
        return ti_result;

    //configure db using pragma statements (if provided by derived)
    auto config_pragma_result = configureDBUsingPragmas();
    if (!config_pragma_result.ok())
        return config_pragma_result;

    loginf << "done";

    return Result::succeeded();
}

/**
 * Closes the instance and destroys all open connections.
 */
void DBInstance::close()
{
    logdbg << "db open " << db_open_;

    if (!dbOpen())
        return;

    logdbg << "closing";

    //destroy connections
    if (default_connection_)
    {
        default_connection_->disconnect();
        default_connection_.reset();
    }
    destroyCustomConnections();
    destroyConcurrentConnections();

    close_impl();

    db_open_      = false;
    db_connected_ = false;
    db_in_mem_    = false;
    db_filename_  = "";

    table_info_.clear();
}

/**
 */
Result DBInstance::reconnect(bool cleanup_db, Result* cleanup_result)
{
    traced_assert(dbReady());

    // if (cleanup_result)
    //     *cleanup_result = Result::succeeded();

    // return Result::succeeded();

    //no reconnection to in-mem db
    if (db_in_mem_)
    {
        logwrn << "trying to reconnect to in-memory db, skipping";
        return Result::succeeded();
    }

    auto fn = db_filename_;

    loginf << "closing db for reconnect...";

    //close db
    close();

    if (cleanup_db)
    {
        loginf << "cleaning db...";

        //cleanup closed db file (cleanup might need closing of db handles)
        auto res = cleanupDB(fn);
        
        if (cleanup_result)
            *cleanup_result = res;
    }

    loginf << "reopening db...";

    //re-open db
    auto open_res = open(fn);

    return open_res;
}

/**
 */
Result DBInstance::exportToFile(const std::string& file_name)
{
    traced_assert(dbReady());

    return exportToFile_impl(file_name);
}

/**
 */
Result DBInstance::cleanupDB(const std::string& db_fn)
{
    traced_assert(!dbOpen());
    traced_assert(!db_fn.empty());

    return cleanupDB_impl(db_fn);
}

/**
 */
Result DBInstance::cleanupDB_impl(const std::string& db_fn)
{
    //derive if some special action is needed
    return Result::succeeded();
}

/**
 */
DBConnection& DBInstance::defaultConnection()
{
    traced_assert(dbReady());
    traced_assert(default_connection_);

    return *default_connection_;
}

/**
 */
DBConnectionWrapper DBInstance::createConnectionWrapper(DBConnection* conn, 
                                                        bool verbose, 
                                                        const std::function<void(DBConnection*)>& destroyer)
{
    if (conn == nullptr)
    {
        //create new connection
        auto r = createConnection(verbose);

        if (!r.ok())
            return DBConnectionWrapper(r.error());

        conn = r.result();
    }

    traced_assert(conn);

    return DBConnectionWrapper(this, conn, destroyer);
}

/**
 */
DBInstance::ConnectionWrapperPtr DBInstance::createConnectionWrapperPtr(DBConnection* conn, 
                                                                        bool verbose, 
                                                                        const std::function<void(DBConnection*)>& destroyer)
{
    auto wrapper = createConnectionWrapper(conn, verbose, destroyer);
    return ConnectionWrapperPtr(new DBConnectionWrapper(wrapper));
}

/**
 */
DBConnectionWrapper DBInstance::concurrentConnection(size_t tIdx)
{
    traced_assert(dbReady());
    traced_assert(sqlConfiguration().supports_mt);

    DBConnectionWrapper wrapper; 

    {
        #ifdef PROTECT_CONNECTION
        boost::mutex::scoped_lock locker(connection_mutex_);
        #endif

        auto it = concurrent_connections_.find(tIdx);

        if (it == concurrent_connections_.end())
        {
            //create wrapper with new connection
            wrapper = createConnectionWrapper(nullptr, false, {});

            //no errors => insert
            if (!wrapper.hasError())
                concurrent_connections_[ tIdx ].reset(wrapper.connection_);
        }
        else
        {
            //wrap existing connection
            wrapper = createConnectionWrapper(it->second.get(), false, {});
        }
    }

    traced_assert(!wrapper.isEmpty());

    return wrapper;
}

/**
 */
DBInstance::ConnectionWrapperPtr DBInstance::newCustomConnection()
{
    traced_assert(dbReady());
    traced_assert(sqlConfiguration().supports_mt);

    auto d = [ this ] (DBConnection* conn)
    {
        traced_assert(conn);
        this->destroyCustomConnection(conn);
    };

    //create wrapper with new connection
    auto wrapper_ptr = createConnectionWrapperPtr(nullptr, false, d);
    traced_assert(wrapper_ptr);

    //errors?
    if (wrapper_ptr->hasError())
        return wrapper_ptr;

    {
        #ifdef PROTECT_CONNECTION
        boost::mutex::scoped_lock locker(connection_mutex_);
        #endif

        //insert
        custom_connections_.emplace_back(wrapper_ptr->connection_);
    }

    return wrapper_ptr;
}

/**
 */
void DBInstance::destroyCustomConnection(DBConnection* conn)
{
    traced_assert(conn);

    {
        #ifdef PROTECT_CONNECTION
        boost::mutex::scoped_lock locker(connection_mutex_);
        #endif

        auto it = std::find_if(custom_connections_.begin(), custom_connections_.end(), 
            [ & ] (const ConnectionPtr& c) { return c.get() == conn; });
        
        bool found = (it != custom_connections_.end());
        if (found)
        {
            it->get()->disconnect();
            custom_connections_.erase(it);
        }

        //connection already destroyed? => doom
        traced_assert(found);
    }
}

/**
 */
void DBInstance::destroyCustomConnections()
{
    #ifdef PROTECT_CONNECTION
    boost::mutex::scoped_lock locker(connection_mutex_);
    #endif

    for (auto& cc : custom_connections_)
        cc->disconnect();
    
    custom_connections_.clear();
}

/**
 */
void DBInstance::destroyConcurrentConnections()
{
    #ifdef PROTECT_CONNECTION
    boost::mutex::scoped_lock locker(connection_mutex_);
    #endif

    for (auto& cc : concurrent_connections_)
        cc.second->disconnect();
    
    concurrent_connections_.clear();
}

/**
 */
ResultT<DBConnection*> DBInstance::createConnection(bool verbose)
{
    traced_assert(dbOpen());

    auto r = createConnection_impl(verbose);
    traced_assert(!r.ok() || r.result() != nullptr);

    return r;
}

/**
 * Updates info about tables in the currently opened database.
 */
Result DBInstance::updateTableInfo()
{
    traced_assert(dbReady());
    traced_assert(default_connection_);

    auto res = default_connection_->createTableInfo();
    if (!res.ok())
        return res;

    table_info_ = res.result();

    return Result::succeeded();
}

/**
 */
void DBInstance::printTableInfo() const
{
    traced_assert(dbReady());

    for (const auto& table : tableInfo())
    {
        auto table_name = table.first;

        loginf << "[" << table_name << "]";
        loginf << "columns: " << table.second.columns().size();

        const auto& tinfo = table.second;
        for (const auto& ci : tinfo.columns())
        {
            std::stringstream ss;
            ss << "name: " << ci.name() << " ";
            ss << "dtype_prop: " << (ci.hasPropertyType() ? Property::asString(ci.propertyType()) : "-") << " ";
            ss << "dtype_db: " << (ci.hasDBType() ? ci.dbType() : "-") << " ";
            ss << "key: " << ci.key() << " ";
            ss << "null_allowed: " << ci.nullAllowed() << " ";
            ss << "comment: " << ci.comment();

            loginf << ss.str();
        }

        loginf << "";
    }
}

/**
 */
Result DBInstance::configureDBUsingPragmas()
{
    traced_assert(dbReady());
    traced_assert(default_connection_);

    auto pragmas = sqlPragmas();

    for (const auto& p : pragmas)
    {
        auto r = default_connection_->executePragma(p);
        if (!r.ok())
            return r;
    }

    return Result::succeeded();
}

/**
 */
std::string DBInstance::dbInfo()
{
    if (!dbReady())
        return "db not ready";

    traced_assert(default_connection_);

    auto info = default_connection_->dbInfo();
    if (info.empty())
        return "no info available";

    return info;
}
