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

#include "dbinterface.h"

#include "dbinstance.h"
#include "dbcommand.h"
#include "dbresult.h"
#include "dbtableinfo.h"
#include "dbconnection.h"
#include "sqlgenerator.h"

#include "duckdbinstance.h"

#include "sector.h"
#include "sectorlayer.h"
#include "sectorinsidecache.h"

#include "reporttable.h"

#include "viewpointkeys.h"

#include "appstate.h"
#include "logmodel.h"
#include "buffer.h"
#include "config.h"
#include "files.h"
#include "timeconv.h"
#include "number.h"
#include "asynctask.h"

#include "tbbhack.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <QApplication>
#include <QMessageBox>
#include <QMutexLocker>
#include <QThread>
#include <QElapsedTimer>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/filesystem/path.hpp>

#include <fstream>
#include <tuple>
#include <functional>

using namespace Utils;
using namespace std;

const string PROP_TIMESTAMP_MIN_NAME {"timestamp_min"};
const string PROP_TIMESTAMP_MAX_NAME {"timestamp_max"};
const string PROP_LATITUDE_MIN_NAME  {"latitude_min"};
const string PROP_LATITUDE_MAX_NAME  {"latitude_max"};
const string PROP_LONGITUDE_MIN_NAME {"longitude_min"};
const string PROP_LONGITUDE_MAX_NAME {"longitude_max"};

const size_t DBInterface::TableBulkUpdateMinRows = 50;

#define PROTECT_INSTANCE

/**
 */
// DBInterface::DBInterface(string class_name, string instance_name, COMPASS* compass)
//     : Configurable(class_name, instance_name, compass), ...

DBInterface::DBInterface(nlohmann::json& config,
                         Configurable& parent,
                         AppState& app_state,
                         LogStore& log_store,
                         const std::string& app_version)
    : Configurable(config, &parent)
    , app_state_(app_state)
    , log_store_(log_store)
    , app_version_(app_version)
    , insert_mt_(true)
{
    registerParameter("log_verbose", &log_verbose_, log_verbose_);
    registerParameter("read_chunk_size", &read_chunk_size_, 50000u);
    registerParameter("max_ram_file_gb", &max_ram_file_gb_, max_ram_file_gb_);
    registerParameter("max_ram_inmem_gb", &max_ram_inmem_gb_, max_ram_inmem_gb_);
    registerParameter("num_threads", &num_threads_, num_threads_);
    registerParameter("live_cleanup_time_secs", &live_cleanup_time_secs_, live_cleanup_time_secs_);
    registerParameter("use_live_inmem_db", &use_live_inmem_db_, use_live_inmem_db_);

    if (use_live_inmem_db_)
    {
        logwrn << "version not reliable with live in-mem db, deactivating";
        use_live_inmem_db_ = false; // TODO REMOVE HACK
    }

    registerParameter("preserve_insert_order", &preserve_insert_order_, preserve_insert_order_);

    sector_inside_cache_ = std::make_shared<SectorInsideCache>();

    createSubConfigurables();
}

/**
 */
DBInterface::~DBInterface()
{
    logdbg;

    waitForPendingTaskLogWrites();

    db_instance_ = nullptr;

    logdbg << "end";
}

/**
 * Returns a connection-specific SQL generator.
 */
SQLGenerator DBInterface::sqlGenerator() const
{
    traced_assert(db_instance_);
    return SQLGenerator(db_instance_->sqlConfiguration());
}

/**
 * Internal reset on fail.
 */
void DBInterface::reset()
{
    properties_loaded_ = false;
    properties_.clear();
    properties_dirty_ = false;

    dbcolumn_content_flags_.clear();

    if (db_instance_)
    {
        db_instance_->close();
        db_instance_.reset();
    }
}

/**
 */
std::string DBInterface::dbFilename() const
{
    if (!db_instance_)
        return "";

    return db_instance_->dbFilename();
}

/**
 */
bool DBInterface::dbInMemory() const
{
    if (!db_instance_)
        return false;

    return db_instance_->dbInMem();
}

/**
 */
bool DBInterface::canCreateDBFileFromMemory() const
{
    return (ready() && dbInMemory());
}

/**
 */
bool DBInterface::ready() const
{
    if (!db_instance_)
        return false;
    
    return db_instance_->dbReady(); //ready means open and connected
}

/**
 * Opens a database file.
 */
void DBInterface::openDBFile(const std::string& filename, bool overwrite)
{
    traced_assert(!filename.empty());

    bool new_file = !Files::fileExists(filename) || overwrite;

    openDBFileInternal(filename, overwrite);

    loginf << "new_file " << new_file;

    if (new_file)
        log_store_.logInfo("DBInterface", {}, {}) << "database '" << filename << "' created";
}

/**
 * Opens an in-memory database.
 */
void DBInterface::openDBInMemory()
{
    openDBFileInternal("", false);
}

/**
 */
SectorInsideCache& DBInterface::sectorInsideCache()
{
    traced_assert(sector_inside_cache_);
    return *sector_inside_cache_;
}

void DBInterface::openDBFileInternal(const std::string& filename, bool overwrite)
{
    try
    {
        bool in_mem = filename.empty();
        std::string filename_internal = in_mem ? DBInstance::InMemFilename : filename;

        //delete old file?
        if (!in_mem && overwrite && Files::fileExists(filename))
        {
            loginf << "deleting pre-existing file '" << filename << "'";
            Files::deleteFile(filename);
        }

        bool created_new_db = in_mem || !Files::fileExists(filename);

        //create connection
        db_instance_.reset(new DuckDBInstance(this));
        
        loginf << "opening file '" << filename_internal << "'";

        //connect to db
        traced_assert(db_instance_);

        auto open_result = in_mem ? db_instance_->openInMemory() : db_instance_->open(filename);

        if (!open_result.ok())
        {
            logerr << "database could not be opened: " << open_result.second;
            throw std::runtime_error ("database could not be opened: " + open_result.second);
        }

        if (created_new_db)
        {
            //create some default tables
            traced_assert(!existsPropertiesTable());
            createPropertiesTable();
            properties_loaded_ = true;

            setProperty("APP_VERSION", app_version_);
            saveProperties();

            traced_assert(!existsSectorsTable());
            createSectorsTable();
        }
        else
        {
            traced_assert(existsPropertiesTable());
            loadProperties();

            //check db version against app version
            if (!hasProperty("APP_VERSION")
                    || getProperty("APP_VERSION") != app_version_)
            {
                string reason = hasProperty("APP_VERSION") ?
                            "DB from Version " + getProperty("APP_VERSION") + ", current "
                            + app_version_ : "DB from Version older than 0.7.0";

                properties_loaded_ = false;
                properties_.clear();
                db_instance_->close();

                logerr << "Incorrect application version for database:\n " << reason;
                throw std::runtime_error ("Incorrect application version for database:\n "+reason);
            }

            traced_assert(existsSectorsTable());
        }

        // the targets table is created by DBContentDB when the database was opened

        if (!existsTaskLogTable())
            createTaskLogTable();

        //the sector_inside() function of the Sectors filter, per database instance
        auto function_result = db_instance_->defaultConnection().registerSectorInsideFunction(sector_inside_cache_);
        if (!function_result.ok())
            logwrn << "sector inside function not registered: " << function_result.error();

        //emit databaseOpenedSignal();

        loginf << "done";
    }
    catch(const std::exception& ex)
    {
        reset();

        logerr << "error: " << ex.what();
        throw std::runtime_error(ex.what());
    }
}

/**
 * Transfers an opened memory database to the given file and switches to this version.
 */
void DBInterface::openDBFileFromMemory(const std::string& filename)
{
    traced_assert(canCreateDBFileFromMemory());
    traced_assert(!filename.empty());

    try
    {
        if (Files::fileExists(filename))
            Files::deleteFile(filename);

        //export memory database to given file
        auto export_result = db_instance_->exportToFile(filename);

        if (!export_result.ok())
        {
            logerr << "database could not be exported: " << export_result.second;
            throw std::runtime_error ("database could not be exported: " + export_result.second);
        }

        //close current (in-mem) db
        closeDB();

        //open new physical db
        openDBFile(filename, false);
    }
    catch(const std::exception& ex)
    {
        reset();

        logerr << "error: " << ex.what();
        throw std::runtime_error(ex.what());
    }
}

/**
 * Closes the currently open database.
 */
void DBInterface::closeDB()
{
    loginf;

    waitForPendingTaskLogWrites();

    if (properties_loaded_)  // false if database not opened
        saveProperties();

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        traced_assert(db_instance_);
        db_instance_->close();

        properties_loaded_ = false;
        properties_.clear();
        properties_dirty_ = false;

        dbcolumn_content_flags_.clear();
    }

    // signal emitted in COMPASS
}

/**
 * Exports the currently opened database to the given file.
 */
void DBInterface::exportDBFile(const std::string& filename)
{
    traced_assert(ready());

    Result export_result;

    //delete any existing file
    if (Utils::Files::fileExists(filename))
        Utils::Files::deleteFile(filename);

    try
    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        export_result = db_instance_->exportToFile(filename);
    }
    catch(const std::exception& ex)
    {
        logerr << "database could not be exported: " << ex.what();
        throw std::runtime_error ("database could not be exported: " + std::string(ex.what()));
    }

    if (!export_result.ok())
    {
        logerr << "database could not be exported: " << export_result.second;
        throw std::runtime_error ("database could not be exported: " + export_result.second);
    }
}

/**
 * Cleans the current db.
 * Note that the current database will be closed and reopened for this purpose.
 */
bool DBInterface::cleanupDB(bool show_dialog)
{
    loginf;

    traced_assert(ready());

    //in-mem => nothing to do
    if (dbInMemory())
        return true;

    bool ok;

    if (show_dialog)
    {
        auto cb = [ this ] (const AsyncTaskState&, AsyncTaskProgressWrapper&) 
        { 
            return this->cleanupDBInternal();
        };

        AsyncFuncTask task(cb, "Optimizing Database", "Optimizing database", false);
        ok = task.runAsyncDialog();
    }
    else
    {
        ok = cleanupDBInternal().ok();
    }

    return ok;
}

/**
 */
Result DBInterface::cleanupDBInternal()
{
    // [FIX] Protect the critical section where the DB instance is destroyed/recreated.
    // Also required because we call execute(), which expects the lock to be held.
    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    logdbg << "cleaning db";

    traced_assert(ready());
    traced_assert(!cleanup_in_progress_);

    cleanup_in_progress_ = true;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    std::function<std::tuple<string, string, string, std::uint64_t>()> get_db_status = [&]() -> std::tuple<string, string, string, std::uint64_t>
    {
        try
        {
            DBCommand command;
            // Query raw bytes from duckdb_memory() for precision, while keeping other info from pragma_database_size
            command.set("SELECT (SELECT sum(memory_usage_bytes) FROM duckdb_memory()) as mem_bytes, memory_limit, wal_size FROM pragma_database_size();");

            PropertyList list;
            list.addProperty("mem_bytes", PropertyDataType::ULONGINT);
            list.addProperty("memory_limit", PropertyDataType::STRING);
            list.addProperty("wal_size", PropertyDataType::STRING);
            command.list(list);

            shared_ptr<DBResult> result = execute(command);
            
            if (result->containsData())
            {
                shared_ptr<Buffer> buffer = result->buffer();
                if (buffer->size() > 0)
                {
                    // Get raw bytes
                    std::uint64_t mem_bytes = buffer->get<std::uint64_t>("mem_bytes").get(0);
                    string mem_limit = buffer->get<string>("memory_limit").get(0);
                    string wal_size = buffer->get<string>("wal_size").get(0);

                    // Convert to MB with high precision
                    double mem_mb = mem_bytes / (1024.0 * 1024.0);

                    std::stringstream ss;
                    ss << std::fixed << std::setprecision(2) << mem_mb;

                    return std::make_tuple(ss.str(), mem_limit, wal_size, mem_bytes);
                }
            }
        }
        catch(const std::exception& ex)
        {
            logwrn << "could not query precise db status: " << ex.what();
        }
        return std::make_tuple("?", "?", "?", 0);
    };

    auto get_db_file_size = [&]()
    {
        DBCommand command;
        // Query raw bytes from duckdb_memory() for precision, while keeping other info from pragma_database_size
        command.set("SELECT database_size FROM pragma_database_size();");

        PropertyList list;
        list.addProperty("database_size", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        if (result->containsData())
        {
            shared_ptr<Buffer> buffer = result->buffer();
            if (buffer->size() > 0)
            {
                return buffer->get<std::string>("database_size").get(0);
            }
        }

        return std::string("?");
    };

    auto get_db_row_count = [&]()
    {
        std::uint64_t row_count = 0;

        try
        {
            auto info = db_instance_->defaultConnection().createTableInfo();
            for (const auto& ti : info.result())
            {
                const auto& table_name = ti.second.name();

                PropertyList list;
                list.addProperty("num_rows", PropertyDataType::ULONGINT);

                DBCommand command;
                command.set("SELECT COUNT(*) FROM " + table_name);
                command.list(list);

                std::shared_ptr<DBResult> result = execute(command);
                if (result->containsData() && result->buffer() && result->buffer()->size() > 0)
                {
                    row_count += result->buffer()->get<std::uint64_t>("num_rows").get(0);
                }
                else
                {
                    logwrn << "could not query row count for table " << table_name;
                }
            }
        }
        catch(const std::exception& ex)
        {
            logwrn << "could not query db row count: " << ex.what();
        }

        return row_count;
    };

    string mem_before, limit_before, wal_before;
    std::uint64_t mem_bytes_before;
    std::tie(mem_before, limit_before, wal_before, mem_bytes_before) = get_db_status();

    auto db_size_before = get_db_file_size();

    if (app_state_.appMode() == AppMode::LiveRunning)
    {
        if (db_instance_->dbInMem())
        {
            auto res = execute("CHECKPOINT");

            if (!res.ok())
            {
                logerr << "'CHECKPOINT' failed: " << res.error();
                throw runtime_error("DBInterface: cleanupDBInternal: 'CHECKPOINT' failed: " + res.error());
            }

            res = execute("VACUUM");

            if (!res.ok())
            {
                logerr << "'VACUUM' failed: " << res.error();
                throw runtime_error("DBInterface: cleanupDBInternal: 'VACUUM' failed: " + res.error());
            }
        }
        else
        {
            //loginf << "LIIIIIIVE IN FIIIIIIIILE";

            //reconnect and cleanup to make sure the filesize does not grow
            Result cleanup_result;
            auto res = db_instance_->reconnect(true, &cleanup_result);

            if (!res.ok() || !cleanup_result.ok())
            {
                logerr << "Cleanup failed: " << res.error();
                throw runtime_error("DBInterface: cleanupDBInternal: Cleanup failed: " + res.error());
            }
        }
    }
    else
    {
        auto res = db_instance_->cleanupDB();
        if (!res.ok())
        {
            logerr << "'cleanup failed: '" << res.error() << "'";
        }
    }

    boost::posix_time::ptime elapsed_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration time_diff = elapsed_time - start_time;

    string mem_after, limit_after, wal_after;
    std::uint64_t mem_bytes_after;
    std::tie(mem_after, limit_after, wal_after, mem_bytes_after) = get_db_status();

    std::uint64_t total_row_count = get_db_row_count();
    //double kb_per_row = (double)mem_bytes_after / (double)total_row_count / 1000.0;

    auto db_size_after = get_db_file_size();

    loginf << "duckDB cleanup (" << time_diff.total_milliseconds() << "ms): "
           << "FILE: " << db_size_before << " -> " << db_size_after
           << " RAM: " << mem_before << " -> " << mem_after << " MB" 
           << " (Limit: " << limit_after << ")"
           << " " << (total_row_count / 1000.0) << "K.rows";
           //<< " " << kb_per_row << " KB/row";

    last_live_cleanup_time_ = boost::posix_time::microsec_clock::local_time();

    cleanup_in_progress_ = false;

    return true;
}

bool DBInterface::logVerbose() const
{
    return log_verbose_;
}

Result DBInterface::validateSQL(const std::string& sql)
{
    traced_assert(ready());

    return db_instance_->defaultConnection().execute("EXPLAIN " + sql);
}

Result DBInterface::validateFilter(const std::string& table_name, const std::string& where_clause)
{
    return validateSQL("SELECT * FROM " + table_name + " WHERE " + where_clause);
}

/**
 * !Protect by mutex when calling!
 */
Result DBInterface::execute(const std::string& sql)
{
    traced_assert(ready());

    auto val_res = validateSQL(sql);
    if (!val_res.ok())
    {
        logerr << "validateSQL failed '" << sql << "': " << val_res.error();
        throw std::runtime_error("validateSQL failed '" + sql + "': " + val_res.error());
    }

    auto res = db_instance_->defaultConnection().execute(sql);

    if (!res.ok())
    {
        logerr << "error executing statement '" << sql << "': " << res.error();
        throw std::runtime_error("error executing statement '" + sql + "': " + res.error());
    }

    return res;
}

/**
 * !Protect by mutex when calling!
 */
std::shared_ptr<DBResult> DBInterface::execute(const DBCommand& cmd)
{
    traced_assert(ready());

    auto val_res = validateSQL(cmd.get());
    if (!val_res.ok())
    {
        logerr << "validateSQL failed '" << cmd.get() << "': " << val_res.error();
        throw std::runtime_error("validateSQL failed '" + cmd.get() + "': " + val_res.error());
    }

    auto res = db_instance_->defaultConnection().execute(cmd);

    if (res->hasError())
    {
        logerr << "error executing command '" << cmd.get() << "': " << res->error();
        throw std::runtime_error("error executing command '" + cmd.get() + "': " + res->error());
    }

    return res;
}

/**
 * !Protect by mutex when calling!
 */
void DBInterface::updateTableInfo()
{
    traced_assert(ready());

    auto res = db_instance_->updateTableInfo();

    if (!res.ok())
    {
        logerr << "error updating table info: " << res.error();
        throw std::runtime_error("error updating table info: " + res.error());
    }
}

/**
 */
const DBInstance& DBInterface::dbInstance() const
{
    traced_assert(db_instance_);
    return *db_instance_;
}

/**
 */
const std::map<std::string, DBTableInfo>& DBInterface::tableInfo() const
{
    traced_assert(ready());

    //@TODO: needs protection?
    return db_instance_->tableInfo();
}

/**
 */
bool DBInterface::existsTable(const string& table_name) const
{
    return tableInfo().count(table_name) == 1;
}

/**
 */
void DBInterface::removeTable(const std::string& table_name)
{
    loginf << "name " << table_name;

    traced_assert(ready());

    if (!existsTable(table_name))
    {
        logwrn << "table " << table_name << " does not exist";
        return;
    }

    auto res = db_instance_->defaultConnection().deleteTable(table_name);
    
    loginf << "checking " << table_name;

    traced_assert(res.ok());

    updateTableInfo();

    traced_assert(!existsTable(table_name));
}

/**
 */
bool DBInterface::existsPropertiesTable() 
{ 
    return existsTable(TABLE_NAME_PROPERTIES); 
}

/**
 * Maximum value of an unsigned int column, none if the table is empty (no row, or a row with a NULL max).
 */
boost::optional<std::uint64_t> DBInterface::getMaxUIntValue(const std::string& table_name,
                                                            const std::string& col_name)
{
    traced_assert(ready());

    unsigned int max_tn = 0;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getMaxUIntValueCommand(table_name, col_name);
        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();
        traced_assert(buffer->size() <= 1);

        if (buffer->size() == 0 || buffer->get<unsigned int>(col_name).isNull(0))
            return boost::optional<std::uint64_t>();

        max_tn = buffer->get<unsigned int>(col_name).get(0);
    }

    return max_tn;
}

/**
 * Maximum value of an std::uint64_t column, none if the table is empty (no row, or a row with a NULL max).
 */
boost::optional<std::uint64_t> DBInterface::getMaxULongValue(const std::string& table_name,
                                                             const std::string& col_name)
{
    traced_assert(ready());

    std::uint64_t max_value = 0;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getMaxULongIntValueCommand(table_name, col_name);
        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        if (!buffer->size() || buffer->get<std::uint64_t>(col_name).isNull(0))
            return boost::optional<std::uint64_t>();

        max_value = buffer->get<std::uint64_t>(col_name).get(0);
    }

    return max_value;
}

//std::map<unsigned int, std::tuple<std::set<unsigned int>, std::tuple<bool, unsigned int, unsigned int>,
//std::tuple<bool, unsigned int, unsigned int>>> DBInterface::queryADSBInfo()
//{
//    DBContent& object = COMPASS::instance().dbContentManager().dbContent("ADSB");

//    traced_assert(object.existsInDB());

//    boost::mutex::scoped_lock locker(connection_mutex_);

//    std::map<unsigned int, std::tuple<std::set<unsigned int>, std::tuple<bool, unsigned int, unsigned int>,
//            std::tuple<bool, unsigned int, unsigned int>>> data;

//    shared_ptr<DBCommand> command = sql_generator_.getADSBInfoCommand(object);

//    shared_ptr<DBResult> result = execute(*command);

//    traced_assert(result->containsData());

//    shared_ptr<Buffer> buffer = result->buffer();
//    traced_assert(buffer->has<int>("TARGET_ADDR"));
//    traced_assert(buffer->has<int>("MOPS_VERSION"));
//    traced_assert(buffer->has<char>("MIN_NUCP_NIC"));
//    traced_assert(buffer->has<char>("MAX_NUCP_NIC"));
//    traced_assert(buffer->has<char>("MIN_NACP"));
//    traced_assert(buffer->has<char>("MAX_NACP"));

//    NullableVector<int>& tas = buffer->get<int>("TARGET_ADDR");
//    NullableVector<int>& mops = buffer->get<int>("MOPS_VERSION");
//    NullableVector<char>& min_nus = buffer->get<char>("MIN_NUCP_NIC");
//    NullableVector<char>& max_nus = buffer->get<char>("MAX_NUCP_NIC");
//    NullableVector<char>& min_nas = buffer->get<char>("MIN_NACP");
//    NullableVector<char>& max_nas = buffer->get<char>("MAX_NACP");

//    for (unsigned int cnt = 0; cnt < buffer->size(); cnt++)
//    {
//        if (tas.isNull(cnt))
//            continue;

//        if (!mops.isNull(cnt))
//            get<0>(data[tas.get(cnt)]).insert(mops.get(cnt));

//        if (!min_nus.isNull(cnt) && !max_nus.isNull(cnt))
//            get<1>(data[tas.get(cnt)]) =
//                    std::tuple<bool, unsigned int, unsigned int>(true, min_nus.get(cnt), max_nus.get(cnt));

//        if (!min_nas.isNull(cnt) && !max_nas.isNull(cnt))
//            get<2>(data[tas.get(cnt)]) =
//                    std::tuple<bool, unsigned int, unsigned int>(true, min_nas.get(cnt), max_nas.get(cnt));
//    }

//    return data;
//}

/**
 */
size_t DBInterface::count(const string& table)
{
    logdbg << "table " << table;

    traced_assert(ready());
    traced_assert(existsTable(table));

    int tmp;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string sql = sqlGenerator().getCountStatement(table);

        logdbg << "sql '" << sql << "'";

        DBCommand command;
        command.set(sql);

        PropertyList list;
        list.addProperty("count", PropertyDataType::INT);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(result->containsData());

        tmp = result->buffer()->get<int>("count").get(0);
    }

    logdbg << "start" << table << ": " << tmp << " end";

    return static_cast<size_t>(tmp);
}

/**
 */
void DBInterface::setProperty(const string& id, const string& value)
{
    auto it = properties_.find(id);
    if (it != properties_.end() && it->second == value)
        return;

    properties_[id] = value;
    properties_dirty_ = true;
}

/**
 */
string DBInterface::getProperty(const string& id)
{
    traced_assert(hasProperty(id));
    return properties_.at(id);
}

/**
 */
void DBInterface::removeProperty(const std::string& id)
{
    traced_assert(hasProperty(id));
    properties_.erase(id);
    properties_dirty_ = true;
}

/**
 */
bool DBInterface::hasProperty(const string& id) 
{ 
    return properties_.count(id); 
}

/**
 */
void DBInterface::loadProperties()
{
    loginf;

    traced_assert(ready());
    traced_assert(!properties_loaded_);

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        DBCommand command;
        command.set(sqlGenerator().getSelectAllPropertiesStatement());

        PropertyList list;
        list.addProperty("id", PropertyDataType::STRING);
        list.addProperty("value", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<string>("id"));
        traced_assert(buffer->has<string>("value"));

        NullableVector<string>& id_vec = buffer->get<string>("id");
        NullableVector<string>& value_vec = buffer->get<string>("value");

        for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
        {
            traced_assert(!id_vec.isNull(cnt));

            if (properties_.count(id_vec.get(cnt)))
                logerr << "property '" << id_vec.get(cnt) << "' already exists";

            traced_assert(!properties_.count(id_vec.get(cnt)));
            if (!value_vec.isNull(cnt))
                properties_[id_vec.get(cnt)] = value_vec.get(cnt);
        }

        for (auto& prop_it : properties_)
        {
            std::string value_str = prop_it.second;
            if (value_str.size() > 200)
                value_str = value_str.substr(0, 200) + "...";

            loginf << "id '" << prop_it.first << "' value '"
                << value_str << "'";
        }

        // column with content
        if (properties_.count(dbcolumn_content_property_name_))
        {
            nlohmann::json dbcolumn_content_json = nlohmann::json::parse(properties_.at(dbcolumn_content_property_name_));
            dbcolumn_content_flags_ = dbcolumn_content_json.get<std::map<std::string, std::set<std::string>>>();
        }

        properties_loaded_ = true;
        properties_dirty_ = false;
    }
}

/**
 */
bool DBInterface::hasContentIn(const std::string& table_name, const std::string& column_name) const
{
    if (!properties_loaded_)
        return false;

    if (dbcolumn_content_flags_.count(table_name))
        return dbcolumn_content_flags_.at(table_name).count(column_name);

    return false;
}

/**
 */
void DBInterface::setContentIn(const std::string& table_name, const std::string& column_name)
{
    if (!properties_loaded_)
        return;

    if (!dbcolumn_content_flags_[table_name].count(column_name))
    {
        dbcolumn_content_flags_[table_name].insert(column_name);
        properties_dirty_ = true; // serialized into properties in saveProperties()
    }
}

/**
 */
void DBInterface::saveProperties()
{
    if (!properties_dirty_)
    {
        logdbg << "skipped, not dirty";
        return;
    }

    loginf << "num " << properties_.size();

    if (!db_instance_)
    {
        logwrn << "failed since no database instance exists";
        return;
    }

    // boost::mutex::scoped_lock locker(connection_mutex_); // done in closeConnection
    traced_assert(properties_loaded_);

    nlohmann::json dbcolumn_content_json = dbcolumn_content_flags_;
    properties_[dbcolumn_content_property_name_] = dbcolumn_content_json.dump();

    string str;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        for (auto& prop_it : properties_)
        {
            auto sql = sqlGenerator().getInsertPropertyStatement(prop_it.first, prop_it.second);
            execute(sql);
        }
    }

    properties_dirty_ = false;

    logdbg << "done";
}

/**
 */
std::vector<std::shared_ptr<SectorLayer>> DBInterface::loadSectors()
{
    loginf;

    traced_assert(ready());

    std::vector<std::shared_ptr<SectorLayer>> sector_layers;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        DBCommand command;
        command.set(sqlGenerator().getSelectAllSectorsStatement());

        PropertyList list;
        list.addProperty("id", PropertyDataType::INT);
        list.addProperty("name", PropertyDataType::STRING);
        list.addProperty("layer_name", PropertyDataType::STRING);
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<int>("id"));
        traced_assert(buffer->has<string>("name"));
        traced_assert(buffer->has<string>("layer_name"));
        traced_assert(buffer->has<string>("json"));

        NullableVector<int>& id_vec = buffer->get<int>("id");
        NullableVector<string>& name_vec = buffer->get<string>("name");
        NullableVector<string>& layer_name_vec = buffer->get<string>("layer_name");
        NullableVector<string>& json_vec = buffer->get<string>("json");

        int id;
        string name;
        string layer_name;
        string json_str;

        for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
        {
            traced_assert(!id_vec.isNull(cnt));
            traced_assert(!name_vec.isNull(cnt));
            traced_assert(!layer_name_vec.isNull(cnt));
            traced_assert(!json_vec.isNull(cnt));

            id = id_vec.get(cnt);
            name = name_vec.get(cnt);
            layer_name = layer_name_vec.get(cnt);
            json_str = json_vec.get(cnt);

            auto lay_it = std::find_if(sector_layers.begin(), sector_layers.end(),
                                    [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name;});

            if (lay_it == sector_layers.end())
            {
                sector_layers.push_back(make_shared<SectorLayer>(layer_name));

                lay_it = std::find_if(
                            sector_layers.begin(), sector_layers.end(),
                            [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name;});
            }

            auto eval_sector = new Sector(id, name, layer_name, true);
            bool ok = eval_sector->readJSON(json_str);
            traced_assert(ok);

            string layer_name = eval_sector->layerName();

            (*lay_it)->addSector(shared_ptr<Sector>(eval_sector));
            traced_assert((*lay_it)->hasSector(name));

            logdbg << "loaded sector '" << name << "' in layer '"
                << layer_name << "' num points " << (*lay_it)->sector(name)->size();
        }
    }

    loginf << "loaded " << sector_layers.size() << " sector layers";

    return sector_layers;
}

/**
 */
bool DBInterface::areColumnsNull(const std::string& table_name, 
                                 const std::vector<std::string> columns)
{
    traced_assert(ready());

    bool cols_null = false;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string str = sqlGenerator().getSelectNullCount(table_name, columns);

        loginf << "sql '" << str << "'";

        DBCommand command;
        command.set(str);

        PropertyList list;
        list.addProperty("count", PropertyDataType::INT);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<int>("count"));

        NullableVector<int>& count_vec = buffer->get<int>("count");
        traced_assert(count_vec.contentSize() == 1);

        cols_null = count_vec.get(0) != 0;

        loginf << "null count " << count_vec.get(0);
    }

    return cols_null;
}

/**
 */
bool DBInterface::existsViewPointsTable()
{
    return existsTable(TABLE_NAME_VIEWPOINTS);
}

/**
 */
void DBInterface::createViewPointsTable()
{
    traced_assert(ready());
    traced_assert(!existsViewPointsTable());

    setProperty("view_points_version", ViewPointKeys::VP_COLLECTION_CONTENT_VERSION);

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTableViewPointsCreateStatement());
        updateTableInfo();
    }
}

/**
 */
/**
 * Can be called from the main thread while a live cleanup reconnect runs (view point edited
 * in live mode). The lock is taken first, it waits for the reconnect to finish.
 */
void DBInterface::setViewPoint(const unsigned int id, const string& value)
{
    if (!db_instance_)
    {
        logwrn << "failed since no database instance exists";
        return;
    }

    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    if (!existsTable(TABLE_NAME_VIEWPOINTS))
    {
        setProperty("view_points_version", ViewPointKeys::VP_COLLECTION_CONTENT_VERSION);

        execute(sqlGenerator().getTableViewPointsCreateStatement());
        updateTableInfo();
    }

    string str = sqlGenerator().getInsertViewPointStatement(id, value);
    logdbg << "cmd '" << str << "'";

    execute(str);
}

/**
 */
map<unsigned int, string> DBInterface::viewPoints()
{
    loginf;

    traced_assert(ready());
    traced_assert(existsViewPointsTable());

    map<unsigned int, string> view_points;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        DBCommand command;
        command.set(sqlGenerator().getSelectAllViewPointsStatement());

        PropertyList list;
        list.addProperty("id", PropertyDataType::UINT);
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<unsigned int>("id"));
        traced_assert(buffer->has<string>("json"));
      
        NullableVector<unsigned int>& id_vec = buffer->get<unsigned int>("id");
        NullableVector<string>& json_vec = buffer->get<string>("json");

        for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
        {
            traced_assert(!id_vec.isNull(cnt));
            traced_assert(!view_points.count(id_vec.get(cnt)));
            if (!json_vec.isNull(cnt))
                view_points[id_vec.get(cnt)] = json_vec.get(cnt);
        }
    }

    loginf << "loaded " << view_points.size() << " view points";

    return view_points;
}

/**
 */
void DBInterface::deleteViewPoint(const unsigned int id)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        auto sql = sqlGenerator().getDeleteStatement(TABLE_NAME_VIEWPOINTS, "id="+to_string(id));
        execute(sql);
    }
}

/**
 */
void DBInterface::deleteAllViewPoints()
{
    clearTableContent(TABLE_NAME_VIEWPOINTS);
}

/**
 */
bool DBInterface::existsSectorsTable()
{
    return existsTable(TABLE_NAME_SECTORS);
}

/**
 */
void DBInterface::createSectorsTable()
{
    loginf;

    traced_assert(ready());
    traced_assert(!existsSectorsTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        auto sql = sqlGenerator().getTableSectorsCreateStatement();

        loginf << "sql '" << sql << "'";

        execute(sql);
        updateTableInfo();
    }
}

/**
 */
void DBInterface::clearSectorsTable()
{
    clearTableContent(TABLE_NAME_SECTORS);
}

/**
 */
void DBInterface::saveSector(shared_ptr<Sector> sector)
{
    loginf << "sector " << sector->name() << " layer " << sector->layerName()
           << " id " << sector->id();

    if (!db_instance_)
    {
        logwrn << "failed since no database instance exists";
        return;
    }

    traced_assert(ready());

    if (!existsSectorsTable())
        createSectorsTable();

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        // insert and replace
        string str = sqlGenerator().getReplaceSectorStatement(sector->id(), sector->name(), sector->layerName(), sector->jsonDataStr());
        logdbg << "cmd '" << str << "'";

        execute(str);
    }
}

/**
 */
void DBInterface::deleteSector(shared_ptr<Sector> sector)
{
    traced_assert(ready());

    unsigned int sector_id = sector->id();

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string cmd = sqlGenerator().getDeleteStatement(TABLE_NAME_SECTORS,"id="+to_string(sector_id));
        execute(cmd);
    }
}

/**
 */
void DBInterface::deleteAllSectors()
{
    clearTableContent(TABLE_NAME_SECTORS);
}

bool DBInterface::existsTaskLogTable()
{
    return existsTable(TABLE_NAME_TASK_LOG);
}
void DBInterface::createTaskLogTable()
{
    loginf;

    traced_assert(ready());
    traced_assert(!existsTaskLogTable());

    {
#ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
#endif

        auto sql = sqlGenerator().getTableTaskLogCreateStatement();

        loginf << "sql '" << sql << "'";

        execute(sql);
        updateTableInfo();
    }
}
std::vector<nlohmann::json> DBInterface::loadTaskLogInfo()
{
    loginf;

    traced_assert(ready());
    traced_assert(existsTaskLogTable());

    std::vector<nlohmann::json> info;

    {
#ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
#endif

        DBCommand command;
        command.set(sqlGenerator().getSelectAllTaslLogMessagesStatement());

        PropertyList list;
        list.addProperty("msg_id", PropertyDataType::UINT);
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(!result->hasError());
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<unsigned int>("msg_id"));
        traced_assert(buffer->has<string>("json"));

        NullableVector<unsigned int>& msg_id_vec = buffer->get<unsigned int>("msg_id");
        NullableVector<string>& json_vec = buffer->get<string>("json");

        unsigned int msg_id;
        string json_str;

        std::set<unsigned int> existing_msg_ids;

        for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
        {
            traced_assert(!msg_id_vec.isNull(cnt));
            traced_assert(!json_vec.isNull(cnt));

            msg_id = msg_id_vec.get(cnt);
            json_str = json_vec.get(cnt);

            traced_assert(!existing_msg_ids.count(msg_id));
            existing_msg_ids.insert(msg_id);

            info.emplace_back(nlohmann::json::parse(json_str));
        }
    }

    return info;
}

/**
 * Task log entries are written from the main thread, also while the live cleanup reconnects
 * the database on a worker thread. The cleanup holds instance_mutex_ for its whole duration,
 * so owning the lock means no cleanup is in progress and ready() is stable. The main thread
 * never waits for the lock: a busy instance defers the write to a background thread.
 */
void DBInterface::saveTaskLogInfo(unsigned int msg_id, const nlohmann::json& info)
{
    {
        boost::unique_lock<boost::mutex> locker(instance_mutex_, boost::try_to_lock);

        if (locker.owns_lock())
        {
            if (ready())
                saveTaskLogInfoInternal(msg_id, info);
            // no database open: the entry stays in memory only

            return;
        }
    }

    loginf << "instance busy, deferring task log write " << msg_id;

    ++num_deferred_task_log_writes_;

    boost::mutex::scoped_lock locker(pending_task_log_writes_mutex_);

    // drop finished writes
    pending_task_log_writes_.erase(
        std::remove_if(pending_task_log_writes_.begin(), pending_task_log_writes_.end(),
                       [] (const std::future<void>& f) 
                       { return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready; }),
        pending_task_log_writes_.end());

    pending_task_log_writes_.push_back(std::async(std::launch::async, [ this, msg_id, info ] ()
    {
        boost::mutex::scoped_lock locker(instance_mutex_);

        if (!ready()) // database closed in the meantime
            return;

        try
        {
            saveTaskLogInfoInternal(msg_id, info);
        }
        catch (const std::exception& e)
        {
            logerr << "deferred task log write failed: " << e.what();
        }
    }));
}

/**
 * Writes the task log entry. instance_mutex_ must be held by the caller, the database must be ready.
 */
void DBInterface::saveTaskLogInfoInternal(unsigned int msg_id, const nlohmann::json& info)
{
    traced_assert(ready());
    traced_assert(existsTaskLogTable());

    //storing all targets at once via a buffer is faster
    std::shared_ptr<Buffer> buffer(new Buffer(LogStore::LogEntry::DBPropertyList));

    auto& msg_id_vec = buffer->get<unsigned int>(LogStore::LogEntry::DBColumnID.name());
    auto& info_vec   = buffer->get<nlohmann::json>(LogStore::LogEntry::DBColumnInfo.name());

    msg_id_vec.set(0, msg_id);
    info_vec.set(0, info);

    insertBufferInternal(TABLE_NAME_TASK_LOG, buffer);
}

/**
 * Waits for deferred task log writes. Called before the database is closed or the interface destroyed.
 */
void DBInterface::waitForPendingTaskLogWrites()
{
    std::vector<std::future<void>> pending;

    {
        boost::mutex::scoped_lock locker(pending_task_log_writes_mutex_);
        pending.swap(pending_task_log_writes_);
    }

    for (auto& f : pending)
        if (f.valid())
            f.wait();
}

// ============================================================
// db_context table
// ============================================================

bool DBInterface::existsDBContextTable()
{
    return existsTable(TABLE_NAME_DB_CONTEXT);
}

void DBInterface::createDBContextTable()
{
    traced_assert(ready());
    traced_assert(!existsDBContextTable());

    {
        boost::mutex::scoped_lock locker(instance_mutex_);
        execute(sqlGenerator().getTableDBContextCreateStatement());
        updateTableInfo();
    }
}

/**
 * Can be called from the main thread while a live cleanup reconnect runs (e.g. a new data
 * source in live data). The lock is taken first, it waits for the reconnect to finish.
 */
void DBInterface::saveDBContextSection(const std::string& section, const std::string& json_str)
{
    boost::mutex::scoped_lock locker(instance_mutex_);

    traced_assert(ready());

    if (!existsTable(TABLE_NAME_DB_CONTEXT))
    {
        execute(sqlGenerator().getTableDBContextCreateStatement());
        updateTableInfo();
    }

    execute(sqlGenerator().getReplaceDBContextSectionStatement(section, json_str));
}

std::string DBInterface::loadDBContextSection(const std::string& section)
{
    traced_assert(ready());

    if (!existsDBContextTable())
        return {};

    {
        boost::mutex::scoped_lock locker(instance_mutex_);

        DBCommand command;
        command.set(sqlGenerator().getSelectDBContextSectionStatement(section));

        PropertyList list;
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        auto result = execute(command);
        auto buffer = result->buffer();

        if (buffer->size() == 0)
            return {};

        return buffer->get<std::string>("json").get(0);
    }
}

std::map<std::string, std::string> DBInterface::loadAllDBContextSections()
{
    traced_assert(ready());

    std::map<std::string, std::string> sections;

    if (!existsDBContextTable())
        return sections;

    {
        boost::mutex::scoped_lock locker(instance_mutex_);

        DBCommand command;
        command.set(sqlGenerator().getSelectAllDBContextSectionsStatement());

        PropertyList list;
        list.addProperty("section", PropertyDataType::STRING);
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        auto result = execute(command);
        auto buffer = result->buffer();

        auto& section_vec = buffer->get<std::string>("section");
        auto& json_vec = buffer->get<std::string>("json");

        for (size_t i = 0; i < buffer->size(); ++i)
            sections[section_vec.get(i)] = json_vec.get(i);
    }

    return sections;
}

void DBInterface::clearDBContextTable()
{
    if (existsDBContextTable())
        clearTableContent(TABLE_NAME_DB_CONTEXT);
}

// ============================================================
// db_info table
// ============================================================

bool DBInterface::existsDBInfoTable()
{
    return existsTable(TABLE_NAME_DB_INFO);
}

void DBInterface::createDBInfoTable()
{
    traced_assert(ready());
    traced_assert(!existsDBInfoTable());

    {
        boost::mutex::scoped_lock locker(instance_mutex_);
        execute(sqlGenerator().getTableDBInfoCreateStatement());
        updateTableInfo();
    }
}

/**
 * Can be called from the main thread while a live cleanup reconnect runs on a worker thread.
 * The cleanup holds instance_mutex_ while the database is closed, so the lock is taken first:
 * it waits for the reconnect to finish (a few seconds at most), then the database is ready.
 */
void DBInterface::saveDBInfo(const std::string& name, const std::string& json_str)
{
    boost::mutex::scoped_lock locker(instance_mutex_);

    traced_assert(ready());

    if (!existsTable(TABLE_NAME_DB_INFO))
    {
        execute(sqlGenerator().getTableDBInfoCreateStatement());
        updateTableInfo();
    }

    execute(sqlGenerator().getReplaceDBInfoStatement(name, json_str));
}

std::string DBInterface::loadDBInfo(const std::string& name)
{
    traced_assert(ready());

    if (!existsDBInfoTable())
        return {};

    {
        boost::mutex::scoped_lock locker(instance_mutex_);

        DBCommand command;
        command.set(sqlGenerator().getSelectDBInfoStatement(name));

        PropertyList list;
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        auto result = execute(command);
        auto buffer = result->buffer();

        if (buffer->size() == 0)
            return {};

        return buffer->get<std::string>("json").get(0);
    }
}

bool DBInterface::hasDBInfo(const std::string& name)
{
    return !loadDBInfo(name).empty();
}

void DBInterface::removeDBInfo(const std::string& name)
{
    traced_assert(ready());

    if (!existsDBInfoTable())
        return;

    {
        boost::mutex::scoped_lock locker(instance_mutex_);
        execute(sqlGenerator().getDeleteDBInfoStatement(name));
    }
}

/**
 * Sets all values of a column to NULL.
 */
void DBInterface::setColumnNull(const std::string& table_name, const std::string& col_name)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getSetNullStatement(table_name, col_name));
    }
}

/**
 * Inserts a row or replaces the row with the same key, values in column order.
 */
void DBInterface::replaceRow(const std::string& table_name, const std::vector<std::string>& values)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().replaceStatement(table_name, values));
    }
}

/**
 * Creates a table with the given columns, primary_key is the index of the key column (-1 = none).
 */
void DBInterface::createTable(const std::string& table_name,
                              const PropertyList& properties,
                              int primary_key)
{
    traced_assert(ready());
    traced_assert(!existsTable(table_name));

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getCreateTableStatement(table_name, properties, primary_key));
        updateTableInfo();
    }
}

/**
 * Creates a table with the given columns and indices.
 */
void DBInterface::createTable(const std::string& table_name,
                              const std::vector<DBTableColumnInfo>& column_infos,
                              const std::vector<db::Index>& indices)
{
    traced_assert(ready());
    traced_assert(!existsTable(table_name));

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getCreateTableStatement(table_name, column_infos, indices));
        updateTableInfo();
    }
}

/**
 * Deletes the rows of a table matching the given filter (empty = all rows).
 */
Result DBInterface::deleteRows(const std::string& table_name, const std::string& filter)
{
    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    auto cmd = sqlGenerator().getDeleteCommand(table_name, filter);
    auto res = execute(*cmd);
    if (res->hasError())
        return Result::failed(res->error());

    return Result::succeeded();
}

/**
 * Sets a single cell, the row is identified by the value of its key column.
 */
Result DBInterface::updateCell(const std::string& table_name,
                               const std::string& col_name,
                               const nlohmann::json& col_value,
                               const std::string& key_col_name,
                               const nlohmann::json& key_col_value)
{
    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    string str = sqlGenerator().getUpdateCellStatement(table_name, col_name, col_value, key_col_name, key_col_value);
    return execute(str);
}

/**
 * Creates a report table, replacing an existing table of the same name.
 */
void DBInterface::createReportTable(const std::string& table_name,
                                    const PropertyList& properties,
                                    const std::string& key_column)
{
    traced_assert(ready());
    traced_assert(ReportTableDefinition::isReportTableName(table_name));
    traced_assert(properties.size());

    if (existsTable(table_name))
        removeReportTable(table_name);

    std::vector<DBTableColumnInfo> column_infos;
    for (const auto& p : properties.properties())
        column_infos.emplace_back(p.name(), p.dataType(), p.name() == key_column);

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getCreateTableStatement(table_name, column_infos));
        updateTableInfo();
    }

    traced_assert(existsTable(table_name));
}

/**
 */
void DBInterface::removeReportTable(const std::string& table_name)
{
    traced_assert(ready());
    traced_assert(ReportTableDefinition::isReportTableName(table_name));

    if (!existsTable(table_name))
    {
        logwrn << "table '" << table_name << "' does not exist";
        return;
    }

    loginf << "dropping report table '" << table_name << "'";

    Result res;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        res = db_instance_->defaultConnection().deleteTable(table_name);
        updateTableInfo();
    }

    if (!res.ok())
    {
        logerr << "dropping report table '" << table_name << "' failed: " << res.error();
        throw runtime_error("DBInterface: removeReportTable: dropping table '" + table_name + "' failed: " + res.error());
    }
}

/**
 * Names of all report tables in the database.
 */
std::set<std::string> DBInterface::reportTableNames() const
{
    std::set<std::string> names;

    for (const auto& t : tableInfo())
        if (ReportTableDefinition::isReportTableName(t.first))
            names.insert(t.first);

    return names;
}

/**
 * Names of the report tables of the given result.
 */
std::set<std::string> DBInterface::reportTableNames(unsigned int result_id) const
{
    auto prefix = ReportTableDefinition::tableNamePrefix(result_id);

    std::set<std::string> names;

    for (const auto& name : reportTableNames())
        if (name.compare(0, prefix.size(), prefix) == 0)
            names.insert(name);

    return names;
}

/**
 * Drops the report tables of the given result, except the ones to keep. Returns the number of dropped tables.
 */
size_t DBInterface::removeReportTables(unsigned int result_id, const std::set<std::string>& keep)
{
    size_t n = 0;

    for (const auto& name : reportTableNames(result_id))
    {
        if (keep.count(name))
            continue;

        removeReportTable(name);
        ++n;
    }

    return n;
}

/**
 * Drops all report tables not in the given set, left behind by aborted runs. Returns the number of dropped tables.
 */
size_t DBInterface::removeOrphanReportTables(const std::set<std::string>& referenced_tables)
{
    size_t n = 0;

    for (const auto& name : reportTableNames())
    {
        if (referenced_tables.count(name))
            continue;

        logwrn << "dropping orphan report table '" << name << "'";

        removeReportTable(name);
        ++n;
    }

    if (n)
        cleanupDB(false);

    return n;
}

/**
 * Persistent storage size of a table in bytes, from the DuckDB block usage.
 */
ResultT<size_t> DBInterface::tableStorageSize(const std::string& table_name)
{
    traced_assert(ready());

    if (!existsTable(table_name))
        return ResultT<size_t>::failed("Table '" + table_name + "' does not exist");

    try
    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        DBCommand command;
        command.set("SELECT CAST(count(DISTINCT block_id) AS BIGINT) AS num_blocks, "
                    "CAST((SELECT block_size FROM pragma_database_size()) AS BIGINT) AS block_size "
                    "FROM pragma_storage_info('" + table_name + "') WHERE block_id >= 0");

        PropertyList list;
        list.addProperty("num_blocks", PropertyDataType::LONGINT);
        list.addProperty("block_size", PropertyDataType::LONGINT);
        command.list(list);

        auto result = execute(command);
        if (!result->containsData() || result->buffer()->size() != 1)
            return ResultT<size_t>::failed("No storage info for table '" + table_name + "'");

        const auto& buffer = *result->buffer();

        std::int64_t num_blocks = buffer.get<std::int64_t>("num_blocks").isNull(0) ? 0 : buffer.get<std::int64_t>("num_blocks").get(0);
        std::int64_t block_size = buffer.get<std::int64_t>("block_size").isNull(0) ? 0 : buffer.get<std::int64_t>("block_size").get(0);

        return ResultT<size_t>::succeeded((size_t)(num_blocks * block_size));
    }
    catch (const std::exception& ex)
    {
        return ResultT<size_t>::failed(ex.what());
    }
}

/**
 */
void DBInterface::insertBuffer(const string& table_name, 
                               shared_ptr<Buffer> buffer)
{
    logdbg << "table name " << table_name << " buffer size " << buffer->size();

    traced_assert(ready());
    traced_assert(buffer);
    traced_assert(!cleanup_in_progress_);

    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    insertBufferInternal(table_name, buffer);
}

/**
 * Inserts the buffer into the given table. instance_mutex_ must be held by the caller.
 */
void DBInterface::insertBufferInternal(const string& table_name, 
                                       shared_ptr<Buffer> buffer)
{
    if (!existsTable(table_name))
    {
        logerr << "table with name '" << table_name << "' does not exist";
        throw runtime_error("DBInterface: insertBuffer: table with name '" + table_name + "' does not exist");
    }

    Result res = db_instance_->defaultConnection().insertBuffer(table_name, buffer);

    if (!res.ok())
    {
        logerr << "inserting into table '" << table_name << "' failed: " << res.error();
        throw runtime_error("DBInterface: insertBuffer: inserting into table '" + table_name + "' failed: " + res.error());
    }
}

/**
 * Inserts multiple buffers (table name -> buffer) at once, possibly utilizing parallelization.
 */
void DBInterface::insertBuffers(const std::map<std::string, std::shared_ptr<Buffer>>& buffers)
{
    traced_assert(ready());
    traced_assert (!cleanup_in_progress_);

    bool db_supports_mt = db_instance_->sqlConfiguration().supports_mt;
    bool exec_mt        = insert_mt_ && db_supports_mt && buffers.size() > 1;

    size_t max_size = 0;
    size_t min_size = std::numeric_limits<size_t>::max();
    for (const auto& it : buffers)
    {
        if (it.second->size() < min_size)
            min_size = it.second->size();
        if (it.second->size() > max_size)
            max_size = it.second->size();
    }

    logdbg << "inserting " << buffers.size() << " object(s) " 
           << (exec_mt ? "multi-threaded" : "single-threaded") 
           << " min_size " << (buffers.size() ? min_size : 0) << " max_size " << (buffers.size() ? max_size : 0);

    struct Job
    {
        Job() = default;
        Job(size_t idx,
            const std::string& tname,
            const std::shared_ptr<Buffer>& b,
            const boost::optional<size_t>& from,
            const boost::optional<size_t>& to)
        :   index   (idx  )
        ,   table   (tname)
        ,   buffer  (b    )
        ,   idx_from(from )
        ,   idx_to  (to   ) {}

        size_t                  index;
        std::string             table;
        std::shared_ptr<Buffer> buffer;
        boost::optional<size_t> idx_from;
        boost::optional<size_t> idx_to;
    };

    std::vector<Job> insert_jobs;

    int chunk_size = -1;

    //init single-threaded
    size_t idx = 0;
    for (auto& it : buffers)
    {
        traced_assert(it.second);
        traced_assert(existsTable(it.first));

        if (chunk_size < 1 || !exec_mt)
        {
            insert_jobs.push_back(Job(idx++, it.first, it.second, {}, {}));
        }
        else
        {
            auto n = it.second->size();
            for (size_t idx = 0; idx < n; idx += (size_t)chunk_size)
            {
                size_t idx_start = idx;
                size_t idx_end   = std::min(idx_start + chunk_size, n);

                traced_assert(idx_start < idx_end);

                insert_jobs.push_back(Job(idx++, it.first, it.second, idx_start, idx_end - 1));
            }
        }
    }

    {
#ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
#endif

        unsigned int n = insert_jobs.size();

        logdbg << "created " << n << " job(s)";

        // create connections for multithreading (if supported)
        std::vector<DBConnectionWrapper> mt_connections;
        if (db_supports_mt)
        {
            mt_connections.resize(n);

            for (unsigned int i = 0; i < n; ++i)
            {
                mt_connections[i] = db_instance_->concurrentConnection(i);
                traced_assert(!mt_connections[i].isEmpty());

                if (mt_connections[i].hasError())
                {
                    logerr << "creating mt connection failed: " << mt_connections[i].error();
                    throw runtime_error(
                        "DBInterface: insertBuffers: creating mt connection failed: " +
                        mt_connections[i].error());
                }
            }
        }

        // insert buffers
        std::vector<Result> results(n, 0);
        if (exec_mt)
        {
            // insert multithreaded (if supported)
            tbb::parallel_for(uint(0), n,
                              [&](unsigned int i)
                              {
                                  auto& job = insert_jobs.at(i);
                                  results.at(i) = mt_connections.at(i).connection().insertBuffer(
                                      job.table, job.buffer, job.idx_from, job.idx_to);
                              });
        }
        else
        {
            // single threaded insert
            for (unsigned int i = 0; i < n; ++i)
            {
                auto& job = insert_jobs.at(i);
                results.at(i) = db_instance_->defaultConnection().insertBuffer(
                    job.table, job.buffer, job.idx_from, job.idx_to);
            }
        }

        // check results
        for (unsigned int i = 0; i < n; ++i)
        {
            const auto& table_name = insert_jobs.at(i).table;
            const auto& result = results.at(i);
            if (!result.ok())
            {
                logerr << "inserting into table '" << table_name << "' failed: " << result.error();
                throw runtime_error("DBInterface: insertBuffer: inserting into table '" +
                                    table_name + "' failed: " + result.error());
            }
        }
    }
}

/**
 */
void DBInterface::updateBuffer(const std::string& table_name,
                               const std::string& key_col,
                               shared_ptr<Buffer> buffer,
                               const boost::optional<size_t>& idx_from,
                               const boost::optional<size_t>& idx_to)
{
    logdbg << "table " << table_name << " buffer size " << buffer->size() << " key " << key_col;

    traced_assert(ready());
    traced_assert(buffer);

    if (!existsTable(table_name))
    {
        logerr << "table with name '" << table_name << "' does not exist";
        throw runtime_error("DBInterface: updateBuffer: table with name '" + table_name + "' does not exist");
    }

    Result res;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        res = db_instance_->defaultConnection().updateBuffer(table_name, buffer, key_col, idx_from, idx_to);
    }

    if (!res.ok())
    {
        logerr << "updating table '" << table_name << "' failed: " << res.error();
        throw runtime_error("DBInterface: updateBuffer: updating table '" + table_name + "' failed: " + res.error());
    }
}

/**
 * Starts a chunked read of the given columns, see readDataChunk().
 */
void DBInterface::prepareRead(const std::string& table_name,
                              const PropertyList& properties,
                              const std::string& filter,
                              bool use_order,
                              const std::string& order_column,
                              const std::string& join_clause)
{
    logdbg << "table " << table_name;

    traced_assert(ready());

    shared_ptr<DBCommand> read = sqlGenerator().getSelectCommand(
        table_name, properties, filter, use_order, order_column, join_clause);

    loginf << "sql '" << read->get() << "'";

    Result res;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        res = db_instance_->defaultConnection().startRead(read, 0, read_chunk_size_);
    }
    
    if (!res.ok())
    {
        logerr << "preparing read for table '" << table_name << "' command '" << read->get() << "' failed: " << res.error();
        throw runtime_error("DBInterface: prepareRead: preparing read for table '" + table_name + "' command '" + read->get() + "'failed: " + res.error());
    }
}

/**
 */
std::pair<std::shared_ptr<Buffer>, bool> DBInterface::readDataChunk()
{
    // locked by prepareRead
    traced_assert(ready());

    shared_ptr<DBResult> result;
    bool last_one = false;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        result = db_instance_->defaultConnection().readChunk();
    }

    if (!result)
    {
        logerr << "connection returned unknown error";
        throw runtime_error("DBInterface: readDataChunk: connection returned unknown error");
    }
    if (result->hasError())
    {
        logerr << "connection returned error: " << result->error();
        throw runtime_error("DBInterface: readDataChunk: connection returned unknown error: " + result->error());
    }
    if (!result->containsData())
    {
        logerr << "buffer does not contain data";
        throw runtime_error("DBInterface: readDataChunk: buffer does not contain data");
    }

    last_one = !result->hasMore();

    shared_ptr<Buffer> buffer = result->buffer();

    traced_assert(buffer);

    return {buffer, last_one};
}

/**
 */
void DBInterface::finalizeReadStatement()
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        db_instance_->defaultConnection().stopRead();
    }
}

/**
 * Runs the database cleanup if live_cleanup_time_min passed since the first call or the last cleanup.
 */
void DBInterface::liveCleanupIfDue()
{
    traced_assert(ready());
    traced_assert(!cleanup_in_progress_);

    if (last_live_cleanup_time_.is_not_a_date_time())
        last_live_cleanup_time_ = boost::posix_time::microsec_clock::local_time();

    if (boost::posix_time::microsec_clock::local_time() - last_live_cleanup_time_ > boost::posix_time::seconds(live_cleanup_time_secs_))
    {
        loginf << "live cleanup time reached, doing cleanup";
        cleanupDBInternal();
    }
}

/**
 */
void DBInterface::clearTableContent(const string& table_name)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        // DELETE FROM tablename;
        execute("DELETE FROM " + table_name + ";");
    }
}

/**
 */
ResultT<std::shared_ptr<Buffer>> DBInterface::select(const std::string& table_name, 
                                                     const PropertyList& properties,
                                                     const std::string& filter)
{
    if (!existsTable(table_name))
        return ResultT<std::shared_ptr<Buffer>>::failed("Table does not exist");

    #ifdef PROTECT_INSTANCE
    boost::mutex::scoped_lock locker(instance_mutex_);
    #endif

    std::shared_ptr<Buffer> buffer;

    try
    {
        auto cmd = sqlGenerator().getSelectCommand(table_name, properties, filter);
        auto result = execute(*cmd);
        if (result->hasError() || !result->containsData() || !result->buffer())
            throw std::runtime_error("Could not obtain results table");

        buffer = result->buffer();
    }
    catch(const std::exception& ex)
    {
        logerr << "could not select data: " << ex.what();
        return ResultT<std::shared_ptr<Buffer>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not select data: Unknown error";
        return ResultT<std::shared_ptr<Buffer>>::failed("Unknown error");
    }

    return ResultT<std::shared_ptr<Buffer>>::succeeded(buffer);
}

/**
 */
void DBInterface::createPropertiesTable()
{
    traced_assert(ready());
    traced_assert(!existsPropertiesTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTablePropertiesCreateStatement());
        updateTableInfo();
    }
}

/**
 */
void DBInterface::startPerformanceMetrics() const
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        db_instance_->defaultConnection().startPerformanceMetrics();
    }
}

/**
 */
db::PerformanceMetrics DBInterface::stopPerformanceMetrics() const
{
    traced_assert(ready());

    db::PerformanceMetrics pm;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        pm = db_instance_->defaultConnection().stopPerformanceMetrics();
    }

    return pm;
}

/**
 */
bool DBInterface::hasActivePerformanceMetrics() const
{
    if (!ready())
        return false;

    bool ok;
    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        ok = db_instance_->defaultConnection().hasActivePerformanceMetrics();
    }

    return ok;
}

/**
 */
std::string DBInterface::dbInfo()
{
    if (!ready())
        return "db not ready";

    return db_instance_->dbInfo();
}

