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

#include "configurable.h"
#include "dbdefs.h"
#include "result.h"

#include <QObject>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <future>
#include <vector>

#include <memory>
#include <set>

class DBInstance;
class DBConnection;
class DBResult;
class DBTableInfo;
class DBTableColumnInfo;
class DBCommand;
class SQLGenerator;

class AppState;
class LogStore;

class Buffer;
class BufferWriter;
class Job;
class Sector;
class SectorLayer;
class SectorInsideCache;
class Result;

class PropertyList;

class QWidget;
class QProgressDialog;

static const std::string TABLE_NAME_PROPERTIES = "properties";
static const std::string TABLE_NAME_SECTORS    = "sectors";
static const std::string TABLE_NAME_VIEWPOINTS = "viewpoints";
static const std::string TABLE_NAME_TARGETS    = "targets";
static const std::string TABLE_NAME_TASK_LOG    = "task_log";
static const std::string TABLE_NAME_DB_CONTEXT  = "db_context";
static const std::string TABLE_NAME_DB_INFO     = "db_info";

extern const std::string PROP_TIMESTAMP_MIN_NAME;
extern const std::string PROP_TIMESTAMP_MAX_NAME;
extern const std::string PROP_LATITUDE_MIN_NAME;
extern const std::string PROP_LATITUDE_MAX_NAME;
extern const std::string PROP_LONGITUDE_MIN_NAME;
extern const std::string PROP_LONGITUDE_MAX_NAME;

/**
 * Generic database access (tables, buffers, properties, default tables).
 * The DB content specific access is built on top of it by DBContentDB.
 */
class DBInterface : public QObject, public Configurable
{
    Q_OBJECT

public:
    DBInterface(nlohmann::json& config,
                Configurable& parent,
                AppState& app_state,
                LogStore& log_store,
                const std::string& app_version);
    virtual ~DBInterface();

    bool logVerbose() const;

    void openDBFile(const std::string& filename, bool overwrite);
    void openDBInMemory();
    void openDBFileFromMemory(const std::string& filename);
    void exportDBFile(const std::string& filename);
    void closeDB();

    bool cleanupDB(bool show_dialog = false);
    bool cleanupInProgress() const { return cleanup_in_progress_; }

    const std::map<std::string, DBTableInfo>& tableInfo() const;
    std::string dbFilename() const;
    bool dbInMemory() const;
    bool canCreateDBFileFromMemory() const;

    bool ready() const;

    const DBInstance& dbInstance() const;

    /// sector geometry behind the sector_inside() database function, fed by the Sectors filter.
    /// The function is registered on every database open.
    SectorInsideCache& sectorInsideCache();
    std::shared_ptr<SectorInsideCache> sectorInsideCachePtr() const { return sector_inside_cache_; }

    void insertBuffer(const std::string& table_name, std::shared_ptr<Buffer> buffer);
    void insertBuffers(const std::map<std::string, std::shared_ptr<Buffer>>& buffers); // table name -> buffer

    void updateBuffer(const std::string& table_name, const std::string& key_col, std::shared_ptr<Buffer> buffer,
                      const boost::optional<size_t>& idx_from = boost::none,
                      const boost::optional<size_t>& idx_to = boost::none);  // no indexes means full buffer

    // chunked read of one select statement
    void prepareRead(const std::string& table_name,
                     const PropertyList& properties,
                     const std::string& filter,
                     bool use_order = false,
                     const std::string& order_column = "",
                     const std::string& join_clause = "");
    std::pair<std::shared_ptr<Buffer>, bool> readDataChunk(); // last one flag
    void finalizeReadStatement();

    void liveCleanupIfDue(); // cleanup after live deletions, at most every live_cleanup_time_min

    size_t count(const std::string& table);

    bool existsPropertiesTable();
    void createPropertiesTable();
    void setProperty(const std::string& id, const std::string& value);
    std::string getProperty(const std::string& id);
    void removeProperty(const std::string& id);
    bool hasProperty(const std::string& id);

    bool hasContentIn(const std::string& table_name, const std::string& column_name) const;
    void setContentIn(const std::string& table_name, const std::string& column_name);

    void saveProperties();

    bool existsTable(const std::string& table_name) const;
    void removeTable(const std::string& table_name);

    bool areColumnsNull (const std::string& table_name, const std::vector<std::string> columns);

    bool existsViewPointsTable();
    void createViewPointsTable();
    void setViewPoint(const unsigned int id, const std::string& value);
    std::map<unsigned int, std::string> viewPoints();
    void deleteViewPoint(const unsigned int id);
    void deleteAllViewPoints();

    bool existsSectorsTable();
    void createSectorsTable();
    std::vector<std::shared_ptr<SectorLayer>> loadSectors ();
    void clearSectorsTable();
    void saveSector(std::shared_ptr<Sector> sector); // write to db and add
    void deleteSector(std::shared_ptr<Sector> sector);
    void deleteAllSectors();

    bool existsTaskLogTable();
    void createTaskLogTable();
    std::vector<nlohmann::json> loadTaskLogInfo();
    void saveTaskLogInfo(unsigned int msg_id, const nlohmann::json& info);

    // db_context table - stores the active DBContext per DB
    bool existsDBContextTable();
    void createDBContextTable();
    void saveDBContextSection(const std::string& section, const std::string& json_str);
    std::string loadDBContextSection(const std::string& section);
    std::map<std::string, std::string> loadAllDBContextSections();
    void clearDBContextTable();

    // db_info table - generic key-value metadata (runtime state, etc.)
    bool existsDBInfoTable();
    void createDBInfoTable();
    void saveDBInfo(const std::string& name, const std::string& json_str);
    std::string loadDBInfo(const std::string& name);
    bool hasDBInfo(const std::string& name);
    void removeDBInfo(const std::string& name);

    // generic table operations, e.g. for the task result tables (see TaskResultDB)
    void createTable(const std::string& table_name,
                     const PropertyList& properties,
                     int primary_key = -1);
    void createTable(const std::string& table_name,
                     const std::vector<DBTableColumnInfo>& column_infos,
                     const std::vector<db::Index>& indices);
    Result deleteRows(const std::string& table_name, const std::string& filter);
    void setColumnNull(const std::string& table_name, const std::string& col_name);
    void replaceRow(const std::string& table_name, const std::vector<std::string>& values); // insert or replace by key
    Result updateCell(const std::string& table_name,
                      const std::string& col_name,
                      const nlohmann::json& col_value,
                      const std::string& key_col_name,
                      const nlohmann::json& key_col_value);

    // report tables, see reporttable.h
    void createReportTable(const std::string& table_name,
                           const PropertyList& properties,
                           const std::string& key_column);
    void removeReportTable(const std::string& table_name);
    std::set<std::string> reportTableNames() const;
    std::set<std::string> reportTableNames(unsigned int result_id) const;
    size_t removeReportTables(unsigned int result_id,
                              const std::set<std::string>& keep = std::set<std::string>());
    size_t removeOrphanReportTables(const std::set<std::string>& referenced_tables);
    ResultT<size_t> tableStorageSize(const std::string& table_name);

    void clearTableContent(const std::string& table_name);
    ResultT<std::shared_ptr<Buffer>> select(const std::string& table_name, 
                                            const PropertyList& properties,
                                            const std::string& filter);

    boost::optional<unsigned long> getMaxUIntValue(const std::string& table_name,
                                                   const std::string& col_name);  // none if empty or null
    boost::optional<unsigned long> getMaxULongValue(const std::string& table_name,
                                                    const std::string& col_name); // none if empty or null

    void startPerformanceMetrics() const;
    db::PerformanceMetrics stopPerformanceMetrics() const;
    bool hasActivePerformanceMetrics() const;

    Result validateSQL(const std::string& sql);
    Result validateFilter(const std::string& table_name, const std::string& where_clause);

    std::string dbInfo();

    //std::map<unsigned int, std::tuple<std::set<unsigned int>, std::tuple<bool, unsigned int, unsigned int>,
    //std::tuple<bool, unsigned int, unsigned int>>> queryADSBInfo();
    // ta -> mops versions, nucp_nics, nac_ps

    static const size_t TableBulkUpdateMinRows;

    bool useLiveInMemDB() const { return use_live_inmem_db_; }
    unsigned int maxRAMFileGB() const { return max_ram_file_gb_; }
    unsigned int maxRAMinMemGB() const { return max_ram_inmem_gb_; }
    unsigned int numThreads() const { return num_threads_; }
    bool preserveInsertOrder() const { return preserve_insert_order_; }

protected:
    void insertBufferInternal(const std::string& table_name, std::shared_ptr<Buffer> buffer);
    void saveTaskLogInfoInternal(unsigned int msg_id, const nlohmann::json& info);
    void waitForPendingTaskLogWrites();

    AppState&   app_state_;
    LogStore&   log_store_;
    std::string app_version_; // checked against the APP_VERSION property of a database

    bool log_verbose_ {false};

    unsigned int max_ram_file_gb_{2};
    unsigned int max_ram_inmem_gb_{8};
    unsigned int num_threads_{8};
    unsigned int live_cleanup_time_min_{60};
    bool use_live_inmem_db_{true};

    bool preserve_insert_order_{true};

    boost::posix_time::ptime last_live_cleanup_time_;

    void openDBFileInternal(const std::string& filename, bool overwrite);

    void loadProperties();
    void reset();

    void recreateConcurrentConnections();

    SQLGenerator sqlGenerator() const;
    
    Result execute(const std::string& sql);
    std::shared_ptr<DBResult> execute(const DBCommand& cmd);

    void updateTableInfo();
    Result cleanupDBInternal();

    std::unique_ptr<DBInstance> db_instance_;

    std::shared_ptr<SectorInsideCache> sector_inside_cache_;

    bool properties_loaded_ {false};
    const std::string dbcolumn_content_property_name_{"dbcolumn_content"};

    mutable boost::mutex instance_mutex_;

    // task log writes deferred while instance_mutex_ was busy (live cleanup reconnect)
    std::vector<std::future<void>> pending_task_log_writes_;
    boost::mutex                   pending_task_log_writes_mutex_;

    unsigned int read_chunk_size_;

    std::map<std::string, std::string> properties_;
    bool properties_dirty_ {false}; // in-memory properties differ from db, saveProperties() pending
    std::map<std::string, std::set<std::string>> dbcolumn_content_flags_; // dbtable -> dbcols with content

    bool insert_mt_ = false;

    bool cleanup_in_progress_ = false;
};
