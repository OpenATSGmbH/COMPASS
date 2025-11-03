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
#include "dbcontent/variable/variableset.h"
#include "dbdefs.h"
#include "result.h"

#include <QObject>

#include <boost/thread/mutex.hpp>

#include <memory>
#include <set>

class DBInstance;
class DBConnection;
class DBResult;
class DBTableInfo;
class DBCommand;
class SQLGenerator;

class COMPASS;

class Buffer;
class BufferWriter;
class Job;
class Sector;
class SectorLayer;
class Result;
class DBFFT;
class DBContent;

class PropertyList;

namespace dbContent
{
    class DBDataSource;
    class Variable;
    class Target;
}

class TaskResult;

namespace ResultReport
{
    class Section;
    class SectionContent;
}

class QWidget;
class QProgressDialog;

static const std::string TABLE_NAME_PROPERTIES = "properties";
static const std::string TABLE_NAME_SECTORS    = "sectors";
static const std::string TABLE_NAME_VIEWPOINTS = "viewpoints";
static const std::string TABLE_NAME_TARGETS    = "targets";
static const std::string TABLE_NAME_TASK_LOG   = "task_log";

extern const std::string PROP_TIMESTAMP_MIN_NAME;
extern const std::string PROP_TIMESTAMP_MAX_NAME;
extern const std::string PROP_LATITUDE_MIN_NAME;
extern const std::string PROP_LATITUDE_MAX_NAME;
extern const std::string PROP_LONGITUDE_MIN_NAME;
extern const std::string PROP_LONGITUDE_MAX_NAME;

/**
 */
class DBInterface : public QObject, public Configurable
{
    Q_OBJECT

signals:
    //void databaseOpenedSignal();
    //void databaseContentChangedSignal();
    //void databaseClosedSignal();

public:
    DBInterface(std::string class_id, std::string instance_id, COMPASS* compass);
    virtual ~DBInterface();

    virtual void generateSubConfigurable(const std::string& class_id,
                                         const std::string& instance_id);

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

    // data sources
    bool existsDataSourcesTable();
    void createDataSourcesTable();
    std::vector<std::unique_ptr<dbContent::DBDataSource>> getDataSources();
    void saveDataSources(const std::vector<std::unique_ptr<dbContent::DBDataSource>>& data_sources);
    // clears previous and saves new ones

    // ffts
    bool existsFFTsTable();
    void createFFTsTable();
    std::vector<std::unique_ptr<DBFFT>> getFFTs();
    void saveFFTs(const std::vector<std::unique_ptr<DBFFT>>& ffts);
    // clears previous and saves new ones

    // insert data and create associated data sources
    void insertDBContent(DBContent& dbcontent, std::shared_ptr<Buffer> buffer);
    void insertDBContent(const std::map<std::string, std::shared_ptr<Buffer>>& buffers);
    void insertBuffer(const std::string& table_name, std::shared_ptr<Buffer> buffer);
    
    void updateBuffer(const std::string& table_name, const std::string& key_col, std::shared_ptr<Buffer> buffer,
                      int from_index = -1, int to_index = -1);  // no indexes means full buffer

    void prepareRead(const DBContent& dbcontent, dbContent::VariableSet read_list,
                     std::string custom_filter_clause,
                     bool use_order = false, dbContent::Variable* order_variable = nullptr);

    std::pair<std::shared_ptr<Buffer>, bool> readDataChunk(const DBContent& dbcontent); // last one flag
    void finalizeReadStatement(const DBContent& dbcontent);

    void deleteBefore(const DBContent& dbcontent, boost::posix_time::ptime before_timestamp);
    void deleteAll(const DBContent& dbcontent);
    void deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic);
    void deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic, unsigned int line_id);

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
    void createTable(const DBContent& object);
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

    bool existsTargetsTable();
    void createTargetsTable();
    void clearTargetsTable();
    std::vector<std::unique_ptr<dbContent::Target>> loadTargets();

    void saveTargets(const std::map<unsigned int, nlohmann::json>& targets_info);
    void updateTargets(const std::map<unsigned int, nlohmann::json>& targets_info);

    bool existsTaskLogTable();
    void createTaskLogTable();
    std::vector<nlohmann::json> loadTaskLogInfo();
    void saveTaskLogInfo(unsigned int msg_id, const nlohmann::json& info);

    void clearAssociations(const DBContent& dbcontent);

    bool existsTaskResultsTable() const;
    bool existsReportContentsTable() const;
    void createTaskResultsTable();
    void createReportContentsTable();
    Result saveResult(const TaskResult& result, 
                      bool cleanup_db_if_needed);
    Result deleteResult(const TaskResult& result, 
                        bool cleanup_db_if_needed,
                        bool* deleted = nullptr);
    Result updateResultHeader(const TaskResult& result);
    Result updateResultContent(const TaskResult& result);
    ResultT<std::vector<std::shared_ptr<TaskResult>>> loadResults();
    ResultT<std::shared_ptr<ResultReport::SectionContent>> loadContent(ResultReport::Section* section, unsigned int content_id);

    void clearTableContent(const std::string& table_name);
    ResultT<std::shared_ptr<Buffer>> select(const std::string& table_name, 
                                            const PropertyList& properties,
                                            const std::string& filter);

    unsigned long getMaxRecordNumber(DBContent& object);
    unsigned int getMaxRefTrackTrackNum();
    boost::optional<unsigned long> getMaxReportContentID();

    void startPerformanceMetrics() const;
    db::PerformanceMetrics stopPerformanceMetrics() const;
    bool hasActivePerformanceMetrics() const;

    std::string dbInfo();

    //std::map<unsigned int, std::tuple<std::set<unsigned int>, std::tuple<bool, unsigned int, unsigned int>,
    //std::tuple<bool, unsigned int, unsigned int>>> queryADSBInfo();
    // ta -> mops versions, nucp_nics, nac_ps

    static const size_t TableBulkUpdateMinRows;

protected:
    virtual void checkSubConfigurables() override {}

    void openDBFileInternal(const std::string& filename, bool overwrite);

    void loadProperties();
    void reset();

    void recreateConcurrentConnections();

    void initDBContentBuffer(DBContent& dbcontent, 
                             std::shared_ptr<Buffer> buffer);

    SQLGenerator sqlGenerator() const;
    
    Result execute(const std::string& sql);
    std::shared_ptr<DBResult> execute(const DBCommand& cmd);

    void updateTableInfo();
    Result cleanupDBInternal();

    std::unique_ptr<DBInstance> db_instance_;

    bool properties_loaded_ {false};
    const std::string dbcolumn_content_property_name_{"dbcolumn_content"};

    mutable boost::mutex instance_mutex_;

    unsigned int read_chunk_size_;

    std::map<std::string, std::string> properties_;
    std::map<std::string, std::set<std::string>> dbcolumn_content_flags_; // dbtable -> dbcols with content

    bool insert_mt_ = false;

    bool cleanup_in_progress_ = false;
};
