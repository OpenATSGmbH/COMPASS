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

#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/metavariable.h"
#include "dbcontent/target/target.h"

#include "sector.h"
#include "sectorlayer.h"
#include "source/dbdatasource.h"
#include "fft/dbfft.h"

#include "task/taskmanager.h"
#include "task/result/taskresult.h"
#include "task/result/report/section.h"
#include "task/result/report/sectioncontent.h"
#include "task/result/report/sectioncontentfigure.h"
#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/sectioncontenttext.h"

#include "eval/results/evaluationtaskresult.h"

#include "viewpoint.h"

#include "compass.h"
#include "buffer.h"
#include "config.h"
#include "files.h"
#include "timeconv.h"
#include "number.h"
#include "asynctask.h"

#include "tbbhack.h"

#include <QApplication>
#include <QMessageBox>
#include <QMutexLocker>
#include <QThread>
#include <QElapsedTimer>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/path.hpp>

#include <fstream>

using namespace Utils;
using namespace std;
using namespace dbContent;

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
DBInterface::DBInterface(string class_id, 
                         string instance_id, 
                         COMPASS* compass)
:   Configurable(class_id, instance_id, compass)
,   insert_mt_(true)
{
    registerParameter("read_chunk_size", &read_chunk_size_, 50000u);

    createSubConfigurables();
}

/**
 */
DBInterface::~DBInterface()
{
    logdbg;

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
        COMPASS::instance().logInfo("DBInterface") << "database '" << filename << "' created";
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

            setProperty("APP_VERSION", COMPASS::instance().config().getString("version"));
            saveProperties();

            traced_assert(!existsDataSourcesTable());
            createDataSourcesTable();

            traced_assert(!existsSectorsTable());
            createSectorsTable();
        }
        else
        {
            traced_assert(existsPropertiesTable());
            loadProperties();

            //check db version against app version
            if (!hasProperty("APP_VERSION")
                    || getProperty("APP_VERSION") != COMPASS::instance().config().getString("version"))
            {
                string reason = hasProperty("APP_VERSION") ?
                            "DB from Version " + getProperty("APP_VERSION") + ", current "
                            + COMPASS::instance().config().getString("version") : "DB from Version older than 0.7.0";

                properties_loaded_ = false;
                properties_.clear();
                db_instance_->close();

                logerr << "Incorrect application version for database:\n " << reason;
                throw std::runtime_error ("Incorrect application version for database:\n "+reason);
            }

            traced_assert(existsDataSourcesTable());
            traced_assert(existsSectorsTable());
        }

        if (!existsTargetsTable())
            createTargetsTable();

        if (!existsTaskLogTable())
            createTaskLogTable();

        //determine maximum report content id
        auto max_id = getMaxReportContentID();
        ResultReport::Section::setCurrentContentID(max_id.has_value() ? max_id.value() + 1 : 0);

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
    loginf << "cleaning db...";

    traced_assert(ready());
    traced_assert(!cleanup_in_progress_);
    traced_assert(!dbInMemory());

    cleanup_in_progress_ = true;

    Result res_cleanup, res_critical;

    try
    {
        auto res_reconnect = db_instance_->reconnect(true, &res_cleanup);

        //reconnection shall never fail
        if (!res_reconnect.ok())
            throw std::runtime_error("Reconnecting to database failed: " + res_reconnect.error());

        if (!res_cleanup.ok())
        {
            //cleanup didn't work => log and return false
            logerr << "Cleanup failed: " << res_cleanup.error();
        }

        res_critical = Result::succeeded();
    }
    catch(const std::exception& ex)
    {
        res_critical = Result::failed(ex.what());
    }

    cleanup_in_progress_ = false;

    if (!res_critical.ok())
    {
        //@TODO: correct way to resolve this worst case?

        reset();

        logerr << "error: " << res_critical.error();
        throw std::runtime_error(res_critical.error());
    }

    return res_cleanup;
}

/**
 */
void DBInterface::generateSubConfigurable(const string& class_id,
                                          const string& instance_id)
{
    throw std::runtime_error("DBInterface: generateSubConfigurable: unknown class_id " + class_id);
}



/**
 * !Protect by mutex when calling!
 */
Result DBInterface::execute(const std::string& sql)
{
    traced_assert(ready());

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
void DBInterface::createTable(const DBContent& object)
{
    loginf << "obj " << object.name();

    traced_assert(ready());

    if (existsTable(object.dbTableName()))
    {
        logerr << "table " << object.dbTableName() << " already exists";
        return;
    }

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string statement = sqlGenerator().getCreateTableStatement(object);

        execute(statement);
        updateTableInfo();
    }

    loginf << "checking " << object.dbTableName();

    traced_assert(existsTable(object.dbTableName()));
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
 */
unsigned long DBInterface::getMaxRecordNumber(DBContent& object)
{
    traced_assert(ready());
    traced_assert(object.existsInDB());

    traced_assert(COMPASS::instance().dbContentManager().existsMetaVariable(DBContent::meta_var_rec_num_.name()));
    traced_assert(COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_rec_num_.name()).existsIn(object.name()));

    Variable& rec_num_var = COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_rec_num_.name()).getFor(object.name());

    traced_assert(object.hasVariable(rec_num_var.name()));

    unsigned long max_rn = 0;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getMaxULongIntValueCommand(object.dbTableName(),
                                                                                  rec_num_var.dbColumnName());
        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        if (!buffer->size())
        {
            logwrn << "no max record number found";
            return 0;
        }

        traced_assert(!buffer->get<unsigned long>(rec_num_var.dbColumnName()).isNull(0));

        max_rn = buffer->get<unsigned long>(rec_num_var.dbColumnName()).get(0);
    }

    return max_rn;
}

/**
 */
unsigned int DBInterface::getMaxRefTrackTrackNum()
{
    traced_assert(ready());

    DBContent& reftraj_content = COMPASS::instance().dbContentManager().dbContent("RefTraj");

    if(!reftraj_content.existsInDB())
        return 0;

    traced_assert(COMPASS::instance().dbContentManager().existsMetaVariable(DBContent::meta_var_track_num_.name()));
    traced_assert(COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_track_num_.name()).existsIn("RefTraj"));

    Variable& track_num_var = COMPASS::instance().dbContentManager().metaVariable(
                DBContent::meta_var_track_num_.name()).getFor("RefTraj");

    traced_assert(reftraj_content.hasVariable(track_num_var.name()));

    unsigned int max_tn = 0;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getMaxUIntValueCommand(reftraj_content.dbTableName(),
                                                                            track_num_var.dbColumnName());
        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        if (!buffer->size())
        {
            logwrn << "no max track number found";
            return 0;
        }

        traced_assert(!buffer->get<unsigned int>(track_num_var.dbColumnName()).isNull(0));

        max_tn = buffer->get<unsigned int>(track_num_var.dbColumnName()).get(0);
    }

    return max_tn;
}

/**
 */
boost::optional<unsigned long> DBInterface::getMaxReportContentID()
{
    traced_assert(ready());

    if (!existsReportContentsTable())
        return boost::optional<unsigned long>();

    unsigned int max_tn = 0;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getMaxUIntValueCommand(ResultReport::SectionContent::DBTableName,
                                                                              ResultReport::SectionContent::DBColumnContentID.name());
        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();
        traced_assert(buffer->size() <= 1);

        if (buffer->size() == 0)
            return boost::optional<unsigned long>();

        traced_assert(!buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name()).isNull(0));

        max_tn = buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name()).get(0);
    }

    return max_tn;
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
bool DBInterface::existsDataSourcesTable()
{
    return existsTable(dbContent::DBDataSource::table_name_);
}

/**
 */
void DBInterface::createDataSourcesTable()
{
    traced_assert(ready());
    traced_assert(!existsDataSourcesTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTableDataSourcesCreateStatement());
        updateTableInfo();
    }
}

/**
 */
std::vector<std::unique_ptr<dbContent::DBDataSource>> DBInterface::getDataSources()
{
    logdbg;

    traced_assert(ready());

    using namespace dbContent;

    std::vector<std::unique_ptr<DBDataSource>> sources;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getDataSourcesSelectCommand();

        loginf << "sql '" << command->get() << "'";

        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());
        shared_ptr<Buffer> buffer = result->buffer();

        logdbg << "json '" << buffer->asJSON().dump(4) << "'";

        traced_assert(buffer->properties().hasProperty(DBDataSource::id_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::ds_type_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::sac_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::sic_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::name_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::short_name_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::info_column_));
        traced_assert(buffer->properties().hasProperty(DBDataSource::counts_column_));

        for (unsigned cnt = 0; cnt < buffer->size(); cnt++)
        {
            if (buffer->get<unsigned int>(DBDataSource::id_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL id, will be omitted";
                continue;
            }

            if (buffer->get<string>(DBDataSource::ds_type_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL content type, will be omitted";
                continue;
            }

            if (buffer->get<unsigned int>(DBDataSource::sac_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL sac, will be omitted";
                continue;
            }

            if (buffer->get<unsigned int>(DBDataSource::sic_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL sic, will be omitted";
                continue;
            }

            if (buffer->get<string>(DBDataSource::name_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL name, will be omitted";
                continue;
            }

            std::unique_ptr<DBDataSource> src {new DBDataSource()};

            src->id(buffer->get<unsigned int>(DBDataSource::id_column_.name()).get(cnt));
            src->dsType(buffer->get<string>(DBDataSource::ds_type_column_.name()).get(cnt));
            src->sac(buffer->get<unsigned int>(DBDataSource::sac_column_.name()).get(cnt));
            src->sic(buffer->get<unsigned int>(DBDataSource::sic_column_.name()).get(cnt));
            src->name(buffer->get<string>(DBDataSource::name_column_.name()).get(cnt));

            if (!buffer->get<string>(DBDataSource::short_name_.name()).isNull(cnt))
                src->shortName(buffer->get<string>(DBDataSource::short_name_.name()).get(cnt));

            if (!buffer->get<string>(DBDataSource::info_column_.name()).isNull(cnt))
                src->info(buffer->get<string>(DBDataSource::info_column_.name()).get(cnt));

            if (!buffer->get<string>(DBDataSource::counts_column_.name()).isNull(cnt))
                src->counts(buffer->get<string>(DBDataSource::counts_column_.name()).get(cnt));

            sources.emplace_back(move(src));
        }
    }

    return sources;
}

/**
 */
void DBInterface::saveDataSources(const std::vector<std::unique_ptr<dbContent::DBDataSource>>& data_sources)
{
    loginf << "num " << data_sources.size();

    using namespace dbContent;

    traced_assert(ready());

    clearTableContent(DBDataSource::table_name_);

    PropertyList list;
    list.addProperty(DBDataSource::id_column_);
    list.addProperty(DBDataSource::ds_type_column_);
    list.addProperty(DBDataSource::sac_column_);
    list.addProperty(DBDataSource::sic_column_);
    list.addProperty(DBDataSource::name_column_);
    list.addProperty(DBDataSource::short_name_);
    list.addProperty(DBDataSource::info_column_);
    list.addProperty(DBDataSource::counts_column_);

    shared_ptr<Buffer> buffer = make_shared<Buffer>(list);

    unsigned int cnt = 0;
    for (auto& ds_it : data_sources)
    {
        buffer->get<unsigned int>(DBDataSource::id_column_.name()).set(cnt, ds_it->id());
        buffer->get<string>(DBDataSource::ds_type_column_.name()).set(cnt, ds_it->dsType());
        buffer->get<unsigned int>(DBDataSource::sac_column_.name()).set(cnt, ds_it->sac());
        buffer->get<unsigned int>(DBDataSource::sic_column_.name()).set(cnt, ds_it->sic());
        buffer->get<string>(DBDataSource::name_column_.name()).set(cnt, ds_it->name());

        if (ds_it->hasShortName())
            buffer->get<string>(DBDataSource::short_name_.name()).set(cnt, ds_it->shortName());

        buffer->get<string>(DBDataSource::info_column_.name()).set(cnt, ds_it->infoStr());
        buffer->get<string>(DBDataSource::counts_column_.name()).set(cnt, ds_it->countsStr());

        ++cnt;
    }

    logdbg << "buffer size " << buffer->size();

    insertBuffer(DBDataSource::table_name_, buffer);

    logdbg << "done";
}

/**
 */
bool DBInterface::existsFFTsTable()
{
    return existsTable(DBFFT::table_name_);
}

/**
 */
void DBInterface::createFFTsTable()
{
    traced_assert(ready());
    traced_assert(!existsFFTsTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTableFFTsCreateStatement());
        updateTableInfo();
    }
}

/**
 */
std::vector<std::unique_ptr<DBFFT>> DBInterface::getFFTs()
{
    logdbg;

    traced_assert(ready());

    using namespace dbContent;

    std::vector<std::unique_ptr<DBFFT>> sources;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        shared_ptr<DBCommand> command = sqlGenerator().getFFTSelectCommand();

        logdbg << "sql '" << command->get() << "'";

        shared_ptr<DBResult> result = execute(*command);
        traced_assert(result->containsData());
        shared_ptr<Buffer> buffer = result->buffer();

        logdbg << "json '" << buffer->asJSON().dump(4) << "'";

        traced_assert(buffer->properties().hasProperty(DBFFT::name_column_));
        traced_assert(buffer->properties().hasProperty(DBFFT::info_column_));

        for (unsigned cnt = 0; cnt < buffer->size(); cnt++)
        {
            if (buffer->get<string>(DBFFT::name_column_.name()).isNull(cnt))
            {
                logerr << "data source cnt " << cnt
                    << " has NULL name, will be omitted";
                continue;
            }

            std::unique_ptr<DBFFT> src {new DBFFT()};

            src->name(buffer->get<string>(DBFFT::name_column_.name()).get(cnt));

            if (!buffer->get<string>(DBFFT::info_column_.name()).isNull(cnt))
                src->info(nlohmann::json::parse(buffer->get<string>(DBFFT::info_column_.name()).get(cnt)));

            sources.emplace_back(std::move(src));
        }
    }

    return sources;
}

/**
 */
void DBInterface::saveFFTs(const std::vector<std::unique_ptr<DBFFT>>& ffts)
{
    logdbg << "num " << ffts.size();

    using namespace dbContent;

    traced_assert(ready());

    if (existsFFTsTable())
        clearTableContent(DBFFT::table_name_);
    else
        createFFTsTable();

    PropertyList list;
    list.addProperty(DBFFT::name_column_);
    list.addProperty(DBFFT::info_column_);

    shared_ptr<Buffer> buffer = make_shared<Buffer>(list);

    unsigned int cnt = 0;
    for (auto& ds_it : ffts)
    {
        buffer->get<string>(DBFFT::name_column_.name()).set(cnt, ds_it->name());
        buffer->get<string>(DBFFT::info_column_.name()).set(cnt, ds_it->infoStr());

        ++cnt;
    }

    logdbg << "buffer size " << buffer->size();

    insertBuffer(DBFFT::table_name_, buffer);

    logdbg << "done";
}

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
    properties_[id] = value;
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
            loginf << "id '" << prop_it.first << "' value '"
                << prop_it.second << "'";

        // column with content
        if (properties_.count(dbcolumn_content_property_name_))
        {
            nlohmann::json dbcolumn_content_json = nlohmann::json::parse(properties_.at(dbcolumn_content_property_name_));
            dbcolumn_content_flags_ = dbcolumn_content_json.get<std::map<std::string, std::set<std::string>>>();
        }

        properties_loaded_ = true;
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
        dbcolumn_content_flags_[table_name].insert(column_name);
}

/**
 */
void DBInterface::saveProperties()
{
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

            loginf << "loaded sector '" << name << "' in layer '"
                << layer_name << "' num points " << (*lay_it)->sector(name)->size();
        }
    }

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

    setProperty("view_points_version", ViewPoint::VP_COLLECTION_CONTENT_VERSION);

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
void DBInterface::setViewPoint(const unsigned int id, const string& value)
{
    if (!db_instance_)
    {
        logwrn << "failed since no database instance exists";
        return;
    }

    if (!existsViewPointsTable())
        createViewPointsTable();

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string str = sqlGenerator().getInsertViewPointStatement(id, value);
        logdbg << "cmd '" << str << "'";

        execute(str);
    }
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

/**
 */
bool DBInterface::existsTargetsTable()
{
    return existsTable(TABLE_NAME_TARGETS);
}

/**
 */
void DBInterface::createTargetsTable()
{
    loginf;

    traced_assert(ready());
    traced_assert(!existsTargetsTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        auto sql = sqlGenerator().getTableTargetsCreateStatement();

        loginf << "sql '" << sql << "'";

        execute(sql);
        updateTableInfo();
    }
}

/**
 */
void DBInterface::clearTargetsTable()
{
    clearTableContent(TABLE_NAME_TARGETS);
}

/**
 */
std::vector<std::unique_ptr<dbContent::Target>> DBInterface::loadTargets()
{
    loginf;

    traced_assert(ready());

    std::vector<std::unique_ptr<dbContent::Target>> targets;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        DBCommand command;
        command.set(sqlGenerator().getSelectAllTargetsStatement());

        PropertyList list;
        list.addProperty("utn", PropertyDataType::UINT);
        list.addProperty("json", PropertyDataType::STRING);
        command.list(list);

        shared_ptr<DBResult> result = execute(command);
        traced_assert(!result->hasError());
        traced_assert(result->containsData());

        shared_ptr<Buffer> buffer = result->buffer();

        traced_assert(buffer);
        traced_assert(buffer->has<unsigned int>("utn"));
        traced_assert(buffer->has<string>("json"));

        NullableVector<unsigned int>& utn_vec = buffer->get<unsigned int>("utn");
        NullableVector<string>& json_vec = buffer->get<string>("json");

        unsigned int utn;
        string json_str;

        std::set<unsigned int> existing_utns;

        for (size_t cnt = 0; cnt < buffer->size(); ++cnt)
        {
            traced_assert(!utn_vec.isNull(cnt));
            traced_assert(!json_vec.isNull(cnt));

            utn = utn_vec.get(cnt);
            json_str = json_vec.get(cnt);

            traced_assert(!existing_utns.count(utn));
            existing_utns.insert(utn);

            //shared_ptr<dbContent::Target> target = make_shared<dbContent::Target>(utn, nlohmann::json::parse(json_str));

            targets.emplace_back(new dbContent::Target(utn, nlohmann::json::parse(json_str)));

            //        loginf << "loaded target " << utn << " json '"
            //               << json_str << "'";
        }
    }

    return targets;
}

/**
 */
void DBInterface::saveTargets(const std::map<unsigned int, nlohmann::json>& targets_info)
{
    logdbg;

    traced_assert(ready());

    clearTargetsTable();

    {
        //storing all targets at once via a buffer is faster
        std::shared_ptr<Buffer> buffer(new Buffer(dbContent::Target::DBPropertyList));

        auto& id_vec   = buffer->get<unsigned int>(dbContent::Target::DBColumnID.name());
        auto& info_vec = buffer->get<nlohmann::json>(dbContent::Target::DBColumnInfo.name());

        size_t idx = 0;
        for (auto& tgt_it : targets_info)
        {
            id_vec.set(idx, tgt_it.first);
            info_vec.set(idx, tgt_it.second);
            ++idx;
        }

        insertBuffer(TABLE_NAME_TARGETS, buffer);
    }

    logdbg << "done";
}

/**
 */
void DBInterface::updateTargets(const std::map<unsigned int, nlohmann::json>& targets_info)
{
    loginf << "updating " << targets_info.size() << " utn(s)";

    traced_assert(ready());

    if (targets_info.size() > TableBulkUpdateMinRows)
    {
        //updating many targets at once via a buffer is faster
        std::shared_ptr<Buffer> buffer(new Buffer(dbContent::Target::DBPropertyList));

        auto& id_vec   = buffer->get<unsigned int>(dbContent::Target::DBColumnID.name());
        auto& info_vec = buffer->get<nlohmann::json>(dbContent::Target::DBColumnInfo.name());

        size_t idx = 0;
        for (auto& tgt_it : targets_info)
        {
            id_vec.set(idx, tgt_it.first);
            info_vec.set(idx, tgt_it.second);
            ++idx;
        }

        updateBuffer(TABLE_NAME_TARGETS, Target::DBColumnID.name(), buffer);
    }
    else
    {
        //run individual update for each target
        for (auto& tgt_it : targets_info)
        {
            #ifdef PROTECT_INSTANCE
            boost::mutex::scoped_lock locker(instance_mutex_);
            #endif

            string str = sqlGenerator().getInsertTargetStatement(tgt_it.first, tgt_it.second.dump());

            // uses replace with utn as unique key
            execute(str);
        }
    }

    loginf << "done";
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

void DBInterface::saveTaskLogInfo(unsigned int msg_id, const nlohmann::json& info)
{
    traced_assert(ready());
    traced_assert(existsTaskLogTable());

    {
        //storing all targets at once via a buffer is faster
        std::shared_ptr<Buffer> buffer(new Buffer(LogStore::LogEntry::DBPropertyList));

        auto& msg_id_vec   = buffer->get<unsigned int>(LogStore::LogEntry::DBColumnID.name());
        auto& info_vec = buffer->get<nlohmann::json>(LogStore::LogEntry::DBColumnInfo.name());

        msg_id_vec.set(0, msg_id);
        info_vec.set(0, info);

        insertBuffer(TABLE_NAME_TASK_LOG, buffer);
    }
}

/**
 */
void DBInterface::clearAssociations(const DBContent& dbcontent)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string str = sqlGenerator().getSetNullStatement(dbcontent.dbTableName(),
                                                        dbcontent.variable("UTN").dbColumnName());      
        // uses replace with utn as unique key
        execute(str);
    }
}

/**
 */
bool DBInterface::existsTaskResultsTable() const
{
    return existsTable(TaskResult::DBTableName);
}

/**
 */
bool DBInterface::existsReportContentsTable() const
{
    return existsTable(ResultReport::SectionContent::DBTableName);
}

/**
 */
void DBInterface::createTaskResultsTable()
{
    traced_assert(ready());
    traced_assert(!existsTaskResultsTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTableTaskResultsCreateStatement());
        updateTableInfo();
    }
}

/**
 */
void DBInterface::createReportContentsTable()
{
    traced_assert(ready());
    traced_assert(!existsReportContentsTable());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        execute(sqlGenerator().getTableReportContentsCreateStatement());
        updateTableInfo();
    }
}

/**
 */
Result DBInterface::saveResult(const TaskResult& result, bool cleanup_db_if_needed)
{
    traced_assert(ready());

    try
    {
        // removeTable(TaskResult::DBTableName); // UGA TABLEHACK

        //create needed tables
        if (!existsTaskResultsTable())
            createTaskResultsTable();

        if (!existsReportContentsTable())
            createReportContentsTable();
        
        //remove any old result with the same id/name
        bool result_deleted = false;
        auto del_result = deleteResult(result, false, &result_deleted);
        if (!del_result.ok())
            throw std::runtime_error(del_result.error());

        auto result_id   = result.id();
        auto result_name = result.name();

        //write result
        {
            std::shared_ptr<Buffer> buffer(new Buffer(TaskResult::DBPropertyList));

            auto& id_vec      = buffer->get<unsigned int>(TaskResult::DBColumnID.name());
            auto& name_vec    = buffer->get<std::string>(TaskResult::DBColumnName.name());
            auto& header_vec  = buffer->get<nlohmann::json>(TaskResult::DBColumnJSONHeader.name());
            auto& content_vec = buffer->get<nlohmann::json>(TaskResult::DBColumnJSONContent.name());
            auto& type_vec    = buffer->get<int>(TaskResult::DBColumnResultType.name());

            id_vec.set(0, result_id);
            name_vec.set(0, result.name());
            header_vec.set(0, result.header().toJSON());
            content_vec.set(0, result.toJSON());
            type_vec.set(0, (int)result.type());

            insertBuffer(TaskResult::DBTableName, buffer);
        }

        //write contents
        auto report_contents = result.report()->reportContents(true);
        {
            size_t chunk_size_bytes = 1e09;
            size_t current_bytes    = 0;
            size_t current_row      = 0;

            std::shared_ptr<Buffer> buffer;

            NullableVector<unsigned int>*   content_id_vec = nullptr;
            NullableVector<unsigned int>*   result_id_vec  = nullptr;
            NullableVector<int>*            type_vec       = nullptr;
            NullableVector<nlohmann::json>* content_vec    = nullptr;

            for (const auto& c : report_contents)
            {
                if (!buffer || current_bytes > chunk_size_bytes)
                {
                    //insert old buffer if available
                    if (buffer)
                    {
                        //loginf << "writing" << buffer->size() << " row(s)";
                        insertBuffer(ResultReport::SectionContent::DBTableName, buffer);
                    }

                    //create new buffer
                    buffer.reset(new Buffer(ResultReport::SectionContent::DBPropertyList));

                    content_id_vec = &buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name());
                    result_id_vec  = &buffer->get<unsigned int>(ResultReport::SectionContent::DBColumnResultID.name());
                    type_vec       = &buffer->get<int>(ResultReport::SectionContent::DBColumnType.name());
                    content_vec    = &buffer->get<nlohmann::json>(ResultReport::SectionContent::DBColumnJSONContent.name());

                    current_bytes = 0;
                    current_row   = 0;
                }

                traced_assert(buffer);

                //!this might trigger recomputations from temporarily generated data,
                //which is immediately thrown away afterwards!
                auto   c_json  = c->toJSON();
                size_t c_bytes = c_json.dump().size();

                content_id_vec->set(current_row, c->contentID());
                result_id_vec->set(current_row, result_id);
                type_vec->set(current_row, (int)c->contentType());
                content_vec->set(current_row, c_json);

                current_bytes += c_bytes;
                current_row   += 1;
            }

            //insert remaining buffer content
            if (current_row > 0)
            {
                //loginf << "writing" << buffer->size() << " row(s)";
                insertBuffer(ResultReport::SectionContent::DBTableName, buffer);
            }
        }

        //cleanup db?
        if (result_deleted && cleanup_db_if_needed)
            cleanupDB(false);
    }
    catch(const std::exception& ex)
    {
        logerr << "could not store result: " << ex.what();
        return Result::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not store result: Unknown error";
        return Result::failed("Unknown error");
    }

    return Result::succeeded();
}

/**
 */
Result DBInterface::deleteResult(const TaskResult& result, 
                                 bool cleanup_db_if_needed,
                                 bool* deleted)
{
    if (deleted)
        *deleted = false;

    auto id = result.id();

    if (!existsTaskResultsTable() || 
        !existsReportContentsTable())
    {
        logerr << "result tables do not exist";
        return Result::failed("Result tables do not exist");
    }

    bool results_deleted = false;

    try
    {
        //get old result(s) which resemble(s) the new one (either in id or name)
        auto sel_filter = db::SQLFilter(TaskResult::DBColumnID.name(), std::to_string(id), db::SQLFilter::ComparisonOp::Is)
            .OR(TaskResult::DBColumnName.name(), "'" + result.name() + "'", db::SQLFilter::ComparisonOp::Is).statement();

        PropertyList properties = PropertyList({ TaskResult::DBColumnID });

        auto sel_result = select(TaskResult::DBTableName, properties, sel_filter);
        if (!sel_result.ok())
            throw std::runtime_error("Locating old result failed");

        //collect id(s) of old result(s)
        std::vector<unsigned int> ids_to_remove;

        size_t n = sel_result.result()->size();
        const auto& id_vector = sel_result.result()->get<unsigned int>(TaskResult::DBColumnID.name());
        for (size_t i = 0; i < n; ++i)
            ids_to_remove.push_back(id_vector.get(i));

        loginf << "deleting " << ids_to_remove.size() << " old result(s)";

        //delete old result(s)
        for (auto old_id : ids_to_remove)
        {
            auto del_content_cmd = sqlGenerator().getDeleteCommand(ResultReport::SectionContent::DBTableName,
                                                                   db::SQLFilter(ResultReport::SectionContent::DBColumnResultID.name(), 
                                                                                 std::to_string(old_id), 
                                                                                 db::SQLFilter::ComparisonOp::Is).statement());
            auto del_result_cmd = sqlGenerator().getDeleteCommand(TaskResult::DBTableName,
                                                                  db::SQLFilter(TaskResult::DBColumnID.name(), 
                                                                                std::to_string(old_id), 
                                                                                db::SQLFilter::ComparisonOp::Is).statement());
            auto result_del_content = execute(*del_content_cmd);
            auto result_del_result  = execute(*del_result_cmd );

            if (result_del_content->hasError())
                throw std::runtime_error(result_del_content->error());
            if (result_del_result->hasError())
                throw std::runtime_error(result_del_result->error());

            results_deleted = true;
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "could not delete result: " << ex.what();
        return Result::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not delete result: Unknown error";
        return Result::failed("Unknown error");
    }

    if (deleted)
        *deleted = results_deleted;

    //cleanup db?
    if (results_deleted && cleanup_db_if_needed)
        cleanupDB(false);

    return Result::succeeded();
}

/**
 */
Result DBInterface::updateResultHeader(const TaskResult& result)
{
    loginf << "updating header of result '" << result.name() << "'";

    traced_assert(ready());

    try
    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string str = sqlGenerator().getUpdateCellStatement(TaskResult::DBTableName,
                                                           TaskResult::DBColumnJSONHeader.name(),
                                                           result.header().toJSON(),
                                                           TaskResult::DBColumnID.name(),
                                                           result.id());
        // uses replace with utn as unique key
        auto res = execute(str);
        if (!res.ok())
            return res;
    }
    catch(const std::exception& ex)
    {
        return Result::failed(std::string(ex.what()));
    }
    catch(...)
    {
        return Result::failed("Unknown error");
    }

    loginf << "done";

    return Result::succeeded();
}

/**
 */
Result DBInterface::updateResultContent(const TaskResult& result)
{
    loginf << "updating content of result '" << result.name() << "'";

    traced_assert(ready());

    try
    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        string str = sqlGenerator().getUpdateCellStatement(TaskResult::DBTableName,
                                                           TaskResult::DBColumnJSONContent.name(),
                                                           result.toJSON(),
                                                           TaskResult::DBColumnID.name(),
                                                           result.id());
        // uses replace with utn as unique key
        auto res = execute(str);
        if (!res.ok())
            return res;
    }
    catch(const std::exception& ex)
    {
        return Result::failed(std::string(ex.what()));
    }
    catch(...)
    {
        return Result::failed("Unknown error");
    }

    loginf << "done";

    return Result::succeeded();
}

/**
 */
ResultT<std::vector<std::shared_ptr<TaskResult>>> DBInterface::loadResults()
{
    traced_assert(ready());

    std::vector<std::shared_ptr<TaskResult>> results;

    //no results stored yet?
    if (!existsTaskResultsTable())
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::succeeded(results);

    try
    {
        auto cmd = sqlGenerator().getSelectCommand(TaskResult::DBTableName, TaskResult::DBPropertyList, "");
        auto result = execute(*cmd);
        if (result->hasError() || !result->containsData() || !result->buffer())
            throw std::runtime_error("Could not obtain results table");

        auto b = result->buffer();

        //@TODO: check more tables obtained this way for having the right properties
        if (!b->hasProperty(TaskResult::DBColumnID) ||
            !b->hasProperty(TaskResult::DBColumnName) ||
            !b->hasProperty(TaskResult::DBColumnJSONHeader) ||
            !b->hasProperty(TaskResult::DBColumnJSONContent) ||
            !b->hasProperty(TaskResult::DBColumnResultType))
            throw std::runtime_error("Results table invalid");

        size_t nr = b->size();

        auto& id_vec      = b->get<unsigned int>(TaskResult::DBColumnID.name());
        auto& name_vec    = b->get<std::string>(TaskResult::DBColumnName.name());
        auto& header_vec  = b->get<nlohmann::json>(TaskResult::DBColumnJSONHeader.name());
        auto& content_vec = b->get<nlohmann::json>(TaskResult::DBColumnJSONContent.name());
        auto& type_vec    = b->get<int>(TaskResult::DBColumnResultType.name());

        results.resize(nr);

        auto& task_man = COMPASS::instance().taskManager();

        for (size_t i = 0; i < nr; ++i)
        {
            const auto& result_name  = name_vec.get(i);
            const auto& result_type  = type_vec.get(i);
            const auto& result_id    = id_vec.get(i);
            const auto& json_header  = header_vec.get(i);
            const auto& json_content = content_vec.get(i);

            //create result of given type
            results[ i ] = task_man.createResult(result_id, (task::TaskResultType)result_type);
            if (!results[ i ])
                throw std::runtime_error("Could not create result from type");

            //try to read result header
            TaskResultHeader header;
            if (!header.fromJSON(json_header))
                throw std::runtime_error("Could not read header of result '" + result_name + "'");

            //try to read result
            if (!results[ i ]->fromJSON(json_content))
                throw std::runtime_error("Could not read content of result '" + result_name + "'");

            //configure result using header information (update state etc.)
            results[ i ]->configure(header);

            //final checks
            if (results[ i ]->name() != result_name ||
                results[ i ]->type() != result_type ||
                results[ i ]->id()   != result_id)
                throw std::runtime_error("Result '" + result_name + "' obtains invalid content");
        }
    }
    catch(const std::exception& ex)
    {
        logerr << "could not load results: " << ex.what();
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not load results: Unknown error";
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed("Unknown error");
    }

    return ResultT<std::vector<std::shared_ptr<TaskResult>>>::succeeded(results);
}

/**
 */
ResultT<std::shared_ptr<ResultReport::SectionContent>> DBInterface::loadContent(ResultReport::Section* section, 
                                                                                unsigned int content_id)
{
    traced_assert(ready());

    std::shared_ptr<ResultReport::SectionContent> content;

    loginf << "loading content id " << content_id << "...";

    try
    {
        PropertyList properties;
        properties.addProperty(ResultReport::SectionContent::DBColumnContentID);
        properties.addProperty(ResultReport::SectionContent::DBColumnResultID);
        properties.addProperty(ResultReport::SectionContent::DBColumnType);
        properties.addProperty(ResultReport::SectionContent::DBColumnJSONContent);

        auto filter_col = ResultReport::SectionContent::DBColumnContentID.name();

        auto cmd = sqlGenerator().getSelectCommand(ResultReport::SectionContent::DBTableName, 
                                                   ResultReport::SectionContent::DBPropertyList,
                                                   db::SQLFilter(filter_col, std::to_string(content_id), db::SQLFilter::ComparisonOp::Is).statement());
        auto result = execute(*cmd);

        if (result->hasError() || !result->containsData() || !result->buffer())
            throw std::runtime_error("Could not obtain content from table");

        auto b = result->buffer();

        size_t nr = b->size();
        if (nr == 0)
            throw std::runtime_error("Content id not found in table");
        else if (nr > 1)
            throw std::runtime_error("Multiple content ids found in table");

        auto& content_id_vec = b->get<unsigned int>(ResultReport::SectionContent::DBColumnContentID.name());
        auto& type_vec       = b->get<int>(ResultReport::SectionContent::DBColumnType.name());
        auto& content_vec    = b->get<nlohmann::json>(ResultReport::SectionContent::DBColumnJSONContent.name());

        ResultReport::SectionContent::ContentType type = (ResultReport::SectionContent::ContentType)type_vec.get(0);

        //create empty content depending on type
        //@TODO: small factory?
        if (type == ResultReport::SectionContent::ContentType::Figure)
        {
            content.reset(new ResultReport::SectionContentFigure(section));
        }
        else if (type == ResultReport::SectionContent::ContentType::Table)
        {
            content.reset(new ResultReport::SectionContentTable(section));
        }
        else if (type == ResultReport::SectionContent::ContentType::Text)
        {
            content.reset(new ResultReport::SectionContentText(section));
        }
        else
        {
            throw std::runtime_error("Invalid content type");
        }

        //read content
        bool ok = content->fromJSON(content_vec.get(0));
        if (!ok)
            throw std::runtime_error("Could not read content from JSON");

        //check content
        if (content->contentID() != content_id_vec.get(0) ||
            content->contentType() != type)
            throw std::runtime_error("contents invalid");
    }
    catch(const std::exception& ex)
    {
        logerr << "Could not load content: " << ex.what();
        return ResultT<std::shared_ptr<ResultReport::SectionContent>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "Could not load content: Unknown error";
        return ResultT<std::shared_ptr<ResultReport::SectionContent>>::failed("Unknown error");
    }

    loginf << "loaded";

    return ResultT<std::shared_ptr<ResultReport::SectionContent>>::succeeded(content);
}

/**
 */
void DBInterface::initDBContentBuffer(DBContent& dbcontent, 
                                      std::shared_ptr<Buffer> buffer)
{
    // create record numbers & and store new max rec num
    traced_assert(dbcontent.hasVariable(DBContent::meta_var_rec_num_.name()));

    Variable& rec_num_var = dbcontent.variable(DBContent::meta_var_rec_num_.name());
    traced_assert(rec_num_var.dataType() == PropertyDataType::ULONGINT);

    string rec_num_col_str = rec_num_var.dbColumnName();
    traced_assert(!buffer->has<unsigned long>(rec_num_col_str));

    buffer->addProperty(rec_num_col_str, PropertyDataType::ULONGINT);

    traced_assert(COMPASS::instance().dbContentManager().hasMaxRecordNumberWODBContentID());
    unsigned long max_rec_num = COMPASS::instance().dbContentManager().maxRecordNumberWODBContentID();

    NullableVector<unsigned long>& rec_num_vec = buffer->get<unsigned long>(rec_num_col_str);

    unsigned int buffer_size = buffer->size();
    unsigned int dbcont_id = dbcontent.id();

    for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
    {
        ++max_rec_num;
        rec_num_vec.set(cnt, Number::recNumAddDBContId(max_rec_num, dbcont_id));
    }

    COMPASS::instance().dbContentManager().maxRecordNumberWODBContentID(max_rec_num);
}

/**
 */
void DBInterface::insertDBContent(DBContent& dbcontent, std::shared_ptr<Buffer> buffer)
{
    logdbg << "dbcont " << dbcontent.name() << " buffer size " << buffer->size();

    traced_assert(ready());
    traced_assert(buffer);

    // create table if required
    if (!existsTable(dbcontent.dbTableName()))
        createTable(dbcontent);

    initDBContentBuffer(dbcontent, buffer);
    insertBuffer(dbcontent.dbTableName(), buffer);
}

/**
 */
void DBInterface::insertBuffer(const string& table_name, 
                               shared_ptr<Buffer> buffer)
{
    logdbg << "table name " << table_name << " buffer size " << buffer->size();

    traced_assert(ready());
    traced_assert(buffer);

    if (!existsTable(table_name))
    {
        logerr << "table with name '" << table_name << "' does not exist";
        throw runtime_error("DBInterface: insertBuffer: table with name '" + table_name + "' does not exist");
    }

    Result res;

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        res = db_instance_->defaultConnection().insertBuffer(table_name, buffer);
    }

    if (!res.ok())
    {
        logerr << "inserting into table '" << table_name << "' failed: " << res.error();
        throw runtime_error("DBInterface: insertBuffer: inserting into table '" + table_name + "' failed: " + res.error());
    }
}

/**
 * Inserts multiple dbcontent buffers at once, possibly utilizing parallelization.
 */
void DBInterface::insertDBContent(const std::map<std::string, std::shared_ptr<Buffer>>& buffers)
{
    traced_assert(ready());

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

    auto& dbc_manager = COMPASS::instance().dbContentManager();

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

        auto& dbcontent = dbc_manager.dbContent(it.first);

        //create table if needed
        if (!existsTable(dbcontent.dbTableName()))
            createTable(dbcontent);

        //init buffer
        initDBContentBuffer(dbcontent, it.second);

        if (chunk_size < 1 || !exec_mt)
        {
            insert_jobs.push_back(Job(idx++, dbcontent.dbTableName(), it.second, {}, {}));
        }
        else
        {
            auto n = it.second->size();
            for (size_t idx = 0; idx < n; idx += (size_t)chunk_size)
            {
                size_t idx_start = idx;
                size_t idx_end   = std::min(idx_start + chunk_size, n);

                traced_assert(idx_start < idx_end);

                insert_jobs.push_back(Job(idx++, dbcontent.dbTableName(), it.second, idx_start, idx_end - 1));
            }
        }
    }

    unsigned int n = insert_jobs.size();

    logdbg << "created " << n << " job(s)";

    //create connections for multithreading (if supported)
    std::vector<DBConnectionWrapper> mt_connections;
    if (db_supports_mt)
    {
        mt_connections.resize(n);

        for (unsigned int i = 0; i < n; ++i)
        {
            mt_connections[ i ] = db_instance_->concurrentConnection(i);
            traced_assert(!mt_connections[ i ].isEmpty());

            if (mt_connections[ i ].hasError())
            {
                logerr << "creating mt connection failed: " << mt_connections[ i ].error();
                throw runtime_error("DBInterface: insertDBContent: creating mt connection failed: " + mt_connections[ i ].error());
            }
        }
    }

    //insert buffers
    std::vector<Result> results(n, 0);
    if (exec_mt)
    {
        //insert multithreaded (if supported)
        tbb::parallel_for(uint(0), n, [ & ](unsigned int i) 
        {
            auto& job = insert_jobs.at(i);
            results.at(i) = mt_connections.at(i).connection().insertBuffer(job.table, job.buffer, job.idx_from, job.idx_to);
        });
    }
    else
    {
        //single threaded insert
        for (unsigned int i = 0; i < n; ++i)
        {
            auto& job = insert_jobs.at(i);
            results.at(i) = db_instance_->defaultConnection().insertBuffer(job.table, job.buffer, job.idx_from, job.idx_to);
        }
    }

    //check results
    for (unsigned int i = 0; i < n; ++i)
    {
        const auto& table_name = insert_jobs.at(i).table;
        const auto& result     = results.at(i);
        if (!result.ok())
        {
            logerr << "inserting into table '" << table_name << "' failed: " << result.error();
            throw runtime_error("DBInterface: insertBuffer: inserting into table '" + table_name + "' failed: " + result.error());
        }
    }
}

/**
 */
void DBInterface::updateBuffer(const std::string& table_name, 
                               const std::string& key_col,
                               shared_ptr<Buffer> buffer, 
                               int from_index, 
                               int to_index)
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

        res = db_instance_->defaultConnection().updateBuffer(table_name, buffer, key_col, from_index, to_index);
    }

    if (!res.ok())
    {
        logerr << "updating table '" << table_name << "' failed: " << res.error();
        throw runtime_error("DBInterface: updateBuffer: updating table '" + table_name + "' failed: " + res.error());
    }
}

/**
 */
void DBInterface::prepareRead(const DBContent& dbcontent, 
                              VariableSet read_list, 
                              string custom_filter_clause,
                              bool use_order, 
                              Variable* order_variable)
{
    logdbg << "dbcont_name " << dbcontent.name();

    traced_assert(ready());
    traced_assert(dbcontent.existsInDB());

    shared_ptr<DBCommand> read = sqlGenerator().getSelectCommand(
        dbcontent, read_list, custom_filter_clause, use_order, order_variable);

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
        logerr << "preparing read for dbcontent '" << dbcontent.name() << "' command '" << read->get() << "' failed: " << res.error();
        throw runtime_error("DBInterface: prepareRead: preparing read for dbcontent '" + dbcontent.name() + "' command '" + read->get() + "'failed: " + res.error());
    }
}

/**
 */
std::pair<std::shared_ptr<Buffer>, bool> DBInterface::readDataChunk(const DBContent& dbcontbject)
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

    buffer->dbContentName(dbcontbject.name());

    traced_assert(buffer);

    return {buffer, last_one};
}

/**
 */
void DBInterface::finalizeReadStatement(const DBContent& dbcontbject)
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
 */
void DBInterface::deleteBefore(const DBContent& dbcontent, 
                               boost::posix_time::ptime before_timestamp)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        std::shared_ptr<DBCommand> command = sqlGenerator().getDeleteCommand(dbcontent, before_timestamp);
        execute(*command.get());
    }
}

/**
 */
void DBInterface::deleteAll(const DBContent& dbcontent)
{
    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        std::shared_ptr<DBCommand> command = sqlGenerator().getDeleteCommand(dbcontent);
        execute(*command.get());
    }
}

/**
 */
void DBInterface::deleteContent(const DBContent& dbcontent, 
                                unsigned int sac, 
                                unsigned int sic)
{
    loginf << "dbcontent " << dbcontent.name() << " sac/sic " << sac << "/" << sic;

    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        std::shared_ptr<DBCommand> command = sqlGenerator().getDeleteCommand(dbcontent, sac, sic);

        execute(*command.get());
    }
}

/**
 */
void DBInterface::deleteContent(const DBContent& dbcontent, unsigned int sac, unsigned int sic, unsigned int line_id)
{
    loginf << "dbcontent " << dbcontent.name() << " sac/sic " << sac << "/" << sic << " line " << line_id;

    traced_assert(ready());

    {
        #ifdef PROTECT_INSTANCE
        boost::mutex::scoped_lock locker(instance_mutex_);
        #endif

        std::shared_ptr<DBCommand> command = sqlGenerator().getDeleteCommand(dbcontent, sac, sic, line_id);

        execute(*command.get());
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
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed(ex.what());
    }
    catch(...)
    {
        logerr << "could not select data: Unknown error";
        return ResultT<std::vector<std::shared_ptr<TaskResult>>>::failed("Unknown error");
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
