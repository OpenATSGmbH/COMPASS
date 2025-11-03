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

#include "dbcontent/dbcontentmanager.h"
#include "compass.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanagerwidget.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "dbcontent/target/target.h"
#include "dbcontent/target/targetlistwidget.h"
#include "logger.h"
#include "dbcontent/variable/metavariable.h"
#include "datasourcemanager.h"
#include "stringconv.h"
#include "viewmanager.h"
#include "jobmanager.h"
#include "evaluationmanager.h"
#include "filtermanager.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "dbcontent/variable/metavariableconfigurationdialog.h"
#include "dbcontentdeletedbjob.h"
#include "dbcontent_commands.h"
#include "viewpoint.h"
#include "dbcontentinsertdbjob.h"
#include "timeconv.h"

#include "util/tbbhack.h"

#include <QApplication>
#include <QMessageBox>

#include <algorithm>
#include <string>

using namespace std;
using namespace Utils;
using namespace dbContent;

/**
 */
DBContentManager::DBContentManager(const std::string& class_id, const std::string& instance_id,
                                   COMPASS* compass)
    : Configurable(class_id, instance_id, compass, "db_content.json"), compass_(*compass)
{
    logdbg << "creating subconfigurables";

    registerParameter("max_live_data_age_cache", &max_live_data_age_cache_, 5u);
    registerParameter("max_live_data_age_db", &max_live_data_age_db_, 60u);

    createSubConfigurables();

    // check uniqueness of dbcontent ids
    set<unsigned int> dbcont_ids;

    for (auto& object_it : dbcontent_)
    {
        traced_assert(object_it.second->id() < 256);
        traced_assert(dbcont_ids.count(object_it.second->id()) == 0);
        dbcont_ids.insert(object_it.second->id());
    }

    qRegisterMetaType<std::shared_ptr<Buffer>>("std::shared_ptr<Buffer>"); // for dbcont read job
    // for signal about new data
    qRegisterMetaType<std::map<std::string, std::shared_ptr<Buffer>>>("std::map<std::string, std::shared_ptr<Buffer>>");

    dbContent::init_dbcontent_commands();

    traced_assert(!target_model_);
    target_model_.reset(new dbContent::TargetModel(*this));
    traced_assert(target_model_);
}

/**
 */
DBContentManager::~DBContentManager()
{
    logdbg;

    data_.clear();

    for (auto it : dbcontent_)
        delete it.second;
    dbcontent_.clear();

    meta_variables_.clear();

    widget_ = nullptr;

    logdbg << "done";
}

/**
 */
void DBContentManager::generateSubConfigurable(const std::string& class_id,
                                               const std::string& instance_id)
{
    logdbg << "class_id " << class_id << " instance_id "
           << instance_id;

    if (class_id == "DBContent")
    {
        DBContent* object = new DBContent(compass_, class_id, instance_id, this);
        loginf << "adding content " << object->name()
               << " id " << object->id();
        traced_assert(!dbcontent_.count(object->name()));
        traced_assert(!dbcontent_ids_.count(object->id()));

        dbcontent_[object->name()] = object;
        dbcontent_ids_[object->id()] = object;
    }
    else if (class_id == "MetaVariable")
    {
        MetaVariable* meta_var = new MetaVariable(class_id, instance_id, this);
        logdbg << "adding meta var type "
               << meta_var->name();

        traced_assert(!existsMetaVariable(meta_var->name()));
        //meta_variables_.emplace(meta_var->name(), meta_var);
        //meta_variables_.emplace_back(meta_var);

        meta_variables_.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(meta_var->name()),   // args for key
            std::forward_as_tuple(meta_var));  // args for mapped value
    }
    else
        throw std::runtime_error("DBContentManager: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

/**
 */
void DBContentManager::checkSubConfigurables()
{
}

/**
 */
bool DBContentManager::existsDBContent(const std::string& dbcontent_name)
{
    logdbg << "'" << dbcontent_name << "'";

    return (dbcontent_.find(dbcontent_name) != dbcontent_.end());
}

/**
 */
DBContent& DBContentManager::dbContent(const std::string& dbcontent_name)
{
    logdbg << "name " << dbcontent_name;

    traced_assert(dbcontent_.find(dbcontent_name) != dbcontent_.end());

    return *dbcontent_.at(dbcontent_name);
}

/**
 */
void DBContentManager::deleteDBContent(const std::string& dbcontent_name)
{
    logdbg << "name " << dbcontent_name;
    traced_assert(existsDBContent(dbcontent_name));
    delete dbcontent_.at(dbcontent_name);
    dbcontent_.erase(dbcontent_name);

    emit dbObjectsChangedSignal();
}

/**
 */
void DBContentManager::deleteDBContentData(boost::posix_time::ptime before_timestamp)
{
    loginf;

    traced_assert(!delete_job_);

    delete_job_ = make_shared<DBContentDeleteDBJob>(COMPASS::instance().dbInterface());
    delete_job_->setBeforeTimestamp(before_timestamp);

    connect(delete_job_.get(), &DBContentDeleteDBJob::doneSignal, this, &DBContentManager::deleteJobDoneSlot,
            Qt::QueuedConnection);

    JobManager::instance().addDBJob(delete_job_);
}

/**
 */
bool DBContentManager::hasData()
{
    for (auto& object_it : dbcontent_)
        if (object_it.second->hasData())
            return true;

    return false;
}

/**
 */
unsigned int DBContentManager::getMaxDBContentID()
{
    unsigned int ret = 0;

    for (auto& object_it : dbcontent_)
        ret = max(ret, object_it.second->id());

    return ret;
}

/**
 */
bool DBContentManager::existsDBContentWithId (unsigned int id)
{
    return dbcontent_ids_.count(id);
}

/**
 */
const std::string& DBContentManager::dbContentWithId (unsigned int id)
{
    traced_assert(dbcontent_ids_.count(id));
    return dbcontent_ids_.at(id)->name();
}

unsigned int DBContentManager::dbContentId(const std::string& dbcont_name)
{
    assert (existsDBContent(dbcont_name));
    return dbContent(dbcont_name).id();
}

/**
 */
bool DBContentManager::existsMetaVariable(const std::string& var_name)
{
    return meta_variables_.count(var_name);
}

/**
 */
MetaVariable& DBContentManager::metaVariable(const std::string& var_name)
{
    logdbg << "name " << var_name;

    traced_assert(meta_variables_.count(var_name));
    return *(meta_variables_.at(var_name).get());
}

/**
 */
void DBContentManager::renameMetaVariable(const std::string& old_var_name, const std::string& new_var_name)
{
    traced_assert(existsMetaVariable(old_var_name));

    std::unique_ptr<dbContent::MetaVariable> meta_var = std::move(meta_variables_.at(old_var_name));
    meta_variables_.erase(old_var_name);
    meta_var->name(new_var_name);
    meta_variables_.emplace(new_var_name, std::move(meta_var));


    if (meta_cfg_dialog_)
    {
        meta_cfg_dialog_->updateList();
        meta_cfg_dialog_->selectMetaVariable(new_var_name);
    }
}

/**
 */
void DBContentManager::deleteMetaVariable(const std::string& var_name)
{
    logdbg << "name " << var_name;
    traced_assert(existsMetaVariable(var_name));

    meta_variables_.erase(var_name);

    if (meta_cfg_dialog_)
    {
        meta_cfg_dialog_->updateList();
        meta_cfg_dialog_->clearDetails();
    }
}

/**
 */
bool DBContentManager::usedInMetaVariable(const Variable& variable)
{
    for (auto& meta_it : meta_variables_)
        if (meta_it.second->uses(variable))
            return true;

    return false;
}

/**
 */
DBContentManagerWidget* DBContentManager::widget()
{
    if (!widget_)
    {
        widget_.reset(new DBContentManagerWidget(*this));
    }

    traced_assert(widget_);
    return widget_.get();
}

/**
 */
VariableSet DBContentManager::getReadSet(const std::string& dbcontent_name)
{
    EvaluationManager& eval_man = COMPASS::instance().evaluationManager();
    ViewManager& view_man       = COMPASS::instance().viewManager();

    VariableSet read_set = view_man.getReadSet(dbcontent_name);

    // add required vars for processing
    addStandardVariables(dbcontent_name, read_set);

    //label_generator_->addVariables(dbcontent_name, read_set);

    if (eval_man.needsAdditionalVariables())
        eval_man.addVariables(dbcontent_name, read_set);

    return read_set;
}

/**
 */
void DBContentManager::load(const std::string& custom_filter_clause, 
                            bool measure_db_performance,
                            const std::map<std::string, dbContent::VariableSet>* custom_read_set)
{
    loading_done_ = false;

    logdbg << "custom_filter_clause '" << custom_filter_clause << "'";

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    if (load_in_progress_)
    {
        logdbg << "quitting previous load";

        for (auto& object : dbcontent_)
        {
            if (object.second->isLoading())
                object.second->quitLoading();
        }

        while (load_in_progress_) // JobManager::instance().hasDBJobs()
        {
            loginf << "previous load to finish";

            QCoreApplication::processEvents();
            QThread::msleep(1);
        }
    }

    loginf << "starting loading";

    saveSelectedRecNums();
    clearData();

    load_in_progress_ = true;

    bool load_job_created = false;

    DataSourceManager& ds_man =  COMPASS::instance().dataSourceManager();
    DBInterface& db_interface = COMPASS::instance().dbInterface();

    if (measure_db_performance)
        db_interface.startPerformanceMetrics();

    for (auto& object : dbcontent_)
    {
        logdbg << "object " << object.first
               << " loadable " << object.second->loadable()
               << " loading wanted " << ds_man.loadingWanted(object.first)
               << " filters " << COMPASS::instance().filterManager().useFilters();

        if (object.second->loadable() && ds_man.loadingWanted(object.first))
        {
            logdbg << "loading object " << object.first;
            
            auto read_set = getReadSet(object.first);

            if (custom_read_set && custom_read_set->count(object.first))
                read_set.add(custom_read_set->at(object.first));

            if (read_set.getSize() == 0)
            {
                logwrn << "skipping loading of object " << object.first
                       << " since an empty read list was detected";
                continue;
            }

            // load(dbContent::VariableSet& read_set, bool use_datasrc_filters, bool use_filters,
            // const std::string& custom_filter_clause="")
            object.second->load(read_set, true, COMPASS::instance().filterManager().useFilters(),
                                custom_filter_clause);

            load_job_created = true;
        }
    }
    emit loadingStartedSignal();

    if (!load_job_created)
        finishLoading();
}

/**
 */
void DBContentManager::loadBlocking(const std::string& custom_filter_clause, 
                                    bool measure_db_performance,
                                    unsigned int sleep_msecs,
                                    const std::map<std::string, dbContent::VariableSet>* custom_read_set)
{
    load(custom_filter_clause, measure_db_performance, custom_read_set);

    while (!loading_done_)
    {
        QCoreApplication::processEvents();
        QThread::msleep(sleep_msecs);
    }
}

/**
 */
void DBContentManager::addLoadedData(std::map<std::string, std::shared_ptr<Buffer>> data)
{
    logdbg;

    // newest data batch has been finalized, ready to be added

    // add buffers to data

    bool something_changed = false;

    for (auto& buf_it : data)
    {
        if (!buf_it.second->size()) // empty buffer
            continue;

        if (data_.count(buf_it.first))
        {
            logdbg << "adding buffer dbcont " << buf_it.first
                   << " adding size " << buf_it.second->size() << " current size " << data_.at(buf_it.first)->size();

            data_.at(buf_it.first)->seizeBuffer(*buf_it.second.get());

            logdbg << "new buffer dbcont " << buf_it.first
                   << " size " << data_.at(buf_it.first)->size();
        }
        else
        {
            data_[buf_it.first] = std::move(buf_it.second);

            logdbg << "created buffer dbcont " << buf_it.first
                   << " size " << data_.at(buf_it.first)->size();
        }

        something_changed = true;
    }

    if (something_changed)
    {
        updateNumLoadedCounts();
        restoreSelectedRecNums();

        logdbg << "emitting signal";

        emit loadedDataSignal(data_, false);
    }
}

/**
 */
std::map<std::string, std::shared_ptr<Buffer>> DBContentManager::loadedData()
{
    return data_;
}

/**
 */
void DBContentManager::quitLoading()
{
    loginf;

    traced_assert(load_in_progress_);

    for (auto& object : dbcontent_)
    {
        if (object.second->isLoading())
            object.second->quitLoading();
    }

    //load_in_progress_ = true;  // TODO
}

/**
 */
void DBContentManager::databaseOpenedSlot()
{
    loginf;

    loadMaxRecordNumberWODBContentID();
    loadMaxRefTrajTrackNum();

    DBInterface& db_interface = COMPASS::instance().dbInterface();

    if (db_interface.hasProperty("associations_generated"))
    {
        traced_assert(db_interface.hasProperty("associations_id"));

        has_associations_ =
            db_interface.getProperty("associations_generated") == "1";
        associations_id_ = db_interface.getProperty("associations_id");
    }
    else
    {
        has_associations_ = false;
        associations_id_ = "";
    }

    // load min max values
    if (db_interface.hasProperty(PROP_TIMESTAMP_MIN_NAME))
    {
        timestamp_min_ = Time::fromLong(stol(db_interface.getProperty(PROP_TIMESTAMP_MIN_NAME)));
        traced_assert(!timestamp_min_->is_not_a_date_time());
    }
    if (db_interface.hasProperty(PROP_TIMESTAMP_MAX_NAME))
    {
        timestamp_max_ = Time::fromLong(stol(db_interface.getProperty(PROP_TIMESTAMP_MAX_NAME)));
        traced_assert(!timestamp_max_->is_not_a_date_time());
    }

    if (hasMinMaxTimestamp())
        loginf << "timestamp_min_ " << Time::toString(*timestamp_min_)
               << " timestamp_max_ " << Time::toString(*timestamp_max_);
    else
        loginf << "no min/max timestamp";

    if (db_interface.hasProperty(PROP_LATITUDE_MIN_NAME))
        latitude_min_ = stod(db_interface.getProperty(PROP_LATITUDE_MIN_NAME));
    if (db_interface.hasProperty(PROP_LATITUDE_MAX_NAME))
        latitude_max_ = stod(db_interface.getProperty(PROP_LATITUDE_MAX_NAME));

    if (db_interface.hasProperty(PROP_LONGITUDE_MIN_NAME))
        longitude_min_ = stod(db_interface.getProperty(PROP_LONGITUDE_MIN_NAME));
    if (db_interface.hasProperty(PROP_LONGITUDE_MAX_NAME))
        longitude_max_ = stod(db_interface.getProperty(PROP_LONGITUDE_MAX_NAME));

    if (hasMinMaxPosition())
        loginf << "latitude_min_ " << *latitude_min_
               << " latitude_max_ " << *latitude_max_ << " longitude_min_ " << *longitude_min_
               << " longitude_max_ " << *longitude_max_;
    else
        loginf << "no min/max position";

    for (auto& object : dbcontent_)
        object.second->databaseOpenedSlot();

    loadTargets();

    emit associationStatusChangedSignal();
    emit dbContentStatusChanged();

    loginf << "done";
}

/**
 */
void DBContentManager::databaseClosedSlot()
{
    loginf;

    max_rec_num_wo_dbcontid_ = 0;
    has_max_rec_num_wo_dbcontid_ = false;

    max_reftraj_track_num_ = 0;
    has_max_reftraj_track_num_ = false;

    has_associations_ = false;
    associations_id_ = "";

    for (auto& object : dbcontent_)
        object.second->databaseClosedSlot();

    target_model_->clear();

    timestamp_min_.reset();
    timestamp_max_.reset();
    latitude_min_.reset();
    latitude_max_.reset();
    longitude_min_.reset();
    longitude_max_.reset();

    emit associationStatusChangedSignal();
    emit dbContentStatusChanged();
}

/**
 */
void DBContentManager::loadingDone(DBContent& object)
{
    bool done = true;

    for (auto& object_it : dbcontent_)
    {
        if (object_it.second->isLoading())
        {
            logdbg << "start" << object_it.first << " still loading";
            done = false;
            break;
        }
    }

    if (done)
        finishLoading();
    else
        logdbg << "not done";
}

/**
 */
void DBContentManager::deleteJobDoneSlot()
{
    logdbg;

    traced_assert(delete_job_);

    delete_job_ = nullptr;
}

/**
 */
void DBContentManager::metaDialogOKSlot()
{
    traced_assert(meta_cfg_dialog_);
    meta_cfg_dialog_->hide();
}

/**
 */
void DBContentManager::finishLoading()
{
    loginf << "all done";
    load_in_progress_ = false;

    tmp_selected_rec_nums_.clear();

    COMPASS::instance().viewManager().doViewPointAfterLoad();

    DBInterface& db_interface = COMPASS::instance().dbInterface();
    if (db_interface.hasActivePerformanceMetrics())
        loginf << db_interface.stopPerformanceMetrics().asString();

    emit loadingDoneSignal();

    //COMPASS::instance().dbContentManager().labelGenerator().updateAvailableLabelLines(); // update available lines

    QApplication::restoreOverrideCursor();

    loading_done_ = true;
}

/**
 */
bool DBContentManager::hasAssociations() const
{
    return has_associations_;
}

/**
 */
void DBContentManager::setAssociationsIdentifier(const std::string& assoc_id)
{
    auto& dbinterface = COMPASS::instance().dbInterface();

    dbinterface.setProperty("associations_generated", "1");
    dbinterface.setProperty("associations_id", assoc_id);
    dbinterface.saveProperties();

    has_associations_ = true;
    associations_id_ = assoc_id;

    COMPASS::instance().dataSourceManager().updateWidget();

    emit associationStatusChangedSignal();
}

/**
 */
std::string DBContentManager::associationsID() const { return associations_id_; }

/**
 */
void DBContentManager::clearAssociationsIdentifier()
{
    has_associations_ = false;
    associations_id_ = "";

    auto& dbinterface = COMPASS::instance().dbInterface();

    if (dbinterface.hasProperty("associations_generated"))
        dbinterface.removeProperty("associations_generated");

    if (dbinterface.hasProperty("associations_id"))
        dbinterface.removeProperty("associations_id");

    dbinterface.saveProperties();

    COMPASS::instance().dataSourceManager().updateWidget();

    emit associationStatusChangedSignal();
}

/**
 */
bool DBContentManager::loadInProgress() const
{
    return load_in_progress_;
}

/**
 */
void DBContentManager::clearData()
{
    loginf;

    data_.clear();

    COMPASS::instance().viewManager().clearDataInViews();
}

/**
 */
void DBContentManager::insertData(std::map<std::string, std::shared_ptr<Buffer>> data)
{
    logdbg;

    while (load_in_progress_) // pending insert
    {
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(1);
    }

    traced_assert(!insert_in_progress_);
    traced_assert(!insert_data_.size());
    traced_assert(!insert_job_);

    insert_in_progress_ = true;
    logdbg << "insert in progress " << insert_in_progress_;

    insert_data_ = data;

    //@TODO: prepare insert in dbcontents in parallel
    for (auto& buf_it : insert_data_)
    {
        traced_assert(existsDBContent(buf_it.first));
        dbContent(buf_it.first).prepareInsert(buf_it.second);
    }

    //update data sources from dbcontents (single-threaded)
    for (auto& buf_it : insert_data_)
    {
        dbContent(buf_it.first).updateDataSourcesBeforeInsert(buf_it.second);
    }

    insert_job_ = make_shared<DBContentInsertDBJob>(COMPASS::instance().dbInterface(), *this, data, false);

    connect(insert_job_.get(), &DBContentInsertDBJob::doneSignal,
            this, &DBContentManager::finishInserting, Qt::QueuedConnection);

    JobManager::instance().addDBJob(insert_job_);
}

/**
 */
void DBContentManager::finishInserting()
{
    logdbg;

    //delete insert job
    insert_job_ = nullptr;

    using namespace boost::posix_time;

    ptime start_time = microsec_clock::local_time();
    ptime tmp_time = microsec_clock::local_time();

    //finish inserting in db contents
    for (auto& buf_it : insert_data_)
    {
        dbContent(buf_it.first).finalizeInsert(buf_it.second);
    }

    unsigned int buffer_cnt = 0;
    for (auto& buf_it : insert_data_)
        buffer_cnt += buf_it.second->size();

    logdbg << "size " << buffer_cnt;

    traced_assert(existsMetaVariable(DBContent::meta_var_timestamp_.name()));

    insert_in_progress_ = false;
    logdbg << "insert in progress " << insert_in_progress_;
    emit insertDoneSignal();

    logdbg << "done signal took "
           << String::timeStringFromDouble((microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
           << " full " << String::timeStringFromDouble((microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

    // calculate min/max values

    tmp_time = microsec_clock::local_time();

    // ts calc for resume action

    boost::posix_time::ptime min_time_found; // for max latency calculation

    for (auto& buf_it : insert_data_)
    {
        string dbcont_name = buf_it.first;

        traced_assert(metaCanGetVariable(dbcont_name, DBContent::meta_var_timestamp_));

        unsigned int buffer_size = buf_it.second->size();

        //auto updateMinMaxRange = [ & ] (Property prop, const std::string& dbcont_name, )
        // TODO template function this

        // use dbcolum name since buffer has been transformed during insert

        // timestamp
        {
            Variable& var = metaGetVariable(dbcont_name, DBContent::meta_var_timestamp_);
            if (buf_it.second->has<boost::posix_time::ptime>(var.dbColumnName()))
            {
                NullableVector<boost::posix_time::ptime>& data_vec = buf_it.second->get<boost::posix_time::ptime>(
                    var.dbColumnName());

                bool has_vec_min_max;
                ptime ts_vec_min, ts_vec_max;

                tie(has_vec_min_max, ts_vec_min, ts_vec_max) = data_vec.minMaxValues();

                if (has_vec_min_max)
                {
                    // set min time found
                    if (min_time_found.is_not_a_date_time())
                        min_time_found = ts_vec_min;
                    else
                        min_time_found = min(min_time_found, ts_vec_min);

                    if (hasMinMaxTimestamp())
                    {
                        timestamp_min_ = std::min(timestamp_min_.get(), ts_vec_min);
                        timestamp_max_ = std::max(timestamp_max_.get(), ts_vec_max);
                    }
                    else
                    {
                        timestamp_min_ = ts_vec_min;
                        timestamp_max_ = ts_vec_max;
                    }
                }
            }
        }

        if (hasMinMaxTimestamp())
        {
            COMPASS::instance().dbInterface().setProperty(PROP_TIMESTAMP_MIN_NAME,
                                                        to_string(Time::toLong(timestamp_min_.get())));
            COMPASS::instance().dbInterface().setProperty(PROP_TIMESTAMP_MAX_NAME,
                                                        to_string(Time::toLong(timestamp_max_.get())));

            logdbg << "tod min " << timestamp_min_.get()
                   << " max " << timestamp_max_.get();
        }

        // lat & long
        if (metaCanGetVariable(dbcont_name, DBContent::meta_var_longitude_)
            && metaCanGetVariable(dbcont_name, DBContent::meta_var_latitude_))
        {
            Variable& lat_var = metaGetVariable(dbcont_name, DBContent::meta_var_latitude_);
            Variable& lon_var = metaGetVariable(dbcont_name, DBContent::meta_var_longitude_);

            if (buf_it.second->has<double>(lat_var.dbColumnName())
                && buf_it.second->has<double>(lon_var.dbColumnName()))
            {
                NullableVector<double>& lat_vec = buf_it.second->get<double>(lat_var.dbColumnName());
                NullableVector<double>& lon_vec = buf_it.second->get<double>(lon_var.dbColumnName());

                bool has_min_max = hasMinMaxPosition();

                for (unsigned int cnt=0; cnt < buffer_size; cnt++)
                {
                    if (!lat_vec.isNull(cnt) && !lon_vec.isNull(cnt))
                    {
                        if (has_min_max)
                        {
                            latitude_min_ = std::min(latitude_min_.get(), lat_vec.get(cnt));
                            latitude_max_ = std::max(latitude_max_.get(), lat_vec.get(cnt));

                            longitude_min_ = std::min(longitude_min_.get(), lon_vec.get(cnt));
                            longitude_max_ = std::max(longitude_max_.get(), lon_vec.get(cnt));
                        }
                        else
                        {
                            latitude_min_ = lat_vec.get(cnt);
                            latitude_max_ = lat_vec.get(cnt);

                            longitude_min_ = lon_vec.get(cnt);
                            longitude_max_ = lon_vec.get(cnt);

                            has_min_max = true;
                        }
                    }
                }

                if (has_min_max)
                {
                    COMPASS::instance().dbInterface().setProperty(PROP_LATITUDE_MIN_NAME, to_string(latitude_min_.get()));
                    COMPASS::instance().dbInterface().setProperty(PROP_LATITUDE_MAX_NAME, to_string(latitude_max_.get()));

                    COMPASS::instance().dbInterface().setProperty(PROP_LONGITUDE_MIN_NAME, to_string(longitude_min_.get()));
                    COMPASS::instance().dbInterface().setProperty(PROP_LONGITUDE_MAX_NAME, to_string(longitude_max_.get()));

                    logdbg << "lat min " << latitude_min_.get()
                           << " max " << latitude_max_.get()
                           << " lon min " << longitude_min_.get()
                           << " max " << longitude_max_.get();
                }
            }
        }
    }

    logdbg << "min/max took "
           << String::timeStringFromDouble((microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
           << " full " << String::timeStringFromDouble((microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

    tmp_time = microsec_clock::local_time();

    if (COMPASS::instance().appMode() == AppMode::Offline || COMPASS::instance().appMode() == AppMode::LivePaused)
        insert_data_.clear();

    // start clearing old data

    if (COMPASS::instance().appMode() == AppMode::LiveRunning)
    {
        using namespace boost::posix_time;

        ptime old_time = Time::currentUTCTime() - minutes(max_live_data_age_db_);

        logdbg << "deleting data before " << Time::toString(old_time);

        deleteDBContentData(old_time);
    }

    logdbg << "clear old took "
           << String::timeStringFromDouble(
                  (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
           << " full " << String::timeStringFromDouble(
                  (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

    tmp_time = microsec_clock::local_time();

    // add inserted to loaded data

    bool had_data = data_.size();

    if (COMPASS::instance().appMode() == AppMode::LiveRunning) // do tod cleanup
    {
        addInsertedDataToChache();

        logdbg << "insert cache took "
               << String::timeStringFromDouble((microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
               << " full " << String::timeStringFromDouble((microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

        tmp_time = microsec_clock::local_time();

        buffer_cnt = 0;
        for (auto& buf_it : data_)
            buffer_cnt += buf_it.second->size();

        logdbg << "before cut data size " << buffer_cnt;

        cutCachedData();

        logdbg << "cut cache took "
               << String::timeStringFromDouble(
                      (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
               << " full " << String::timeStringFromDouble(
                      (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

        tmp_time = microsec_clock::local_time();

        buffer_cnt = 0;
        for (auto& buf_it : data_)
            buffer_cnt += buf_it.second->size();

        logdbg << "after cut data size " << buffer_cnt;

        // INFO] DBContentManager: finishInserting: size 220692
        // filter ds took 00:00:13.266 full 00:00:13.395
        filterDataSources();

        logdbg << "filterDataSources took "
               << String::timeStringFromDouble(
                      (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
               << " full " << String::timeStringFromDouble(
                      (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

        buffer_cnt = 0;
        for (auto& buf_it : data_)
            buffer_cnt += buf_it.second->size();

        logdbg << "after ds filter data size " << buffer_cnt;

        tmp_time = microsec_clock::local_time();

        if (COMPASS::instance().filterManager().useFilters())
        {
            COMPASS::instance().filterManager().filterBuffers(data_);

            logdbg << "filter buffs took "
                   << String::timeStringFromDouble(
                          (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
                   << " full " << String::timeStringFromDouble(
                          (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

            tmp_time = microsec_clock::local_time();
        }

        logdbg << "distributing data " << (bool) data_.size();

        if (data_.size())
            emit loadedDataSignal(data_, true);
        else if (had_data)
            COMPASS::instance().viewManager().clearDataInViews();

        logdbg << "distribute took "
               << String::timeStringFromDouble(
                      (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
               << " full " << String::timeStringFromDouble(
                      (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

        tmp_time = microsec_clock::local_time();

        updateNumLoadedCounts();

        logdbg << "update cnts took "
               << String::timeStringFromDouble(
                      (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
               << " full " << String::timeStringFromDouble(
                      (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);

        tmp_time = microsec_clock::local_time();

        if (!min_time_found.is_not_a_date_time())
            loginf << "max latency "
                   << Time::toString(Time::currentUTCTime() - min_time_found);
    }

    COMPASS::instance().dataSourceManager().updateWidget();

    //COMPASS::instance().dbContentManager().labelGenerator().updateAvailableLabelLines(); // update available lines

    logdbg << "update lines took "
           << String::timeStringFromDouble(
                  (microsec_clock::local_time() - tmp_time).total_milliseconds() / 1000.0, true)
           << " full " << String::timeStringFromDouble(
                  (microsec_clock::local_time() - start_time).total_milliseconds() / 1000.0, true);
}

/**
 */
void DBContentManager::addInsertedDataToChache()
{
    loginf;

    //assert (label_generator_);

    //tbb::parallel_for(uint(0), num_targets, [&](unsigned int cnt)

    //for (auto& buf_it : insert_data_)
    unsigned int num_buffers = insert_data_.size();
    boost::mutex data_mutex;

    tbb::parallel_for(uint(0), num_buffers, [&](unsigned int buffer_cnt)
                      {
                          std::map<std::string, std::shared_ptr<Buffer>>::iterator buf_it = insert_data_.begin();
                          std::advance(buf_it, buffer_cnt);

                          VariableSet read_set = COMPASS::instance().viewManager().getReadSet(buf_it->first);
                          addStandardVariables(buf_it->first, read_set);
                          //label_generator_->addVariables(buf_it->first, read_set);

                          vector<Property> buffer_properties_to_be_removed;

                          // remove all unused
                          for (const auto& prop_it : buf_it->second->properties().properties())
                          {
                              if (!read_set.hasDBColumnName(prop_it.name()))
                                  buffer_properties_to_be_removed.push_back(prop_it); // remove it later
                          }

                          for (auto& prop_it : buffer_properties_to_be_removed)
                          {
                              logdbg << "deleting property " << prop_it.name();
                              buf_it->second->deleteProperty(prop_it);
                          }

                          // add assoc property if required
                          if (metaCanGetVariable(buf_it->first, DBContent::meta_var_utn_))
                          {
                              Variable& utn_var = metaGetVariable(buf_it->first, DBContent::meta_var_utn_);
                              Property utn_prop (utn_var.dbColumnName(), utn_var.dataType());

                              if (!buf_it->second->hasProperty(utn_prop))
                                  buf_it->second->addProperty(utn_prop);
                          }

                          // change db column names to dbcont var names
                          buf_it->second->transformVariables(read_set, true);

                          // add selection flags
                          buf_it->second->addProperty(DBContent::selected_var);

                          // add buffer to be able to distribute to views
                          if (!data_.count(buf_it->first))
                          {
                              boost::mutex::scoped_lock locker(data_mutex);
                              data_[buf_it->first] = buf_it->second;
                          }
                          else
                          {
                              data_.at(buf_it->first)->seizeBuffer(*buf_it->second.get());

                              // sort by tod
                              traced_assert(metaVariable(DBContent::meta_var_timestamp_.name()).existsIn(buf_it->first));

                              Variable& ts_var = metaVariable(DBContent::meta_var_timestamp_.name()).getFor(buf_it->first);

                              Property ts_prop {ts_var.name(), ts_var.dataType()};

                              traced_assert(data_.at(buf_it->first)->hasProperty(ts_prop));

                              data_.at(buf_it->first)->sortByProperty(ts_prop);
                          }
                      });


    insert_data_.clear();
}

/**
 */
void DBContentManager::filterDataSources()
{
    logdbg;

    std::map<unsigned int, std::set<unsigned int>> wanted_data_sources =
        COMPASS::instance().dataSourceManager().getLoadDataSources();

    unsigned int num_buffers = data_.size();

    tbb::parallel_for(uint(0), num_buffers, [&](unsigned int buffer_cnt)
                      {
                          std::map<std::string, std::shared_ptr<Buffer>>::iterator buf_it = data_.begin();
                          std::advance(buf_it, buffer_cnt);

                          // remove unwanted data sources
                          traced_assert(metaVariable(DBContent::meta_var_ds_id_.name()).existsIn(buf_it->first));
                          traced_assert(metaVariable(DBContent::meta_var_line_id_.name()).existsIn(buf_it->first));

                          Variable& ds_id_var =
                              metaVariable(DBContent::meta_var_ds_id_.name()).getFor(buf_it->first);
                          Variable& line_id_var =
                              metaVariable(DBContent::meta_var_line_id_.name()).getFor(buf_it->first);

                          Property ds_id_prop {ds_id_var.name(), ds_id_var.dataType()};
                          traced_assert(buf_it->second->hasProperty(ds_id_prop));

                          Property line_id_prop {line_id_var.name(), line_id_var.dataType()};
                          traced_assert(buf_it->second->hasProperty(ds_id_prop));

                          NullableVector<unsigned int>& ds_id_vec =
                              buf_it->second->get<unsigned int>(ds_id_var.name());
                          NullableVector<unsigned int>& line_id_vec =
                              buf_it->second->get<unsigned int>(line_id_var.name());

                          unsigned int buffer_size = buf_it->second->size();

                          vector<unsigned int> indexes_to_remove;
                          //assert (ds_id_vec.isNeverNull()); TODO why asserts?

                          for (unsigned int index=0; index < buffer_size; ++index)
                          {
                              traced_assert(!ds_id_vec.isNull(index));
                              traced_assert(!line_id_vec.isNull(index));

                              if (!wanted_data_sources.count(ds_id_vec.get(index)) // unwanted ds
                                  || !wanted_data_sources.at(ds_id_vec.get(index)).count(line_id_vec.get(index))) // unwanted line
                                  indexes_to_remove.push_back(index);
                          }

                          logdbg << "in " << buf_it->first << " remove "
                                 << indexes_to_remove.size() << " of " << buffer_size;

                          // remove unwanted indexes
                          if (indexes_to_remove.size())
                          {
                              buf_it->second->removeIndexes(indexes_to_remove); // huge cost here
                          }
                          //buffer_size = buf_it.second->size();
                      });

    // remove empty buffers
    std::map<std::string, std::shared_ptr<Buffer>> tmp_data = data_;

    for (auto& buf_it : tmp_data)
        if (!buf_it.second->size())
            data_.erase(buf_it.first);
}

/**
 */
void DBContentManager::cutCachedData()
{
    unsigned int buffer_size;

    boost::posix_time::ptime min_ts = Time::currentUTCTime() - boost::posix_time::minutes(max_live_data_age_cache_);
    // max - x minutes

    logdbg << "current ts " << Time::toString(Time::currentUTCTime())
           << " min_ts " << Time::toString(min_ts);

    for (auto& buf_it : data_)
    {
        buffer_size = buf_it.second->size();

        traced_assert(metaVariable(DBContent::meta_var_timestamp_.name()).existsIn(buf_it.first));

        Variable& ts_var = metaVariable(DBContent::meta_var_timestamp_.name()).getFor(buf_it.first);

        Property ts_prop {ts_var.name(), ts_var.dataType()};

        if (buf_it.second->hasProperty(ts_prop))
        {
            NullableVector<boost::posix_time::ptime>& ts_vec = buf_it.second->get<boost::posix_time::ptime>(
                ts_var.name());

            unsigned int index=0;

            for (; index < buffer_size; ++index)
            {
                if (!ts_vec.isNull(index) && ts_vec.get(index) > min_ts)
                {
                    logdbg << "found " << buf_it.first
                           << " cutoff tod index " << index
                           << " ts " << Time::toString(ts_vec.get(index));
                    break;
                }
            }
            // index == buffer_size if none bigger than min_ts

            if (index) // index found
            {
                index--; // cut at previous

                logdbg << "cutting " << buf_it.first
                       << " up to index " << index
                       << " total size " << buffer_size
                       << " index time " << (ts_vec.isNull(index) ? "null" : Time::toString(ts_vec.get(index)));
                traced_assert(index < buffer_size);
                buf_it.second->cutUpToIndex(index);
            }
        }
        else
            logwrn << "buffer " << buf_it.first << " has not tod for cutoff";
    }

    // remove empty buffers
    std::map<std::string, std::shared_ptr<Buffer>> tmp_data = data_;

    for (auto& buf_it : tmp_data)
        if (!buf_it.second->size())
            data_.erase(buf_it.first);
}

/**
 */
void DBContentManager::updateNumLoadedCounts()
{
    logdbg;

    // ds id->dbcont->line->cnt
    std::map<unsigned int, std::map<std::string,
                                    std::map<unsigned int, unsigned int>>> loaded_counts;

    for (auto& buf_it : data_)
    {
        traced_assert(metaCanGetVariable(buf_it.first, DBContent::meta_var_ds_id_));
        traced_assert(metaCanGetVariable(buf_it.first, DBContent::meta_var_line_id_));

        Variable& ds_id_var = metaGetVariable(buf_it.first, DBContent::meta_var_ds_id_);
        Variable& line_id_var = metaGetVariable(buf_it.first, DBContent::meta_var_line_id_);

        NullableVector<unsigned int>& ds_id_vec = buf_it.second->get<unsigned int>(ds_id_var.name());
        NullableVector<unsigned int>& line_id_vec = buf_it.second->get<unsigned int>(line_id_var.name());

        traced_assert(ds_id_vec.isNeverNull());
        traced_assert(line_id_vec.isNeverNull());

        unsigned int buffer_size = buf_it.second->size();

        for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            loaded_counts[ds_id_vec.get(cnt)][buf_it.first][line_id_vec.get(cnt)] += 1;
    }

    COMPASS::instance().dataSourceManager().setLoadedCounts(loaded_counts);
}

/**
 */
unsigned long DBContentManager::maxRecordNumberWODBContentID() const
{
    traced_assert(has_max_rec_num_wo_dbcontid_);
    return max_rec_num_wo_dbcontid_;
}

/**
 */
void DBContentManager::maxRecordNumberWODBContentID(unsigned long value)
{
    logdbg << "start" << value;

    max_rec_num_wo_dbcontid_ = value;
    has_max_rec_num_wo_dbcontid_ = true;
}

/**
 */
unsigned int DBContentManager::maxRefTrajTrackNum() const
{
    traced_assert(has_max_reftraj_track_num_);
    return max_reftraj_track_num_;
}

/**
 */
void DBContentManager::maxRefTrajTrackNum(unsigned int value)
{
    logdbg << "start" << value;

    max_reftraj_track_num_ = value;
    has_max_reftraj_track_num_ = true;
}

/**
 */
bool DBContentManager::hasMinMaxInfo() const
{
    return timestamp_min_.has_value() || timestamp_max_.has_value()
           || latitude_min_.has_value() || latitude_max_.has_value()
           || longitude_min_.has_value() || longitude_max_.has_value();
}

/**
 */
bool DBContentManager::hasMinMaxTimestamp() const
{
    return timestamp_min_.has_value() && timestamp_max_.has_value();
}

/**
 */
void DBContentManager::setMinMaxTimestamp(boost::posix_time::ptime min, boost::posix_time::ptime max)
{
    traced_assert(!min.is_not_a_date_time());
    traced_assert(!max.is_not_a_date_time());

    timestamp_min_ = min;
    timestamp_max_ = max;

    COMPASS::instance().dbInterface().setProperty("timestamp_min", to_string(Time::toLong(timestamp_min_.get())));
    COMPASS::instance().dbInterface().setProperty("timestamp_max", to_string(Time::toLong(timestamp_max_.get())));
}

/**
 */
std::pair<boost::posix_time::ptime , boost::posix_time::ptime> DBContentManager::minMaxTimestamp() const
{
    traced_assert(hasMinMaxTimestamp());
    return {timestamp_min_.get(), timestamp_max_.get()};
}

/**
 */
bool DBContentManager::hasMinMaxPosition() const
{
    return latitude_min_.has_value() || latitude_max_.has_value()
           || longitude_min_.has_value() || longitude_max_.has_value();
}

/**
 */
void DBContentManager::setMinMaxLatitude(double min, double max)
{
    latitude_min_ = min;
    latitude_max_ = max;

    COMPASS::instance().dbInterface().setProperty("latitude_min", to_string(latitude_min_.get()));
    COMPASS::instance().dbInterface().setProperty("latitude_max", to_string(latitude_max_.get()));
}

/**
 */
std::pair<double, double> DBContentManager::minMaxLatitude() const
{
    traced_assert(hasMinMaxPosition());
    return {latitude_min_.get(), latitude_max_.get()};
}

/**
 */
void DBContentManager::setMinMaxLongitude(double min, double max)
{
    longitude_min_ = min;
    longitude_max_ = max;

    COMPASS::instance().dbInterface().setProperty("longitude_min", to_string(longitude_min_.get()));
    COMPASS::instance().dbInterface().setProperty("longitude_max", to_string(longitude_max_.get()));
}

/**
 */
std::pair<double, double> DBContentManager::minMaxLongitude() const
{
    traced_assert(hasMinMaxPosition());
    return {longitude_min_.get(), longitude_max_.get()};
}

/**
 */
const std::map<std::string, std::shared_ptr<Buffer>>& DBContentManager::data() const
{
    return data_;
}

/**
 */
bool DBContentManager::canGetVariable (const std::string& dbcont_name, const Property& property)
{
    traced_assert(dbcontent_.count(dbcont_name));

    return dbcontent_.at(dbcont_name)->hasVariable(property.name());
}

/**
 */
dbContent::Variable& DBContentManager::getVariable (const std::string& dbcont_name, const Property& property)
{
    traced_assert(canGetVariable(dbcont_name, property));
    traced_assert(dbcontent_.at(dbcont_name)->hasVariable(property.name()));

    Variable& variable = dbcontent_.at(dbcont_name)->variable(property.name());

    traced_assert(variable.dataType() == property.dataType());

    return variable;
}

/**
 */
bool DBContentManager::metaCanGetVariable (const std::string& dbcont_name, const Property& meta_property)
{
    traced_assert(dbcontent_.count(dbcont_name));

    if (!existsMetaVariable(meta_property.name()))
        return false;

    return metaVariable(meta_property.name()).existsIn(dbcont_name);
}

/**
 */
dbContent::Variable& DBContentManager::metaGetVariable (const std::string& dbcont_name, const Property& meta_property)
{
    if (!metaCanGetVariable(dbcont_name, meta_property))
    {
        logerr << "defined '" << meta_property.name()
               << "' in '" << dbcont_name << "'";
        traced_assert(false);
    }

    return metaVariable(meta_property.name()).getFor(dbcont_name);
}

/**
 */
bool DBContentManager::hasTargetsInfo() const
{
    return target_model_->hasTargetsInfo();
}

/**
 */
void DBContentManager::deleteAllTargets()
{
    target_model_->deleteAllTargets();
}

/**
 */
bool DBContentManager::existsTarget(unsigned int utn)
{
    return target_model_->existsTarget(utn);
}

/**
 */
void DBContentManager::createNewTargets(const std::map<unsigned int, dbContent::ReconstructorTarget>& targets)
{
    target_model_->createNewTargets(targets);

    if (target_list_widget_)
        target_list_widget_->resizeColumnsToContents();
}

/**
 */
dbContent::Target& DBContentManager::target(unsigned int utn)
{
    traced_assert(existsTarget(utn));
    return target_model_->target(utn);
}

/**
 */
// void DBContentManager::removeDBContentFromTargets(const std::string& dbcont_name)
// {
//     target_model_->removeDBContentFromTargets(dbcont_name);
//     saveTargets();
// }

/**
 */
void DBContentManager::loadTargets()
{
    loginf;

    target_model_->loadFromDB();

    if (target_list_widget_)
        target_list_widget_->resizeColumnsToContents();
}

/**
 */
void DBContentManager::saveTargets()
{
    loginf;

    target_model_->saveToDB();
}

unsigned int DBContentManager::numTargets() const
{
    return target_model_->size();
}

/**
 */
nlohmann::json DBContentManager::targetsInfoAsJSON() const
{
    traced_assert(hasAssociations());
    traced_assert(hasTargetsInfo());

    return target_model_->asJSON();
}

/**
 */
nlohmann::json DBContentManager::targetInfoAsJSON(unsigned int utn) const
{
    traced_assert(hasAssociations());
    traced_assert(hasTargetsInfo());

    return target_model_->targetAsJSON(utn);
}

/**
 */
nlohmann::json DBContentManager::targetStatsAsJSON() const
{
    traced_assert(hasAssociations());
    traced_assert(hasTargetsInfo());

    return target_model_->targetStatsAsJSON();
}

/**
 */
nlohmann::json DBContentManager::utnsAsJSON() const
{
    traced_assert(hasAssociations());
    traced_assert(hasTargetsInfo());

    return target_model_->utnsAsJSON();
}

/**
 */
unsigned int DBContentManager::maxLiveDataAgeCache() const
{
    return max_live_data_age_cache_;
}

/**
 */
void DBContentManager::resetToStartupConfiguration()
{
    //    if (label_generator_)
    //    {
    //        label_generator_->setTmpDisableRemoveConfigOnDelete(true);

    //        label_generator_ = nullptr;

    //        generateSubConfigurable("DBContentLabelGenerator", "DBContentLabelGenerator0");
    //        traced_assert(label_generator_);
    //    }
}

/**
 */
const dbContent::TargetModel* DBContentManager::targetModel() const
{
    traced_assert(target_model_);
    return target_model_.get();
}

/**
 */
dbContent::TargetListWidget* DBContentManager::targetListWidget()
{
    if (!target_list_widget_)
    {
        traced_assert (target_model_);
        target_list_widget_ = new dbContent::TargetListWidget(*target_model_, *this);
    }

    return target_list_widget_;
}

/**
 */
void DBContentManager::resizeTargetListWidget()
{
    if (target_list_widget_)
        target_list_widget_->resizeColumnsToContents();
}

/**
 */
//void DBContentManager::updateMetaVarNames()
//{
//    //std::map<std::string, std::unique_ptr<dbContent::MetaVariable>> tmp_meta_variables = std::move(meta_variables_);
//    std::map<std::string, std::unique_ptr<dbContent::MetaVariable>> tmp_meta_variables;

//    tmp_meta_variables.insert(make_move_iterator(std::begin(meta_variables_)),
//                              make_move_iterator(std::end(meta_variables_)));
//    traced_assert(!meta_variables_.size());

//    for (auto it = tmp_meta_variables.begin(); it != tmp_meta_variables.end() /* not hoisted */; /* no increment */)
//    {
//        meta_variables_.emplace(it->second->name(), std::move(it->second));
//        it = tmp_meta_variables.erase(it);
//    }

//    meta_variables_.insert(make_move_iterator(std::begin(tmp_meta_variables)),
//                              make_move_iterator(std::end(tmp_meta_variables)));
//}

/**
 */
bool DBContentManager::insertInProgress() const
{
    return insert_in_progress_;
}

/**
 */
void DBContentManager::loadMaxRecordNumberWODBContentID()
{
    traced_assert(COMPASS::instance().dbInterface().ready());

    max_rec_num_wo_dbcontid_ = 0;
    unsigned long max_rec_num_with_dbcontid = 0;

    for (auto& obj_it : dbcontent_)
    {
        if (obj_it.second->existsInDB())
        {
            max_rec_num_with_dbcontid = COMPASS::instance().dbInterface().getMaxRecordNumber(*obj_it.second);
            max_rec_num_wo_dbcontid_ = max(Number::recNumGetWithoutDBContId(max_rec_num_with_dbcontid),
                                           max_rec_num_wo_dbcontid_);
        }
    }

    has_max_rec_num_wo_dbcontid_ = true;

    loginf << "start" << max_rec_num_wo_dbcontid_;
}

/**
 */
void DBContentManager::loadMaxRefTrajTrackNum()
{
    traced_assert(COMPASS::instance().dbInterface().ready());

    max_reftraj_track_num_ = COMPASS::instance().dbInterface().getMaxRefTrackTrackNum();
    has_max_reftraj_track_num_ = true;

    loginf << "start" << max_reftraj_track_num_;
}

/**
 */
void DBContentManager::addStandardVariables(std::string dbcont_name, dbContent::VariableSet& read_set)
{
    traced_assert(metaCanGetVariable(dbcont_name, DBContent::meta_var_rec_num_));
    read_set.add(metaGetVariable(dbcont_name, DBContent::meta_var_rec_num_));

    traced_assert(metaCanGetVariable(dbcont_name, DBContent::meta_var_ds_id_));
    read_set.add(metaGetVariable(dbcont_name, DBContent::meta_var_ds_id_));

    traced_assert(metaCanGetVariable(dbcont_name, DBContent::meta_var_line_id_));
    read_set.add(metaGetVariable(dbcont_name, DBContent::meta_var_line_id_));

    traced_assert(metaCanGetVariable(dbcont_name, DBContent::meta_var_timestamp_));
    read_set.add(metaGetVariable(dbcont_name, DBContent::meta_var_timestamp_));

    if(metaCanGetVariable(dbcont_name, DBContent::meta_var_utn_))
        read_set.add(metaGetVariable(dbcont_name, DBContent::meta_var_utn_));
}

/**
 */
MetaVariableConfigurationDialog* DBContentManager::metaVariableConfigdialog()
{
    if (!meta_cfg_dialog_)
    {
        meta_cfg_dialog_.reset(new MetaVariableConfigurationDialog(*this));

        connect(meta_cfg_dialog_.get(), &MetaVariableConfigurationDialog::okSignal,
                this, &DBContentManager::metaDialogOKSlot);
    }

    traced_assert(meta_cfg_dialog_);
    return meta_cfg_dialog_.get();
}

/**
 */
void DBContentManager::setViewableDataConfig (const nlohmann::json::object_t& data)
{
    viewable_data_cfg_.reset(new ViewableDataConfig(data));

    COMPASS::instance().viewManager().setCurrentViewPoint(viewable_data_cfg_.get());
}

void DBContentManager::storeSelectedRecNums(const std::vector<unsigned long>& selected)
{
    clearSelectedRecNums(); // no other selected

    auto& dbcont_man = COMPASS::instance().dbContentManager();

    for (auto rec_num : selected)
    {
        tmp_selected_rec_nums_[dbcont_man.dbContentWithId(Number::recNumGetDBContId(rec_num))].insert(rec_num);
    }
}

void DBContentManager::clearSelectedRecNums()
{
    loginf;

    tmp_selected_rec_nums_.clear();

    for (const auto& buf_it : data_) // std::map<std::string, std::shared_ptr<Buffer>>
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));

        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());
        selected_vec.setAll(false);
    }
}

/**
 */
void DBContentManager::saveSelectedRecNums()
{
    loginf;

    if(tmp_selected_rec_nums_.size())
        return; // already stored from view point

    for (const auto& buf_it : data_) // std::map<std::string, std::shared_ptr<Buffer>>
    {
        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));

        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        traced_assert(buf_it.second->has<unsigned long>(DBContent::meta_var_rec_num_.name()));
        NullableVector<unsigned long>& rec_num_vec = buf_it.second->get<unsigned long>(
            DBContent::meta_var_rec_num_.name());

        size_t data_size = selected_vec.contentSize();

        for (unsigned int cnt=0; cnt < data_size; ++cnt)
        {
            if (!selected_vec.isNull(cnt) && selected_vec.get(cnt))
                tmp_selected_rec_nums_[buf_it.first].insert(rec_num_vec.get(cnt));
        }

        loginf << "start" << buf_it.first << " has "
               << tmp_selected_rec_nums_[buf_it.first].size() << " selected" ;
    }
}

/**
 */
void DBContentManager::restoreSelectedRecNums()
{
    logdbg << "start" << tmp_selected_rec_nums_.size();

    if (tmp_selected_rec_nums_.empty())
        return;

    for (const auto& buf_it : data_) // std::map<std::string, std::shared_ptr<Buffer>>
    {
        if (!tmp_selected_rec_nums_.count(buf_it.first))
            continue;

        auto& sel_recnums = tmp_selected_rec_nums_.at(buf_it.first);
        if (sel_recnums.empty())
            continue;

        traced_assert(buf_it.second->has<bool>(DBContent::selected_var.name()));

        NullableVector<bool>& selected_vec = buf_it.second->get<bool>(DBContent::selected_var.name());

        traced_assert(buf_it.second->has<unsigned long>(DBContent::meta_var_rec_num_.name()));
        NullableVector<unsigned long>& rec_num_vec = buf_it.second->get<unsigned long>(
            DBContent::meta_var_rec_num_.name());

        // select existing & erase, keep still unselected

        std::map<unsigned long, unsigned int> unique_rec_nums =
            rec_num_vec.uniqueValuesWithIndexes(sel_recnums); // indexes of selected rec nums, value->index

        for (auto& rec_num_it : unique_rec_nums)
        {
            selected_vec.set(rec_num_it.second, true);
            sel_recnums.erase(rec_num_it.first);
        }
    }
}

/**
 */
void DBContentManager::showSurroundingData (unsigned int utn)
{
    nlohmann::json::object_t data;

    traced_assert(target_model_);
    traced_assert(target_model_->existsTarget(utn));

    dbContent::Target& target = target_model_->target(utn);

    using namespace boost::posix_time;

    ptime time_begin = target.timeBegin();
    time_begin -= seconds(60);

    ptime time_end = target.timeEnd();
    time_end += seconds(60);

    //    "Timestamp": {
    //    "Timestamp Maximum": "05:56:32.297",
    //    "Timestamp Minimum": "05:44:58.445"
    //    },

        // TODO_TIMESTAMP
    data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Maximum"] = Time::toString(time_end);
    data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Minimum"] = Time::toString(time_begin);

    //    "Aircraft Address": {
    //    "Aircraft Address Values": "FEFE10"
    //    },
    if (target.aircraftAddresses().size())
        data[ViewPoint::VP_FILTERS_KEY]["Aircraft Address"]["Aircraft Address Values"] =
            target.aircraftAddressesStr()+",NULL";

    //    "Mode 3/A Code": {
    //    "Mode 3/A Code Values": "7000"
    //    }

    if (target.modeACodes().size())
        data[ViewPoint::VP_FILTERS_KEY]["Mode 3/A Codes"]["Mode 3/A Codes Values"] = target.modeACodesStr()+",NULL";

    //    VP_FILTERS_KEY: {
    //    "Barometric Altitude": {
    //    "Barometric Altitude Maximum": "43000",
    //    "Barometric Altitude Minimum": "500",
    //    "Barometric Altitude NULL": false
    //    },

    if (target.hasModeC())
    {
        float alt_min = target.modeCMin();
        alt_min -= 300;
        float alt_max = target.modeCMax();
        alt_max += 300;

        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude Maximum"] = alt_max;
        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude Minimum"] = alt_min;
        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude NULL"] = true;
    }

        //    "Position": {
        //    "Latitude Maximum": "50.78493920733",
        //    "Latitude Minimum": "44.31547147615",
        //    "Longitude Maximum": "20.76559892354",
        //    "Longitude Minimum": "8.5801592186"
        //    }

    if (target.hasPositionBounds())
    {
        double lat_eps = (target.latitudeMax() - target.latitudeMin()) / 10.0;
        lat_eps = min(lat_eps, 0.1); // 10% or 0.1 at max
        double lon_eps = (target.longitudeMax() - target.longitudeMin()) / 10.0; // 10%
        lon_eps = min(lon_eps, 0.1); // 10% or 0.1 at max

        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Maximum"] = to_string(target.latitudeMax()+lat_eps);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Minimum"] = to_string(target.latitudeMin()-lat_eps);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Maximum"] = to_string(target.longitudeMax()+lon_eps);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Minimum"] = to_string(target.longitudeMin()-lon_eps);
    }

    setViewableDataConfig(data);
}

void DBContentManager::showSurroundingData (std::set<unsigned int> utns)
{
    nlohmann::json::object_t data;

    using namespace boost::posix_time;

    traced_assert(target_model_);

    ptime sum_time_begin, sum_time_end;

    std::set<std::string> aircraft_adresses;
    std::set<std::string> mode_a_codes;

    float sum_alt_min{0}, sum_alt_max{0};
    double sum_lat_min{0},sum_lat_max{0},sum_lon_min{0},sum_lon_max{0};

    bool first_ts{true}, first_alt{true}, first_pos{true};

    for (auto utn : utns)
    {
        traced_assert(target_model_->existsTarget(utn));

        dbContent::Target& target = target_model_->target(utn);

        ptime time_begin = target.timeBegin();
        time_begin -= seconds(60);

        ptime time_end = target.timeEnd();
        time_end += seconds(60);

        if (first_ts)
        {
            sum_time_begin = time_begin;
            sum_time_end = time_end;
        }
        else
        {
            sum_time_begin = min(sum_time_begin, time_begin);
            sum_time_end = max(sum_time_end, time_end);
        }
        first_ts = false;

        for (auto acad : target.aircraftAddresses())
        {
            if (!aircraft_adresses.count(String::hexStringFromInt(acad, 6, '0')))
                aircraft_adresses.insert(String::hexStringFromInt(acad, 6, '0'));
        }

        for (auto m3a : target.modeACodes())
        {
            if (!mode_a_codes.count(String::octStringFromInt(m3a, 4, '0')))
                mode_a_codes.insert(String::octStringFromInt(m3a, 4, '0'));
        }

        if (target.hasModeC())
        {
            float alt_min = target.modeCMin() - 300;
            float alt_max = target.modeCMax() + 300;

            if (first_alt)
            {
                sum_alt_min = alt_min;
                sum_alt_max = alt_max;
            }
            else
            {
                sum_alt_min = min(sum_alt_min, alt_min);
                sum_alt_max = max(sum_alt_max, alt_max);
            }

            first_alt = false;
        }

        if (target.hasPositionBounds())
        {
            double lat_eps = (target.latitudeMax() - target.latitudeMin()) / 10.0;
            lat_eps = min(lat_eps, 0.1); // 10% or 0.1 at max
            double lon_eps = (target.longitudeMax() - target.longitudeMin()) / 10.0; // 10%
            lon_eps = min(lon_eps, 0.1); // 10% or 0.1 at max

            double lat_max = target.latitudeMax()+lat_eps;
            double lat_min = target.latitudeMin()-lat_eps;
            double lon_max = target.longitudeMax()+lon_eps;
            double lon_min = target.longitudeMin()-lon_eps;

            if (first_pos)
            {
                sum_lat_min = lat_min;
                sum_lat_max = lat_max;
                sum_lon_min = lon_min;
                sum_lon_max = lon_max;
            }
            else
            {
                sum_lat_min = min(lat_min,sum_lat_min);
                sum_lat_max = max(lat_max,sum_lat_max);
                sum_lon_min = min(lon_min,sum_lon_min);
                sum_lon_max = max(lon_max,sum_lon_max);
            }

            first_pos = false;
        }
    }

    //    "Timestamp": {
    //    "Timestamp Maximum": "05:56:32.297",
    //    "Timestamp Minimum": "05:44:58.445"
    //    },

        // TODO_TIMESTAMP
    data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Maximum"] = Time::toString(sum_time_end);
    data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Minimum"] = Time::toString(sum_time_begin);

    //    "Aircraft Address": {
    //    "Aircraft Address Values": "FEFE10"
    //    },
    if (aircraft_adresses.size())
        data[ViewPoint::VP_FILTERS_KEY]["Aircraft Address"]["Aircraft Address Values"] =
            String::compress(aircraft_adresses,',')+",NULL";

    //    "Mode 3/A Code": {
    //    "Mode 3/A Code Values": "7000"
    //    }

    if (mode_a_codes.size())
        data[ViewPoint::VP_FILTERS_KEY]["Mode 3/A Codes"]["Mode 3/A Codes Values"] =
            String::compress(mode_a_codes,',')+",NULL";

    //    VP_FILTERS_KEY: {
    //    "Barometric Altitude": {
    //    "Barometric Altitude Maximum": "43000",
    //    "Barometric Altitude Minimum": "500",
    //    "Barometric Altitude NULL": false
    //    },

    if (!first_alt)
    {
        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude Maximum"] = sum_alt_max;
        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude Minimum"] = sum_alt_min;
        data[ViewPoint::VP_FILTERS_KEY]["Barometric Altitude"]["Barometric Altitude NULL"] = true;
    }

    //    "Position": {
    //    "Latitude Maximum": "50.78493920733",
    //    "Latitude Minimum": "44.31547147615",
    //    "Longitude Maximum": "20.76559892354",
    //    "Longitude Minimum": "8.5801592186"
    //    }

    if (!first_pos)
    {
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Maximum"] = to_string(sum_lat_max);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Minimum"] = to_string(sum_lat_min);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Maximum"] = to_string(sum_lon_max);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Minimum"] = to_string(sum_lon_min);

    }

    setViewableDataConfig(data);
}

/**
 */
std::string DBContentManager::utnComment (unsigned int utn)
{
    traced_assert(target_model_);
    traced_assert(target_model_->existsTarget(utn));
    return target_model_->target(utn).comment();
}

/**
 */
void DBContentManager::utnComment (unsigned int utn, std::string value)
{
    loginf << "utn " << utn << " comment '" << value << "'";

    traced_assert(target_model_);
    traced_assert(target_model_->existsTarget(utn));
    target_model_->setTargetComment(utn, value);
}

/**
 */
TargetBase::Category DBContentManager::emitterCategory(unsigned int utn) const
{
    traced_assert(target_model_);
    traced_assert(target_model_->existsTarget(utn));

    return target_model_->target(utn).targetCategory();
}

/**
 */
std::string DBContentManager::emitterCategoryStr(unsigned int utn) const
{
    traced_assert(target_model_);
    traced_assert(target_model_->existsTarget(utn));

    return target_model_->target(utn).emitterCategoryStr();
}

/**
 */
void DBContentManager::autoFilterUTNS()
{
    traced_assert(target_model_);
    target_model_->setUseByFilter();

    //    data_.setUseAllTargetData(true);
    //    data_.clearComments();
    //    data_.setUseByFilter();
}

/**
 */
void DBContentManager::showUTN (unsigned int utn)
{
    loginf << "utn " << utn;

    nlohmann::json data;
    data[ViewPoint::VP_FILTERS_KEY]["UTNs"]["utns"] = to_string(utn);

    logdbg << "showing";
    setViewableDataConfig(data);
}

/**
 */
void DBContentManager::showUTNs (std::set<unsigned int> utns)
{
    loginf << "len " << utns.size();

    if (utns.size())
    {
        nlohmann::json data;
        data[ViewPoint::VP_FILTERS_KEY]["UTNs"]["utns"] = String::compress(utns, ',');

        loginf << "showing '" << String::compress(utns, ',') << "'";
        setViewableDataConfig(data);
    }
    else
        clearData();
}

