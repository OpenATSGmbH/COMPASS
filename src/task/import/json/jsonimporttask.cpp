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

#include "jsonimporttask.h"
#include "compass.h"
#include "buffer.h"
//#include "createartasassociationstask.h"
#include "dbinterface.h"
//#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
//#include "dbcontent/variable/variable.h"
#include "files.h"
#include "jobmanager.h"
#include "jsonimporttaskdialog.h"
#include "jsonmappingjob.h"
#include "jsonparsejob.h"
#include "jsonparsingschema.h"
//#include "propertylist.h"
//#include "radarplotpositioncalculatortask.h"
#include "readjsonfilejob.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "asteriximporttask.h"
#include "datasourcemanager.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>
#include <algorithm>
#include <fstream>
#include <memory>
#include <stdexcept>

using namespace Utils;
using namespace nlohmann;
using namespace std;

const unsigned int num_objects_chunk = 10000;

JSONImportTask::JSONImportTask(const std::string& class_id, const std::string& instance_id,
                               TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_import_json.json")
{
    tooltip_ = "Allows importing of JSON data in several variants into the opened database.";

    registerParameter("current_schema_name", &current_schema_name_, std::string());

    date_ = boost::posix_time::ptime(boost::gregorian::day_clock::universal_day());

    createSubConfigurables();
}

JSONImportTask::~JSONImportTask()
{
}

void JSONImportTask::generateSubConfigurable(const std::string& class_id,
                                             const std::string& instance_id)
{
    if (class_id == "JSONParsingSchema")
    {
        std::string name = getSubConfiguration(class_id, instance_id).getParameterConfigValue<std::string>("name");

        traced_assert(schemas_.find(name) == schemas_.end());

        logdbg << "generating schema " << instance_id
               << " with name " << name;

        schemas_[name] = make_shared<JSONParsingSchema>(class_id, instance_id, this);
    }
    else
        throw std::runtime_error("JSONImporterTask: generateSubConfigurable: unknown class_id " +
                                 class_id);
}

JSONImportTaskDialog* JSONImportTask::dialog()
{
    if (!dialog_)
    {
        dialog_.reset(new JSONImportTaskDialog(*this));

        connect(dialog_.get(), &JSONImportTaskDialog::testTmportSignal,
                this, &JSONImportTask::dialogTestImportSlot);

        connect(dialog_.get(), &JSONImportTaskDialog::importSignal,
                this, &JSONImportTask::dialogImportSlot);

        connect(dialog_.get(), &JSONImportTaskDialog::cancelSignal,
                this, &JSONImportTask::dialogCancelSlot);
    }

    traced_assert(dialog_);
    return dialog_.get();
}

void JSONImportTask::importFilename(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    import_filename_ = filename;

    if (dialog_)
        dialog_->updateSource();

}

bool JSONImportTask::hasSchema(const std::string& name)
{
    if (name == "jASTERIX")
        return true;

    return schemas_.count(name) > 0;
}

bool JSONImportTask::hasCurrentSchema() const
{
    if (!current_schema_name_.size())
        return false;

    if (current_schema_name_ == "jASTERIX")
        return true;

    return schemas_.count(current_schema_name_) > 0;
}

bool JSONImportTask::hasJSONSchema() const
{
    return hasCurrentSchema() && current_schema_name_ != "jASTERIX";
}

std::shared_ptr<JSONParsingSchema> JSONImportTask::currentJSONSchema()
{
    traced_assert(hasCurrentSchema());
    traced_assert(hasJSONSchema());

    traced_assert(current_schema_name_ != "jASTERIX");

    return schemas_.at(current_schema_name_);
}

void JSONImportTask::removeCurrentSchema()
{
    traced_assert(hasCurrentSchema());

    traced_assert(current_schema_name_ != "jASTERIX");

    schemas_.erase(current_schema_name_);
    traced_assert(!hasCurrentSchema());

    current_schema_name_ = "";

    if (schemas_.size())
        current_schema_name_ = schemas_.begin()->first;

    loginf << "set current schema '" << currentSchemaName() << "'";
}

std::string JSONImportTask::currentSchemaName() const { return current_schema_name_; }

void JSONImportTask::currentSchemaName(const std::string& current_schema)
{
    loginf << "start" << current_schema;

    current_schema_name_ = current_schema;

    loginf << "done";
}

void JSONImportTask::test(bool test) { test_ = test; }

size_t JSONImportTask::objectsInserted() const { return records_inserted_; }

unsigned int JSONImportTask::fileLineID() const
{
    return file_line_id_;
}

void JSONImportTask::fileLineID(unsigned int value)
{
    loginf << "value " << value;

    file_line_id_ = value;
}

const boost::posix_time::ptime &JSONImportTask::date() const
{
    return date_;
}

void JSONImportTask::date(const boost::posix_time::ptime& date)
{
    date_ = date;
}


bool JSONImportTask::canImportFile()
{
    if (!import_filename_.size())
        return false;

    if (!Files::fileExists(import_filename_))
    {
        loginf << "not possible since file '" << import_filename_
               << "does not exist";
        return false;
    }

    if (!current_schema_name_.size())
        return false;

    if (current_schema_name_ == "jASTERIX")
        return true;

    if (!schemas_.count(current_schema_name_))
    {
        current_schema_name_ = "";
        return false;
    }

    return true;
}

bool JSONImportTask::canRun() { return canImportFile(); }

void JSONImportTask::run()
{
    loginf << "filename '" << import_filename_ << "' test " << test_;

    done_ = false; // since can be run multiple times

    std::string tmp;

    if (test_)
        tmp = "test import of file ";
    else
        tmp = "import of file ";

    start_time_ = boost::posix_time::microsec_clock::local_time();

    COMPASS::instance().logInfo("JSON Import") << "started";

    traced_assert(canImportFile());

    objects_read_ = 0;
    objects_parsed_ = 0;
    objects_parse_errors_ = 0;

    objects_mapped_ = 0;
    objects_not_mapped_ = 0;

    objects_created_ = 0;
    records_inserted_ = 0;

    all_done_ = false;

    traced_assert(hasCurrentSchema());

    if (hasJSONSchema()) // native json schema
    {
        for (auto& map_it : *currentJSONSchema())
            if (!map_it.second->initialized())
                map_it.second->initialize();
    }
    else // jASTERIX schema
    {
        for (auto& map_it : *task_manager_.asterixImporterTask().schema())
            if (!map_it.second->initialized())
                map_it.second->initialize();
    }

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    if (current_schema_name_ == "jASTERIX")
        read_json_job_ = std::make_shared<ReadJSONFileJob>(import_filename_, 1);
    else
        read_json_job_ = std::make_shared<ReadJSONFileJob>(import_filename_, num_objects_chunk);

    connect(read_json_job_.get(), &ReadJSONFileJob::readJSONFilePartSignal, this,
            &JSONImportTask::addReadJSONSlot, Qt::QueuedConnection);
    connect(read_json_job_.get(), &ReadJSONFileJob::obsoleteSignal, this,
            &JSONImportTask::readJSONFileObsoleteSlot, Qt::QueuedConnection);
    connect(read_json_job_.get(), &ReadJSONFileJob::doneSignal, this,
            &JSONImportTask::readJSONFileDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addBlockingJob(read_json_job_);

    updateMsgBox();

    logdbg << "JSONImporterTask: run done";
}

void JSONImportTask::dialogImportSlot()
{
    loginf;

    traced_assert(dialog_);
    dialog_->hide();

    traced_assert(canRun());
    run();
}

void JSONImportTask::dialogTestImportSlot()
{
    loginf;

    traced_assert(dialog_);
    dialog_->hide();

    traced_assert(canRun());
    test_ = true;

    run();

}
void JSONImportTask::dialogCancelSlot()
{
    loginf;

    traced_assert(dialog_);
    dialog_->hide();
}

void JSONImportTask::addReadJSONSlot()
{
    loginf;

    traced_assert(read_json_job_);

    loginf << "moving objects";

//    if (maxLoadReached())
//        read_json_job_->pause();

    std::vector<std::string> objects = read_json_job_->objects(); // json objects?

    bytes_read_ = read_json_job_->bytesRead();
    bytes_to_read_ = read_json_job_->bytesToRead();
    read_status_percent_ = read_json_job_->getStatusPercent();
    objects_read_ += objects.size();
    loginf << "bytes " << bytes_read_ << " to read "
           << bytes_to_read_ << " percent " << read_status_percent_;

    while (json_parse_job_)  // only one can exist at a time
    {
        if (read_json_job_)
            read_json_job_->pause();

        updateMsgBox();

        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents); // TODO not smart
        QThread::msleep(10);
    }

    // start parse job
    traced_assert(!json_parse_job_);

    loginf << "starting parse job";
    json_parse_job_ =
            std::make_shared<JSONParseJob>(std::move(objects), current_schema_name_, post_process_);

    connect(json_parse_job_.get(), &JSONParseJob::obsoleteSignal, this,
            &JSONImportTask::parseJSONObsoleteSlot, Qt::QueuedConnection);
    connect(json_parse_job_.get(), &JSONParseJob::doneSignal, this,
            &JSONImportTask::parseJSONDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addNonBlockingJob(json_parse_job_);

    loginf << "updating message box";
    updateMsgBox();
}

void JSONImportTask::readJSONFileDoneSlot()
{
    logdbg << "start";

    read_status_percent_ = 100.0;
    read_json_job_ = nullptr;

    logdbg << "updating message box";
    updateMsgBox();

    logdbg << "done";
}

void JSONImportTask::readJSONFileObsoleteSlot()
{
    logdbg << "start";
}

void JSONImportTask::parseJSONDoneSlot()
{
    loginf;

    traced_assert(json_parse_job_);

    objects_parsed_ += json_parse_job_->objectsParsed();
    objects_parse_errors_ += json_parse_job_->parseErrors();
    std::unique_ptr<nlohmann::json> json_objects = json_parse_job_->jsonObjects();

    json_parse_job_ = nullptr;

    traced_assert(hasCurrentSchema());

    std::vector<std::string> keys;

    std::shared_ptr<JSONMappingJob> json_map_job;

    if (current_schema_name_ == "jASTERIX")
    {
        //loginf << "UGA '" << json_objects->dump(4) << "'";

        if (json_objects->contains("frames")) // framed
        {
            keys = {"frames", "content", "data_blocks", "content", "records"};
        }
        else // not framed
        {
            traced_assert(json_objects->contains("data_blocks"));
            keys = {"data_blocks", "content", "records"};
        }

        json_map_job = std::make_shared<JSONMappingJob>(
                    std::move(json_objects), keys, file_line_id_,
                    COMPASS::instance().taskManager().asterixImporterTask().schema()->parsers());

        //loginf << "UGA2";
    }
    else // native json schema
    {
        traced_assert(json_objects->contains("data")); // always written in data array
        keys = {"data"};

        size_t count = json_objects->at("data").size();
        logdbg << "start" << count << " parsed objects";

        traced_assert(hasJSONSchema());

        json_map_job = std::make_shared<JSONMappingJob>(
                    std::move(json_objects), keys, file_line_id_, currentJSONSchema()->parsers());
    }

    traced_assert(json_map_job);

    //loginf << "UGA3";

    connect(json_map_job.get(), &JSONMappingJob::obsoleteSignal, this,
            &JSONImportTask::mapJSONObsoleteSlot, Qt::QueuedConnection);
    connect(json_map_job.get(), &JSONMappingJob::doneSignal, this, &JSONImportTask::mapJSONDoneSlot,
            Qt::QueuedConnection);

    json_map_jobs_.push_back(json_map_job);

    JobManager::instance().addNonBlockingJob(json_map_job);

    updateMsgBox();

    if (read_json_job_)
    {
        if (maxLoadReached())
            read_json_job_->pause();
        else
            read_json_job_->unpause();
    }

    loginf << "done";
}

void JSONImportTask::parseJSONObsoleteSlot()
{
    logdbg << "start";
}

void JSONImportTask::mapJSONDoneSlot()
{
    loginf;

    JSONMappingJob* map_job = dynamic_cast<JSONMappingJob*>(QObject::sender());
    traced_assert(map_job);

    loginf << "skipped " << map_job->numNotMapped()
           << " all skipped " << objects_not_mapped_;

    objects_mapped_ += map_job->numMapped();  // TODO done twice?
    objects_not_mapped_ += map_job->numNotMapped();
    objects_parse_errors_ += map_job->numErrors();

    objects_created_ += map_job->numCreated();

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers = map_job->buffers();

    json_map_jobs_.erase(json_map_jobs_.begin());

    for (auto& buf_it : job_buffers)
        if (buf_it.second && buf_it.second->size())
            objects_mapped_ += buf_it.second->size();

    if (test_ || !objects_mapped_)
    {
        checkAllDone();
        updateMsgBox();
        return;
    }

    updateMsgBox();

    if (read_json_job_)
    {
        if (maxLoadReached())
            read_json_job_->pause();
        else
            read_json_job_->unpause();
    }

    // date_

    traced_assert(!ts_calculator_.processing());
    ts_calculator_.setBuffers(std::move(job_buffers));

    ts_calculator_.calculate(import_filename_,
                             boost::posix_time::ptime(boost::gregorian::day_clock::universal_day()), false,
                             false, 0,
                             false, 0);

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers2 {ts_calculator_.buffers()};
    ts_calculator_.setProcessingDone();

    std::shared_ptr<ASTERIXPostprocessJob> postprocess_job =
            make_shared<ASTERIXPostprocessJob>(std::move(job_buffers2));

    postprocess_jobs_.push_back(postprocess_job);

    // check for future when net import

    connect(postprocess_job.get(), &ASTERIXPostprocessJob::obsoleteSignal, this,
            &JSONImportTask::postprocessObsoleteSlot, Qt::QueuedConnection);
    connect(postprocess_job.get(), &ASTERIXPostprocessJob::doneSignal, this,
            &JSONImportTask::postprocessDoneSlot, Qt::QueuedConnection);

    JobManager::instance().addNonBlockingJob(postprocess_job);

    //insertData(std::move(job_buffers));

    logdbg << "done";
}

void JSONImportTask::mapJSONObsoleteSlot()
{
     logdbg << "start"; 
}


void JSONImportTask::postprocessDoneSlot()
{
    logdbg << "import_file " << import_filename_;

    if (stopped_)
    {
        postprocess_jobs_.clear();

        checkAllDone();

        return;
    }

    ASTERIXPostprocessJob* post_job = dynamic_cast<ASTERIXPostprocessJob*>(QObject::sender());
    traced_assert(post_job);

    std::map<std::string, std::shared_ptr<Buffer>> job_buffers {post_job->buffers()};

    traced_assert(postprocess_jobs_.size());
    traced_assert(postprocess_jobs_.begin()->get() == post_job);
    post_job = nullptr;
    postprocess_jobs_.erase(postprocess_jobs_.begin()); // remove

    insertData(std::move(job_buffers));

    // queue data
//    if (!stopped_)
//    {
//        queued_job_buffers_.emplace_back(std::move(job_buffers));

//        if (!insert_active_)
//        {
//            logdbg << "inserting, thread " << QThread::currentThreadId();
//            traced_assert(!COMPASS::instance().dbContentManager().insertInProgress());
//            insertData();
//        }
//    }
}

void JSONImportTask::postprocessObsoleteSlot()
{
    ASTERIXPostprocessJob* post_job = dynamic_cast<ASTERIXPostprocessJob*>(QObject::sender());
    traced_assert(post_job);

    traced_assert(postprocess_jobs_.size());
    traced_assert(postprocess_jobs_.begin()->get() == post_job);
    post_job = nullptr;
    postprocess_jobs_.erase(postprocess_jobs_.begin()); // remove
}


void JSONImportTask::insertData(std::map<std::string, std::shared_ptr<Buffer>> job_buffers)
{
    loginf << "inserting into database";

    if (!job_buffers.size())
    {
        loginf << "no job buffers";
        return;
    }

    unsigned int old_records_size = records_inserted_;

    for (auto& job_it : job_buffers)
        records_inserted_ += job_it.second->size();

    if (records_inserted_ == old_records_size)
    {
        loginf << "no data in job buffers";
        return;
    }

    while (insert_active_)
    {
        loginf << "waiting on insert done";

        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(1);
    }

    DBContentManager& dbcont_manager = COMPASS::instance().dbContentManager();

    if (!insert_slot_connected_)
    {
        loginf << "connecting slot";

        connect(&dbcont_manager, &DBContentManager::insertDoneSignal,
                this, &JSONImportTask::insertDoneSlot, Qt::QueuedConnection);
        insert_slot_connected_ = true;
    }

    ++insert_active_;
    dbcont_manager.insertData(job_buffers);

    // checkAllDone();


//    traced_assert(hasCurrentSchema());
//    shared_ptr<JSONParsingSchema> current_schema = currentJSONSchema();
//    //assert(schemas_.count(current_schema_));

//    if (!dbcont_variable_sets_.size())  // initialize if empty
//    {
//        for (auto& parser_it : *current_schema)
//        {
//            std::string dbcontent_name = parser_it.second->dbContentName();

//            if (!job_buffers.count(dbcontent_name))
//                continue;

//            //DBContent& dbcontent = parser_it.second->dbContent();

//            std::string data_source_var_name = parser_it.second->dataSourceVariableName();
//            traced_assert(data_source_var_name.size());
//            //assert(dbcontent.currentDataSourceDefinition().localKey() == data_source_var_name);

//            const dbContent::VariableSet& set = parser_it.second->variableList();

//            if (dbcont_variable_sets_.count(dbcontent_name))  // add variables
//            {
//                traced_assert(std::get<0>(dbcont_variable_sets_.at(dbcontent_name)) == data_source_var_name);
//                std::get<1>(dbcont_variable_sets_.at(dbcontent_name)).add(set);
//            }
//            else  // create it
//                dbcont_variable_sets_[dbcontent_name] = std::make_tuple(data_source_var_name, set);
//        }
//    }

//    while (insert_active_)
//    {
//        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
//        QThread::msleep(1);
//    }

//    bool has_sac_sic = false;

//    DBContentManager& dbcont_manager = COMPASS::instance().dbContentManager();

//    for (auto& buf_it : job_buffers)
//    {
//        std::string dbcontent_name = buf_it.first;

//        std::shared_ptr<Buffer> buffer = buf_it.second;

//        if (!buffer->size())
//        {
//            logdbg << "dbcont " << dbcontent_name << " with empty buffer";
//            continue;
//        }

//        if (!dbcont_variable_sets_.count(dbcontent_name))
//        {
//            logerr << "dbcont " << dbcontent_name << " has no variable set, buffer size "
//                   << buffer->size();
//            continue;
//        }

//        traced_assert(dbcont_variable_sets_.count(dbcontent_name));

//        loginf << "insert dbcont " << dbcontent_name << " size " << buffer->size()
//               << " num prop " << buffer->properties().size();
//        //buffer->properties().print();

//        traced_assert(dbcont_manager.existsDBContent(dbcontent_name));
//        DBContent& dbcontent = dbcont_manager.dbContent(dbcontent_name);

//        ++insert_active_;

//        has_sac_sic = dbcontent.hasVariable("sac") && dbcontent.hasVariable("sic") &&
//                buffer->has<unsigned char>("sac") && buffer->has<unsigned char>("sic");

//        logdbg << "start" << dbcontent.name() << " has sac/sic "
//               << has_sac_sic << " buffer size " << buffer->size();

//        TODO_ASSERT

////        connect(&dbcontent, &DBContent::insertDoneSignal, this, &JSONImportTask::insertDoneSlot,
////                Qt::UniqueConnection);
////        connect(&dbcontent, &DBContent::insertProgressSignal, this,
////                &JSONImportTask::insertProgressSlot, Qt::UniqueConnection);

//        std::string data_source_var_name = std::get<0>(dbcont_variable_sets_.at(dbcontent_name));

//        logdbg << "adding new data sources in dbcont " << dbcontent.name()
//               << " ds varname '" << data_source_var_name << "'";

//        TODO_ASSERT

//        // collect existing datasources
//        std::set<int> datasources_existing;
////        if (dbcontent.hasDataSources())
////            for (auto ds_it = dbcontent.dsBegin(); ds_it != dbcontent.dsEnd(); ++ds_it)
////                datasources_existing.insert(ds_it->first);

//        // getting key list and distinct values
//        traced_assert(buffer->properties().hasProperty(data_source_var_name));
//        traced_assert(buffer->properties().get(data_source_var_name).dataType() == PropertyDataType::INT);

//        traced_assert(buffer->has<int>(data_source_var_name));
//        NullableVector<int>& data_source_key_list = buffer->get<int>(data_source_var_name);
//        std::set<int> data_source_keys = data_source_key_list.distinctValues();

//        std::map<int, std::pair<unsigned char, unsigned char>> sac_sics;  // keyvar->(sac,sic)
//        // collect sac/sics
//        if (has_sac_sic)
//        {
//            NullableVector<unsigned char>& sac_list = buffer->get<unsigned char>("sac");
//            NullableVector<unsigned char>& sic_list = buffer->get<unsigned char>("sic");

//            size_t size = buffer->size();
//            int key_val;
//            for (unsigned int cnt = 0; cnt < size; ++cnt)
//            {
//                key_val = data_source_key_list.get(cnt);

//                if (datasources_existing.count(key_val) != 0)
//                    continue;

//                if (sac_sics.count(key_val) == 0)
//                {
//                    logdbg << "found new ds " << key_val
//                           << " for sac/sic";

//                    traced_assert(!sac_list.isNull(cnt) && !sic_list.isNull(cnt));
//                    sac_sics[key_val] = std::pair<unsigned char, unsigned char>(sac_list.get(cnt),
//                                                                                sic_list.get(cnt));

//                    logdbg << "source " << key_val << " sac "
//                           << static_cast<int>(sac_list.get(cnt)) << " sic "
//                           << static_cast<int>(sic_list.get(cnt));
//                }
//            }
//        }

//        // adding datasources
//        std::map<int, std::pair<int, int>> datasources_to_add;

//        for (auto ds_key_it : data_source_keys)
//            if (datasources_existing.count(ds_key_it) == 0 &&
//                    added_data_sources_.count(ds_key_it) == 0)
//            {
//                if (datasources_to_add.count(ds_key_it) == 0)
//                {
//                    logdbg << "adding new data source " << ds_key_it;
//                    if (sac_sics.count(ds_key_it) == 0)
//                        datasources_to_add[ds_key_it] = {-1, -1};
//                    else
//                        datasources_to_add[ds_key_it] = {sac_sics.at(ds_key_it).first,
//                                                         sac_sics.at(ds_key_it).second};

//                    added_data_sources_.insert(ds_key_it);
//                }
//            }

//        TODO_ASSERT

////        if (datasources_to_add.size())
////        {
////            dbcontent.addDataSources(datasources_to_add);
////        }

//        dbContent::VariableSet& set = std::get<1>(dbcont_variable_sets_.at(dbcontent_name));

//        loginf << "calling dbcont insert buffer size " << buffer->size()
//               << " num set " << set.getSize();

//        //set.print();

//        //dbcontent.insertData(set, buffer, false);

//        TODO_ASSERT

//        objects_inserted_ += buffer->size();

////        if (dbcontent.name() == "Radar")
////            num_radar_inserted_ += buffer->size(); // store for later check

//        // status_widget_->addNumInserted(dbcontent.name(), buffer->size());
//    }

    logdbg << "done";
}

void JSONImportTask::checkAllDone()
{
    logdbg << "start";

    loginf << "all done " << all_done_ << " read "
           << (read_json_job_ == nullptr) << " parse jobs " << (json_parse_job_ == nullptr)
           << " map jobs " << json_map_jobs_.empty() << " post jobs " << postprocess_jobs_.empty()
           << " insert active " << (insert_active_ == 0);

    if (!all_done_ && read_json_job_ == nullptr && json_parse_job_ == nullptr &&
            json_map_jobs_.size() == 0 && postprocess_jobs_.size() == 0 && insert_active_ == 0)
    {
        stop_time_ = boost::posix_time::microsec_clock::local_time();

        boost::posix_time::time_duration diff = stop_time_ - start_time_;

        std::string time_str =
                String::timeStringFromDouble(diff.total_milliseconds() / 1000.0, false);

        loginf << "read done after " << time_str;

        all_done_ = true;

        double records_per_second = records_inserted_ / (diff.total_milliseconds() / 1000.0);

        COMPASS::instance().logInfo("JSON Import")
            << "done after " << time_str
            << ", inserted " << records_inserted_ << " rec"
            << " with " << String::doubleToStringPrecision(records_per_second, 2) << " rec/s";

        QApplication::restoreOverrideCursor();

        // if (!test_)
        //     emit COMPASS::instance().interface().databaseContentChangedSignal();


        if (!test_)
        {
            loginf << "setting done";

            done_ = true;

            emit doneSignal();
        }

        test_ = false;  // is set again in case of test import
    }

    logdbg << "done";
}

void JSONImportTask::updateMsgBox()
{
    logdbg << "start";

    if (all_done_ && !allow_user_interactions_)
    {
        if (msg_box_)
            msg_box_->close();

        msg_box_ = nullptr;

        loginf << "deleting";

        return;
    }

    if (!msg_box_)
    {
        msg_box_.reset(new QMessageBox());

        loginf << "creating";

        if (test_)
            msg_box_->setWindowTitle("Test Import JSON Data Status");
        else
            msg_box_->setWindowTitle("Import JSON Data Status");

        traced_assert(msg_box_);
    }

    std::string msg;

    msg = "File '" + import_filename_ + "'\n";

    stop_time_ = boost::posix_time::microsec_clock::local_time();

    boost::posix_time::time_duration diff = stop_time_ - start_time_;

    std::string elapsed_time_str =
            String::timeStringFromDouble(diff.total_milliseconds() / 1000.0, false);

    // calculate insert rate
    double objects_per_second = 0.0;
    bool objects_per_second_updated = false;
    if (records_inserted_ && statistics_calc_objects_inserted_ != records_inserted_)
    {
        objects_per_second = records_inserted_ / (diff.total_milliseconds() / 1000.0);
        objects_per_second_updated = true;

        statistics_calc_objects_inserted_ = records_inserted_;
        object_rate_str_ = std::to_string(static_cast<int>(objects_per_second));
    }

    // calculate remaining time
    if (objects_per_second_updated && bytes_to_read_ && objects_parsed_ && objects_mapped_)
    {
        double avg_time_per_obj_s = 1.0 / objects_per_second;

        double avg_mapped_obj_bytes =
                static_cast<double>(bytes_read_) / static_cast<double>(objects_mapped_);
        double num_obj_total = static_cast<double>(bytes_to_read_) / avg_mapped_obj_bytes;

        double remaining_obj_num = 0.0;

        if (objects_not_mapped_ < objects_parsed_)  // skipped objects ok
        {
            double not_skipped_ratio = static_cast<double>(objects_parsed_ - objects_not_mapped_) /
                    static_cast<double>(objects_parsed_);
            remaining_obj_num = (num_obj_total * not_skipped_ratio) - records_inserted_;

            //        loginf << "UGA avg bytes " << avg_obj_bytes << " num total " << num_obj_total
            //        << " not skipped ratio "
            //               << not_skipped_ratio << " all mapped " <<
            //               num_obj_total*not_skipped_ratio
            //               << " obj ins " << objects_inserted_ << " remain obj " <<
            //               remaining_obj_num;
        }
        else  // unknown number of skipped objects
        {
            remaining_obj_num = num_obj_total - records_inserted_;
        }

        if (remaining_obj_num < 0.0)
            remaining_obj_num = 0.0;

        double time_remaining_s = remaining_obj_num * avg_time_per_obj_s;
        remaining_time_str_ = String::timeStringFromDouble(time_remaining_s, false);
    }

    msg += "Elapsed Time: " + elapsed_time_str + "\n";

    if (bytes_read_ > 1e9)
        msg += "Data read: " +
                String::doubleToStringPrecision(static_cast<double>(bytes_read_) * 1e-9, 2) + " GB";
    else
        msg += "Data read: " +
                String::doubleToStringPrecision(static_cast<double>(bytes_read_) * 1e-6, 2) + " MB";

    if (bytes_to_read_)
        msg += " (" + std::to_string(static_cast<int>(read_status_percent_)) + "%)\n\n";
    else
        msg += "\n\n";

    msg += "Objects read: " + std::to_string(objects_read_) + "\n";
    msg += "Objects parsed: " + std::to_string(objects_parsed_) + "\n";
    msg += "Objects parse errors: " + std::to_string(objects_parse_errors_) + "\n\n";

    msg += "Objects mapped: " + std::to_string(objects_mapped_) + "\n";
    msg += "Objects not mapped: " + std::to_string(objects_not_mapped_) + "\n\n";

    msg += "Objects created: " + std::to_string(objects_created_) + "\n";
    msg += "Objects inserted: " + std::to_string(records_inserted_) + "\n\n";

    if (object_rate_str_.size())
        msg += "Object rate: " + object_rate_str_ + " e/s";

    if (!all_done_ && remaining_time_str_.size())
        msg += "\nEstimated remaining time: " + remaining_time_str_;

    msg_box_->setText(msg.c_str());

    if (all_done_)
        msg_box_->setStandardButtons(QMessageBox::Ok);
    else
        msg_box_->setStandardButtons(QMessageBox::NoButton);

    msg_box_->show();

    logdbg << "done";
}

bool JSONImportTask::maxLoadReached()
{
    return json_parse_job_ != nullptr || json_map_jobs_.size() > 2;
}

//void JSONImportTask::insertProgressSlot(float percent)
//{
//    logdbg << "start" << String::percentToString(percent) << "%";
//}

void JSONImportTask::insertDoneSlot()
{
    loginf;
    --insert_active_;

    bool test = test_; // test_ cleared by checkAllDone

    checkAllDone();
    updateMsgBox();

    if (all_done_ && !test)
    {
        loginf << "finalizing";

        disconnect(&COMPASS::instance().dbContentManager(), &DBContentManager::insertDoneSignal,
                   this, &JSONImportTask::insertDoneSlot);
        insert_slot_connected_ = false;

        COMPASS::instance().dataSourceManager().saveDBDataSources();
        emit COMPASS::instance().dataSourceManager().dataSourcesChangedSignal();

        //emit doneSignal(name_); emitted in checkAllDone
    }

    loginf << "done";
}
