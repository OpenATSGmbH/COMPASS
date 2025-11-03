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

#include "createartasassociationstask.h"

#include "compass.h"
#include "createartasassociationstaskdialog.h"
#include "createartasassociationsstatusdialog.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "datasourcemanager.h"
#include "jobmanager.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "viewmanager.h"

#include <QApplication>
#include <QMessageBox>

using namespace std;
using namespace Utils;
using namespace dbContent;

const std::string CreateARTASAssociationsTask::DONE_PROPERTY_NAME = "artas_associations_created"; // really needed

CreateARTASAssociationsTask::CreateARTASAssociationsTask(const std::string& class_id,
                                                         const std::string& instance_id,
                                                         TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_calc_artas_assoc.json")
{
    tooltip_ = "Allows creation of target report association based on ARTAS tracks and the TRI "
               "information.";

    registerParameter("current_data_source_name", &settings_.current_data_source_name_, Settings().current_data_source_name_);
    registerParameter("current_data_source_line_id", &settings_.current_data_source_line_id_, Settings().current_data_source_line_id_);

    // time stuff
    registerParameter("end_track_time", &settings_.end_track_time_, Settings().end_track_time_);

    registerParameter("association_time_past", &settings_.association_time_past_, Settings().association_time_past_);
    registerParameter("association_time_future", &settings_.association_time_future_, Settings().association_time_future_);

    registerParameter("misses_acceptable_time", &settings_.misses_acceptable_time_, Settings().misses_acceptable_time_);

    registerParameter("associations_dubious_distant_time", &settings_.associations_dubious_distant_time_, Settings().associations_dubious_distant_time_);
    registerParameter("association_dubious_close_time_past", &settings_.association_dubious_close_time_past_, Settings().association_dubious_close_time_past_);
    registerParameter("association_dubious_close_time_future", &settings_.association_dubious_close_time_future_, Settings().association_dubious_close_time_future_);

    // track flag stuff
    registerParameter("ignore_track_end_associations", &settings_.ignore_track_end_associations_, Settings().ignore_track_end_associations_);
    registerParameter("mark_track_end_associations_dubious", &settings_.mark_track_end_associations_dubious_, Settings().mark_track_end_associations_dubious_);
    registerParameter("ignore_track_coasting_associations", &settings_.ignore_track_coasting_associations_, Settings().ignore_track_coasting_associations_);
    registerParameter("mark_track_coasting_associations_dubious", &settings_.mark_track_coasting_associations_dubious_, Settings().mark_track_coasting_associations_dubious_);
}

CreateARTASAssociationsTask::~CreateARTASAssociationsTask() {}

void CreateARTASAssociationsTask::showDialog()
{
    CreateARTASAssociationsTaskDialog dialog(*this);

    if (dialog.exec() == QDialog::Rejected)
        return;

    traced_assert(canRun());
    run();

}

CreateARTASAssociationsTask::Error CreateARTASAssociationsTask::checkError() const
{
    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();
    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    logdbg << "tracker " << dbcontent_man.existsDBContent("CAT062");

    if (!dbcontent_man.existsDBContent("CAT062"))
        return CreateARTASAssociationsTask::Error::NoDataSource;

    DBContent& tracker_object = dbcontent_man.dbContent("CAT062");

    // tracker stuff
    logdbg << "tracker loadable " << tracker_object.loadable();

    if (!tracker_object.loadable())
        return CreateARTASAssociationsTask::Error::NoDataSource;

    logdbg << "tracker count " << tracker_object.count();
    if (!tracker_object.count())
        return CreateARTASAssociationsTask::Error::NoDataSource;

    // no data sources
    logdbg << "num tracker data sources "
           << ds_man.hasDataSourcesOfDBContent("CAT062");

    if (!ds_man.hasDataSourcesOfDBContent("CAT062"))
        return CreateARTASAssociationsTask::Error::NoDataSource;

    bool ds_found{false};
    unsigned int current_ds_id {0};
    unsigned int line_count = 0;

    for (auto& ds_it : ds_man.dbDataSources())
    {
        if (!ds_it->numInsertedMap().count("CAT062")) // check if track data exists
            continue;

        if ((ds_it->hasShortName() &&
             ds_it->shortName() == settings_.current_data_source_name_) ||
                (ds_it->name() == settings_.current_data_source_name_))
        {
            ds_found = true;
            current_ds_id = ds_it->id();

            line_count = ds_it->hasNumInserted("CAT062", settings_.current_data_source_line_id_) ? 
                ds_it->numInsertedMap().at("CAT062").at(settings_.current_data_source_line_id_) : 0;

            break;
        }
    }

    logdbg << "tracker ds_found " << ds_found << " id " << current_ds_id;

    if (!ds_found)
        return CreateARTASAssociationsTask::Error::NoDataSource;

    logdbg << "line count " << line_count << " line id " << settings_.current_data_source_line_id_;

    if (!line_count)
        return CreateARTASAssociationsTask::Error::NoDataForLineID;

    logdbg << "tracker vars";

    bool has_needed_cat_62_vars = tracker_object.hasVariable(DBContent::var_cat062_tris_.name()) &&
                                  tracker_object.hasVariable(DBContent::var_cat062_track_begin_.name()) &&
                                  tracker_object.hasVariable(DBContent::var_cat062_coasting_.name()) &&
                                  tracker_object.hasVariable(DBContent::var_cat062_track_end_.name());
    
    if (!has_needed_cat_62_vars)
        logerr << "needed CAT062 vars not available";

    traced_assert(has_needed_cat_62_vars);

    bool has_needed_metavars = dbcontent_man.existsMetaVariable(DBContent::meta_var_rec_num_.name()) &&
                               dbcontent_man.existsMetaVariable(DBContent::meta_var_ds_id_.name()) &&
                               dbcontent_man.existsMetaVariable(DBContent::meta_var_timestamp_.name()) &&
                               dbcontent_man.existsMetaVariable(DBContent::meta_var_track_num_.name()) &&
                               dbcontent_man.existsMetaVariable(DBContent::meta_var_artas_hash_.name()) &&
                               dbcontent_man.existsMetaVariable(DBContent::meta_var_utn_.name());

    if (!has_needed_metavars)
        logerr << "needed metavars not available";

    traced_assert(has_needed_metavars);

    loginf << "no error";

    return CreateARTASAssociationsTask::Error::NoError;
}

bool CreateARTASAssociationsTask::canRun()
{
    return (checkError() == Error::NoError);
}

void CreateARTASAssociationsTask::run()
{
    traced_assert(canRun());

    loginf << "started";

    save_associations_ = true;

    start_time_ = boost::posix_time::microsec_clock::local_time();

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    traced_assert(!status_dialog_);
    status_dialog_.reset(new CreateARTASAssociationsStatusDialog(*this));
    connect(status_dialog_.get(), &CreateARTASAssociationsStatusDialog::closeSignal, this,
            &CreateARTASAssociationsTask::closeStatusDialogSlot);
    status_dialog_->markStartTime();
    status_dialog_->setAssociationStatus("Loading Data");
    status_dialog_->show();

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();
    dbcontent_man.clearData();

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    COMPASS::instance().viewManager().disableDataDistribution(true);

    connect(&dbcontent_man, &DBContentManager::loadedDataSignal,
            this, &CreateARTASAssociationsTask::loadedDataDataSlot);
    connect(&dbcontent_man, &DBContentManager::loadingDoneSignal,
            this, &CreateARTASAssociationsTask::loadingDoneSlot);

    for (auto& dbcont_it : dbcontent_man)
    {
        if (!dbcont_it.second->hasData())
            continue;

        if (!dbcont_it.second->containsTargetReports() ||
            dbcont_it.second->isReferenceContent()) // not covered by ARTAS
            continue;

        VariableSet read_set = getReadSetFor(dbcont_it.first);

        if (dbcont_it.first == "CAT062")
        {
            bool ds_found{false};
            unsigned int current_ds_id;

            for (auto& ds_it : ds_man.dbDataSources())
            {
                if (!ds_it->numInsertedMap().count("CAT062")) // check if track data exists
                    continue;

                if ((ds_it->hasShortName() &&
                     ds_it->shortName() == settings_.current_data_source_name_) ||
                        (ds_it->name() == settings_.current_data_source_name_))
                {
                    ds_found = true;
                    current_ds_id = ds_it->id();
                    break;
                }
            }

            traced_assert(ds_found);
            std::string custom_filter_clause {
                dbcontent_man.metaGetVariable(dbcont_it.first, DBContent::meta_var_ds_id_).dbColumnName()
                        + " in (" + std::to_string(current_ds_id) + ") AND " +
                dbcontent_man.metaGetVariable(dbcont_it.first, DBContent::meta_var_line_id_).dbColumnName()
                        + " in (" + std::to_string(settings_.current_data_source_line_id_) + ")"
            };

            dbcont_it.second->loadFiltered(read_set, custom_filter_clause);
        }
        else
            dbcont_it.second->load(read_set, false, false);

    }
}

bool CreateARTASAssociationsTask::wasRun()
{
    return COMPASS::instance().dbInterface().hasProperty(DONE_PROPERTY_NAME)
             && COMPASS::instance().dbInterface().getProperty(DONE_PROPERTY_NAME) == "1";
}

void CreateARTASAssociationsTask::loadedDataDataSlot(
        const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset)
{
    data_ = data;
}

void CreateARTASAssociationsTask::loadingDoneSlot()
{
    loginf;

    traced_assert(status_dialog_);

    dbcont_loading_done_ = true;

    traced_assert(!create_job_);

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    disconnect(&dbcontent_man, &DBContentManager::loadedDataSignal,
            this, &CreateARTASAssociationsTask::loadedDataDataSlot);
    disconnect(&dbcontent_man, &DBContentManager::loadingDoneSignal,
            this, &CreateARTASAssociationsTask::loadingDoneSlot);

    dbcontent_man.clearData();

    COMPASS::instance().viewManager().disableDataDistribution(false);

    create_job_ = std::make_shared<CreateARTASAssociationsJob>(
                *this, COMPASS::instance().dbInterface(), data_);

    connect(create_job_.get(), &CreateARTASAssociationsJob::doneSignal, this,
            &CreateARTASAssociationsTask::createDoneSlot, Qt::QueuedConnection);
    connect(create_job_.get(), &CreateARTASAssociationsJob::obsoleteSignal, this,
            &CreateARTASAssociationsTask::createObsoleteSlot, Qt::QueuedConnection);
    connect(create_job_.get(), &CreateARTASAssociationsJob::statusSignal, this,
            &CreateARTASAssociationsTask::associationStatusSlot, Qt::QueuedConnection);
    connect(create_job_.get(), &CreateARTASAssociationsJob::saveAssociationsQuestionSignal,
            this, &CreateARTASAssociationsTask::saveAssociationsQuestionSlot,
            Qt::QueuedConnection);

    JobManager::instance().addDBJob(create_job_);

    status_dialog_->setAssociationStatus("In Progress");
}

void CreateARTASAssociationsTask::dialogRunSlot()
{
    loginf;

    traced_assert(canRun());
    run ();
}

void CreateARTASAssociationsTask::createDoneSlot()
{
    loginf;

    traced_assert(create_job_);

    create_job_done_ = true;

    status_dialog_->setAssociationCounts(create_job_->associationCounts());
    status_dialog_->setFoundHashes(create_job_->foundHashes());
    status_dialog_->setMissingHashesAtBeginning(create_job_->missingHashesAtBeginning());
    status_dialog_->setMissingHashes(create_job_->missingHashes());
    status_dialog_->setDubiousAssociations(create_job_->dubiousAssociations());
    status_dialog_->setFoundDuplicates(create_job_->foundHashDuplicates());
    status_dialog_->setAssociationStatus("Done");

    status_dialog_->setDone();

    if (!allow_user_interactions_)
        status_dialog_->close();

    create_job_ = nullptr;

    stop_time_ = boost::posix_time::microsec_clock::local_time();

    boost::posix_time::time_duration diff = stop_time_ - start_time_;

    std::string time_str = String::timeStringFromDouble(diff.total_milliseconds() / 1000.0, false);

    if (save_associations_)
    {
        COMPASS::instance().dbInterface().setProperty(DONE_PROPERTY_NAME, "1");

        COMPASS::instance().dbInterface().saveProperties();

        done_ = true;
    }
    else
        loginf << "done after " << time_str << " without saving";

    QApplication::restoreOverrideCursor();

    emit doneSignal();
}

void CreateARTASAssociationsTask::createObsoleteSlot() { create_job_ = nullptr; }

std::string CreateARTASAssociationsTask::currentDataSourceName() const
{
    return settings_.current_data_source_name_;
}

void CreateARTASAssociationsTask::currentDataSourceName(const std::string& current_data_source_name)
{
    loginf << "start" << current_data_source_name;

    settings_.current_data_source_name_ = current_data_source_name;

    emit dataSourceChanged();
}

unsigned int CreateARTASAssociationsTask::currentDataSourceLineID() const
{
    return settings_.current_data_source_line_id_;
}

void CreateARTASAssociationsTask::currentDataSourceLineID(unsigned int line_id)
{
    loginf << "start" << line_id;

    settings_.current_data_source_line_id_ = line_id;

    emit dataSourceChanged();
}

float CreateARTASAssociationsTask::endTrackTime() const
{
    traced_assert(settings_.end_track_time_);
    return settings_.end_track_time_;
}

void CreateARTASAssociationsTask::endTrackTime(float end_track_time)
{
    loginf << "start" << end_track_time;

    settings_.end_track_time_ = end_track_time;
}

float CreateARTASAssociationsTask::associationTimePast() const { return settings_.association_time_past_; }

void CreateARTASAssociationsTask::associationTimePast(float association_time_past)
{
    loginf << "start" << association_time_past;

    settings_.association_time_past_ = association_time_past;
}

float CreateARTASAssociationsTask::associationTimeFuture() const
{
    return settings_.association_time_future_;
}

void CreateARTASAssociationsTask::associationTimeFuture(float association_time_future)
{
    loginf << "start" << association_time_future;

    settings_.association_time_future_ = association_time_future;
}

float CreateARTASAssociationsTask::missesAcceptableTime() const { return settings_.misses_acceptable_time_; }

void CreateARTASAssociationsTask::missesAcceptableTime(float misses_acceptable_time)
{
    loginf << "start" << misses_acceptable_time;

    settings_.misses_acceptable_time_ = misses_acceptable_time;
}

float CreateARTASAssociationsTask::associationsDubiousDistantTime() const
{
    return settings_.associations_dubious_distant_time_;
}

void CreateARTASAssociationsTask::associationsDubiousDistantTime(
        float associations_dubious_distant_time)
{
    loginf << "start"
           << associations_dubious_distant_time;

    settings_.associations_dubious_distant_time_ = associations_dubious_distant_time;
}

float CreateARTASAssociationsTask::associationDubiousCloseTimePast() const
{
    return settings_.association_dubious_close_time_past_;
}

void CreateARTASAssociationsTask::associationDubiousCloseTimePast(
        float association_dubious_close_time_past)
{
    loginf << association_dubious_close_time_past;

    settings_.association_dubious_close_time_past_ = association_dubious_close_time_past;
}

float CreateARTASAssociationsTask::associationDubiousCloseTimeFuture() const
{
    return settings_.association_dubious_close_time_future_;
}

void CreateARTASAssociationsTask::associationDubiousCloseTimeFuture(
        float association_dubious_close_time_future)
{
    loginf << "start"
           << association_dubious_close_time_future;

    settings_.association_dubious_close_time_future_ = association_dubious_close_time_future;
}

bool CreateARTASAssociationsTask::ignoreTrackEndAssociations() const
{
    return settings_.ignore_track_end_associations_;
}

void CreateARTASAssociationsTask::ignoreTrackEndAssociations(bool value)
{
    loginf << "value " << value;
    settings_.ignore_track_end_associations_ = value;
}

bool CreateARTASAssociationsTask::markTrackEndAssociationsDubious() const
{
    return settings_.mark_track_end_associations_dubious_;
}

void CreateARTASAssociationsTask::markTrackEndAssociationsDubious(bool value)
{
    loginf << "value " << value;
    settings_.mark_track_end_associations_dubious_ = value;
}

bool CreateARTASAssociationsTask::ignoreTrackCoastingAssociations() const
{
    return settings_.ignore_track_coasting_associations_;
}

void CreateARTASAssociationsTask::ignoreTrackCoastingAssociations(bool value)
{
    loginf << "value " << value;
    settings_.ignore_track_coasting_associations_ = value;
}

bool CreateARTASAssociationsTask::markTrackCoastingAssociationsDubious() const
{
    return settings_.mark_track_coasting_associations_dubious_;
}

void CreateARTASAssociationsTask::markTrackCoastingAssociationsDubious(bool value)
{
    loginf << "value " << value;
    settings_.mark_track_coasting_associations_dubious_ = value;
}

VariableSet CreateARTASAssociationsTask::getReadSetFor(const std::string& dbcontent_name)
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    VariableSet read_set;

    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_utn_));

    if (dbcontent_name == "CAT062")
    {
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_));
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_begin_));
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_end_));
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_coasting_));

        read_set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat062_tris_));
        read_set.add(dbcont_man.getVariable(dbcontent_name, DBContent::var_cat062_tri_recnums_));
    }
    else if (dbcont_man.dbContent(dbcontent_name).containsTargetReports())
    {
        read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_artas_hash_));
    }

    // must be last for update process
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));

    return read_set;
}

void CreateARTASAssociationsTask::associationStatusSlot(QString status)
{
    traced_assert(status_dialog_);
    status_dialog_->setAssociationStatus(status.toStdString());
}

void CreateARTASAssociationsTask::saveAssociationsQuestionSlot(QString question_str)
{
    traced_assert(status_dialog_);
    traced_assert(create_job_);

    status_dialog_->setAssociationCounts(create_job_->associationCounts());
    status_dialog_->setFoundHashes(create_job_->foundHashes());
    status_dialog_->setMissingHashesAtBeginning(create_job_->missingHashesAtBeginning());
    status_dialog_->setMissingHashes(create_job_->missingHashes());
    status_dialog_->setDubiousAssociations(create_job_->dubiousAssociations());
    status_dialog_->setFoundDuplicates(create_job_->foundHashDuplicates());

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Malformed Associations", question_str,
                                  QMessageBox::Yes | QMessageBox::No);

    save_associations_ = reply == QMessageBox::Yes;

    create_job_->setSaveQuestionAnswer(save_associations_);
}


void CreateARTASAssociationsTask::closeStatusDialogSlot()
{
    traced_assert(status_dialog_);
    status_dialog_->close();
    status_dialog_ = nullptr;
}
