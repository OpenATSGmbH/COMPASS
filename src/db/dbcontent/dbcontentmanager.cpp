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
#include "viewabledataconfig.h"
#include "buffer_utils.h"
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
#include "db_context_manager.h"
#include "stringconv.h"
#include "viewmanager.h"
#include "jobmanager.h"
#include "evaluationmanager.h"
#include "filtermanager.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "dbcontent/db_content_edit_dialog.h"
#include "dbcontentdeletedbjob.h"
#include "dbcontentdataengine.h"
#include "loadoperation.h"
#include "livedatafeed.h"
#include "dbcontent_commands.h"
#include "viewpoint.h"
#include "dbcontentinsertdbjob.h"
#include "timeconv.h"

#include "util/tbbhack.h"

#include <QApplication>
#include <QMessageBox>
#include <QPushButton>
#include <QThread>

#include <algorithm>
#include <boost/none.hpp>
#include <string>

using namespace std;
using namespace Utils;
using namespace dbContent;


DBContentManager::DBContentManager(nlohmann::json& config, COMPASS& compass)
    : Configurable(config, &compass), compass_(compass)
{
    registerParameter("show_data_counts", &show_data_counts_, false);

    logdbg << "creating subconfigurables";

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

    data_engine_.reset(new DBContentDataEngine(*this));
    // forward the engine's delete completion as the manager's public signal so
    // existing consumers keep working
    connect(data_engine_.get(), &DBContentDataEngine::dataDeletedSignal,
            this, &DBContentManager::dataDeletedSignal);
    connect(data_engine_.get(), &DBContentDataEngine::insertDoneSignal,
            this, &DBContentManager::insertDoneSignal);

    // the live feed is owned by ViewManager (the displayer); the engine's
    // insertedDataSignal feeds it and ViewManager drives the live tick
}

/**
 */
DBContentManager::~DBContentManager()
{
    logdbg;

    for (auto it : dbcontent_)
        delete it.second;
    dbcontent_.clear();

    meta_variables_.clear();

    widget_ = nullptr;

    logdbg << "done";
}

/**
 */
void DBContentManager::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);
    logdbg << "class_name " << class_name;

    if (class_name == "DBContent")
    {
        DBContent* object = new DBContent(child_json, this);
        loginf << "adding content " << object->name()
               << " id " << object->id();
        traced_assert(!dbcontent_.count(object->name()));
        traced_assert(!dbcontent_ids_.count(object->id()));

        dbcontent_[object->name()] = object;
        dbcontent_ids_[object->id()] = object;
    }
    else if (class_name == "MetaVariable")
    {
        MetaVariable* meta_var = new MetaVariable(child_json, this);
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
        throw std::runtime_error("DBContentManager: generateSubConfigurable: unknown class_name " +
                                 class_name);
}

/**
 */
bool DBContentManager::existsDBContent(const std::string& dbcontent_name) const
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
const DBContent& DBContentManager::dbContent(const std::string& dbcontent_name) const
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
bool DBContentManager::hasActiveDeleteJob() const
{
    return data_engine_->hasActiveDeleteJob();
}

/**
 */
void DBContentManager::deleteData(const nlohmann::json& delete_info)
{
    loginf;

    traced_assert(!hasActiveDeleteJob()); // caller checks hasActiveDeleteJob()

    // wipe the display before the DB delete (a view concern kept out of the engine)
    compass_.viewManager().clearDataInViews();

    data_engine_->deleteData(delete_info);
}

/**
 */
void DBContentManager::deleteDBContentData(boost::posix_time::ptime before_timestamp)
{
    loginf;

    data_engine_->deleteOlderThan(before_timestamp);
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
unsigned int DBContentManager::getMaxDBContentID() const
{
    unsigned int ret = 0;

    for (auto& object_it : dbcontent_)
        ret = max(ret, object_it.second->id());

    return ret;
}

/**
 */
bool DBContentManager::existsDBContentWithId (unsigned int id) const
{
    return dbcontent_ids_.count(id);
}

/**
 */
const std::string& DBContentManager::dbContentWithId (unsigned int id) const
{
    traced_assert(dbcontent_ids_.count(id));
    return dbcontent_ids_.at(id)->name();
}

/**
 */
unsigned int DBContentManager::dbContentId(const std::string& dbcont_name) const
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


    if (db_content_edit_dialog_)
        db_content_edit_dialog_->rebuildTree();
}

/**
 */
void DBContentManager::deleteMetaVariable(const std::string& var_name)
{
    logdbg << "name " << var_name;
    traced_assert(existsMetaVariable(var_name));

    meta_variables_.erase(var_name);

    if (db_content_edit_dialog_)
        db_content_edit_dialog_->rebuildTree();
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
    EvaluationManager& eval_man = compass_.evaluationManager();
    ViewManager& view_man       = compass_.viewManager();

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
void DBContentManager::databaseOpenedSlot()
{
    loginf;

    data_engine_->loadMaxRecordNumberWODBContentID();
    data_engine_->loadMaxRefTrajTrackNum();
    data_engine_->loadMinMaxInfo();

    DBInterface& db_interface = compass_.dbInterface();

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

    data_engine_->clearMaxNumbers();
    data_engine_->clearMinMaxInfo();

    has_associations_ = false;
    associations_id_ = "";

    for (auto& object : dbcontent_)
        object.second->databaseClosedSlot();

    target_model_->clear();

    emit associationStatusChangedSignal();
    emit dbContentStatusChanged();
}

/**
 */
void DBContentManager::dbContentEditDialogOKSlot()
{
    traced_assert(db_content_edit_dialog_);
    db_content_edit_dialog_->hide();
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
    auto& dbinterface = compass_.dbInterface();

    dbinterface.setProperty("associations_generated", "1");
    dbinterface.setProperty("associations_id", assoc_id);
    dbinterface.saveProperties();

    has_associations_ = true;
    associations_id_ = assoc_id;

    // updateWidgets removed - handled by signals

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

    auto& dbinterface = compass_.dbInterface();

    if (dbinterface.hasProperty("associations_generated"))
        dbinterface.removeProperty("associations_generated");

    if (dbinterface.hasProperty("associations_id"))
        dbinterface.removeProperty("associations_id");

    dbinterface.saveProperties();

    // updateWidgets removed - handled by signals

    emit associationStatusChangedSignal();
}

/**
 */
void DBContentManager::insertData(std::map<std::string, std::shared_ptr<Buffer>> data)
{
    logdbg;

    data_engine_->insert(std::move(data));
}

/**
 * No-op: the live-session load bookends moved to ViewManager::appModeSwitchSlot (it owns
 * the feed and the load lifecycle). Kept as a hook for any future manager-side app-mode
 * reaction; the connection in COMPASS is retained for the same reason.
 */
void DBContentManager::appModeSwitchSlot(AppMode app_mode_previous, AppMode app_mode_current)
{
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

std::set<unsigned int> DBContentManager::getIgnoredUTNs() const
{
    traced_assert(hasAssociations());
    traced_assert(hasTargetsInfo());

    return target_model_->getIgnoredUTNs();
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
    return data_engine_->insertInProgress();
}

/**
 * Adds the always-present load core (record number, data source id, line id and
 * the ordering timestamp); with add_utn_if_available also adds utn when the
 * content has it. Single source for the standard load variables.
 */
void DBContentManager::addStandardVariables(std::string dbcont_name, dbContent::VariableSet& read_set,
                                            bool add_utn_if_available)
{
    traced_assert(metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_rec_num_));
    read_set.add(metaGetVariable(dbcont_name, dbcontent_vars::meta_var_rec_num_));

    traced_assert(metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_ds_id_));
    read_set.add(metaGetVariable(dbcont_name, dbcontent_vars::meta_var_ds_id_));

    traced_assert(metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_line_id_));
    read_set.add(metaGetVariable(dbcont_name, dbcontent_vars::meta_var_line_id_));

    traced_assert(metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_));
    read_set.add(metaGetVariable(dbcont_name, dbcontent_vars::meta_var_timestamp_));

    if(add_utn_if_available && metaCanGetVariable(dbcont_name, dbcontent_vars::meta_var_utn_))
        read_set.add(metaGetVariable(dbcont_name, dbcontent_vars::meta_var_utn_));
}

/**
 */
DBContentEditDialog* DBContentManager::dbContentEditDialog()
{
    if (!db_content_edit_dialog_)
    {
        db_content_edit_dialog_.reset(new DBContentEditDialog(*this));

        connect(db_content_edit_dialog_.get(), &DBContentEditDialog::okSignal,
                this, &DBContentManager::dbContentEditDialogOKSlot);
    }

    traced_assert(db_content_edit_dialog_);
    return db_content_edit_dialog_.get();
}

/**
 */
void DBContentManager::setViewableDataConfig (const nlohmann::json::object_t& data)
{
    viewable_data_cfg_.reset(new ViewableDataConfig(data));

    compass_.viewManager().setCurrentViewPoint(viewable_data_cfg_.get());
}

void DBContentManager::storeSelectedRecNums(const std::vector<unsigned long>& selected)
{
    // selection carry-over lives in ViewManager; this is a thin façade for lower layers
    compass_.viewManager().storeSelectedRecNums(selected);
}

bool DBContentManager::hasMaxLatency() const
{
    // the live session (owned by ViewManager) owns the latency; the manager only forwards it
    return compass_.viewManager().hasMaxLatency();
}

boost::posix_time::time_duration DBContentManager::maxLatency() const
{
    return compass_.viewManager().maxLatency();
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
    {
        compass_.viewManager().clearDataInViews();
    }
}

