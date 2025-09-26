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

#include "evaluationmanager.h"
#include "eval/results/report/pdfgeneratordialog.h"
#include "evaluationstandard.h"
#include "evaluationdialog.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/baseconfig.h"
#include "eval/evaluation_commands.h"
#include "evaluationsettings.h"
#include "evaluationtargetfilter.h"
#include "evaluationtaskresult.h"

#include "sectorlayer.h"
#include "sector.h"
#include "airspace.h"

#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/variable/variable.h"

#include "compass.h"
#include "dbinterface.h"
#include "datasourcemanager.h"
#include "filtermanager.h"
#include "viewmanager.h"
#include "taskmanager.h"

#include "buffer.h"
#include "dbfilter.h"
#include "viewpoint.h"
#include "viewabledataconfig.h"

#include "taskdefs.h"

#include "stringconv.h"
#include "timeconv.h"

#include "json.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QThread>
#include <QMessageBox>

#include <memory>
#include <fstream>
#include <cstdlib>
#include <system.h>

using namespace Utils;
using namespace std;
using namespace nlohmann;
using namespace boost::posix_time;

const std::string EVAL_TIME_CONSTRAINTS_PROPRTY_NAME {"eval_time_constraints"};
const std::string EVAL_TIME_CONSTRAINTS_USE {"use"};
const std::string EVAL_TIME_CONSTRAINTS_BEGIN {"begin"};
const std::string EVAL_TIME_CONSTRAINTS_END {"end"};
const std::string EVAL_TIME_CONSTRAINTS_EXCLUDED_WINDOWS {"excluded_windows"};

/**
 */
EvaluationManager::EvaluationManager(const std::string& class_id, 
                                     const std::string& instance_id, 
                                     COMPASS* compass)
:   Configurable(class_id, instance_id, compass, "eval.json")
//,   compass_    (*compass)
    , dbcontent_man_(compass->dbContentManager())
{
    createSubConfigurables();
    init_evaluation_commands();
}

/**
 */
EvaluationManager::~EvaluationManager()
{
    sector_layers_.clear();
}

/**
*/
void EvaluationManager::generateSubConfigurable(const std::string& class_id,
                                                const std::string& instance_id)
{
    if (class_id == "EvaluationTargetFilter")
    {
        traced_assert(!target_filter_);
        target_filter_.reset(new EvaluationTargetFilter(class_id, instance_id, *this));
    }
    else if (class_id == "EvaluationCalculator")
    {
        traced_assert(!calculator_);

        EvaluationCalculator* calculator = new EvaluationCalculator(class_id, instance_id, *this, dbcontent_man_);
        calculator_.reset(calculator);
    }
    else
    {
        throw std::runtime_error("EvaluationManager: generateSubConfigurable: unknown class_id " + class_id);
    }
}

/**
*/
void EvaluationManager::checkSubConfigurables()
{
    if (!target_filter_)
    {
        generateSubConfigurable("EvaluationTargetFilter", "EvaluationTargetFilter0");
        traced_assert(target_filter_);
    }

    if (!calculator_)
    {
        //generate default calculator
        generateSubConfigurable("EvaluationCalculator", "EvaluationCalculator0");
        traced_assert(calculator_);
    }
}

/**
 */
void EvaluationManager::init()
{
    loginf;

    traced_assert(!initialized_);
    initialized_ = true;

    auto& dbc_manager = COMPASS::instance().dbContentManager();

    connect (&dbc_manager, &DBContentManager::associationStatusChangedSignal,
            this, &EvaluationManager::associationStatusChangedSlot);
    connect (dbc_manager.targetModel(), &dbContent::TargetModel::targetInfoChangedSignal,
            this, &EvaluationManager::targetInfoChangedSlot);
    connect (dbc_manager.targetModel(), &dbContent::TargetModel::targetEvalUsageChangedSignal,
            this, &EvaluationManager::partialResultsUpdateNeededSlot);
    connect (dbc_manager.targetModel(), &dbContent::TargetModel::targetEvalFullChangeSignal,
            this, &EvaluationManager::fullResultsUpdateNeededSlot);
    connect (dbc_manager.targetModel(), &dbContent::TargetModel::targetsDeletedSignal,
            this, &EvaluationManager::lockResultsSlot);

    auto& task_manager = COMPASS::instance().taskManager();

    connect (&task_manager, &TaskManager::taskRadarPlotPositionsDoneSignal, 
             this, &EvaluationManager::lockResultsSlot);
}

/**
 */
void EvaluationManager::close()
{
    initialized_ = false;
}

/**
 */
void EvaluationManager::clearData()
{
    traced_assert(calculator_);
    return calculator_->clearData();
}

/**
 */
Result EvaluationManager::canEvaluate() const
{
    traced_assert(initialized_);
    traced_assert(calculator_);

    return calculator_->canEvaluate();
}

/**
 */
Result EvaluationManager::evaluate(bool show_dialog)
{
    loginf;

    traced_assert(initialized_);
    traced_assert(calculator_);

    //show config dialog?
    if (show_dialog)
    {
        EvaluationDialog dlg(*calculator_);
        if (dlg.exec() == QDialog::Rejected)
            return Result::succeeded();
    }

    //create clone of current calculator
    auto res = calculator_->clone();

    if (!res.ok())
        logerr << "evaluation error: " << res.error();
    traced_assert(res.ok());

    auto calculator_local = res.result();
    traced_assert(calculator_local);

    //evaluate
    auto eval_res = calculator_local->evaluate(true);

    if (!eval_res.ok())
    {
        //interaction mode => show error immediately
        if (show_dialog)
            QMessageBox::critical(nullptr, "Evaluation Failed", QString::fromStdString(eval_res.error()));

        return eval_res;
    }

    traced_assert(calculator_local->evaluated());

    //store calculator to task result
    auto& task_man = COMPASS::instance().taskManager();

    traced_assert(task_man.hasResult(calculator_local->resultName()));

    auto task_result = task_man.result(calculator_local->resultName());
    traced_assert(task_result);

    auto eval_result = dynamic_cast<EvaluationTaskResult*>(task_result.get());
    traced_assert(eval_result);

    eval_result->injectCalculator(calculator_local);

    last_result_name_ = calculator_local->resultName();

    emit evaluationDoneSignal();

    return Result::succeeded();
}

/**
 */
void EvaluationManager::databaseOpenedSlot()
{
    loginf;

    traced_assert(calculator_);

    traced_assert(!sectors_loaded_);
    loadSectors();

    //load sectors before locking any results via this connections
    connect(this, &EvaluationManager::sectorsChangedSignal, this, &EvaluationManager::lockResultsSlot);

    auto& dbinterface = COMPASS::instance().dbInterface();

    use_timestamp_filter_ = false;
    load_timestamp_begin_ = {};
    load_timestamp_end_ = {};
    load_filtered_time_windows_.clear();

    if (dbinterface.hasProperty(EVAL_TIME_CONSTRAINTS_PROPRTY_NAME))
    {
        std::string constraints_str = dbinterface.getProperty(EVAL_TIME_CONSTRAINTS_PROPRTY_NAME);

        try
        {
            nlohmann::json constraints_json = nlohmann::json::parse(constraints_str);

            if (constraints_json.contains(EVAL_TIME_CONSTRAINTS_USE))
                use_timestamp_filter_ = constraints_json.at(EVAL_TIME_CONSTRAINTS_USE);

            if (constraints_json.contains(EVAL_TIME_CONSTRAINTS_BEGIN))
                load_timestamp_begin_ = Time::fromString(constraints_json.at(EVAL_TIME_CONSTRAINTS_BEGIN));

            if (constraints_json.contains(EVAL_TIME_CONSTRAINTS_END))
                load_timestamp_end_ = Time::fromString(constraints_json.at(EVAL_TIME_CONSTRAINTS_END));

            if (constraints_json.contains(EVAL_TIME_CONSTRAINTS_EXCLUDED_WINDOWS))
                load_filtered_time_windows_.setFrom(constraints_json.at(EVAL_TIME_CONSTRAINTS_EXCLUDED_WINDOWS));
        }
        catch (std::exception& e)
        {
            logerr << "unsupported eval_time_ '"
                   << constraints_str << "': " << e.what();
        }
    }

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcont_man.hasMinMaxTimestamp())
    {
        std::pair<boost::posix_time::ptime , boost::posix_time::ptime> minmax_ts =  dbcont_man.minMaxTimestamp();

        if (load_timestamp_begin_.is_not_a_date_time())
            load_timestamp_begin_ = get<0>(minmax_ts);

        if (load_timestamp_end_.is_not_a_date_time())
            load_timestamp_end_ = get<1>(minmax_ts);
    }

    // init with false values if not in cfg
    calculator_->checkReferenceDataSources();
    calculator_->checkTestDataSources();
    calculator_->checkMinHeightFilterValid();
}

/**
 */
void EvaluationManager::databaseClosedSlot()
{
    loginf;

    //disconnect result locking before clearing the sectors
    disconnect(this, &EvaluationManager::sectorsChangedSignal, this, &EvaluationManager::lockResultsSlot);

    use_timestamp_filter_ = false;
    load_timestamp_begin_ = {};
    load_timestamp_end_ = {};
    load_filtered_time_windows_.clear();

    traced_assert(calculator_);

    calculator_->reset();

    clearSectors();
    resetViewableDataConfig(true);
}

/**
 */
void EvaluationManager::dataSourcesChangedSlot()
{
    traced_assert(calculator_);

    calculator_->checkReferenceDataSources();
    calculator_->checkTestDataSources();
}

/**
 */
void EvaluationManager::associationStatusChangedSlot()
{
    // react on association status change
}

void EvaluationManager::targetInfoChangedSlot()
{
    loginf;

    emit resultsNeedUpdate(task::UpdateState::ContentUpdateNeeded);
}

void EvaluationManager::partialResultsUpdateNeededSlot()
{
    loginf;

    emit resultsNeedUpdate(task::UpdateState::PartialUpdateNeeded);
}

void EvaluationManager::fullResultsUpdateNeededSlot()
{
    loginf;

    emit resultsNeedUpdate(task::UpdateState::FullUpdateNeeded);
}

void EvaluationManager::lockResultsSlot()
{
    loginf;

    emit resultsNeedUpdate(task::UpdateState::Locked);
}

void EvaluationManager::saveTimeConstraints()
{
    loginf;

    nlohmann::json constraints_json = nlohmann::json::object();

    constraints_json[EVAL_TIME_CONSTRAINTS_USE] = use_timestamp_filter_;

    constraints_json[EVAL_TIME_CONSTRAINTS_BEGIN] = Time::toString(load_timestamp_begin_);
    constraints_json[EVAL_TIME_CONSTRAINTS_END] = Time::toString(load_timestamp_end_);

    constraints_json[EVAL_TIME_CONSTRAINTS_EXCLUDED_WINDOWS] = load_filtered_time_windows_.asJSON();

    COMPASS::instance().dbInterface().setProperty(EVAL_TIME_CONSTRAINTS_PROPRTY_NAME, constraints_json.dump());
}

/**
 */
bool EvaluationManager::needsAdditionalVariables() const
{
    return needs_additional_variables_;
}

/**
 */
void EvaluationManager::addVariables (const std::string dbcontent_name, dbContent::VariableSet& read_set)
{
    loginf << "dbcontent_name " << dbcontent_name;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    if (!dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_latitude_))
        return;

    // TODO add required variables from standard requirements

    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rec_num_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_utn_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_latitude_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_longitude_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_max_stddev_xy))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_max_stddev_xy));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_));

    // flight level
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_g_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_g_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_v_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_v_));

    //if (settings_.dbcontent_name_ref_ == dbcontent_name && settings_.dbcontent_name_ref_ == "CAT062")

    // flight level trusted
    if (dbcontent_name == "CAT062")
    {
        read_set.add(dbcontent_man.getVariable("CAT062", DBContent::var_cat062_baro_alt_));
        read_set.add(dbcontent_man.getVariable("CAT062", DBContent::var_cat062_fl_measured_));
    }

    // m3a
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_g_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_g_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_v_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_v_));

    // tn
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_num_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_));

    // ground bit
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ground_bit_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_bit_));

    // speed & track angle
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_angle_));

    // accs
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ax_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ax_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ay_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ay_));

    // rocd
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_rocd_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rocd_));

    // moms
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mom_long_acc_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_long_acc_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mom_trans_acc_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_trans_acc_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mom_vert_rate_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_vert_rate_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_coasting_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_coasting_));
}

/**
*/
bool EvaluationManager::hasSectorLayer(const std::string& layer_name) const
{
    traced_assert(sectors_loaded_);

    auto iter = std::find_if(sector_layers_.begin(), sector_layers_.end(),
                             [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name;});

    return iter != sector_layers_.end();
}

/**
*/
//void EvaluationManager::renameSectorLayer (const std::string& name, const std::string& new_name)
//{
//    // TODO
//}

/**
*/
std::shared_ptr<SectorLayer> EvaluationManager::sectorLayer (const std::string& layer_name) const
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSectorLayer(layer_name));

    auto iter = std::find_if(sector_layers_.begin(), sector_layers_.end(),
                             [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name;});
    traced_assert(iter != sector_layers_.end());

    return *iter;
}

/**
*/
void EvaluationManager::updateMaxSectorID()
{
    for (auto& sec_lay_it : sector_layers_)
    {
        for (auto& sec_it : sec_lay_it->sectors())
            max_sector_id_ = std::max(max_sector_id_, sec_it->id());
    }
}

/**
*/
void EvaluationManager::loadSectors()
{
    loginf;

    traced_assert(!sectors_loaded_);

    if (!COMPASS::instance().dbInterface().ready())
    {
        sectors_loaded_ = false;
        return;
    }

    sector_layers_ = COMPASS::instance().dbInterface().loadSectors();
    sectors_loaded_ = true;

    updateMaxSectorID();
    
    emit sectorsChangedSignal();
}

/**
*/
void EvaluationManager::clearSectors()
{
    loginf;

    sector_layers_.clear();
    sectors_loaded_ = false;

    emit sectorsChangedSignal();
}

/**
 */
void EvaluationManager::updateSectorLayers()
{
    if (sectors_loaded_)
    {
        for (const auto& layer : sectorsLayers())
            for (const auto& s : layer->sectors())
                s->createFastInsideTest();
    }
}

/**
 */
unsigned int EvaluationManager::getMaxSectorId ()
{
    traced_assert(sectors_loaded_);
    return max_sector_id_;
}

/**
 */
void EvaluationManager::createNewSector(const std::string& name, 
                                        const std::string& layer_name,
                                        bool exclude, 
                                        QColor color, 
                                        std::vector<std::pair<double,double>> points)
{
    loginf << "start"
           << " name " << name 
           << " layer_name " << layer_name
           << " num points " << points.size();

    traced_assert(sectors_loaded_);
    traced_assert(!hasSector(name, layer_name));
    traced_assert(calculator_);

    ++max_sector_id_; // new max

    shared_ptr<Sector> sector(new Sector(max_sector_id_, name, layer_name, true, exclude, color, points));

    // add to existing sectors
    if (!hasSectorLayer(layer_name))
    {
        sector_layers_.push_back(make_shared<SectorLayer>(layer_name));
    }

    traced_assert(hasSectorLayer(layer_name));

    sectorLayer(layer_name)->addSector(sector);

    traced_assert(hasSector(name, layer_name));
    sector->save();

    calculator_->clearData();

    emit sectorsChangedSignal();
}

/**
 */
bool EvaluationManager::hasSector (const string& name, const string& layer_name)
{
    traced_assert(sectors_loaded_);

    if (!hasSectorLayer(layer_name))
        return false;

    return sectorLayer(layer_name)->hasSector(name);
}

/**
 */
bool EvaluationManager::hasSector (unsigned int id)
{
    traced_assert(sectors_loaded_);

    for (auto& sec_lay_it : sector_layers_)
    {
        auto& sectors = sec_lay_it->sectors();
        auto iter = std::find_if(sectors.begin(), sectors.end(),
                                 [id](const shared_ptr<Sector>& x) { return x->id() == id;});
        if (iter != sectors.end())
            return true;
    }
    return false;
}

/**
 */
std::shared_ptr<Sector> EvaluationManager::sector (const string& name, const string& layer_name)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(name, layer_name));

    return sectorLayer(layer_name)->sector(name);
}

/**
 */
std::shared_ptr<Sector> EvaluationManager::sector (unsigned int id)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));

    for (auto& sec_lay_it : sector_layers_)
    {
        auto& sectors = sec_lay_it->sectors();
        auto iter = std::find_if(sectors.begin(), sectors.end(),
                                 [id](const shared_ptr<Sector>& x) { return x->id() == id;});
        if (iter != sectors.end())
            return *iter;
    }

    logerr << "id " << id << " not found";
    traced_assert(false);
}

/**
 */
void EvaluationManager::moveSector(unsigned int id, const std::string& old_layer_name, const std::string& new_layer_name)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));
    traced_assert(calculator_);

    shared_ptr<Sector> tmp_sector = sector(id);

    traced_assert(hasSectorLayer(old_layer_name));
    sectorLayer(old_layer_name)->removeSector(tmp_sector);

    if (!hasSectorLayer(new_layer_name))
        sector_layers_.push_back(make_shared<SectorLayer>(new_layer_name));

    traced_assert(hasSectorLayer(new_layer_name));
    sectorLayer(new_layer_name)->addSector(tmp_sector);

    traced_assert(hasSector(tmp_sector->name(), new_layer_name));
    tmp_sector->save();

    calculator_->clearData();

    emit sectorsChangedSignal();
}

/**
 */
std::vector<std::shared_ptr<SectorLayer>>& EvaluationManager::sectorsLayers()
{
    traced_assert(sectors_loaded_);

    return sector_layers_;
}

/**
 */
void EvaluationManager::saveSector(unsigned int id)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(id));

    saveSector(sector(id));
}

/**
 */
void EvaluationManager::saveSector(std::shared_ptr<Sector> sector)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(sector->name(), sector->layerName()));
    COMPASS::instance().dbInterface().saveSector(sector);
}

/**
 */
void EvaluationManager::deleteSector(shared_ptr<Sector> sector)
{
    traced_assert(sectors_loaded_);
    traced_assert(hasSector(sector->name(), sector->layerName()));
    traced_assert(calculator_);

    string layer_name = sector->layerName();

    traced_assert(hasSectorLayer(layer_name));

    sectorLayer(layer_name)->removeSector(sector);

    // remove sector layer if empty
    if (!sectorLayer(layer_name)->size())
    {
        auto iter = std::find_if(sector_layers_.begin(), sector_layers_.end(),
                                 [&layer_name](const shared_ptr<SectorLayer>& x) { return x->name() == layer_name;});

        traced_assert(iter != sector_layers_.end());
        sector_layers_.erase(iter);

        calculator_->checkMinHeightFilterValid();
    }

    COMPASS::instance().dbInterface().deleteSector(sector);

    calculator_->clearData();

    emit sectorsChangedSignal();
}

/**
 */
void EvaluationManager::deleteAllSectors()
{
    traced_assert(sectors_loaded_);
    traced_assert(calculator_);

    sector_layers_.clear();

    COMPASS::instance().dbInterface().deleteAllSectors();

    calculator_->checkMinHeightFilterValid();
    calculator_->clearData();

    emit sectorsChangedSignal();
}

/**
 */
void EvaluationManager::importSectors(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    traced_assert(sectors_loaded_);
    traced_assert(calculator_);

    sector_layers_.clear();
    COMPASS::instance().dbInterface().clearSectorsTable();

    std::ifstream input_file(filename, std::ifstream::in);

    try
    {
        json j = json::parse(input_file);

        if (!j.contains("sectors"))
        {
            logerr << "file does not contain sectors";
            return;
        }

        json& sectors = j["sectors"];

        if (!sectors.is_array())
        {
            logerr << "file sectors is not an array";
            return;
        }

        unsigned int id;
        string name;
        string layer_name;
        string json_str;

        for (auto& j_sec_it : sectors.get<json::array_t>())
        {
            if (!j_sec_it.contains("id")
                    || !j_sec_it.contains("name")
                    || !j_sec_it.contains("layer_name")
                    || !j_sec_it.contains("points"))
            {
                logerr << "ill-defined sectors skipped, json '" << j_sec_it.dump(4)
                       << "'";
                continue;
            }

            id = j_sec_it.at("id");
            name = j_sec_it.at("name");
            layer_name = j_sec_it.at("layer_name");

            auto eval_sector = new Sector(id, name, layer_name, true);
            eval_sector->readJSON(j_sec_it.dump());

            if (!hasSectorLayer(layer_name))
                sector_layers_.push_back(make_shared<SectorLayer>(layer_name));

            sectorLayer(layer_name)->addSector(shared_ptr<Sector>(eval_sector));

            traced_assert(hasSector(name, layer_name));

            eval_sector->save();

            loginf << "loaded sector '" << name << "' in layer '"
                   << layer_name << "' num points " << sector(name, layer_name)->size() << " id " << id;
        }
    }
    catch (json::exception& e)
    {
        logerr << "could not load file '"
               << filename << "'";
        throw e;
    }

    calculator_->checkMinHeightFilterValid();
    calculator_->clearData();

    emit sectorsChangedSignal();
}

/**
 */
void EvaluationManager::exportSectors (const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    traced_assert(sectors_loaded_);

    json j;

    j["sectors"] = json::array();
    json& sectors = j["sectors"];

    unsigned int cnt = 0;

    for (auto& sec_lay_it : sector_layers_)
    {
        for (auto& sec_it : sec_lay_it->sectors())
        {
            sectors[cnt] = sec_it->jsonData();
            ++cnt;
        }
    }

    std::ofstream output_file;
    output_file.open(filename, std::ios_base::out);

    output_file << j.dump(4);
}

/**
 */
bool EvaluationManager::importAirSpace(const AirSpace& air_space,
                                       const boost::optional<std::set<std::string>>& sectors_to_import)
{
    auto layers = air_space.layers();
    if (layers.empty())
        return false;

    std::vector<std::shared_ptr<SectorLayer>> new_layers;

    for (auto l : layers)
    {
        std::vector<std::shared_ptr<Sector>> sectors;
        for (auto s : l->sectors())
        {
            if (sectors_to_import.has_value() && sectors_to_import->find(s->name()) == sectors_to_import->end())
                continue;

            sectors.push_back(s);
        }

        if (!sectors.empty())
        {
            l->clearSectors();
            for (auto s : sectors)
            {
                //serialize from now on
                s->serializeSector(true);

                l->addSector(s);
            }
            new_layers.push_back(l);
        }
    }

    if (new_layers.empty())
        return false;

    sector_layers_.insert(sector_layers_.begin(), new_layers.begin(), new_layers.end());

    for (auto& sec_lay_it : new_layers)
        for (auto& sec_it : sec_lay_it->sectors())
            sec_it->save();


    updateMaxSectorID();

    emit sectorsChangedSignal();

    return true;
}

/**
 */
bool EvaluationManager::sectorsLoaded() const
{
    return sectors_loaded_;
}

/**
 */
void EvaluationManager::setViewableDataConfig (const nlohmann::json::object_t& data)
{
    viewable_data_cfg_.reset(new ViewableDataConfig(data));

    COMPASS::instance().viewManager().setCurrentViewPoint(viewable_data_cfg_.get());
}

/**
 */
void EvaluationManager::resetViewableDataConfig(bool reset_view_point)
{
    if (reset_view_point)
        COMPASS::instance().viewManager().unsetCurrentViewPoint();

    viewable_data_cfg_.reset();
}

/**
*/
void EvaluationManager::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    // @TODO: react on config changes
}

/**
 */
void EvaluationManager::loadData(const EvaluationCalculator& calculator,
                                 bool blocking)
{
    traced_assert(!raw_data_available_);

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    auto ds_ids = calculator.usedDataSources();

    //load only needed data sources
    ds_man.setLoadDSTypes(true); // load all ds types
    ds_man.setLoadOnlyDataSources(ds_ids); // limit loaded data sources

    //configure filters for load
    configureLoadFilters(calculator);

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();
    dbcontent_man.clearData(); //clear previously loaded data

    //!do not distribute this reload to views!
    COMPASS::instance().viewManager().disableDataDistribution(true);

    //add variables needed by evaluation
    needs_additional_variables_ = true;

    if (blocking)
    {
        dbcontent_man.loadBlocking();
        loadingDone();
    }
    else
    {
        connect(&dbcontent_man, &DBContentManager::loadingDoneSignal, this, &EvaluationManager::loadingDone);
        active_load_connection_ = true;
        dbcontent_man.load();
    }

    needs_additional_variables_ = false;
}

/**
 */
void EvaluationManager::configureLoadFilters(const EvaluationCalculator& calculator)
{
    FilterManager& fil_man = COMPASS::instance().filterManager();

    // set use filters
    fil_man.useFilters(true);
    fil_man.disableAllFilters();

    const auto& roi      = calculator.sectorROI();
    const auto& utns     = calculator.evaluationUTNs();
    
    // position data
    if (roi.has_value())
    {
        traced_assert(fil_man.hasFilter("Position"));
        DBFilter* pos_fil = fil_man.getFilter("Position");

        json filter;

        pos_fil->setActive(true);

        filter["Position"]["Latitude Maximum" ] = to_string(roi->latitude_max );
        filter["Position"]["Latitude Minimum" ] = to_string(roi->latitude_min );
        filter["Position"]["Longitude Maximum"] = to_string(roi->longitude_max);
        filter["Position"]["Longitude Minimum"] = to_string(roi->longitude_min);

        pos_fil->loadViewPointConditions(filter); 
    }

    if (!utns.empty())
    {
        traced_assert(fil_man.hasFilter("UTNs"));
        DBFilter* utn_fil = fil_man.getFilter("UTNs");

        json filter;

        utn_fil->setActive(true);

        std::vector<std::string> utn_strings;
        for (auto utn : utns)
             utn_strings.push_back(std::to_string(utn));

        std::string utns_str = Utils::String::compress(utn_strings, ',');

        filter["UTNs"]["utns" ] = utns_str;

        utn_fil->loadViewPointConditions(filter);
    }

    // timestamp filter

    if (use_timestamp_filter_)
    {
        traced_assert(fil_man.hasFilter("Timestamp"));
        DBFilter* fil = fil_man.getFilter("Timestamp");

        fil->setActive(true);

        json filter;

        filter["Timestamp"]["Timestamp Minimum"] = Time::toString(load_timestamp_begin_);
        filter["Timestamp"]["Timestamp Maximum"] = Time::toString(load_timestamp_end_);

        if (load_filtered_time_windows_.size())
        {
            filter["Excluded Time Windows"]["Windows"] =
                load_filtered_time_windows_.asJSON();
        }

        fil->loadViewPointConditions(filter);
    }

    // load filters

    // if (settings.use_load_filter_)
    // {
    //     if (settings.use_ref_traj_accuracy_filter_)
    //     {
    //         traced_assert(fil_man.hasFilter("RefTraj Accuracy"));
    //         DBFilter* fil = fil_man.getFilter("RefTraj Accuracy");

    //         fil->setActive(true);

    //         json filter;

    //         filter["RefTraj Accuracy"]["Accuracy Minimum"] = to_string(settings.ref_traj_minimum_accuracy_);

    //         fil->loadViewPointConditions(filter);
    //     }

    //     if (settings.use_adsb_filter_)
    //     {
    //         traced_assert(fil_man.hasFilter("ADSB Quality"));
    //         DBFilter* adsb_fil = fil_man.getFilter("ADSB Quality");

    //         adsb_fil->setActive(true);

    //         json filter;

    //         filter["ADSB Quality"]["use_v0"] = settings.use_v0_;
    //         filter["ADSB Quality"]["use_v1"] = settings.use_v1_;
    //         filter["ADSB Quality"]["use_v2"] = settings.use_v2_;

    //         // nucp
    //         filter["ADSB Quality"]["use_min_nucp"] = settings.use_min_nucp_;
    //         filter["ADSB Quality"]["min_nucp"] = settings.min_nucp_;
    //         filter["ADSB Quality"]["use_max_nucp"] = settings.use_max_nucp_;
    //         filter["ADSB Quality"]["max_nucp"] = settings.max_nucp_;

    //         // nic
    //         filter["ADSB Quality"]["use_min_nic"] = settings.use_min_nic_;
    //         filter["ADSB Quality"]["min_nic"] = settings.min_nic_;
    //         filter["ADSB Quality"]["use_max_nic"] = settings.use_max_nic_;
    //         filter["ADSB Quality"]["max_nic"] = settings.max_nic_;

    //         // nacp
    //         filter["ADSB Quality"]["use_min_nacp"] = settings.use_min_nacp_;
    //         filter["ADSB Quality"]["min_nacp"] = settings.min_nacp_;
    //         filter["ADSB Quality"]["use_max_nacp"] = settings.use_max_nacp_;
    //         filter["ADSB Quality"]["max_nacp"] = settings.max_nacp_;

    //         // sil v1
    //         filter["ADSB Quality"]["use_min_sil_v1"] = settings.use_min_sil_v1_;
    //         filter["ADSB Quality"]["min_sil_v1"] = settings.min_sil_v1_;
    //         filter["ADSB Quality"]["use_max_sil_v1"] = settings.use_max_sil_v1_;
    //         filter["ADSB Quality"]["max_sil_v1"] = settings.max_sil_v1_;

    //         // sil v2
    //         filter["ADSB Quality"]["use_min_sil_v2"] = settings.use_min_sil_v2_;
    //         filter["ADSB Quality"]["min_sil_v2"] = settings.min_sil_v2_;
    //         filter["ADSB Quality"]["use_max_sil_v2"] = settings.use_max_sil_v2_;
    //         filter["ADSB Quality"]["max_sil_v2"] = settings.max_sil_v2_;

    //         adsb_fil->loadViewPointConditions(filter);
    //     }
    // }
}

/**
 */
void EvaluationManager::loadingDone()
{
    loginf;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    if (active_load_connection_)
    {
        disconnect(&dbcontent_man, &DBContentManager::loadingDoneSignal, this, &EvaluationManager::loadingDone);
        active_load_connection_ = false;
    }

    //!reenable distribution to views!
    COMPASS::instance().viewManager().disableDataDistribution(false);

    traced_assert(!raw_data_available_);

    //obtain data
    raw_data_ = dbcontent_man.loadedData();
    raw_data_available_ = true;

    //clear local data
    dbcontent_man.clearData();

    //signal new data
    emit hasNewData();
}

/**
*/
EvaluationTargetFilter& EvaluationManager::targetFilter() const
{
    traced_assert(target_filter_);
    return *target_filter_.get();
}

/**
 */
std::map<std::string, std::shared_ptr<Buffer>> EvaluationManager::fetchData()
{
    traced_assert(raw_data_available_);

    auto data_cpy = raw_data_;
    raw_data_ = {};
    raw_data_available_ = false;

    return data_cpy;
}

bool EvaluationManager::useTimestampFilter() const
{
    return use_timestamp_filter_;
}

void EvaluationManager::useTimestampFilter(bool value)
{
    use_timestamp_filter_ = value;

    saveTimeConstraints();
}

std::string EvaluationManager::timestampFilterStr() const
{
    ostringstream ss;

    ss << "Use: " << use_timestamp_filter_ << endl;

    if (use_timestamp_filter_)
    {
        ss << "Begin: " << Time::toString(load_timestamp_begin_) << endl;
        ss << "End: " << Time::toString(load_timestamp_end_) << endl;
        ss << "Excluded: " << load_filtered_time_windows_.asString();
    }

    return ss.str();
}

/**
 */
boost::posix_time::ptime EvaluationManager::loadTimestampBegin() const
{
    return load_timestamp_begin_;
}

/**
 */
void EvaluationManager::loadTimestampBegin(boost::posix_time::ptime value)
{
    loginf << "value " << Time::toString(value);

    load_timestamp_begin_ = value;

    saveTimeConstraints();
}

/**
 */
boost::posix_time::ptime EvaluationManager::loadTimestampEnd() const
{
    return load_timestamp_end_;
}

/**
 */
void EvaluationManager::loadTimestampEnd(boost::posix_time::ptime value)
{
    loginf << "value " << Time::toString(value);

    load_timestamp_end_ = value;

    saveTimeConstraints();
}

/**
 */
Utils::TimeWindowCollection& EvaluationManager::excludedTimeWindows()
{
    return load_filtered_time_windows_;
}

/**
 */
bool EvaluationManager::hasCurrentStandard() const
{
    traced_assert(calculator_);
    return calculator_->hasCurrentStandard();
}

/**
 */
const EvaluationStandard& EvaluationManager::currentStandard() const
{
    traced_assert(calculator_);
    return calculator_->currentStandard();
}
