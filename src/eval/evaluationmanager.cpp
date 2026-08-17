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

#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontentdataengine.h"
#include "dbcontent/loadoperation.h"
#include "dbcontent/variable/variable.h"

#include "compass.h"
#include "dbinterface.h"
#include "db_context_manager.h"
#include "filtermanager.h"
#include "viewmanager.h"
#include "taskmanager.h"

#include "buffer.h"
#include "dbfilter.h"
#include "dbfiltercondition.h"
#include "timestampfilter.h"
#include "utnfilter.h"
#include "filterclause.h"
#include "idbvariableresolver.h"
#include "global.h"
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
EvaluationManager::EvaluationManager(nlohmann::json& config,
                                     COMPASS& compass)
    : Configurable(config, &compass)
    , compass_(compass)
    , dbcontent_man_(compass.dbContentManager())
{
    createSubConfigurables();
    init_evaluation_commands(compass_);
}

/**
 */
EvaluationManager::~EvaluationManager()
{
}

/**
*/
void EvaluationManager::generateSubConfigurable(nlohmann::json& child_json)
{
    const auto& class_name = Configuration::getClassName(child_json);

    if (class_name == "EvaluationTargetFilter")
    {
        traced_assert(!target_filter_);
        target_filter_.reset(new EvaluationTargetFilter(child_json, *this));
    }
    else if (class_name == "EvaluationCalculator")
    {
        traced_assert(!calculator_);

        EvaluationCalculator* calculator = new EvaluationCalculator(child_json, *this, dbcontent_man_, true);
        calculator_.reset(calculator);
    }
    else
    {
        throw std::runtime_error("EvaluationManager: generateSubConfigurable: unknown class_name " + class_name);
    }
}

/**
*/
void EvaluationManager::checkSubConfigurables()
{
    if (!target_filter_)
    {
        generateSubConfigurableFromConfig("EvaluationTargetFilter", "EvaluationTargetFilter0");
        traced_assert(target_filter_);
    }

    if (!calculator_)
    {
        //generate default calculator
        generateSubConfigurableFromConfig("EvaluationCalculator", "EvaluationCalculator0");
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

    registerParameter("remove_disabled_utn_data", &remove_disabled_utn_data_, remove_disabled_utn_data_);

    auto& dbc_manager = dbcontent_man_;

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

    auto& task_manager = compass_.taskManager();

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
Result EvaluationManager::evaluate(bool show_dialog,
                                   const std::string& custom_result_name)
{
    loginf << "show_dialog " << show_dialog;

    traced_assert(initialized_);
    traced_assert(calculator_);

    calculator_->resetCustomReportName();

    //show config dialog?
    std::string report_name;
    if (show_dialog)
    {
        EvaluationDialog dlg(*calculator_, QApplication::activeWindow());

        if (!custom_result_name.empty())
            dlg.setReportName(custom_result_name);

        auto ret = dlg.exec();

        if (ret == QDialog::Rejected)
            return Result::succeeded();

        //obtain suitable report name from dialog
        report_name = dlg.reportName();
    }
    else
    {
        //obtain suitable report name from calculator
        report_name = custom_result_name.empty() ? calculator_->suggestReportName() : custom_result_name;
    }

    //create clone of current calculator
    auto res = calculator_->clone();

    if (!res.ok())
        logerr << "evaluation error: " << res.error();
    traced_assert(res.ok());

    auto calculator_local = res.result();
    traced_assert(calculator_local);

    //we always set a custom report name
    calculator_local->setCustomReportName(report_name);

    //evaluate with updated constraints
    auto eval_res = calculator_local->evaluate();

    if (!eval_res.ok())
    {
        //interaction mode => show error immediately
        if (show_dialog)
            QMessageBox::critical(QApplication::activeWindow(), "Evaluation Failed", QString::fromStdString(eval_res.error()));

        return eval_res;
    }

    traced_assert(calculator_local->evaluated());

    //store calculator to task result
    auto& task_man = compass_.taskManager();

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

    // sectors are now loaded by DBContextManager - forward its signal
    connect(&compass_.dbContextManager(), &context::DBContextManager::sectorsChangedSignal,
            this, &EvaluationManager::sectorsChangedSignal);
    connect(&compass_.dbContextManager(), &context::DBContextManager::sectorsChangedSignal,
            this, &EvaluationManager::sectorsChangedSlot);

    //load sectors before locking any results via this connections
    connect(this, &EvaluationManager::sectorsChangedSignal, this, &EvaluationManager::lockResultsSlot);

    auto& dbinterface = compass_.dbInterface();

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

    DBContentManager& dbcont_man = dbcontent_man_;

    if (dbcont_man.dataEngine().hasMinMaxTimestamp())
    {
        std::pair<boost::posix_time::ptime , boost::posix_time::ptime> minmax_ts =  dbcont_man.dataEngine().minMaxTimestamp();

        if (load_timestamp_begin_.is_not_a_date_time())
            load_timestamp_begin_ = get<0>(minmax_ts);

        if (load_timestamp_end_.is_not_a_date_time())
            load_timestamp_end_ = get<1>(minmax_ts);
    }

    // check if configuration is still valid after db open
    calculator_->checkConfiguration();
}

/**
 */
void EvaluationManager::databaseClosedSlot()
{
    loginf;

    //disconnect result locking before clearing the sectors
    disconnect(this, &EvaluationManager::sectorsChangedSignal, this, &EvaluationManager::lockResultsSlot);

    // disconnect forwarding of DBContextManager signal
    disconnect(&compass_.dbContextManager(), &context::DBContextManager::sectorsChangedSignal,
               this, &EvaluationManager::sectorsChangedSignal);
    disconnect(&compass_.dbContextManager(), &context::DBContextManager::sectorsChangedSignal,
               this, &EvaluationManager::sectorsChangedSlot);

    use_timestamp_filter_ = false;
    load_timestamp_begin_ = {};
    load_timestamp_end_ = {};
    load_filtered_time_windows_.clear();

    traced_assert(calculator_);

    calculator_->reset();

    // sectors are now cleared by DBContextManager
    emit sectorsChangedSignal();
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

void EvaluationManager::sectorsChangedSlot()
{
    loginf;

    if (calculator_)
    {
        calculator_->checkMinHeightFilterValid();
        calculator_->clearData();
    }
}

void EvaluationManager::saveTimeConstraints()
{
    loginf;

    nlohmann::json constraints_json = nlohmann::json::object();

    constraints_json[EVAL_TIME_CONSTRAINTS_USE] = use_timestamp_filter_;

    constraints_json[EVAL_TIME_CONSTRAINTS_BEGIN] = Time::toString(load_timestamp_begin_);
    constraints_json[EVAL_TIME_CONSTRAINTS_END] = Time::toString(load_timestamp_end_);

    constraints_json[EVAL_TIME_CONSTRAINTS_EXCLUDED_WINDOWS] = load_filtered_time_windows_.asJSON();

    compass_.dbInterface().setProperty(EVAL_TIME_CONSTRAINTS_PROPRTY_NAME, constraints_json.dump());
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

    DBContentManager& dbcontent_man = dbcontent_man_;

    if (!dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_))
        return;

    // TODO add required variables from standard requirements

    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_rec_num_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ds_id_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_line_id_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_utn_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_max_stddev_xy_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_max_stddev_xy_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_));

    // flight level
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_g_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_v_));

    //if (settings_.dbcontent_name_ref_ == dbcontent_name && settings_.dbcontent_name_ref_ == "CAT062")

    // flight level trusted
    if (dbcontent_name == "CAT062")
    {
        read_set.add(dbcontent_man.getVariable("CAT062", dbcontent_vars::var_cat062_baro_alt_));
        read_set.add(dbcontent_man.getVariable("CAT062", dbcontent_vars::var_cat062_fl_measured_));
    }

    // m3a
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_g_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_v_));

    // tn
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_num_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_num_));

    // ground bit
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_bit_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_bit_));

    // speed & track angle
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_));
    read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_));

    // accs
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ax_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ax_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ay_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ay_));

    // rocd
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_rocd_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_rocd_));

    // moms
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_long_acc_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_long_acc_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_trans_acc_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_trans_acc_));
    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_vert_rate_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mom_vert_rate_));

    if (dbcontent_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_coasting_))
        read_set.add(dbcontent_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_coasting_));
}

/**
*/
/**
 */
void EvaluationManager::updateSectorLayers()
{
    auto& ctx = compass_.dbContextManager();

    if (ctx.sectorsLoaded())
    {
        for (const auto& layer : ctx.sectorLayers())
            for (const auto& s : layer->sectors())
                s->createFastInsideTest();
    }
}

/**
 */
void EvaluationManager::setViewableDataConfig (const nlohmann::json::object_t& data)
{
    viewable_data_cfg_ = std::make_shared<ViewableDataConfig>(data);

    compass_.viewManager().setCurrentViewPoint(viewable_data_cfg_);
}

/**
 */
void EvaluationManager::resetViewableDataConfig(bool reset_view_point)
{
    if (reset_view_point)
        compass_.viewManager().unsetCurrentViewPoint();

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
void EvaluationManager::loadData(const EvaluationCalculator& calculator)
{
    loginf;

    traced_assert(!raw_data_available_);

    // Modal wait dialog around the blocking load below. The load pumps events (op->wait) and,
    // being an issuer-private batch load, raises no progress dialog of its own - without a
    // modal the user could drive the UI into another evaluation or load meanwhile.
    QMessageBox msg_box(QApplication::activeWindow());
    msg_box.setWindowTitle("Evaluation");
    msg_box.setText("Loading data ...");
    msg_box.setStandardButtons(QMessageBox::NoButton);
    msg_box.setWindowModality(Qt::ApplicationModal);
    // do not steal os focus from other applications when popping up
    msg_box.setAttribute(Qt::WA_ShowWithoutActivating, true);
    msg_box.show();
    QCoreApplication::processEvents(); // paint before the load busies the main thread

    DBContentManager& dbcontent_man = dbcontent_man_;

    // add variables needed by evaluation - getReadSet picks these up while the
    // flag is set, synchronously inside the engine load below
    needs_additional_variables_ = true;

    LoadRequest req;
    req.dbcontents_ = { calculator.dbContentNameRef(),
                        calculator.dbContentNameTst() };

    // only the needed data sources - per-op, so the user's selection stays untouched
    req.datasrc_selection_ = calculator.usedDataSources();

    // build eval's load WHERE from the shared clause toolkit (ROI bbox / UTN set /
    // timestamp bounds) instead of hijacking the global FilterManager, so the user's
    // filters stay untouched. rendered per content up front and captured.
    req.apply_view_filters_ = false;
    req.custom_filter_clause_ = LoadRequest::perContentClause(
        req.dbcontents_,
        [this, &calculator](const std::string& name) { return loadFilterClause(name, calculator); });

    // isolated batch load: filled by the engine, read in loadingDone; the view
    // dataset is never touched
    load_op_ = std::make_shared<LoadOperation>(dbcontent_man, req);

    dbcontent_man.dataEngine().load(load_op_);
    load_op_->wait();
    loadingDone();

    needs_additional_variables_ = false;
}

/**
 * Per-content load WHERE for eval, built from the shared clause toolkit: ROI position
 * bbox (Latitude/Longitude <=/>=), UTN set, and timestamp bounds - AND-combined. Mirrors
 * the old configureLoadFilters values without mutating the global FilterManager. Excluded
 * time windows are applied in-memory during evaluation, not at load (as before).
 */
std::string EvaluationManager::loadFilterClause(const std::string& dbcontent_name,
                                                const EvaluationCalculator& calculator)
{
    IDBVariableResolver& resolver = compass_.filterManager().variableResolver();

    std::vector<FilterClause> parts;

    // sqlFor yields an empty clause for contents lacking the given variable, so no guards needed
    const auto& roi = calculator.sectorROI();
    if (roi.has_value())
    {
        parts.push_back(DBFilterCondition::sqlFor(
            resolver, dbcontent_name, dbcontent_vars::meta_var_latitude_.name(),
            META_OBJECT_NAME, filter_op::less_equal, std::to_string(roi->latitude_max)));
        parts.push_back(DBFilterCondition::sqlFor(
            resolver, dbcontent_name, dbcontent_vars::meta_var_latitude_.name(),
            META_OBJECT_NAME, filter_op::greater_equal, std::to_string(roi->latitude_min)));

        parts.push_back(DBFilterCondition::sqlFor(
            resolver, dbcontent_name, dbcontent_vars::meta_var_longitude_.name(),
            META_OBJECT_NAME, filter_op::less_equal, std::to_string(roi->longitude_max)));
        parts.push_back(DBFilterCondition::sqlFor(
            resolver, dbcontent_name, dbcontent_vars::meta_var_longitude_.name(),
            META_OBJECT_NAME, filter_op::greater_equal, std::to_string(roi->longitude_min)));
    }

    const auto& utns = calculator.evaluationUTNs();
    if (!utns.empty())
        parts.push_back(UTNFilter::sqlFor(resolver, utns, false, dbcontent_name));

    if (use_timestamp_filter_)
        parts.push_back(TimestampFilter::sqlFor(
            resolver, load_timestamp_begin_, load_timestamp_end_, dbcontent_name));

    return combineAnd(parts).sql;
}

/**
 */
void EvaluationManager::loadingDone()
{
    loginf;

    traced_assert(!raw_data_available_);

    // obtain data from the isolated operation
    // @TODO: op state unchecked - a Failed/Cancelled load evaluates on empty/partial data
    raw_data_ = load_op_->buffers();
    raw_data_available_ = true;

    load_op_ = nullptr; // release the operation

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

