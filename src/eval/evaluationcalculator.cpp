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

#include "evaluationcalculator.h"
#include "evaluationmanager.h"
#include "evaluationtarget.h"

#include "eval/results/report/pdfgeneratordialog.h"
#include "evaluationstandard.h"
#include "evaluationdialog.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"
#include "eval/requirement/base/baseconfig.h"
#include "eval/results/base/base.h"
#include "eval/results/base/single.h"
#include "eval/results/base/joined.h"

#include "compass.h"
#include "dbinterface.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/dbcontentmanager.h"
#include "datasourcemanager.h"

#include "sectorlayer.h"
//#include "sector.h"
#include "airspace.h"
//#include "dbcontent/variable/metavariable.h"
//#include "dbcontent/variable/variable.h"
//#include "buffer.h"
#include "filtermanager.h"
//#include "dbfilter.h"
//#include "viewabledataconfig.h"
#include "viewmanager.h"
#include "stringconv.h"
#include "util/timeconv.h"
#include "global.h"
#include "viewpoint.h"
#include "projectionmanager.h"
#include "projection.h"
//#include "files.h"

#include "json.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QThread>
#include <QMessageBox>

#include <memory>
//#include <fstream>
#include <cstdlib>
#include <system.h>

using namespace Utils;
using namespace std;
using namespace nlohmann;
using namespace boost::posix_time;

/**
 */
EvaluationCalculator::EvaluationCalculator(const std::string& class_id, 
                                           const std::string& instance_id,
                                           EvaluationManager& eval_man, DBContentManager& dbcontent_man)
:   Configurable      (class_id, instance_id, &eval_man)
,   eval_man_         (eval_man)
,   data_             (new EvaluationData(*this, eval_man_, dbcontent_man))
,   results_gen_      (new EvaluationResultsGenerator(*this))
,   tst_srcs_coverage_(new dbContent::DataSourceCompoundCoverage)
{
    readSettings();
    createSubConfigurables();
}

/**
 */
EvaluationCalculator::EvaluationCalculator(EvaluationManager& eval_man, 
                                           DBContentManager& dbcontent_man,
                                           const nlohmann::json& config)
:   Configurable      ("EvaluationManager", "EvaluationManager0", nullptr, "", &config)
,   eval_man_         (eval_man)
,   data_             (new EvaluationData(*this, eval_man_, dbcontent_man))
,   results_gen_      (new EvaluationResultsGenerator(*this))
,   tst_srcs_coverage_(new dbContent::DataSourceCompoundCoverage)
{
    readSettings();
    createSubConfigurables();
}

/**
 */
EvaluationCalculator::~EvaluationCalculator()
{
}

/**
 */
ResultT<EvaluationCalculator*> EvaluationCalculator::clone() const
{
    //obtain current json config
    nlohmann::json config;
    generateJSON(config, Configurable::JSONExportType::General);

    //clone from config
    return EvaluationCalculator::clone(config);
}

/**
 */
ResultT<EvaluationCalculator*> EvaluationCalculator::clone(const nlohmann::json& config)
{
    if (!config.is_object())
        return Result::failed("Config not available");

    auto& eval_man = COMPASS::instance().evaluationManager();
    auto& dbc_man  = COMPASS::instance().dbContentManager();

    EvaluationCalculator* c = nullptr;

    try
    {
        //create calculator based on given config
        c = new EvaluationCalculator(eval_man, dbc_man, config);
    }
    catch(const std::exception& ex)
    {
        return ResultT<EvaluationCalculator*>::failed("Could not create calculator from stored config: " + std::string(ex.what()));
    }
    catch(...)
    {
        return ResultT<EvaluationCalculator*>::failed("Could not create calculator from stored config: Unknown error");
    }

    traced_assert(c);

    if (c->canEvaluate().ok())
    {
        //update some stuff in case the cloned calculator can evaluate
        c->updateSectorROI();
    }

    auto res = ResultT<EvaluationCalculator*>::succeeded(c);
    traced_assert(res.hasResult() && res.result());

    return res;
}

/**
 */
void EvaluationCalculator::readSettings()
{
    typedef EvaluationSettings Settings;

    registerParameter("dbcontent_name_ref", &settings_.dbcontent_name_ref_, Settings().dbcontent_name_ref_);
    registerParameter("line_id_ref", &settings_.line_id_ref_, Settings().line_id_ref_);
    registerParameter("active_sources_ref", &settings_.active_sources_ref_, Settings().active_sources_ref_);

    registerParameter("dbcontent_name_tst", &settings_.dbcontent_name_tst_, Settings().dbcontent_name_tst_);
    registerParameter("line_id_tst", &settings_.line_id_tst_, Settings().line_id_tst_);
    registerParameter("active_sources_tst", &settings_.active_sources_tst_, Settings().active_sources_tst_);

    registerParameter("current_standard", &settings_.current_standard_, Settings().current_standard_);

    registerParameter("use_grp_in_sector", &settings_.use_grp_in_sector_, Settings().use_grp_in_sector_);
    registerParameter("use_requirement", &settings_.use_requirement_, Settings().use_requirement_);

    // registerParameter("max_ref_time_diff", &settings_.max_ref_time_diff_, Settings().max_ref_time_diff_);

    // // load filter
    // registerParameter("use_load_filter", &settings_.use_load_filter_, Settings().use_load_filter_);

    // registerParameter("use_ref_traj_accuracy_filter_", &settings_.use_ref_traj_accuracy_filter_, Settings().use_ref_traj_accuracy_filter_);
    // registerParameter("ref_traj_minimum_accuracy", &settings_.ref_traj_minimum_accuracy_, Settings().ref_traj_minimum_accuracy_);

    // registerParameter("use_adsb_filter", &settings_.use_adsb_filter_, Settings().use_adsb_filter_);
    // registerParameter("use_v0", &settings_.use_v0_, Settings().use_v0_);
    // registerParameter("use_v1", &settings_.use_v1_, Settings().use_v1_);
    // registerParameter("use_v2", &settings_.use_v2_, Settings().use_v2_);

    // // nucp
    // registerParameter("use_min_nucp", &settings_.use_min_nucp_, Settings().use_min_nucp_);
    // registerParameter("min_nucp", &settings_.min_nucp_, Settings().min_nucp_);

    // registerParameter("use_max_nucp", &settings_.use_max_nucp_, Settings().use_max_nucp_);
    // registerParameter("max_nucp", &settings_.max_nucp_, Settings().max_nucp_);

    // // nic
    // registerParameter("use_min_nic", &settings_.use_min_nic_, Settings().use_min_nic_);
    // registerParameter("min_nic", &settings_.min_nic_, Settings().min_nic_);

    // registerParameter("use_max_nic", &settings_.use_max_nic_, Settings().use_max_nic_);
    // registerParameter("max_nic", &settings_.max_nic_, Settings().max_nic_);

    // // nacp
    // registerParameter("use_min_nacp", &settings_.use_min_nacp_, Settings().use_min_nacp_);
    // registerParameter("min_nacp", &settings_.min_nacp_, Settings().min_nacp_);

    // registerParameter("use_max_nacp", &settings_.use_max_nacp_, Settings().use_max_nacp_);
    // registerParameter("max_nacp", &settings_.max_nacp_, Settings().max_nacp_);

    // // sil v1
    // registerParameter("use_min_sil_v1", &settings_.use_min_sil_v1_, Settings().use_min_sil_v1_);
    // registerParameter("min_sil_v1", &settings_.min_sil_v1_, Settings().min_sil_v1_);

    // registerParameter("use_max_sil_v1", &settings_.use_max_sil_v1_, Settings().use_max_sil_v1_);
    // registerParameter("max_sil_v1", &settings_.max_sil_v1_, Settings().max_sil_v1_);

    // // sil v2
    // registerParameter("use_min_sil_v2", &settings_.use_min_sil_v2_, Settings().use_min_sil_v2_);
    // registerParameter("min_sil_v2", &settings_.min_sil_v2_, Settings().min_sil_v2_);

    // registerParameter("use_max_sil_v2", &settings_.use_max_sil_v2_, Settings().use_max_sil_v2_);
    // registerParameter("max_sil_v2", &settings_.max_sil_v2_, Settings().max_sil_v2_);

    registerParameter("result_detail_zoom", &settings_.result_detail_zoom_, Settings().result_detail_zoom_);

    // min height filter
    registerParameter("min_height_filter_layer", &settings_.min_height_filter_layer_, Settings().min_height_filter_layer_);

    // report stuff
    registerParameter("report_skip_no_data_details", &settings_.report_skip_no_data_details_, Settings().report_skip_no_data_details_);
    registerParameter("report_split_results_by_mops", &settings_.report_split_results_by_mops_, Settings().report_split_results_by_mops_);
    registerParameter("report_split_results_by_aconly_ms", &settings_.report_split_results_by_aconly_ms_, Settings().report_split_results_by_aconly_ms_);

    //grid generation
    registerParameter("grid_num_cells_x", &settings_.grid_num_cells_x, Settings().grid_num_cells_x);
    registerParameter("grid_num_cells_y", &settings_.grid_num_cells_y, Settings().grid_num_cells_y);

    //histogram generation
    registerParameter("histogram_num_bins", &settings_.histogram_num_bins, Settings().histogram_num_bins);
    
    updateDerivedParameters();
}

/**
 */
void EvaluationCalculator::generateSubConfigurable(const std::string& class_id,
                                                   const std::string& instance_id)
{
    if (class_id == "EvaluationStandard")
    {
        EvaluationStandard* standard = new EvaluationStandard(class_id, instance_id, *this);
        logdbg << "adding standard " << standard->name();

        traced_assert(!hasStandard(standard->name()));

        standards_.push_back(std::unique_ptr<EvaluationStandard>(standard));

        // resort by name
        sort(standards_.begin(), standards_.end(),
             [](const std::unique_ptr<EvaluationStandard>& a, const std::unique_ptr<EvaluationStandard>& b) -> bool
        {
            return a->name() > b->name();
        });
    }
    else
        throw std::runtime_error("EvaluationCalculator: generateSubConfigurable: unknown class_id " + class_id);
}

/**
 */
void EvaluationCalculator::checkSubConfigurables()
{
}

/**
 */
void EvaluationCalculator::updateDerivedParameters()
{
    data_sources_ref_ = settings_.active_sources_ref_.get<std::map<std::string, std::map<std::string, bool>>>();
    data_sources_tst_ = settings_.active_sources_tst_.get<std::map<std::string, std::map<std::string, bool>>>();
}

/**
 */
Result EvaluationCalculator::canEvaluate() const
{
    //needs associations
    if (!COMPASS::instance().dbContentManager().hasAssociations())
        return Result::failed("Please run target report association");

    //needs a set standard
    if (!hasCurrentStandard())
        return Result::failed("Please select a standard");

    //needs selected ref data sources
    if (!hasSelectedReferenceDataSources())
        return Result::failed("Please select reference data sources");

    //needs selected test data sources
    if (!hasSelectedTestDataSources())
        return Result::failed("Please select test data sources");

    //needs loaded sectors
    if (!sectorsLoaded())
        return Result::failed("No Database loaded");

    if (sectorLayers().empty())
        return Result::failed("Please add at least one sector");

    if (!anySectorsWithReq())
        return Result::failed("Please set requirements for at least one sector");

    //@TODO
    //return "Please activate at least one requirement group";

    return Result::succeeded();
}

/**
 */
void EvaluationCalculator::reset()
{
    clearData();

    //clear data sources
    data_sources_ref_.clear();
    data_sources_tst_.clear();
}

/**
 */
void EvaluationCalculator::clearData()
{
    loginf;

    data_->clear();
    results_gen_->clear();

    sector_roi_.reset();

    reference_data_loaded_  = false;
    test_data_loaded_       = false;
    data_loaded_            = false;
    evaluated_              = false; 
    active_load_connection_ = false;
}

/**
 */
Result EvaluationCalculator::evaluate(bool update_report,
                                      const std::vector<unsigned int>& utns,
                                      const std::vector<Evaluation::RequirementResultID>& requirements)
{
    loginf;

    traced_assert(canEvaluate().ok());

    //always load blocking for now
    const bool Blocking = true;

    try
    {
        eval_utns_         = utns;
        eval_requirements_ = requirements;
        update_report_     = update_report;

        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

        // remove previous stuff
        eval_man_.resetViewableDataConfig(true);

        clearData();

        emit resultsChanged();

        // actually load
        QApplication::restoreOverrideCursor();

        // clear data
        data_->clear();

        // update stuff before load
        updateCompoundCoverage(activeDataSourcesTst());
        updateSectorROI();

        if (Blocking)
        {
            eval_man_.loadData(*this, true);
            auto res = loadingDone();
            if (!res.ok())
                return res;
        }
        else
        {
            traced_assert(!active_load_connection_);

            QObject::connect(&eval_man_, &EvaluationManager::hasNewData, this, &EvaluationCalculator::loadingDone);
            active_load_connection_ = true;
            eval_man_.loadData(*this, false);
        }
    }
    catch (...)
    {
        return Result::failed("Evaluation failed due to errors");
    }

    return Result::succeeded();
}

/**
 */
Result EvaluationCalculator::loadingDone()
{
    loginf;

    if (active_load_connection_)
    {
        QObject::disconnect(&eval_man_, &EvaluationManager::hasNewData, this, &EvaluationCalculator::loadingDone);
        active_load_connection_ = false;
    }

    auto data = eval_man_.fetchData();
 
    data_->setBuffers(data);

    bool has_ref_data = data.count(settings_.dbcontent_name_ref_);
    bool has_tst_data = data.count(settings_.dbcontent_name_tst_);

    if (eval_utns_.empty() && !has_ref_data)
        return Result::failed("Loading data failed, no reference data was loaded");

    data_->addReferenceData(settings_.dbcontent_name_ref_, settings_.line_id_ref_);
    reference_data_loaded_ = has_ref_data;

    if (eval_utns_.empty() && !has_tst_data)
        return Result::failed("Loading data failed, no test data was loaded");

    data_->addTestData(settings_.dbcontent_name_tst_, settings_.line_id_tst_);
    test_data_loaded_ = has_tst_data;

    data_loaded_ = reference_data_loaded_ || test_data_loaded_;

    //ready to evaluate?
    if (data_loaded_)
    {
        loginf << "finalizing: has ref data " << has_ref_data << " has tst data " << has_tst_data;

        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

        //finalize loaded data
        data_->finalize();

        boost::posix_time::time_duration time_diff =  boost::posix_time::microsec_clock::local_time() - start_time;

        loginf << "finalize done "
                << String::timeStringFromDouble(time_diff.total_milliseconds() / 1000.0, true);

        loginf << "starting to evaluate";

        return evaluateData();
    }

    //no data loaded => no eval needed
    return Result::succeeded();
}

/**
 */
Result EvaluationCalculator::evaluateData()
{
    loginf;

    traced_assert(data_loaded_);
    traced_assert(canEvaluate().ok());

    Projection& projection = ProjectionManager::instance().currentProjection();
    projection.clearCoordinateSystems();
    projection.addAllCoordinateSystems();

    // clean previous
    results_gen_->clear();

    evaluated_ = false;

    emit resultsChanged();
    
    // eval
    results_gen_->evaluate(currentStandard(), eval_utns_, eval_requirements_, update_report_);

    evaluated_ = true;

    emit resultsChanged();
    emit evaluationDone();

    return Result::succeeded();
}

/**
 */
std::map<unsigned int, std::set<unsigned int>> EvaluationCalculator::usedDataSources() const
{
    std::map<unsigned int, std::set<unsigned int>> data_sources;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    std::set<unsigned int> line_ref_set = {settings_.line_id_ref_};

    unsigned int ds_id;

    for (auto& ds_it : data_sources_ref_.at(settings_.dbcontent_name_ref_))
    {
        ds_id = stoul(ds_it.first);

        loginf << "ref ds_id '" << ds_it.first << "' uint " << ds_id;

        for (auto& line_it : line_ref_set)
            loginf << "ref line " << line_it;

        traced_assert(ds_man.hasDBDataSource(ds_id));

        if (ds_it.second)
            data_sources.insert(make_pair(ds_id, line_ref_set));
    }

    std::set<unsigned int> line_tst_set = {settings_.line_id_tst_};

    for (auto& ds_it : data_sources_tst_.at(settings_.dbcontent_name_tst_))
    {
        ds_id = stoul(ds_it.first);

        loginf << "tst ds_id '" << ds_it.first << "' uint " << ds_id;

        for (auto& line_it : line_tst_set)
            loginf << "tst line " << line_it;

        traced_assert(ds_man.hasDBDataSource(ds_id));

        if (ds_it.second)
        {
            if (data_sources.count(ds_id)) // same ds id
                data_sources.at(ds_id).insert(line_tst_set.begin(), line_tst_set.end());
            else
                data_sources.insert(make_pair(ds_id, line_tst_set));
        }
    }

    return data_sources;
}

/**
 */
bool EvaluationCalculator::filterMinimumHeight() const
{
    return !settings_.min_height_filter_layer_.empty();
}   

/**
 */
const std::string& EvaluationCalculator::minHeightFilterLayerName() const
{
    return settings_.min_height_filter_layer_;
}

/**
 */
void EvaluationCalculator::minHeightFilterLayerName(const std::string& layer_name)
{
    traced_assert(layer_name.empty() || eval_man_.hasSectorLayer(layer_name));

    loginf << "layer changed to "
           << (layer_name.empty() ? "null" : "'" + layer_name + "'");

    settings_.min_height_filter_layer_ = layer_name;
}

/**
 */
std::shared_ptr<SectorLayer> EvaluationCalculator::minHeightFilterLayer() const
{
    if (!filterMinimumHeight())
        return {};

    //!will assert on non-existing layer name!
    return eval_man_.sectorLayer(settings_.min_height_filter_layer_);
}

/**
 * Called every time a layer is removed from the eval manager.
 * Checks if the selected min height filter layer is still present and resets it needed.
 */
void EvaluationCalculator::checkMinHeightFilterValid()
{
    if (!settings_.min_height_filter_layer_.empty() && !eval_man_.hasSectorLayer(settings_.min_height_filter_layer_))
    {
        logerr << "Layer '" << settings_.min_height_filter_layer_ << "'"
               << " not present, resetting min height filter";
        
        settings_.min_height_filter_layer_ = "";
    }
}

/**
 */
std::string EvaluationCalculator::dbContentNameRef() const
{
    return settings_.dbcontent_name_ref_;
}

/**
 */
void EvaluationCalculator::dbContentNameRef(const std::string& name)
{
    loginf << "name " << name;

    settings_.dbcontent_name_ref_ = name;

    checkReferenceDataSources();
}

/**
 */
bool EvaluationCalculator::hasValidReferenceDBContent () const
{
    if (!settings_.dbcontent_name_ref_.size())
        return false;

    return COMPASS::instance().dbContentManager().existsDBContent(settings_.dbcontent_name_ref_);
}

/**
 */
const std::map<std::string, bool>& EvaluationCalculator::dataSourcesRef() const 
{ 
    return data_sources_ref_.at(settings_.dbcontent_name_ref_);
}

/**
 */
std::map<std::string, bool>& EvaluationCalculator::dataSourcesRef() 
{ 
    return data_sources_ref_.at(settings_.dbcontent_name_ref_);
}

/**
 */
set<unsigned int> EvaluationCalculator::activeDataSourcesRef()
{
    set<unsigned int> srcs;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    for (auto& ds_it : data_sources_ref_[settings_.dbcontent_name_ref_])
    {
        unsigned int ds_id = stoul(ds_it.first);
        traced_assert(ds_man.hasDBDataSource(ds_id));

        if (ds_it.second)
            srcs.insert(ds_id);
    }

    return srcs;
}

/**
 */
EvaluationCalculator::EvaluationDSInfo EvaluationCalculator::activeDataSourceInfoRef() const
{
    EvaluationDSInfo ds_info;
    ds_info.dbcontent = settings_.dbcontent_name_ref_;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    for (auto& ds_it : data_sources_ref_.at(settings_.dbcontent_name_ref_))
    {
        if (!ds_it.second)
            continue;

        unsigned int ds_id = stoul(ds_it.first);
        traced_assert(ds_man.hasDBDataSource(ds_id));

        const auto& name = ds_man.dbDataSource(ds_id).name();

        ds_info.data_sources.push_back({ name, ds_id });
    }

    return ds_info;
}

/**
 */
void EvaluationCalculator::selectDataSourceRef(const std::string& name, 
                                               bool select, 
                                               bool update_settings)
{
    dataSourcesRef().at(name) = select;

    if (update_settings)
        settings_.active_sources_ref_ = data_sources_ref_;
}

/**
 */
std::string EvaluationCalculator::dbContentNameTst() const
{
    return settings_.dbcontent_name_tst_;
}

/**
 */
void EvaluationCalculator::dbContentNameTst(const std::string& name)
{
    loginf << "name " << name;

    settings_.dbcontent_name_tst_ = name;

    checkTestDataSources();
}

/**
 */
bool EvaluationCalculator::hasValidTestDBContent () const
{
    if (!settings_.dbcontent_name_tst_.size())
        return false;

    return COMPASS::instance().dbContentManager().existsDBContent(settings_.dbcontent_name_tst_);
}

/**
 */
const std::map<std::string, bool>& EvaluationCalculator::dataSourcesTst() const 
{ 
    return data_sources_tst_.at(settings_.dbcontent_name_tst_);
}
/**
 */
std::map<std::string, bool>& EvaluationCalculator::dataSourcesTst() 
{ 
    return data_sources_tst_.at(settings_.dbcontent_name_tst_);
}

/**
 */
set<unsigned int> EvaluationCalculator::activeDataSourcesTst()
{
    set<unsigned int> srcs;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    for (auto& ds_it : data_sources_tst_[settings_.dbcontent_name_tst_])
    {
        unsigned int ds_id = stoul(ds_it.first);
        traced_assert(ds_man.hasDBDataSource(ds_id));

        if (ds_it.second)
            srcs.insert(ds_id);
    }

    return srcs;
}

/**
 */
EvaluationCalculator::EvaluationDSInfo EvaluationCalculator::activeDataSourceInfoTst() const
{
    EvaluationDSInfo ds_info;
    ds_info.dbcontent = settings_.dbcontent_name_tst_;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    for (auto& ds_it : data_sources_tst_.at(settings_.dbcontent_name_tst_))
    {
        if (!ds_it.second)
            continue;

        unsigned int ds_id = stoul(ds_it.first);
        traced_assert(ds_man.hasDBDataSource(ds_id));

        const auto& name = ds_man.dbDataSource(ds_id).name();

        ds_info.data_sources.push_back({ name, ds_id });
    }

    return ds_info;
}

/**
 */
void EvaluationCalculator::selectDataSourceTst(const std::string& name, 
                                               bool select, 
                                               bool update_settings)
{
    dataSourcesTst().at(name) = select;

    if (update_settings)
        settings_.active_sources_tst_ = data_sources_tst_;
}

/**
 */
bool EvaluationCalculator::hasConstraints() const
{
    return !eval_utns_.empty() || !eval_requirements_.empty();
}

/**
 */
bool EvaluationCalculator::dataLoaded() const
{
    return data_loaded_;
}

/**
 */
bool EvaluationCalculator::evaluated() const
{
    return evaluated_;
}

/**
 */
bool EvaluationCalculator::hasCurrentStandard() const
{
    return settings_.current_standard_.size() && hasStandard(settings_.current_standard_);
}

/**
 */
std::string EvaluationCalculator::currentStandardName() const
{
    return settings_.current_standard_;
}

/**
 */
void EvaluationCalculator::currentStandardName(const std::string& current_standard)
{
    settings_.current_standard_ = current_standard;

    if (settings_.current_standard_.size())
        traced_assert(hasStandard(settings_.current_standard_));

    emit currentStandardChanged();
}

/**
 */
void EvaluationCalculator::renameCurrentStandard (const std::string& new_name)
{
    loginf << "new name '" << new_name << "'";

    traced_assert(hasCurrentStandard());
    traced_assert(!hasStandard(new_name));

    currentStandard().name(new_name);
    settings_.current_standard_ = new_name;

    emit standardsChanged();
    emit currentStandardChanged();
}

/**
 */
void EvaluationCalculator::copyCurrentStandard (const std::string& new_name)
{
    loginf << "new name '" << new_name << "'";

    traced_assert(hasCurrentStandard());
    traced_assert(!hasStandard(new_name));

    nlohmann::json data;
    data["parameters"]["name"] = new_name;

    Configurable::generateSubConfigurableFromJSON(currentStandard(), data, "EvaluationStandard");

    settings_.current_standard_ = new_name;

    emit standardsChanged();
    emit currentStandardChanged();
}

/**
 */
EvaluationStandard& EvaluationCalculator::currentStandard()
{
    traced_assert(hasCurrentStandard());

    string name = settings_.current_standard_;

    auto iter = std::find_if(standards_.begin(), standards_.end(),
                             [&name](const unique_ptr<EvaluationStandard>& x) { return x->name() == name;});

    traced_assert(iter != standards_.end());

    return *iter->get();
}

/**
 */
const EvaluationStandard& EvaluationCalculator::currentStandard() const
{
    traced_assert(hasCurrentStandard());

    string name = settings_.current_standard_;

    auto iter = std::find_if(standards_.begin(), standards_.end(),
                             [&name](const unique_ptr<EvaluationStandard>& x) { return x->name() == name;});

    traced_assert(iter != standards_.end());

    return *iter->get();
}

/**
 */
bool EvaluationCalculator::hasStandard(const std::string& name) const
{
    auto iter = std::find_if(standards_.begin(), standards_.end(),
                             [&name](const unique_ptr<EvaluationStandard>& x) { return x->name() == name;});

    return iter != standards_.end();
}

/**
 */
void EvaluationCalculator::addStandard(const std::string& name)
{
    loginf << "name " << name;

    traced_assert(!hasStandard(name));

    std::string instance = "EvaluationStandard" + name + "0";

    auto config = Configuration::create("EvaluationStandard", instance);
    config->addParameter<std::string>("name", name);

    generateSubConfigurableFromConfig(std::move(config));

    emit standardsChanged();

    currentStandardName(name);
}

/**
 */
void EvaluationCalculator::deleteCurrentStandard()
{
    loginf << "name " << settings_.current_standard_;

    traced_assert(hasCurrentStandard());

    string name = settings_.current_standard_;

    auto iter = std::find_if(standards_.begin(), standards_.end(),
                             [&name](const unique_ptr<EvaluationStandard>& x) { return x->name() == name;});

    traced_assert(iter != standards_.end());

    standards_.erase(iter);

    emit standardsChanged();

    currentStandardName("");
}

/**
 */
std::vector<std::string> EvaluationCalculator::currentRequirementNames() const
{
    std::vector<std::string> names;

    if (hasCurrentStandard())
    {
        for (auto& req_grp_it : currentStandard())
        {
            for (auto& req_it : *req_grp_it)
            {
                if (find(names.begin(), names.end(), req_it->name()) == names.end())
                    names.push_back(req_it->name());
            }
        }
    }

    return names;
}

/**
 */
bool EvaluationCalculator::sectorsLoaded() const
{
    return eval_man_.sectorsLoaded();
}

/**
 */
bool EvaluationCalculator::anySectorsWithReq() const
{
    if (!sectorsLoaded())
        return false;

    bool any = false;

    if (hasCurrentStandard())
    {
        const EvaluationStandard& standard = currentStandard();

        const std::vector<std::shared_ptr<SectorLayer>>& sector_layers = sectorLayers();
        for (const auto& sec_it : sector_layers)
        {
            const string& sector_layer_name = sec_it->name();

            for (auto& req_group_it : standard)
            {
                const string& requirement_group_name = req_group_it->name();

                if (useGroupInSectorLayer(sector_layer_name, requirement_group_name))
                {
                    any = true;
                    break;
                }
            }
        }
    }

    return any;
}

/**
 */
std::vector<std::shared_ptr<SectorLayer>>& EvaluationCalculator::sectorLayers()
{
    return eval_man_.sectorsLayers();
}

/**
 */
const std::vector<std::shared_ptr<SectorLayer>>& EvaluationCalculator::sectorLayers() const
{
    return eval_man_.sectorsLayers();
}

/**
 */
void EvaluationCalculator::updateSectorLayers()
{
    if (use_fast_sector_inside_check_)
        eval_man_.updateSectorLayers();
}

/**
 */
void EvaluationCalculator::checkReferenceDataSources(bool update_settings)
{
    loginf;

    if (!hasValidReferenceDBContent())
        return;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    // clear out old ds_ids
    auto ds_copy = data_sources_ref_[settings_.dbcontent_name_ref_];

    unsigned int ds_id;
    for (auto& ds_it : ds_copy)
    {
        ds_id = stoul(ds_it.first);

        if (!ds_man.hasDBDataSource(ds_id))
            data_sources_ref_[settings_.dbcontent_name_ref_].erase(ds_it.first);
    }

    // init non-existing ones with false
    if (ds_man.hasDataSourcesOfDBContent(settings_.dbcontent_name_ref_))
    {
        for (auto& ds_it : ds_man.dbDataSources())
        {
            if (!ds_it->hasNumInserted(settings_.dbcontent_name_ref_))
                continue;

            string ds_id_str = to_string(ds_it->id());

            if (!data_sources_ref_[settings_.dbcontent_name_ref_].count(ds_id_str))
                data_sources_ref_[settings_.dbcontent_name_ref_][ds_id_str] = false; // init with default false
        }
    }

    if (update_settings)
        settings_.active_sources_ref_ = data_sources_ref_;
}

/**
 */
void EvaluationCalculator::checkTestDataSources(bool update_settings)
{
    loginf;

    if (!hasValidTestDBContent())
        return;

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    // clear out old ds_ids
    auto ds_copy = data_sources_tst_[settings_.dbcontent_name_tst_];

    unsigned int ds_id;
    for (auto& ds_it : ds_copy)
    {
        ds_id = stoul(ds_it.first);

        if (!ds_man.hasDBDataSource(ds_id))
            data_sources_tst_[settings_.dbcontent_name_tst_].erase(ds_it.first);
    }

    // init non-existing ones with false
    if (ds_man.hasDataSourcesOfDBContent(settings_.dbcontent_name_tst_))
    {
        for (auto& ds_it : ds_man.dbDataSources())
        {
            if (!ds_it->hasNumInserted(settings_.dbcontent_name_tst_))
                continue;

            string ds_id_str = to_string(ds_it->id());

            if (!data_sources_tst_[settings_.dbcontent_name_tst_].count(ds_id_str))
                data_sources_tst_[settings_.dbcontent_name_tst_][ds_id_str] = false; // init with default false
        }
    }

    if (update_settings)
        settings_.active_sources_tst_ = data_sources_tst_;
}

/**
 */
bool EvaluationCalculator::hasSelectedReferenceDataSources() const
{
    if (!hasValidReferenceDBContent())
        return false;

    for (auto& ds_it : data_sources_ref_.at(settings_.dbcontent_name_ref_))
        if (ds_it.second)
            return true;

    return false;
}

/**
 */
bool EvaluationCalculator::hasSelectedTestDataSources() const
{
    if (!hasValidTestDBContent())
        return false;

    for (auto& ds_it : data_sources_tst_.at(settings_.dbcontent_name_tst_))
        if (ds_it.second)
            return true;

    return false;
}

/**
 */
EvaluationCalculator::ResultIterator EvaluationCalculator::begin()
{
    return results_gen_->begin();
}

/**
 */
EvaluationCalculator::ResultIterator EvaluationCalculator::end()
{
    return results_gen_->end();
}

/**
 */
bool EvaluationCalculator::hasResults() const
{
    return results_gen_->results().size();
}

/**
 */
const EvaluationCalculator::ResultMap& EvaluationCalculator::results() const
{
    return results_gen_->results(); 
}

const std::string& EvaluationCalculator::resultName() const
{
    return results_gen_->resultName(); 
}

/**
 */
EvaluationRequirementResult::Single* EvaluationCalculator::singleResult(const Evaluation::RequirementResultID& id,
                                                                        unsigned int utn) const
{
    for (auto& group_results : results())
    {
        for (auto& result : group_results.second)
        {
            if (!result.second->isSingle())
                continue;

            auto single_ptr = dynamic_cast<EvaluationRequirementResult::Single*>(result.second.get());
            traced_assert(single_ptr);

            if (!single_ptr->isResult(id) || single_ptr->utn() != utn)
                continue;

            return single_ptr;
        }
    }

    return nullptr;
}

/**
 */
EvaluationRequirementResult::Joined* EvaluationCalculator::joinedResult(const Evaluation::RequirementResultID& id) const
{
    for (auto& group_results : results())
    {
        for (auto& result : group_results.second)
        {
            if (!result.second->isJoined())
                continue;

            auto joined_ptr = dynamic_cast<EvaluationRequirementResult::Joined*>(result.second.get());
            traced_assert(joined_ptr);

            if (!joined_ptr->isResult(id))
                continue;

            return joined_ptr;
        }
    }

    return nullptr;
}

/**
 */
void EvaluationCalculator::updateResultsToChanges ()
{
    if (evaluated_)
    {
        results_gen_->updateToChanges();
    }
}

/**
 */
nlohmann::json::object_t EvaluationCalculator::getBaseViewableDataConfig () const
{
    nlohmann::json data;

    // set data sources

    std::map<unsigned int, std::set<unsigned int>> data_sources;

    for (auto& src_it : data_sources_ref_.at(settings_.dbcontent_name_ref_))
        if (src_it.second)
            data_sources[stoul(src_it.first)].insert(settings_.line_id_ref_);

    for (auto& src_it : data_sources_tst_.at(settings_.dbcontent_name_tst_))
        if (src_it.second)
            data_sources[stoul(src_it.first)].insert(settings_.line_id_tst_);

    data["data_sources"] = data_sources;

    if (settings_.load_only_sector_data_ && sector_roi_.has_value())
    {
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Maximum"] = to_string(sector_roi_->latitude_max);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Latitude Minimum"] = to_string(sector_roi_->latitude_min);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Maximum"] = to_string(sector_roi_->longitude_max);
        data[ViewPoint::VP_FILTERS_KEY]["Position"]["Longitude Minimum"] = to_string(sector_roi_->longitude_min);
    }

    // if (settings_.use_ref_traj_accuracy_filter_)
    // {
    //     data[ViewPoint::VP_FILTERS_KEY]["RefTraj Accuracy"]["Accuracy Minimum"] =
    //         to_string(settings_.ref_traj_minimum_accuracy_);
    // }

    // if (settings_.use_adsb_filter_)
    // {
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_v0"] = settings_.use_v0_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_v1"] = settings_.use_v1_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_v2"] = settings_.use_v2_;

    //     // nucp
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_min_nucp"] = settings_.use_min_nucp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["min_nucp"] = settings_.min_nucp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_max_nucp"] = settings_.use_max_nucp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["max_nucp"] = settings_.max_nucp_;

    //     // nic
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_min_nic"] = settings_.use_min_nic_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["min_nic"] = settings_.min_nic_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_max_nic"] = settings_.use_max_nic_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["max_nic"] = settings_.max_nic_;

    //     // nacp
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_min_nacp"] = settings_.use_min_nacp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["min_nacp"] = settings_.min_nacp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_max_nacp"] = settings_.use_max_nacp_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["max_nacp"] = settings_.max_nacp_;

    //     // sil v1
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_min_sil_v1"] =
    //         settings_.use_min_sil_v1_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["min_sil_v1"] = settings_.min_sil_v1_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_max_sil_v1"] =
    //         settings_.use_max_sil_v1_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["max_sil_v1"] = settings_.max_sil_v1_;

    //     // sil v2
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_min_sil_v2"] =
    //         settings_.use_min_sil_v2_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["min_sil_v2"] = settings_.min_sil_v2_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["use_max_sil_v2"] =
    //         settings_.use_max_sil_v2_;
    //     data[ViewPoint::VP_FILTERS_KEY]["ADSB Quality"]["max_sil_v2"] = settings_.max_sil_v2_;
    // }

    if (eval_man_.use_timestamp_filter_)
    {
        data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Minimum"] =
            Time::toString(eval_man_.load_timestamp_begin_);
        data[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Maximum"] =
            Time::toString(eval_man_.load_timestamp_end_);
    }

    return data;
}

/**
 */
nlohmann::json::object_t EvaluationCalculator::getBaseViewableNoDataConfig () const
{
    nlohmann::json data;

    data["data_sources"] = std::map<unsigned int, std::set<unsigned int>>{};

    return data;
}

/**
 */
std::unique_ptr<nlohmann::json::object_t> EvaluationCalculator::getViewableForUTN (unsigned int utn) const
{
    nlohmann::json::object_t data = getBaseViewableDataConfig();
    data[ViewPoint::VP_FILTERS_KEY]["UTNs"]["utns"] = to_string(utn);

    return std::unique_ptr<nlohmann::json::object_t>{new nlohmann::json::object_t(move(data))};
}

/**
 */
std::unique_ptr<nlohmann::json::object_t> EvaluationCalculator::getViewableForEvaluation (const std::string& req_grp_id, 
                                                                                          const std::string& result_id) const
{
    nlohmann::json::object_t data = getBaseViewableNoDataConfig();

    return std::unique_ptr<nlohmann::json::object_t>{new nlohmann::json::object_t(move(data))};
}

/**
 */
std::unique_ptr<nlohmann::json::object_t> EvaluationCalculator::getViewableForEvaluation (unsigned int utn, 
                                                                                          const std::string& req_grp_id, 
                                                                                          const std::string& result_id) const
{
    return getViewableForUTN(utn);
}

/**
 */
void EvaluationCalculator::showUTN (unsigned int utn)
{
    loginf << "utn " << utn;

    nlohmann::json data = getBaseViewableDataConfig();
    data[ViewPoint::VP_FILTERS_KEY]["UTNs"]["utns"] = to_string(utn);

    loginf << "showing";
    eval_man_.setViewableDataConfig(data);
}

/**
 */
void EvaluationCalculator::showFullUTN (unsigned int utn)
{
    nlohmann::json::object_t data;
    data[ViewPoint::VP_FILTERS_KEY]["UTNs"]["utns"] = to_string(utn);

    eval_man_.setViewableDataConfig(data);
}

/**
 */
void EvaluationCalculator::showSurroundingData (const EvaluationTarget& target)
{
    nlohmann::json::object_t data;

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
        data[ViewPoint::VP_FILTERS_KEY]["Aircraft Address"]["Aircraft Address Values"] = target.aircraftAddressesStr()+",NULL";

    //    "Mode 3/A Code": {
    //    "Mode 3/A Code Values": "7000"
    //    }

    if (target.modeACodes().size())
        data[ViewPoint::VP_FILTERS_KEY]["Mode 3/A Codes"]["Mode 3/A Codes Values"] = target.modeACodesStr()+",NULL";

    //    VP_FILTERS_KEY: {
    //    "Barometric Altitude": {
    //    "Barometric Altitude Maximum": "43000",
    //    "Barometric Altitude Minimum": "500"
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

    eval_man_.setViewableDataConfig(data);
}

/**
 */
const json::boolean_t& EvaluationCalculator::useGroupInSectorLayer(const std::string& sector_layer_name,
                                                                   const std::string& group_name) const
{
    traced_assert(hasCurrentStandard());

    nlohmann::json& use_grp_in_sector = const_cast<nlohmann::json&>(settings_.use_grp_in_sector_);

    // standard_name->sector_layer_name->req_grp_name->bool use
    if (!use_grp_in_sector.contains(settings_.current_standard_) || 
        !use_grp_in_sector.at(settings_.current_standard_).contains(sector_layer_name) || 
        !use_grp_in_sector.at(settings_.current_standard_).at(sector_layer_name).contains(group_name))
        use_grp_in_sector[settings_.current_standard_][sector_layer_name][group_name] = false;

    return use_grp_in_sector[settings_.current_standard_][sector_layer_name][group_name].get_ref<const json::boolean_t&>();
}

/**
 */
void EvaluationCalculator::useGroupInSectorLayer(const std::string& sector_layer_name,
                                                 const std::string& group_name, 
                                                 bool value)
{
    traced_assert(hasCurrentStandard());

    loginf << "start"
           << " standard_name " << settings_.current_standard_
           << " sector_layer_name " << sector_layer_name
           << " group_name " << group_name 
           << " value " << value;

    settings_.use_grp_in_sector_[settings_.current_standard_][sector_layer_name][group_name] = value;
}

/**
 */
const json::boolean_t& EvaluationCalculator::useRequirement(const std::string& standard_name, 
                                                            const std::string& group_name,
                                                            const std::string& req_name) const
{
    nlohmann::json& use_requirement = const_cast<nlohmann::json&>(settings_.use_requirement_);

    // standard_name->req_grp_name->req_grp_name->bool use
    if (!use_requirement.contains(standard_name) || 
        !use_requirement.at(standard_name).contains(group_name) || 
        !use_requirement.at(standard_name).at(group_name).contains(req_name))
        use_requirement[standard_name][group_name][req_name] = true;

    return use_requirement[standard_name][group_name][req_name].get_ref<const json::boolean_t&>();
}

/**
 */
void EvaluationCalculator::useRequirement(const std::string& standard_name, 
                                          const std::string& group_name,
                                          const std::string& req_name,
                                          bool value)
{
    loginf << "start"
           << " standard_name " << standard_name
           << " group_name " << group_name 
           << " req_name " << req_name 
           << " value " << value;

    settings_.use_requirement_[standard_name][group_name][req_name] = value;
}

/**
 */
void EvaluationCalculator::updateSectorROI()
{
    loginf;

    sector_roi_.reset();

    if (!settings_.load_only_sector_data_ || 
        !hasCurrentStandard() || 
        !sectorsLoaded() || 
        sectorLayers().empty())
        return;

    bool first = true;
    double lat_min, lat_max, long_min, long_max;
    double tmp_lat_min, tmp_lat_max, tmp_long_min, tmp_long_max;

    const auto& standard      = currentStandard();
    const auto& sector_layers = sectorLayers();
    
    for (auto& sec_it : sector_layers)
    {
        const string& sector_layer_name = sec_it->name();

        for (auto& req_group_it : standard)
        {
            const string& requirement_group_name = req_group_it->name();

            if (!useGroupInSectorLayer(sector_layer_name, requirement_group_name))
                continue; // skip if not used

            if (first)
            {
                tie(lat_min, lat_max) = sec_it->getMinMaxLatitude();
                tie(long_min, long_max) = sec_it->getMinMaxLongitude();
                first = false;
            }
            else
            {
                tie(tmp_lat_min, tmp_lat_max) = sec_it->getMinMaxLatitude();
                tie(tmp_long_min, tmp_long_max) = sec_it->getMinMaxLongitude();

                lat_min = min(lat_min, tmp_lat_min);
                lat_max = max(lat_max, tmp_lat_max);
                long_min = min(long_min, tmp_long_min);
                long_max = max(long_max, tmp_long_max);
            }
        }
    }

    //        lat_min = 46.49;
    //        lat_max = 49.16;
    //        long_min = 11.39;
    //        long_max = 17.43;

    if (!first)
    {
        ROI roi;
        roi.latitude_min  = lat_min  - 0.2;
        roi.latitude_max  = lat_max  + 0.2;
        roi.longitude_min = long_min - 0.2;
        roi.longitude_max = long_max + 0.2;

        sector_roi_ = roi;
    }
}

/**
 */
void EvaluationCalculator::updateCompoundCoverage(std::set<unsigned int> tst_sources)
{
    loginf;

    tst_srcs_coverage_->clear();

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    for (auto ds_id : tst_sources)
    {
        traced_assert(ds_man.hasDBDataSource(ds_id));

        dbContent::DBDataSource& ds = ds_man.dbDataSource(ds_id);

        if (ds.hasRadarRanges())
        {
            bool range_max_set = false;
            double range_max = 0;

            for (auto range_it : ds.radarRanges())
            {
                if (range_max_set)
                    range_max = max(range_max, range_it.second);
                else
                {
                    range_max = range_it.second;
                    range_max_set = true;
                }
            }

            if (range_max_set && ds.hasPosition())
            {
                loginf << "adding src " << ds.name()
                       << " range " << range_max * NM2M;

                tst_srcs_coverage_->addRangeCircle(ds_id, ds.latitude(), ds.longitude(), range_max * NM2M);
            }
        }
    }

    tst_srcs_coverage_->finalize();
}

/**
*/
void EvaluationCalculator::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    //update some derived params after config param change
    updateDerivedParameters();

    //clear data & results
    clearData();

    //for (const auto& p : changed_params)
    //    loginf << "param: " << p;

    //@TODO
}
