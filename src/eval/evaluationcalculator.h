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

#include "evaluationdefs.h"
#include "evaluationsettings.h"
#include "evaluationdata.h"
#include "evaluationresultsgenerator.h"
#include "datasourcecompoundcoverage.h"

#include "target.h"

#include "result.h"

#include "json_fwd.hpp"

#include <boost/optional.hpp>

#include <QObject>

class EvaluationStandard;
class EvaluationTarget;
class EvaluationManager;
class DBContentManager;

class DBContent;
class SectorLayer;
class AirSpace;

namespace dbContent 
{
    class VariableSet;
}

namespace EvaluationRequirementResult
{
    class Base;
    class Single;
    class Joined;
}

/**
 */
class EvaluationCalculator : public QObject, public Configurable
{
    Q_OBJECT
public:
    struct EvaluationDS
    {
        std::string  name;
        unsigned int id;
    };

    struct EvaluationDSInfo
    {
        std::string               dbcontent;
        std::vector<EvaluationDS> data_sources;
    };

    struct ROI
    {
        double latitude_min  {0};
        double latitude_max  {0};
        double longitude_min {0};
        double longitude_max {0};
    };

    typedef std::shared_ptr<EvaluationRequirementResult::Base>      ResultPtr;
    typedef std::map<std::string, std::map<std::string, ResultPtr>> ResultMap;
    typedef ResultMap::const_iterator                               ResultIterator;

    EvaluationCalculator(nlohmann::json& config, EvaluationManager& eval_man,
                         DBContentManager& dbcontent_man, bool is_default_calculator);
    EvaluationCalculator(EvaluationManager& eval_man,
                         DBContentManager& dbcontent_man,
                         const nlohmann::json& config,
                         bool is_default_calculator);
    virtual ~EvaluationCalculator();

    ResultT<EvaluationCalculator*> clone();
    static ResultT<EvaluationCalculator*> clone(EvaluationManager& eval_man,
                                                DBContentManager& dbcontent_man,
                                                const nlohmann::json& config);

    bool hasPartialResult() const;
    bool dataLoaded() const; 
    bool evaluated() const;
    Result canEvaluate() const;

    void reset();
    void clearData();

    Result evaluate();
    Result update();
    Result reloadNeededData(const std::vector<unsigned int>& utns,
                            const std::vector<Evaluation::RequirementResultID>& requirements);
    void updateResultsToChanges();

    // check and correct missing information
    void checkReferenceDataSources(bool update_settings = true);
    void checkTestDataSources(bool update_settings = true);
    void checkMinHeightFilterValid();
    void checkConfiguration();

    // data sources
    std::string dbContentNameRef() const;
    void dbContentNameRef(const std::string& name);

    bool hasValidReferenceDBContent () const;
    const std::map<std::string, bool>& dataSourcesRef() const;
    std::set<unsigned int> activeDataSourcesRef();
    EvaluationDSInfo activeDataSourceInfoRef() const;
    void selectDataSourceRef(const std::string& name, bool select, bool update_settings = true);

    bool hasSelectedReferenceDataSources() const;

    std::string dbContentNameTst() const;
    void dbContentNameTst(const std::string& name);

    bool hasValidTestDBContent () const;
    const std::map<std::string, bool>& dataSourcesTst() const;
    std::set<unsigned int> activeDataSourcesTst();
    EvaluationDSInfo activeDataSourceInfoTst() const;
    void selectDataSourceTst(const std::string& name, bool select, bool update_settings = true);

    bool hasSelectedTestDataSources() const;

    std::map<unsigned int, std::set<unsigned int>> usedDataSources() const;

    // standards & requirements
    bool hasCurrentStandard() const;
    std::string currentStandardName() const; // can return empty string, indicating no standard
    void currentStandardName(const std::string& current_standard);
    void renameCurrentStandard (const std::string& new_name);
    void copyCurrentStandard (const std::string& new_name);
    EvaluationStandard& currentStandard();
    const EvaluationStandard& currentStandard() const;

    bool hasStandard(const std::string& name) const;
    void addStandard(const std::string& name);
    void deleteCurrentStandard();

    using EvaluationStandardIterator = typename std::vector<std::unique_ptr<EvaluationStandard>>::iterator;
    EvaluationStandardIterator standardsBegin() { return standards_.begin(); }
    EvaluationStandardIterator standardsEnd() { return standards_.end(); }
    unsigned int standardsSize () { return standards_.size(); };

    std::vector<std::string> currentRequirementNames() const;

    const nlohmann::json::boolean_t& useRequirement(const std::string& standard_name, 
                                                    const std::string& group_name,
                                                    const std::string& req_name) const;
    void useRequirement(const std::string& standard_name, 
                        const std::string& group_name,
                        const std::string& req_name,
                        bool value);

    // report
    void resetCustomReportName(); 
    void setCustomReportName(const std::string& name);
    bool hasCustomReportName() const;
    const std::string& customReportName() const;
    std::string suggestReportName() const;
    std::string reportName() const;

    // sectors & min height filter
    bool sectorsLoaded() const;
    bool anySectorsWithReq() const;
    std::vector<std::shared_ptr<SectorLayer>>& sectorLayers();
    const std::vector<std::shared_ptr<SectorLayer>>& sectorLayers() const;
    void updateSectorLayers();
    void updateSectorROI();

    bool filterMinimumHeight() const;
    const std::string& minHeightFilterLayerName() const;
    void minHeightFilterLayerName(const std::string& layer_name);
    std::shared_ptr<SectorLayer> minHeightFilterLayer() const;

    const nlohmann::json::boolean_t& useGroupInSectorLayer(const std::string& sector_layer_name,
                                                           const std::string& group_name) const;
    void useGroupInSectorLayer(const std::string& sector_layer_name, 
                               const std::string& group_name, 
                               bool value);

    // base viewables
    void showUTN (unsigned int utn);
    void showFullUTN (unsigned int utn);
    void showSurroundingData (const EvaluationTarget& target);

    std::unique_ptr<nlohmann::json::object_t> getViewableForUTN (unsigned int utn) const;
    std::unique_ptr<nlohmann::json::object_t> getViewableForEvaluation (const std::string& req_grp_id, 
                                                                        const std::string& result_id) const; // empty load
    std::unique_ptr<nlohmann::json::object_t> getViewableForEvaluation (unsigned int utn, 
                                                                        const std::string& req_grp_id, 
                                                                        const std::string& result_id) const; // with data                                                            
    // results
    ResultIterator begin();
    ResultIterator end();

    bool hasResults() const;
    const ResultMap& results() const;
    const std::string& resultName() const;

    EvaluationRequirementResult::Single* singleResult(const Evaluation::RequirementResultID& id,
                                                      unsigned int utn) const;
    EvaluationRequirementResult::Joined* joinedResult(const Evaluation::RequirementResultID& id) const;

    EvaluationManager& manager() { return eval_man_; }
    const EvaluationManager& manager() const { return eval_man_; }
    EvaluationData& data() { return *data_; }
    const EvaluationData& data() const { return *data_; }
    EvaluationResultsGenerator& resultsGenerator() { return *results_gen_; }
    const EvaluationResultsGenerator& resultsGenerator() const { return *results_gen_; }
    EvaluationSettings& settings() { return settings_; }
    const EvaluationSettings& settings() const { return settings_; }
    const dbContent::DataSourceCompoundCoverage& tstSrcsCoverage() const { return *tst_srcs_coverage_; }
    const boost::optional<ROI>& sectorROI() const { return sector_roi_; }
    const std::vector<unsigned int>& evaluationUTNs() const { return eval_utns_; }

    bool globalTimeFilterEnabled() const;
    const Utils::TimeWindow& globalTimeWindow() const { return global_time_window_; }
    const Utils::TimeWindowCollection& globalExclusionTimeWindows() const { return global_exclusion_time_windows_; }

    const dbContent::TargetEvalConstraints* targetConstraint(unsigned int utn) const;
    const std::map<unsigned int, dbContent::TargetEvalConstraints>& targetConstraints() const { return target_constraints_; }

    std::string constraintsAsString() const;

    bool isTimeStampNotExcluded(const boost::posix_time::ptime& ts) const;

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;
signals:
    void standardsChanged();
    void currentStandardChanged();
    void resultsChanged();
    void evaluationDone();
    
protected:
    virtual void checkSubConfigurables() override;

    Result evaluateInternal(bool update_constraints,
                            bool update_report,
                            const std::vector<unsigned int>& utns,
                            const std::vector<Evaluation::RequirementResultID>& requirements);

    void readSettings();

    std::map<std::string, bool>& dataSourcesRef();
    std::map<std::string, bool>& dataSourcesTst();

    nlohmann::json::object_t getBaseViewableDataConfig () const;
    nlohmann::json::object_t getBaseViewableNoDataConfig () const;

    void updateCompoundCoverage(std::set<unsigned int> tst_sources);

    void updateDerivedParameters();
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    void updateConstraints();

    void loadedDataData(const std::map<std::string, std::shared_ptr<Buffer>>& data, bool requires_reset);
    Result loadingDone();
    Result evaluateData();

    void storeGlobalTimeWindow();
    void loadGlobalTimeWindow();

    void storeGlobalExclusionTimeWindows();
    void loadGlobalExclusionTimeWindows();

    void storeTargetConstraints();
    void loadTargetConstraints();

    nlohmann::json globalTimeWindowAsJSON() const;
    nlohmann::json globalExclustionTimeWindowsAsJSON() const;
    nlohmann::json targetConstraintsAsJSON() const;
    Utils::TimeWindow globalTimeWindowFromJSON(const nlohmann::json& j) const;
    Utils::TimeWindowCollection globalExclustionTimeWindowsFromJSON(const nlohmann::json& j) const;
    std::map<unsigned int, dbContent::TargetEvalConstraints> targetConstraintsFromJSON(const nlohmann::json& j) const;

    void clearEvalData();
    void clearConstraints();

    void setCurrentStandardName(const std::string& name);

    EvaluationManager& eval_man_;

    std::vector<unsigned int>                    eval_utns_;
    std::vector<Evaluation::RequirementResultID> eval_requirements_;

    EvaluationSettings settings_;

    //values derived from settings
    std::map<std::string, std::map<std::string, bool>> data_sources_ref_; // db_content -> ds_id -> active flag
    std::map<std::string, std::map<std::string, bool>> data_sources_tst_; // db_content -> ds_id -> active flag

    boost::optional<ROI> sector_roi_;

    bool data_loaded_            {false};
    bool reference_data_loaded_  {false};
    bool test_data_loaded_       {false};
    bool evaluated_              {false};
    
    bool active_load_connection_ {false};
    bool update_report_          {true };

    std::vector<std::unique_ptr<EvaluationStandard>> standards_;

    std::unique_ptr<EvaluationData>                        data_;
    std::unique_ptr<EvaluationResultsGenerator>            results_gen_;
    std::unique_ptr<dbContent::DataSourceCompoundCoverage> tst_srcs_coverage_;

    bool                                                     global_time_filter_enabled_ = false;
    nlohmann::json                                           global_time_window_json_;
    Utils::TimeWindow                                        global_time_window_;
    nlohmann::json                                           global_exclusion_time_windows_json_;
    Utils::TimeWindowCollection                              global_exclusion_time_windows_;
    nlohmann::json                                           target_constraints_json_;
    std::map<unsigned int, dbContent::TargetEvalConstraints> target_constraints_;

    std::string custom_report_name_;

    bool use_fast_sector_inside_check_ = true;

    bool is_default_calculator_ = false;
};
