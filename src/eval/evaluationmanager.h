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
#include "ievalprovider.h"
#include "evaluationdata.h"
#include "evaluationresultsgenerator.h"
#include "evaluationcalculator.h"
#include "viewabledataconfig.h"
#include "result.h"
#include "timewindow.h"

#include <QObject>

#include "json_fwd.hpp"

class COMPASS;
class ViewManager;
class FilterManager;
class EvaluationStandard;
class DBContent;
class DBContentManager;
class LoadOperation;
class SectorLayer;
struct EvaluationSettings;
class EvaluationTargetFilter;

namespace dbContent 
{
    class VariableSet;
}

class QWidget;

/**
 */
class EvaluationManager : public QObject, public Configurable, public IEvalProvider
{
    Q_OBJECT

signals:
    void sectorsChangedSignal();         // sectors changed (due to manual editing, loading, etc.)
    void standardsChangedSignal();       // emitted if standard was added or deleted
    void currentStandardChangedSignal(); // emitted if current standard was changed
    void evaluationDoneSignal();         // evaluation ended
    void hasNewData();                   // new data for evaluation is ready to be fetched
    void resultsNeedUpdate(int type);    // evaluation results need the sent update

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();
    void dataSourcesChangedSlot();
    void associationStatusChangedSlot();

    void sectorsChangedSlot();
    void targetInfoChangedSlot();
    void partialResultsUpdateNeededSlot();
    void fullResultsUpdateNeededSlot();
    void lockResultsSlot();

public:
    EvaluationManager(nlohmann::json& config, COMPASS& compass);
    virtual ~EvaluationManager();

    const EvaluationCalculator* calculator() const { return calculator_.get(); }
    EvaluationCalculator* calculator() { return calculator_.get(); }

    void init();
    void close();
    void clearData();

    Result canEvaluate() const;
    Result evaluate(bool show_dialog, const std::string& custom_result_name = "");

    //data loading
    bool needsAdditionalVariables() const override;
    void addVariables(const std::string& dbcontent_name, dbContent::VariableSet& read_set) override;

    //viewables
    void setViewableDataConfig (const nlohmann::json::object_t& data);
    void resetViewableDataConfig(bool reset_view_point);

    //timestamps
    boost::posix_time::ptime loadTimestampBegin() const;
    void loadTimestampBegin(boost::posix_time::ptime value);

    boost::posix_time::ptime loadTimestampEnd() const;
    void loadTimestampEnd(boost::posix_time::ptime value);

    Utils::TimeWindowCollection& excludedTimeWindows(); // needs to be saved externally
    void saveTimeConstraints();

    bool useTimestampFilter() const override;
    void useTimestampFilter(bool value);
    std::string timestampFilterStr() const;

    bool removeDisabledUTNData() const { return remove_disabled_utn_data_; } // not load disabled utn for performance hack

    //standards
    bool hasCurrentStandard() const override;
    const EvaluationStandard& currentStandard() const;
    std::set<std::string> currentRequirementNames() const override;

    //target list
    void applyTargetFilter(dbContent::TargetCache& target_data) override;

    void editTargetFilter(dbContent::TargetModel& model, QWidget* parent) override;
    bool editTimestampConditions(QWidget* parent) override;
    bool editExcludedTimeWindows(const std::string& utns,
                                 Utils::TimeWindowCollection& time_windows,
                                 QWidget* parent) override;
    bool editExcludedRequirements(const std::string& utns,
                                  std::set<std::string>& selected_requirements,
                                  const std::set<std::string>& all_requirements,
                                  std::string& comment,
                                  QWidget* parent) override;

    //other stuff
    EvaluationTargetFilter& targetFilter() const;
    const std::string& lastResultName() const { return last_result_name_; }

    virtual void generateSubConfigurable(nlohmann::json& child_json) override;

    COMPASS& compass() { return compass_; }
    DBContentManager& dbContentManager() { return dbcontent_man_; }

    // peers, set by the composition root
    void setManagers(ViewManager& view_manager,
                     FilterManager& filter_manager);

    ViewManager& viewManager();
    FilterManager& filterManager();

protected:
    friend class EvaluationCalculator;

    virtual void checkSubConfigurables() override;
    virtual void onConfigurationChanged(const std::vector<std::string>& changed_params) override;

    void updateSectorLayers();

    void loadData(const EvaluationCalculator& calculator); // blocking; harvested in loadingDone
    std::map<std::string, std::shared_ptr<Buffer>> fetchData();

    // update cycles reported by the test data sources, used by the status-message
    // PD method. Empty when the sources report none.
    bool hasStatusCycles(unsigned int ds_id, unsigned int line_id) const;
    const std::vector<boost::posix_time::ptime>& statusCycles(unsigned int ds_id,
                                                              unsigned int line_id) const;

private:
    // per-content load WHERE for eval, built from the shared clause toolkit
    // (ROI bbox / UTN set / timestamp bounds) - no global-filter hijack
    std::string loadFilterClause(const std::string& dbcontent_name,
                                 const EvaluationCalculator& calculator);
    void loadingDone();

    // isolated batch load of the start-of-update-cycle messages of the test data sources
    void loadStatusCycles(const EvaluationCalculator& calculator);

    COMPASS& compass_;
    DBContentManager& dbcontent_man_;

    // peers, set by the composition root
    ViewManager*   view_manager_   = nullptr;
    FilterManager* filter_manager_ = nullptr;

    bool initialized_ {false};

    bool needs_additional_variables_ {false}; // indicates if variables should be added during loading

    std::unique_ptr<EvaluationTargetFilter> target_filter_;
    std::unique_ptr<EvaluationCalculator> calculator_; // sub-configurable

    // shared with ViewManager so it cannot be released while a load it started is in flight
    std::shared_ptr<ViewableDataConfig>            viewable_data_cfg_;
    std::shared_ptr<LoadOperation>                 load_op_; // isolated batch load, released after harvest
    std::map<std::string, std::shared_ptr<Buffer>> raw_data_;
    bool                                           raw_data_available_ = false;

    // ds_id -> line_id -> sorted start-of-update-cycle timestamps
    std::map<unsigned int, std::map<unsigned int, std::vector<boost::posix_time::ptime>>> status_cycles_;

    bool                        use_timestamp_filter_ {false}; // enables/disables BOTH application of timestamp load filter and exclusion windows load filter
    boost::posix_time::ptime    load_timestamp_begin_;         // ts filter begin (added to timestamp load filter)
    boost::posix_time::ptime    load_timestamp_end_;           // ts filter end (added to timestamp load filter)
    Utils::TimeWindowCollection load_filtered_time_windows_;   // exclusion windows (added to exclusion windows load filter)

    bool remove_disabled_utn_data_{false};

    std::string last_result_name_;
};
