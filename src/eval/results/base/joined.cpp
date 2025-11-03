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

#include "eval/results/base/joined.h"
#include "eval/results/base/single.h"
#include "eval/results/base/result_t.h"

#include "task/result/report/sectioncontenttable.h"
#include "task/result/report/section.h"

#include "eval/results/report/evalsectionid.h"

#include "eval/requirement/base/base.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "view/points/viewpointgenerator.h"
#include "view/points/viewpoint.h"

#include "view/gridview/grid2d.h"
#include "view/gridview/grid2dlayer.h"
#include "view/gridview/grid2d_defs.h"

#include "compass.h"
#include "tbbhack.h"

#include <QFileDialog>
#include <QMessageBox>

namespace EvaluationRequirementResult
{

const std::string Joined::SectorTargetsTableName        = "Targets";
const std::string Joined::SectorOverviewTableName       = "Sector Overview";
const std::string Joined::SectorOverviewID              = "Sector Overview";
const int         Joined::SectorOverviewRenderDelayMSec = 5000;

/**
*/
Joined::Joined(const std::string& type, 
                const std::string& result_id,
                std::shared_ptr<EvaluationRequirement::Base> requirement, 
                const SectorLayer& sector_layer,
                EvaluationCalculator& calculator)
:   Base(type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
Joined::~Joined() = default;

/**
*/
void Joined::clearResults()
{
    num_targets_        = 0;
    num_failed_targets_ = 0;

    //invoke derived
    clearResults_impl();
}

/**
*/
void Joined::updateResult()
{
    Base::updateResult(computeResult());
}

/**
*/
boost::optional<double> Joined::computeResult() const
{
    return computeResult_impl();
}

/**
*/
unsigned int Joined::numSingleResults() const
{
    return results_.size();
}

/**
*/
unsigned int Joined::numUsableSingleResults() const
{
    unsigned int cnt {0};

    for (auto& result_it : results_)
        if (result_it->use())
            ++cnt;

    return cnt;
}

/**
*/
unsigned int Joined::numUnusableSingleResults() const
{
    unsigned int cnt {0};

    for (auto& result_it : results_)
        if (!result_it->use())
            ++cnt;

    return cnt;
}

/**
*/
bool Joined::hasReference(const ResultReport::SectionContentTable& table, 
                          const QVariant& annotation) const
{
    if (table.name() == RequirementOverviewTableName)
        return true;
    else
        return false;;
}

/**
*/
std::string Joined::reference(const ResultReport::SectionContentTable& table, 
                              const QVariant& annotation) const
{
    traced_assert(hasReference(table, annotation));
    return EvalSectionID::createForRequirementResult(*this);
}

/**
*/
std::string Joined::getRequirementAnnotationID_impl() const
{
    return (requirement_->name() + ":" + sector_layer_.name());
}

/**
*/
void Joined::iterateDetails(const DetailFunc& func,
                            const DetailSkipFunc& skip_func) const
{
    auto funcSingleResults = [ & ] (const std::shared_ptr<Single>& result)
    {
        result->iterateDetails(func, skip_func);
    };

    iterateSingleResults({}, funcSingleResults, {});
}

/**
*/
void Joined::addToReport(std::shared_ptr<ResultReport::Report> report)
{
    logdbg << "start" <<  requirement_->name();

    if (!results_.size()) // some data must exist
    {
        logerr << "start" <<  requirement_->name() <<": no data";
        return;
    }

    logdbg << "start" <<  requirement_->name() << ": adding joined result";

    addSectorToOverviewTable(report);
    addSectorDetailsToReport(report);
}

/**
*/
void Joined::addSectorToOverviewTable(std::shared_ptr<ResultReport::Report> report)
{
    auto& ov_table = getReqOverviewTable(report);

    auto must_hold_for_any_target = requirement_->mustHoldForAnyTarget();

    auto ref = getRequirementSectionID(); // this gives the associated requirement section link 
                                          // needed to retrieve the correct section from the report
    auto fig = SectorOverviewID;          // overview figure added in addSectorDetailsToReport()

    if (must_hold_for_any_target.has_value() && must_hold_for_any_target.value())
    {
        //just check if any target failed
        std::string result = num_failed_targets_ == 0 ? "Passed" : "Failed";

        // "Sector Layer", "Group", "Req.", "Id", "#Updates", "Result Value", "Condition", "Condition Result"
        ov_table.addRow({ sector_layer_.name(), 
                          requirement_->groupName(),
                          requirement_->shortname(),
                          result_id_, 
                          num_targets_,
                          num_failed_targets_, 
                          "= 0", 
                          result }, {}, ref, fig, {});
    }
    else
    {
        //probabilisitc condition
        auto result_val = resultValue();

        std::string result {"Unknown"};

        if (!result_val.is_null())
            result = conditionResultString();

        // "Sector Layer", "Group", "Req.", "Id", "#Updates", "Result Value", "Condition", "Condition Result"
        ov_table.addRow({ sector_layer_.name(), 
                          requirement_->groupName(),
                          requirement_->shortname(),
                          result_id_, 
                          numUpdates(),
                          result_val, 
                          requirement_->getConditionStr(), 
                          result }, {}, ref, fig, {});
    }
}

/**
*/
std::vector<Joined::SectorInfo> Joined::sectorInfosCommon() const
{
    return { { "Sector Layer"        , "Name of the sector layer"     , sector_layer_.name()       },
             { "Requirement Group"    , "Name of the requirement group", requirement_->groupName()  },
             { "Requirement"          , "Name of the requirement"      , requirement_->name()       },
             { "Num Results"         , "Total number of results"      , numSingleResults()         },
             { "Num Usable Results"  , "Number of usable results"     , numUsableSingleResults()   },
             { "Num Unusable Results", "Number of unusable results"   , numUnusableSingleResults() },
             { "Use"                 , "To be used in results"        , use_                       } };
}

/**
*/
std::vector<Joined::SectorInfo> Joined::sectorConditionInfos() const
{
    std::vector<SectorInfo> infos;

    auto must_hold_for_any_target = requirement_->mustHoldForAnyTarget();
    bool show_target_condition = must_hold_for_any_target.has_value() &&
                                 must_hold_for_any_target.value();

    auto result_val = resultValue();

    if (!show_target_condition)
    {
        infos.push_back({ requirement_->getConditionResultNameShort(true), 
                          requirement_->getConditionResultName(), 
                          result_val });
    }

    std::string result_name   = requirement_->getConditionResultNameShort(false);
    std::string condition_str = requirement_->getConditionStr();

    std::string condition_name   = show_target_condition ? "Single Target Condition" : "Condition";
    std::string confition_string = show_target_condition ? result_name + " " + condition_str : condition_str;

    infos.push_back({condition_name, "", confition_string });

    if (!show_target_condition)
    {
        std::string result {"Unknown"};

        if (!result_val.is_null())
            result = conditionResultString();

        infos.push_back({condition_name + " Fulfilled", "", result});
    }

    if (must_hold_for_any_target.has_value())
    {
        infos.emplace_back("Must hold for any target ", "", must_hold_for_any_target.value());

        if (must_hold_for_any_target.value())
        {
            infos.emplace_back("#Single Targets"       , "", num_targets_       );
            infos.emplace_back("#Failed Single Targets", "", num_failed_targets_);

            infos.push_back({"Sum Condition", "", "= 0" });

            std::string result = num_failed_targets_ == 0 ? "Passed" : "Failed";

            infos.push_back({"Sum Condition Fulfilled", "", result});
        }
    }

    return infos;
}

/**
*/
void Joined::addSectorDetailsToReport(std::shared_ptr<ResultReport::Report> report)
{
    auto& sector_section = getRequirementSection(report);

    if (!sector_section.hasTable(SectorOverviewTableName))
        sector_section.addTable(SectorOverviewTableName, 3, {"Name", "Comment", "Value"}, false);

    auto& sec_det_table = sector_section.getTable(SectorOverviewTableName);

    // callbacks
    if (canExportCSV())
    {
        auto exportAsCSV_lambda = [this]() { this->exportAsCSV(); };

        sec_det_table.registerCallBack("Save Data As CSV", exportAsCSV_lambda);
    }

    // details
    auto infos_common = sectorInfosCommon();

    //@TODO: the following rows were given this result as a pointer, is this even needed?

    for (const auto& info : infos_common)
        sec_det_table.addRow({ info.info_name, info.info_comment, info.info_value });

    auto infos = sectorInfos();

    for (const auto& info : infos)
        sec_det_table.addRow({ info.info_name, info.info_comment, info.info_value });

    auto infos_condition = sectorConditionInfos();

    for (const auto& info : infos_condition)
        sec_det_table.addRow({ info.info_name, info.info_comment, info.info_value });

    // figure
    addOverview(sector_section);
}

/**
*/
bool Joined::exportAsCSV() const
{
    if (!canExportCSV())
        return false;

    loginf << "start" << type();

    QFileDialog dialog(nullptr);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDirectory(COMPASS::instance().lastUsedPath().c_str());
    dialog.setNameFilter("CSV Files (*.csv)");
    dialog.setDefaultSuffix("csv");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);

    if (dialog.exec() != QDialog::Accepted)
        return true;

    QStringList file_names = dialog.selectedFiles();
    traced_assert(file_names.size() == 1);

    std::string filename = file_names.at(0).toStdString();

    std::ofstream output_file;
    output_file.open(filename, std::ios_base::out);

    if (!output_file || !output_file.is_open() || !exportAsCSV(output_file))
    {
        QMessageBox::critical(nullptr, "Error", "Exporting CSV file failed.");
        return false;
    }

    return true;
}

/**
*/
void Joined::addOverview (ResultReport::Section& section,
                          const std::string& name)
{
    section.addFigure(SectorOverviewID, 
                      ResultReport::SectionContentViewable([this]() { return this->viewableOverviewData(); })
                        .setCaption(name).setRenderDelayMS(SectorOverviewRenderDelayMSec));
}

/**
*/
bool Joined::resultUsed(const std::shared_ptr<Single>& result) const
{
    return (result->use() && result->resultUsable());
}

/**
*/
void Joined::iterateSingleResults(const SingleResultFunc& func,
                                  const SingleResultFunc& func_used,
                                  const SingleResultFunc& func_unused) const
{
    traced_assert(func_used);

    for (const auto& result_it : results_)
    {
        if (func) 
            func(result_it);

        if (!resultUsed(result_it))
        {
            if (func_unused) 
                func_unused(result_it);
            continue;
        }

        if (func_used) 
            func_used(result_it);
    }
}

/**
*/
void Joined::addSingleResult(std::shared_ptr<Single> other)
{
    results_.push_back(other);
}

/**
*/
std::vector<std::shared_ptr<Single>>& Joined::singleResults() 
{ 
    return results_; 
}

/**
*/
std::vector<std::shared_ptr<Single>> Joined::usedSingleResults() const
{
    std::vector<std::shared_ptr<Single>> used_results;

    auto funcSingleResults = [ & ] (const std::shared_ptr<Single>& result)
    {
        used_results.push_back(result);
    };

    iterateSingleResults({}, funcSingleResults, {});

    return used_results;
}

/**
*/
bool Joined::hasStoredDetails() const
{
    bool has_details = true;

    for (const auto& single : results_)
    {
        if (!resultUsed(single))
            continue;

        if (!single->hasStoredDetails())
        {
            has_details = false;
            break;
        }
    }

    return has_details;
}

/**
*/
void Joined::updateToChanges(bool reset_viewable)
{
    //clear first
    clearResults();

    //invalidate cached viewable?
    if (reset_viewable)
        viewable_.reset();

    //accumulate single results
    size_t nr = results_.size();

    std::vector<size_t> used;

    //collect used results
    for (size_t i = 0; i < nr; ++i)
    {
        auto& single = results_[ i ];

        //reset interest
        single->setInterestFactor(0, true);

        if (resultUsed(single))
            used.push_back(i);
    }

    size_t nu = used.size();

    //accumulate all used results
    for (size_t i = 0; i < nu; ++i)
    {
        auto& single = results_[ used[ i ] ];

        ++num_targets_;

        if (single->hasFailed())
            ++num_failed_targets_;

        accumulateSingleResult(single, i == 0, i == nu - 1);
    }

    //update result
    updateResult();

    //update interests if result is usable
    auto issues_total = numIssues();

    if (issues_total > 0 && resultUsable())
    {
        for (size_t i = 0; i < nu; ++i)
        {
            auto& single = results_[ used[ i ] ];

            auto issues = single->numIssues();
            traced_assert(issues_total >= issues);

            single->setInterestFactor((double)issues / (double)issues_total);
        }
    }

    //update viewable immediately if details are still stored
    if (hasStoredDetails())
        getOrCreateCachedViewable();
}

/**
*/
std::vector<double> Joined::getValues(const ValueSource<double>& source) const
{
    return EvaluationResultTemplates(this).getValues<double>(source);
}

/**
*/
std::vector<double> Joined::getValues(int value_id) const
{
    return getValues(ValueSource<double>(value_id));
}

/**
*/
std::shared_ptr<nlohmann::json::object_t> Joined::getOrCreateCachedViewable() const
{
    if (!viewable_)
    {
        loginf << "recreating viewable for "
               << "requirement '" << requirement_->name() << "' " 
               << "sector '" << sector_layer_.name() << "'..."; 

        //cache all needed single details in parallel
        auto used_results = usedSingleResults();

        unsigned int n = used_results.size();

        std::vector<Single::TemporaryDetails> temp_details(n);

        tbb::parallel_for(uint(0), n, [ & ] (unsigned int idx)
        {
            temp_details[ idx ] = used_results[ idx ]->temporaryDetails();
        });

        //create new viewable
        viewable_ = createViewable(AnnotationOptions().overview());

        //wipe all single result details
        temp_details.resize(0);
    }

    return viewable_;
}

/**
*/
std::unique_ptr<nlohmann::json::object_t> Joined::createBaseViewable() const
{
    return calculator_.getViewableForEvaluation(req_grp_id_, result_id_);
}

/**
*/
Base::ViewableInfo Joined::createViewableInfo(const AnnotationOptions& options) const
{
    ViewableInfo info;
    info.viewable_type = ViewableType::Overview;

    double lat_min, lat_max, lon_min, lon_max;

    std::tie(lat_min, lat_max) = sector_layer_.getMinMaxLatitude();
    std::tie(lon_min, lon_max) = sector_layer_.getMinMaxLongitude();

    info.bounds = QRectF(lat_min, lon_min, lat_max - lat_min, lon_max - lon_min);

    return info;
}

/**
*/
void Joined::createAnnotations(nlohmann::json& annotations, 
                               const AnnotationOptions& options) const
{
    //everything handled via custom annotations
    addCustomAnnotations(annotations);
}

/**
 */
bool Joined::hasViewableData (const ResultReport::SectionContentTable& table, 
                              const QVariant& annotation) const
{
    if (table.name() != RequirementOverviewTableName)
        return false;

    return true;
}

/**
*/
bool Joined::viewableDataReady() const
{
    return (viewable_ != nullptr);
}

/**
 */
std::shared_ptr<nlohmann::json::object_t> Joined::viewableData(const ResultReport::SectionContentTable& table, 
                                                               const QVariant& annotation) const
{
    traced_assert(hasViewableData(table, annotation));

    //return cached viewable (might recreate the viewable)
    return getOrCreateCachedViewable();
}

/**
*/
std::shared_ptr<nlohmann::json::object_t> Joined::viewableOverviewData() const
{
    //return cached viewable (might recreate the viewable)
    return getOrCreateCachedViewable();
}

/**
 * Legacy code for creating annotations for every single target report in the sector.
*/
// void Joined::addOverviewAnnotations(nlohmann::json& annotations_json) const
// {
//     //add annotations from single results
//     for (auto& single_result : results_)
//     {
//         if (single_result->use())
//         {
//             //create overview annotations for single result
//             single_result->createSumOverviewAnnotations(annotations_json, 
//                                                         eval_man_.settings().show_ok_joined_target_reports_);
//         }
//     }
// }

/**
*/
void Joined::setJoinedContentProperties(ResultReport::SectionContent& content,
                                        const Evaluation::RequirementResultID& id)
{
    Base::setContentProperties(content, id);
}

/**
*/
boost::optional<Evaluation::RequirementResultID> Joined::joinedContentProperties(const ResultReport::SectionContent& content)
{
    return Base::contentProperties(content);
}

}
