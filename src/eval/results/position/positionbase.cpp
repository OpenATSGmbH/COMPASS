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

#include "eval/results/position/positionbase.h"
#include "eval/requirement/position/distance.h"

#include "eval/results/base/featuredefinitions.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/position/distance.h"
#include "evaluationmanager.h"

#include "histograminitializer.h"
#include "histogram.h"
#include "viewpointgenerator.h"

#include "logger.h"

using namespace std;
using namespace Utils;

namespace EvaluationRequirementResult
{

/****************************************************************************
 * PositionBase
 ****************************************************************************/

/**
*/
PositionBase::PositionBase() = default;

/**
*/
PositionBase::PositionBase(unsigned int num_pos,
                           unsigned int num_no_ref,
                           unsigned int num_pos_outside,
                           unsigned int num_pos_inside,
                           unsigned int num_ref_inaccurate,
                           unsigned int num_passed,
                           unsigned int num_failed)
:   num_pos_        (num_pos)
,   num_no_ref_     (num_no_ref)
,   num_pos_outside_(num_pos_outside)
,   num_pos_inside_ (num_pos_inside)
,   num_passed_     (num_passed)
,   num_failed_     (num_failed)
{
}

/**
*/
unsigned int PositionBase::numPassed() const
{
    return num_passed_;
}

/**
*/
unsigned int PositionBase::numFailed() const
{
    return num_failed_;
}

/**
*/
unsigned int PositionBase::numPosOutside() const
{
    return num_pos_outside_;
}

/**
*/
unsigned int PositionBase::numPosInside() const
{
    return num_pos_inside_;
}

unsigned int PositionBase::numRefInaccurate() const
{
    return num_ref_inaccurate_;
}

/**
*/
unsigned int PositionBase::numPos() const
{
    return num_pos_;
}

/**
*/
unsigned int PositionBase::numNoRef() const
{
    return num_no_ref_;
}

/**
*/
const ValueAccumulator& PositionBase::accumulator() const
{
    return accumulator_;
}

/****************************************************************************
 * SinglePositionBaseCommon
 ****************************************************************************/

/**
*/
SinglePositionBaseCommon::SinglePositionBaseCommon(unsigned int num_pos, unsigned int num_no_ref,
                                                   unsigned int num_pos_outside,
                                                   unsigned int num_pos_inside,
                                                   unsigned int num_ref_inaccurate,
                                                   unsigned int num_passed, unsigned int num_failed)
    : PositionBase(num_pos, num_no_ref, num_pos_outside, num_pos_inside,
                   num_ref_inaccurate, num_passed, num_failed)
{
}

/**
*/
boost::optional<double> SinglePositionBaseCommon::common_computeResult(const Single* single_result) const
{
    traced_assert(single_result);
    traced_assert(num_no_ref_ <= num_pos_);
    traced_assert(num_pos_ - num_no_ref_ == num_pos_inside_ + num_pos_outside_);

    accumulator_.reset();

    auto values = single_result->getValues(DetailKey::Value);

    traced_assert(values.size() == num_passed_ + num_failed_);

    unsigned int num_distances = values.size();

    traced_assert(num_passed_ <= num_distances);

    if (num_distances > 0)
        accumulator_.accumulate(values, true);

    return computeFinalResultValue();
}

/**
*/
unsigned int SinglePositionBaseCommon::common_numIssues() const
{
    return num_failed_;
}

/**
*/
bool SinglePositionBaseCommon::common_detailIsOk(const EvaluationDetail& detail,
                                                 const std::shared_ptr<EvaluationRequirement::Base>& requirement) const
{
    const EvaluationRequirement::PositionDistance* req = dynamic_cast<const EvaluationRequirement::PositionDistance*>(requirement.get());

    auto check_passed = detail.getValueAs<bool>(DetailKey::CheckPassed);
    traced_assert(check_passed.has_value());

    if (req)
    {
        //distance requirement => check failed values of interest
        auto failed_values_of_interest = req->failedValuesOfInterest();

        return (( failed_values_of_interest &&  check_passed.value()) ||
                (!failed_values_of_interest && !check_passed.value()));
    }

    return check_passed.value();
}

/**
*/
FeatureDefinitions SinglePositionBaseCommon::common_getCustomAnnotationDefinitions(const Single& single,
                                                                                   const EvaluationCalculator& calculator) const
{
    FeatureDefinitions defs;

    //histograms
    defs.addDefinition<FeatureDefinitionStringCategoryHistogram>("Position Error", calculator, "Error Count", "Error Count")
        .addDataSeries("", { "#CF", "#CP" }, { numFailed(), numPassed() });
    defs.addDefinition<FeatureDefinitionHistogram<double>>("Position Error", calculator, "Full distribution", "Full distribution [m]")
        .addDataSeries("", SinglePositionBaseCommon::DetailKey::Value);

    return defs;
}

/****************************************************************************
 * SinglePositionProbabilityBase
 ****************************************************************************/

/**
*/
SinglePositionProbabilityBase::SinglePositionProbabilityBase(const std::string& result_type,
                                                             const std::string& result_id, 
                                                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                             const SectorLayer& sector_layer,
                                                             unsigned int utn, 
                                                             const EvaluationTargetData* target, 
                                                             EvaluationCalculator& calculator,
                                                             const EvaluationDetails& details,
                                                             unsigned int num_pos, 
                                                             unsigned int num_no_ref,
                                                             unsigned int num_pos_outside, 
                                                             unsigned int num_pos_inside,
                                                             unsigned int num_ref_inaccurate,
                                                             unsigned int num_passed, 
                                                             unsigned int num_failed)
:   SingleProbabilityBase(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
,   SinglePositionBaseCommon(num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_ref_inaccurate, num_passed, num_failed)
{
}

/**
 * Result value for all probability based position requirement results.
 */
boost::optional<double> SinglePositionProbabilityBase::computeFinalResultValue() const
{
    auto total = num_passed_ + num_failed_;

    if (total == 0)
        return {};

    return (double)num_passed_ / (double)total;
}

/**
*/
boost::optional<double> SinglePositionProbabilityBase::computeResult_impl() const
{
    return common_computeResult(this);
}

/**
*/
unsigned int SinglePositionProbabilityBase::numIssues() const
{
    return common_numIssues();
}

/**
*/
bool SinglePositionProbabilityBase::detailIsOk(const EvaluationDetail& detail) const
{
    return common_detailIsOk(detail, requirement_);
}

/**
*/
void SinglePositionProbabilityBase::addAnnotationForDetail(nlohmann::json& annotations_json, 
                                                           const EvaluationDetail& detail, 
                                                           TargetAnnotationType type,
                                                           bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    if (detail.numPositions() == 1) // no ref pos
        return;

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationDistance(annotations_json, detail, AnnotationArrayType::TypeHighlight, true, false);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        addAnnotationDistance(annotations_json, detail, is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError, true, false);
    }
}

/**
*/
FeatureDefinitions SinglePositionProbabilityBase::getCustomAnnotationDefinitions() const
{
    return common_getCustomAnnotationDefinitions(*this, calculator_);
}

/****************************************************************************
 * SinglePositionValueBase
 ****************************************************************************/

/**
*/
SinglePositionValueBase::SinglePositionValueBase(const std::string& result_type,
                                                 const std::string& result_id, 
                                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                 const SectorLayer& sector_layer,
                                                 unsigned int utn, 
                                                 const EvaluationTargetData* target, 
                                                 EvaluationCalculator& calculator,
                                                 const EvaluationDetails& details,
                                                 unsigned int num_pos, 
                                                 unsigned int num_no_ref,
                                                 unsigned int num_pos_outside, 
                                                 unsigned int num_pos_inside,
                                                 unsigned int num_ref_inaccurate,
                                                 unsigned int num_passed, 
                                                 unsigned int num_failed)
:   Single(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
,   SinglePositionBaseCommon(num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_ref_inaccurate, num_passed, num_failed)
{
}

/**
*/
nlohmann::json SinglePositionValueBase::resultValue(double value) const
{
    //by default reformat result value to 2 decimals
    return formatValue(value);
}

/**
*/
boost::optional<double> SinglePositionValueBase::computeResult_impl() const
{
    return common_computeResult(this);
}

/**
*/
unsigned int SinglePositionValueBase::numIssues() const
{
    return common_numIssues();
}

/**
*/
bool SinglePositionValueBase::detailIsOk(const EvaluationDetail& detail) const
{
    return common_detailIsOk(detail, requirement_);
}

/**
*/
void SinglePositionValueBase::addAnnotationForDetail(nlohmann::json& annotations_json, 
                                                     const EvaluationDetail& detail, 
                                                     TargetAnnotationType type,
                                                     bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    if (detail.numPositions() == 1) // no ref pos
        return;

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationDistance(annotations_json, detail, AnnotationArrayType::TypeHighlight, true, false);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        addAnnotationDistance(annotations_json, detail, is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError, true, false);
    }
}

/**
*/
FeatureDefinitions SinglePositionValueBase::getCustomAnnotationDefinitions() const
{
    return common_getCustomAnnotationDefinitions(*this, calculator_);
}

/****************************************************************************
 * JoinedPositionBase
 ****************************************************************************/

/**
*/
JoinedPositionBase::JoinedPositionBase(const std::string& csv_header)
:   csv_header_(csv_header)
{
}

/**
*/
unsigned int JoinedPositionBase::common_numIssues() const
{
    return num_failed_;
}

/**
*/
unsigned int JoinedPositionBase::common_numUpdates() const
{
    return num_passed_ + num_failed_;
}

/**
*/
void JoinedPositionBase::common_clearResults()
{
    num_pos_         = 0;
    num_no_ref_      = 0;
    num_pos_outside_ = 0;
    num_pos_inside_  = 0;
    num_ref_inaccurate_ = 0;
    num_failed_      = 0;
    num_passed_      = 0;

    accumulator_.reset();
}

/**
*/
void JoinedPositionBase::common_accumulateSingleResult(unsigned int utn, const PositionBase& single_result,
                                                       bool last)
{
    num_pos_         += single_result.numPos();
    num_no_ref_      += single_result.numNoRef();
    num_pos_outside_ += single_result.numPosOutside();
    num_pos_inside_  += single_result.numPosInside();
    num_ref_inaccurate_ += single_result.numRefInaccurate();
    num_passed_      += single_result.numPassed();
    num_failed_      += single_result.numFailed();

    if (single_result.accumulator().max() > 10000)
    {
        loginf << "utn " << utn
               << " dist max " <<  single_result.accumulator().max()
               << " count " << single_result.accumulator().numValues();
    }

    accumulator_.join(single_result.accumulator(), last);
}

/**
*/
boost::optional<double> JoinedPositionBase::common_computeResult() const
{
    traced_assert(num_no_ref_ <= num_pos_);
    traced_assert(num_pos_ - num_no_ref_ == num_pos_inside_ + num_pos_outside_);

    // nothing to do

    return computeFinalResultValue();
}

/**
*/
bool JoinedPositionBase::common_exportAsCSV(std::ofstream& strm,
                                            const Joined* result) const
{
    // traced_assert(result);

    // strm << csv_header_ << "\n";

    // auto values = result->getValues(SinglePositionBaseCommon::DetailKey::Value);

    // for (auto v : values)
    //     strm << v << "\n";

    // if (!strm)
    //     return false;

    // return true;

    return false;
}

FeatureDefinitions JoinedPositionBase::common_getCustomAnnotationDefinitions(const Joined& joined,
                                                                             const EvaluationCalculator& calculator) const
{
    FeatureDefinitions defs;

    //@TODO: remove this hack...
    bool invert = false;
    const EvaluationRequirement::PositionDistance* pos_distance = dynamic_cast<const EvaluationRequirement::PositionDistance*>(joined.requirement().get());
    if (pos_distance) 
    {
        if (pos_distance->thresholdValueCheckType() == EvaluationRequirement::COMPARISON_TYPE::GREATER_THAN || 
            pos_distance->thresholdValueCheckType() == EvaluationRequirement::COMPARISON_TYPE::GREATER_THAN_OR_EQUAL)
            invert = true;
    }

    //grids (as geoimages)
    defs.addDefinition<FeatureDefinitionBinaryGrid>("Position Error", calculator, "Comparison Passed")
        .addDataSeries(SinglePositionBaseCommon::DetailKey::CheckPassed, 
                       GridAddDetailMode::AddEvtRefPosition, 
                       invert);
    
    // defs.addDefinition<FeatureDefinitionGrid<double>>("Position Error", calculator, "Error Mean", true)
    //     .addDataSeries(SinglePositionBaseCommon::DetailKey::Value, 
    //                    grid2d::ValueType::ValueTypeMean, 
    //                    GridAddDetailMode::AddEvtRefPosition);
    // defs.addDefinition<FeatureDefinitionGrid<double>>("Position Error", calculator, "Error Stddev", true)
    //     .addDataSeries(SinglePositionBaseCommon::DetailKey::Value, 
    //                    grid2d::ValueType::ValueTypeStddev, 
    //                    GridAddDetailMode::AddEvtRefPosition);
    // defs.addDefinition<FeatureDefinitionGrid<double>>("Position Error", calculator, "Error Max", true)
    //     .addDataSeries(SinglePositionBaseCommon::DetailKey::Value, 
    //                    grid2d::ValueType::ValueTypeMax, 
    //                    GridAddDetailMode::AddEvtRefPosition);

    //histograms
    defs.addDefinition<FeatureDefinitionStringCategoryHistogram>("Position Error", calculator, "Error Count", "Error Count")
        .addDataSeries("", { "#CF", "#CP" }, { numFailed(), numPassed() });
    defs.addDefinition<FeatureDefinitionHistogram<double>>("Position Error", calculator, "Full distribution", "Full distribution [m]")
        .addDataSeries("", SinglePositionBaseCommon::DetailKey::Value);

    //scatterplot
    // defs.addDefinition<FeatureDefinitionCustomScatterSeries>("My Scatter Series", calculator, "Just Some Points", "X", "Y")
    //     .addDataSeries("pointset1", { {0,0}, {1,1}, {2,2}, {3,3} }, Qt::red)
    //     .addDataSeries("pointset2", { {1,0}, {2,1}, {3,2}, {4,3} }, Qt::green)
    //     .addDataSeries("pointset3", { {2,0}, {3,1}, {4,2}, {5,3} }, Qt::blue);
    // defs.addDefinition<FeatureDefinitionTimedScatterSeries>("Position Error", calculator, "Development over Time", "Error [m]")
    //     .addDataSeries("", SinglePositionBaseCommon::DetailKey::Value);

    //grids (as raw grids)
    // defs.addDefinition<FeatureDefinitionGrid<double>>("Position Error", calculator, "Error Mean", false)
    //     .addDataSeries(SinglePositionBaseCommon::DetailKey::Value, 
    //                    grid2d::ValueType::ValueTypeMean, 
    //                    GridAddDetailMode::AddEvtRefPosition);

    return defs;
}

/****************************************************************************
 * JoinedPositionProbabilityBase
 ****************************************************************************/

/**
*/
JoinedPositionProbabilityBase::JoinedPositionProbabilityBase(const std::string& result_type,
                                                             const std::string& result_id, 
                                                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                             const SectorLayer& sector_layer, 
                                                             EvaluationCalculator& calculator,
                                                             const std::string& csv_header)
:   JoinedPositionBase(csv_header)
,   JoinedProbabilityBase(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
 * Result value for all probability based position requirement results.
 */
boost::optional<double> JoinedPositionProbabilityBase::computeFinalResultValue() const
{
    auto total = num_passed_ + num_failed_;

    if (total == 0)
        return {};

    return (double)num_passed_ / (double)total;
}

/**
*/
unsigned int JoinedPositionProbabilityBase::numIssues() const
{
    return common_numIssues();
}

/**
*/
unsigned int JoinedPositionProbabilityBase::numUpdates() const
{
    return common_numUpdates();
}

/**
*/
void JoinedPositionProbabilityBase::clearResults_impl() 
{
    common_clearResults();
}

/**
*/
void JoinedPositionProbabilityBase::accumulateSingleResult(
    const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SinglePositionProbabilityBase> single = std::static_pointer_cast<SinglePositionProbabilityBase>(single_result);

    common_accumulateSingleResult(single->utn(), *single, last);
}

/**
*/
boost::optional<double> JoinedPositionProbabilityBase::computeResult_impl() const
{
    loginf << "start" << type()
            << " num_pos " << num_pos_
            << " num_no_ref " << num_no_ref_
            << " num_ref_inaccurate " << num_ref_inaccurate_
            << " num_failed " << num_failed_
            << " num_passed " << num_passed_;

    return common_computeResult();
}

/**
*/
bool JoinedPositionProbabilityBase::exportAsCSV(std::ofstream& strm) const
{
    return common_exportAsCSV(strm, this);
}

/**
*/
FeatureDefinitions JoinedPositionProbabilityBase::getCustomAnnotationDefinitions() const
{
    return common_getCustomAnnotationDefinitions(*this, calculator_);
}

/****************************************************************************
 * JoinedPositionValueBase
 ****************************************************************************/

/**
*/
JoinedPositionValueBase::JoinedPositionValueBase(const std::string& result_type,
                                                 const std::string& result_id, 
                                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                 const SectorLayer& sector_layer, 
                                                 EvaluationCalculator& calculator,
                                                 const std::string& csv_header)
:   JoinedPositionBase(csv_header)
,   Joined(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
nlohmann::json JoinedPositionValueBase::resultValue(double value) const
{
    //by default reformat result value to 2 decimals
    return formatValue(value);
}

/**
*/
unsigned int JoinedPositionValueBase::numIssues() const
{
    return common_numIssues();
}

/**
*/
unsigned int JoinedPositionValueBase::numUpdates() const
{
    return common_numUpdates();
}

/**
*/
void JoinedPositionValueBase::clearResults_impl() 
{
    common_clearResults();
}

/**
*/
void JoinedPositionValueBase::accumulateSingleResult(
    const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SinglePositionValueBase> single = std::static_pointer_cast<SinglePositionValueBase>(single_result);

    common_accumulateSingleResult(single->utn(), *single, last);
}

/**
*/
boost::optional<double> JoinedPositionValueBase::computeResult_impl() const
{
    loginf << "start" << type()
            << " num_pos " << num_pos_
            << " num_no_ref " << num_no_ref_
            << " num_ref_inaccurate " << num_ref_inaccurate_
            << " num_failed " << num_failed_
            << " num_passed " << num_passed_;

    return common_computeResult();
}

/**
*/
bool JoinedPositionValueBase::exportAsCSV(std::ofstream& strm) const
{
    return common_exportAsCSV(strm, this);
}

/**
*/
FeatureDefinitions JoinedPositionValueBase::getCustomAnnotationDefinitions() const
{
    return common_getCustomAnnotationDefinitions(*this, calculator_);
}

}
