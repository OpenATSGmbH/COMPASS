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

#include "eval/results/detection/detection.h"
#include "eval/results/evaluationdetail.h"
#include "eval/results/base/featuredefinitions.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/detection/detection.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "stringconv.h"
#include "viewpoint.h"

#include "traced_assert.h"

#include <cmath>

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

namespace
{
    // true if the requirement runs in the time-ratio calculation mode
    // (ED-129C Appendix C "Interarrivaltime" method)
    bool isTimeRatio(const std::shared_ptr<EvaluationRequirement::Base>& requirement)
    {
        auto req = std::dynamic_pointer_cast<EvaluationRequirement::Detection>(requirement);
        return req && req->useTimeRatio();
    }

    // table/json representation: whole update interval counts in counting mode,
    // seconds rounded to millisecond precision in time-ratio mode
    nlohmann::json expectedMissedValue(double value, bool time_ratio)
    {
        if (time_ratio)
            return std::round(value * 1000.0) / 1000.0;
        return (unsigned int)(value + 0.5);
    }
}

/**********************************************************************************************
 * DetectionBase
 **********************************************************************************************/

/**
*/
DetectionBase::DetectionBase() = default;

/**
*/
DetectionBase::DetectionBase(double sum_expected,
                             double sum_missed)
:   sum_expected_ (sum_expected)
,   sum_missed_   (sum_missed)
{
}

/**
*/
double DetectionBase::sumExpected() const
{
    return sum_expected_;
}

/**
*/
double DetectionBase::sumMissed() const
{
    return sum_missed_;
}

/**********************************************************************************************
 * SingleDetection
 **********************************************************************************************/

/**
*/
SingleDetection::SingleDetection(const std::string& result_id, 
                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                 const SectorLayer& sector_layer,
                                 unsigned int utn,
                                 const EvaluationTargetData* target,
                                 EvaluationCalculator& calculator,
                                 const EvaluationDetails& details,
                                 double sum_expected,
                                 double sum_missed,
                                 TimePeriodCollection ref_periods)
:   DetectionBase(sum_expected, sum_missed)
,   SingleProbabilityBase("SingleDetection", result_id, requirement, sector_layer, utn, target, calculator, details)
,   ref_periods_(ref_periods)
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleDetection::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedDetection>(result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
boost::optional<double> SingleDetection::computeResult_impl() const
{
    if (sum_expected_ <= 0.0)
        return {};

    logdbg << "utn " << utn_ << " sum_missed " << sum_missed_ << " sum_expected " << sum_expected_;

    traced_assert(sum_missed_ <= sum_expected_ + 1e-06);

    std::shared_ptr<EvaluationRequirement::Detection> req =
            std::static_pointer_cast<EvaluationRequirement::Detection>(requirement_);
    traced_assert(req);

    return (1.0 - (sum_missed_/sum_expected_));
}

/**
*/
unsigned int SingleDetection::numIssues() const
{
    return (unsigned int)(sum_missed_ + 0.5);
}

/**
*/
std::vector<std::string> SingleDetection::targetTableHeadersCustom() const
{
    if (isTimeRatio(requirement_))
        return { "DT [s]", "MT [s]" };

    return { "#EUIs", "#MUIs" };
}

/**
*/
std::string SingleDetection::targetTableCustomSortColumn() const
{
    return isTimeRatio(requirement_) ? "MT [s]" : "#MUIs";
}

/**
*/
nlohmann::json::array_t SingleDetection::targetTableValuesCustom() const
{
    bool time_ratio = isTimeRatio(requirement_);

    return { expectedMissedValue(sum_expected_, time_ratio),
             expectedMissedValue(sum_missed_  , time_ratio) };
}

/**
*/
std::vector<Single::TargetInfo> SingleDetection::targetInfos() const
{
    std::shared_ptr<EvaluationRequirement::Detection> req = std::static_pointer_cast<EvaluationRequirement::Detection>(requirement_);
    traced_assert(req);

    bool time_ratio = isTimeRatio(requirement_);

    std::vector<TargetInfo> infos;

    if (time_ratio)
        infos = { TargetInfo("DT [s]", "Reference Duration", expectedMissedValue(sum_expected_, time_ratio)),
                  TargetInfo("MT [s]", "Missed Time"       , expectedMissedValue(sum_missed_  , time_ratio)) };
    else
        infos = { TargetInfo("#EUIs [1]", "Expected Update Intervals", expectedMissedValue(sum_expected_, time_ratio)),
                  TargetInfo("#MUIs [1]", "Missed Update Intervals"  , expectedMissedValue(sum_missed_  , time_ratio)) };

    for (unsigned int cnt=0; cnt < ref_periods_.size(); ++cnt)
        infos.emplace_back(("Reference Period " + std::to_string(cnt)), "Time inside sector", ref_periods_.period(cnt).str());

    if (!ref_periods_.size())
        infos.emplace_back("Reference Period", "Time inside sector", "None");

    return infos;
}

/**
*/
std::vector<std::string> SingleDetection::detailHeaders() const
{
    if (isTimeRatio(requirement_))
        return { "ToD", "DToD", "MT [s]", "Comment" };

    return { "ToD", "DToD", "MUI", "Comment" };
}

/**
*/
nlohmann::json::array_t SingleDetection::detailValues(const EvaluationDetail& detail,
                                                      const EvaluationDetail* parent_detail) const
{
    auto d_tod = detail.getValue(DetailKey::DiffTOD);

    bool time_ratio = isTimeRatio(requirement_);

    // cumulative missed amount: update interval count or seconds, see evaluate()
    auto missed = detail.getValue(DetailKey::MissedUIs);

    return { Utils::Time::toString(detail.timestamp()),
             d_tod.isValid() ? nlohmann::json(Utils::String::timeStringFromDouble(d_tod.toFloat())) : nlohmann::json(),
             time_ratio ? expectedMissedValue(missed.toDouble(), true) : nlohmann::json(missed.toUInt()),
             detail.comments().generalComment() };
}

/**
*/
bool SingleDetection::detailIsOk(const EvaluationDetail& detail) const
{
    return !detail.getValue(DetailKey::MissOccurred).toBool();
}

/**
*/
void SingleDetection::addAnnotationForDetail(nlohmann::json& annotations_json, 
                                             const EvaluationDetail& detail, 
                                             TargetAnnotationType type,
                                             bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    auto anno_type = is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError;

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationPos(annotations_json, detail.firstPos(), AnnotationArrayType::TypeHighlight);
        addAnnotationPos(annotations_json, detail.lastPos() , AnnotationArrayType::TypeHighlight);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        for (const auto& pos : detail.positions())
            addAnnotationPos(annotations_json, pos, anno_type);
    }
}

/**********************************************************************************************
 * JoinedDetection
 **********************************************************************************************/

/**
*/
JoinedDetection::JoinedDetection(const std::string& result_id, 
                                 std::shared_ptr<EvaluationRequirement::Base> requirement,
                                 const SectorLayer& sector_layer, 
                                 EvaluationCalculator& calculator)
:   DetectionBase()
,   JoinedProbabilityBase("JoinedDetection", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedDetection::numIssues() const
{
    return (unsigned int)(sum_missed_ + 0.5);
}

/**
*/
unsigned int JoinedDetection::numUpdates() const
{
    return (unsigned int)(sum_expected_ + 0.5);
}

/**
*/
void JoinedDetection::clearResults_impl()
{
    sum_missed_   = 0;
    sum_expected_ = 0;
}

/**
*/
void JoinedDetection::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleDetection> single = std::static_pointer_cast<SingleDetection>(single_result);

    assert (single->resultUsable());

    sum_missed_   += single->sumMissed();
    sum_expected_ += single->sumExpected();
}

/**
*/
boost::optional<double> JoinedDetection::computeResult_impl() const
{
    loginf << "start"
            << " sum_missed " << sum_missed_
            << " sum_expected " << sum_expected_;

    traced_assert(sum_missed_ <= sum_expected_ + 1e-06);

    if (sum_expected_ <= 0.0)
        return {};

    return 1.0 - sum_missed_ / sum_expected_;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedDetection::sectorInfos() const
{
    bool time_ratio = isTimeRatio(requirement_);

    if (time_ratio)
        return { { "DT [s]", "Reference Duration", expectedMissedValue(sum_expected_, time_ratio) },
                 { "MT [s]", "Missed Time"       , expectedMissedValue(sum_missed_  , time_ratio) } };

    return { { "#EUIs [1]", "Expected Update Intervals", expectedMissedValue(sum_expected_, time_ratio) },
             { "#MUIs [1]", "Missed Update Intervals"  , expectedMissedValue(sum_missed_  , time_ratio) } };
}

/**
*/
FeatureDefinitions JoinedDetection::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>("Missed Update Intervals", calculator_, "Miss Occurred").
            addDataSeries(SingleDetection::DetailKey::MissOccurred, GridAddDetailMode::AddPositionsAsPolyLine, true);

    return defs;
}

}
