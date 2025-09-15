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

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/detection/detection.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "stringconv.h"
#include "viewpoint.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * DetectionBase
 **********************************************************************************************/

/**
*/
DetectionBase::DetectionBase() = default;

/**
*/
DetectionBase::DetectionBase(int sum_uis, 
                             int missed_uis)
:   sum_uis_    (sum_uis)
,   missed_uis_ (missed_uis)
{
}

/**
*/
unsigned int DetectionBase::sumUIs() const
{
    return sum_uis_;
}

/**
*/
unsigned int DetectionBase::missedUIs() const
{
    return missed_uis_;
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
                                 int sum_uis,
                                 int missed_uis,
                                 TimePeriodCollection ref_periods)
:   DetectionBase(sum_uis, missed_uis)
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
    if (!sum_uis_)
        return {};

    logdbg << "utn " << utn_ << " missed_uis " << missed_uis_ << " sum_uis " << sum_uis_;

    traced_assert(missed_uis_ <= sum_uis_);

    std::shared_ptr<EvaluationRequirement::Detection> req =
            std::static_pointer_cast<EvaluationRequirement::Detection>(requirement_);
    traced_assert(req);

    return (1.0 - ((double)missed_uis_/(double)(sum_uis_)));
}

/**
*/
unsigned int SingleDetection::numIssues() const
{
    return missed_uis_;
}

/**
*/
std::vector<std::string> SingleDetection::targetTableHeadersCustom() const
{
    return { "#EUIs", "#MUIs" };
}

/**
*/
nlohmann::json::array_t SingleDetection::targetTableValuesCustom() const
{
    return { sum_uis_, missed_uis_ };
}

std::string SingleDetection::targetTableCustomSortColumn() const
{
    return "#MUIs";
}

 Qt::SortOrder SingleDetection::targetTableSortOrder() const
 {
    return  Qt::SortOrder::DescendingOrder;
 }

/**
*/
std::vector<Single::TargetInfo> SingleDetection::targetInfos() const
{
    std::shared_ptr<EvaluationRequirement::Detection> req = std::static_pointer_cast<EvaluationRequirement::Detection>(requirement_);
    traced_assert(req);

    std::vector<TargetInfo> infos = { TargetInfo("#EUIs [1]", "Expected Update Intervals", sum_uis_   ),
                                      TargetInfo("#MUIs [1]", "Missed Update Intervals"  , missed_uis_) };

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
    return { "ToD", "DToD", "MUI", "Comment" };
}

/**
*/
nlohmann::json::array_t SingleDetection::detailValues(const EvaluationDetail& detail,
                                                      const EvaluationDetail* parent_detail) const
{
    auto d_tod = detail.getValue(DetailKey::DiffTOD);

    return { Utils::Time::toString(detail.timestamp()),
             d_tod.isValid() ? nlohmann::json(Utils::String::timeStringFromDouble(d_tod.toFloat())) : nlohmann::json(),
             detail.getValue(DetailKey::MissedUIs).toUInt(),
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
    return missed_uis_;
}

/**
*/
unsigned int JoinedDetection::numUpdates() const
{
    return sum_uis_;
}

/**
*/
void JoinedDetection::clearResults_impl() 
{
    missed_uis_ = 0;
    sum_uis_    = 0;
}

/**
*/
void JoinedDetection::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleDetection> single = std::static_pointer_cast<SingleDetection>(single_result);

    assert (single->resultUsable());

    missed_uis_ += single->missedUIs();
    sum_uis_    += single->sumUIs();
}

/**
*/
boost::optional<double> JoinedDetection::computeResult_impl() const
{
    loginf << "start"
            << " missed_uis " << missed_uis_
            << " sum_uis " << sum_uis_;

    traced_assert(missed_uis_ <= sum_uis_);

    if (sum_uis_ == 0)
        return {};

    return 1.0 - (double)missed_uis_ / (double)(sum_uis_);
}

/**
*/
std::vector<Joined::SectorInfo> JoinedDetection::sectorInfos() const
{
    return { { "#EUIs [1]", "Expected Update Intervals", sum_uis_    },
             { "#MUIs [1]", "Missed Update Intervals"  , missed_uis_ } };
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
