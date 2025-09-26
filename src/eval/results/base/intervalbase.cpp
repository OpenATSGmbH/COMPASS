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

#include "eval/results/base/intervalbase.h"
#include "eval/results/base/featuredefinitions.h"

#include "eval/results/evaluationdetail.h"

#include "eval/results/report/rootitem.h"
#include "eval/results/report/section.h"
#include "eval/results/report/sectioncontenttext.h"
#include "eval/results/report/sectioncontenttable.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/base/intervalbase.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "stringconv.h"
#include "viewpoint.h"
#include "sectorlayer.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/***********************************************************************************************
 * SingleIntervalBase
 ***********************************************************************************************/

/**
*/
SingleIntervalBase::SingleIntervalBase(const std::string& result_type, 
                                       const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer,
                                       unsigned int utn,
                                       const EvaluationTargetData* target,
                                       EvaluationCalculator& calculator,
                                       const EvaluationDetails& details,
                                       int sum_uis,
                                       int missed_uis,
                                       TimePeriodCollection ref_periods)
:   IntervalBase         (sum_uis, missed_uis)
,   SingleProbabilityBase(result_type, result_id, requirement, sector_layer, utn, target, calculator, details)
,   ref_periods_         (ref_periods)
{
}

/**
*/
boost::optional<double> SingleIntervalBase::computeResult_impl() const
{
    traced_assert(missed_uis_ <= sum_uis_);

    if (sum_uis_ == 0)
        return {};

    logdbg << type() << ": utn " << utn_ << " missed_uis " << missed_uis_ << " sum_uis " << sum_uis_;

    return (1.0 - (double)missed_uis_ / (double)sum_uis_);
}

/**
*/
unsigned int SingleIntervalBase::numIssues() const
{
    return missed_uis_;
}

/**
*/
std::vector<std::string> SingleIntervalBase::targetTableHeadersCustom() const
{
    return { "#EUIs", "#MUIs" };
}

/**
*/
nlohmann::json::array_t SingleIntervalBase::targetTableValuesCustom() const
{
    return { sum_uis_, missed_uis_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleIntervalBase::targetInfos() const
{
    std::vector<Single::TargetInfo> infos = { { "#EUIs [1]", "Expected Update Intervals", sum_uis_   },
                                              { "#MUIs [1]", "Missed Update Intervals"  , missed_uis_} };
    size_t inside_sectors = 0;

    for (unsigned int cnt=0; cnt < ref_periods_.size(); ++cnt)
    {
        //skip outside sectors
        if (ref_periods_.period(cnt).type() == TimePeriod::Type::OutsideSector)
            continue;

        ++inside_sectors;

        infos.emplace_back(("Reference Period " + std::to_string(cnt)), "Time inside sector", ref_periods_.period(cnt).str());
    }

    if (inside_sectors == 0)
        infos.emplace_back("Reference Period", "Time inside sector", "None");

    return infos;
}

/**
*/
std::vector<std::string> SingleIntervalBase::detailHeaders() const
{
    return {"ToD", "DToD", "#MUIs", "Comment"};
}

/**
*/
nlohmann::json::array_t SingleIntervalBase::detailValues(const EvaluationDetail& detail,
                                                         const EvaluationDetail* parent_detail) const
{
    auto d_tod     = detail.getValue(DetailKey::DiffTOD);
    auto d_tod_str = d_tod.isValid() ? nlohmann::json(String::timeStringFromDouble(d_tod.toFloat())) : nlohmann::json();

    return { Utils::Time::toString(detail.timestamp()),
             d_tod_str,
             detail.getValue(DetailKey::MissedUIs).toUInt(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleIntervalBase::detailIsOk(const EvaluationDetail& detail) const
{
    auto check_failed = detail.getValueAsOrAssert<bool>(EvaluationRequirementResult::SingleIntervalBase::DetailKey::MissOccurred);
    return !check_failed;
}

/**
*/
void SingleIntervalBase::addAnnotationForDetail(nlohmann::json& annotations_json, 
                                           const EvaluationDetail& detail, 
                                           TargetAnnotationType type,
                                           bool is_ok) const
{
    if (detail.numPositions() == 1)
        return;

    traced_assert(detail.numPositions() >= 2);

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

/***********************************************************************************************
 * JoinedIntervalBase
 ***********************************************************************************************/

/**
*/
JoinedIntervalBase::JoinedIntervalBase(const std::string& result_type, 
                                       const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer, 
                                       EvaluationCalculator& calculator)
:   JoinedProbabilityBase(result_type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedIntervalBase::numIssues() const
{
    return missed_uis_;
}

/**
*/
unsigned int JoinedIntervalBase::numUpdates() const
{
    return sum_uis_;
}

/**
*/
void JoinedIntervalBase::clearResults_impl() 
{
    sum_uis_    = 0;
    missed_uis_ = 0;
}

/**
*/
void JoinedIntervalBase::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleIntervalBase> single_interval = std::static_pointer_cast<SingleIntervalBase>(single_result);

    sum_uis_    += single_interval->sumUIs();
    missed_uis_ += single_interval->missedUIs();
}

/**
*/
boost::optional<double> JoinedIntervalBase::computeResult_impl() const
{
    loginf << "start" << type()
            << " sum_uis_ " << sum_uis_
            << " missed_uis_ " << missed_uis_;

    traced_assert(missed_uis_ <= sum_uis_);

    if (sum_uis_ == 0)
        return {};

    return 1.0 - (double)missed_uis_ / (double)(sum_uis_);
}

/**
*/
std::vector<Joined::SectorInfo> JoinedIntervalBase::sectorInfos() const
{
    return { { "#Updates/#EUIs [1]", "Total number update intervals"     , sum_uis_    },
             { "#MUIs [1]"         , "Number of missed update intervals" , missed_uis_ } };
}

/**
*/
FeatureDefinitions JoinedIntervalBase::getCustomAnnotationDefinitions() const
{
    FeatureDefinitions defs;

    defs.addDefinition<FeatureDefinitionBinaryGrid>(requirement()->name(), calculator_, "Passed")
        .addDataSeries(SingleIntervalBase::DetailKey::MissOccurred, 
                       GridAddDetailMode::AddPositionsAsPolyLine, 
                       true);
    return defs;
}

}
