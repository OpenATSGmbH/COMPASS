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

#include "eval/results/identification/change_delay.h"
#include "eval/results/evaluationdetail.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/identification/change_delay.h"

#include "evaluationtargetdata.h"
#include "evaluationmanager.h"

#include "logger.h"
#include "stringconv.h"

#include "traced_assert.h"

using namespace std;
using namespace Utils;
using namespace nlohmann;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * IdentificationChangeDelayBase
 **********************************************************************************************/

/**
*/
IdentificationChangeDelayBase::IdentificationChangeDelayBase(unsigned int num_events,
                                                             unsigned int num_not_assessable,
                                                             unsigned int num_passed,
                                                             unsigned int num_failed)
:   num_events_         (num_events)
,   num_not_assessable_ (num_not_assessable)
,   num_passed_         (num_passed)
,   num_failed_         (num_failed)
{
}

/**********************************************************************************************
 * SingleIdentificationChangeDelay
 **********************************************************************************************/

/**
*/
SingleIdentificationChangeDelay::SingleIdentificationChangeDelay(
        const std::string& result_id,
        std::shared_ptr<EvaluationRequirement::Base> requirement,
        const SectorLayer& sector_layer,
        unsigned int utn,
        const EvaluationTargetData* target,
        EvaluationCalculator& calculator,
        const EvaluationDetails& details,
        unsigned int num_events,
        unsigned int num_not_assessable,
        unsigned int num_passed,
        unsigned int num_failed,
        const std::vector<double>& delays)
:   IdentificationChangeDelayBase(num_events, num_not_assessable, num_passed, num_failed)
,   SingleProbabilityBase("SingleIdentificationChangeDelay", result_id, requirement, sector_layer, utn, target, calculator, details)
{
    accumulator_.accumulate(delays, true);

    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleIdentificationChangeDelay::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedIdentificationChangeDelay>(result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
boost::optional<double> SingleIdentificationChangeDelay::computeResult_impl() const
{
    unsigned int num_checked = num_passed_ + num_failed_;

    if (!num_checked)
        return {};

    return (double)num_passed_ / (double)num_checked;
}

/**
*/
unsigned int SingleIdentificationChangeDelay::numIssues() const
{
    return num_failed_;
}

/**
*/
std::vector<std::string> SingleIdentificationChangeDelay::targetTableHeadersCustom() const
{
    return { "#Events", "CDMin", "CDMax", "CDAvg", "#CDOK", "#CDNOK" };
}

/**
*/
nlohmann::json::array_t SingleIdentificationChangeDelay::targetTableValuesCustom() const
{
    return { num_events_,
             formatValue(accumulator_.min()),
             formatValue(accumulator_.max()),
             formatValue(accumulator_.mean()),
             num_passed_,
             num_failed_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleIdentificationChangeDelay::targetInfos() const
{
    return { { "#Events [1]"  , "Number of identity change events"           , num_events_         },
             { "#NotAssessable [1]", "Number of events without sufficient data", num_not_assessable_ },
             { "CDMin [s]"    , "Minimum of change delay"                    , formatValue(accumulator_.min())    },
             { "CDMax [s]"    , "Maximum of change delay"                    , formatValue(accumulator_.max())    },
             { "CDAvg [s]"    , "Average of change delay"                    , formatValue(accumulator_.mean())   },
             { "CDSDev [s]"   , "Standard Deviation of change delay"         , formatValue(accumulator_.stddev()) },
             { "#CDOK [1]"    , "Number of events with acceptable delay"     , num_passed_         },
             { "#CDNOK [1]"   , "Number of events with unacceptable delay"   , num_failed_         } };
}

/**
*/
std::vector<std::string> SingleIdentificationChangeDelay::detailHeaders() const
{
    return { "ToD", "Delay", "DelayOK", "#CDOK", "#CDNOK", "Comment" };
}

/**
*/
nlohmann::json::array_t SingleIdentificationChangeDelay::detailValues(const EvaluationDetail& detail,
                                                                      const EvaluationDetail* parent_detail) const
{
    auto value = detail.getValue(DetailKey::Value);

    return { Utils::Time::toString(detail.timestamp()),
             value.isValid() ? nlohmann::json(value.toFloat()) : nlohmann::json(),
             detail.getValue(DetailKey::CheckPassed).toBool(),
             detail.getValue(DetailKey::NumCheckPassed).toUInt(),
             detail.getValue(DetailKey::NumCheckFailed).toUInt(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleIdentificationChangeDelay::detailIsOk(const EvaluationDetail& detail) const
{
    auto check_passed = detail.getValue(DetailKey::CheckPassed);

    return check_passed.isValid() && check_passed.toBool();
}

/**
*/
void SingleIdentificationChangeDelay::addAnnotationForDetail(nlohmann::json& annotations_json,
                                                             const EvaluationDetail& detail,
                                                             TargetAnnotationType type,
                                                             bool is_ok) const
{
    traced_assert(detail.numPositions() >= 1);

    if (type == TargetAnnotationType::Highlight)
    {
        addAnnotationPos(annotations_json, detail.firstPos(), AnnotationArrayType::TypeHighlight);
    }
    else if (type == TargetAnnotationType::TargetOverview)
    {
        addAnnotationPos(annotations_json, detail.firstPos(),
                         is_ok ? AnnotationArrayType::TypeOk : AnnotationArrayType::TypeError);
    }
}

/**********************************************************************************************
 * JoinedIdentificationChangeDelay
 **********************************************************************************************/

/**
*/
JoinedIdentificationChangeDelay::JoinedIdentificationChangeDelay(
        const std::string& result_id,
        std::shared_ptr<EvaluationRequirement::Base> requirement,
        const SectorLayer& sector_layer,
        EvaluationCalculator& calculator)
:   IdentificationChangeDelayBase()
,   JoinedProbabilityBase("JoinedIdentificationChangeDelay", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedIdentificationChangeDelay::numIssues() const
{
    return num_failed_;
}

/**
*/
unsigned int JoinedIdentificationChangeDelay::numUpdates() const
{
    return num_passed_ + num_failed_;
}

/**
*/
void JoinedIdentificationChangeDelay::clearResults_impl()
{
    num_events_         = 0;
    num_not_assessable_ = 0;
    num_passed_         = 0;
    num_failed_         = 0;

    accumulator_.reset();
}

/**
*/
void JoinedIdentificationChangeDelay::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleIdentificationChangeDelay> single =
            std::static_pointer_cast<SingleIdentificationChangeDelay>(single_result);

    assert (single->resultUsable());

    num_events_         += single->numEvents();
    num_not_assessable_ += single->numNotAssessable();
    num_passed_         += single->numPassed();
    num_failed_         += single->numFailed();

    accumulator_.join(single->accumulator(), last);
}

/**
*/
boost::optional<double> JoinedIdentificationChangeDelay::computeResult_impl() const
{
    loginf << "start"
           << " num_passed " << num_passed_
           << " num_failed " << num_failed_;

    unsigned int num_checked = num_passed_ + num_failed_;

    if (!num_checked)
        return {};

    return (double)num_passed_ / (double)num_checked;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedIdentificationChangeDelay::sectorInfos() const
{
    return { { "#Events [1]"  , "Number of identity change events"           , num_events_         },
             { "#NotAssessable [1]", "Number of events without sufficient data", num_not_assessable_ },
             { "CDMin [s]"    , "Minimum of change delay"                    , formatValue(accumulator_.min())    },
             { "CDMax [s]"    , "Maximum of change delay"                    , formatValue(accumulator_.max())    },
             { "CDAvg [s]"    , "Average of change delay"                    , formatValue(accumulator_.mean())   },
             { "CDSDev [s]"   , "Standard Deviation of change delay"         , formatValue(accumulator_.stddev()) },
             { "#CDOK [1]"    , "Number of events with acceptable delay"     , num_passed_         },
             { "#CDNOK [1]"   , "Number of events with unacceptable delay"   , num_failed_         } };
}

}
