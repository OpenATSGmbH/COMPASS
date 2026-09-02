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

#include "eval/results/latency/adsblatency.h"
#include "eval/results/evaluationdetail.h"

#include "eval/requirement/base/base.h"
#include "eval/requirement/latency/adsblatency.h"

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
 * ADSBLatencyBase
 **********************************************************************************************/

/**
*/
ADSBLatencyBase::ADSBLatencyBase(unsigned int num_pos,
                                 unsigned int num_no_data,
                                 unsigned int num_pos_outside,
                                 unsigned int num_pos_inside,
                                 unsigned int num_value_ok,
                                 unsigned int num_value_nok)
:   num_pos_         (num_pos)
,   num_no_data_     (num_no_data)
,   num_pos_outside_ (num_pos_outside)
,   num_pos_inside_  (num_pos_inside)
,   num_value_ok_    (num_value_ok)
,   num_value_nok_   (num_value_nok)
{
}

/**********************************************************************************************
 * SingleADSBLatency
 **********************************************************************************************/

/**
*/
SingleADSBLatency::SingleADSBLatency(const std::string& result_id,
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer,
                                     unsigned int utn,
                                     const EvaluationTargetData* target,
                                     EvaluationCalculator& calculator,
                                     const EvaluationDetails& details,
                                     unsigned int num_pos,
                                     unsigned int num_no_data,
                                     unsigned int num_pos_outside,
                                     unsigned int num_pos_inside,
                                     unsigned int num_value_ok,
                                     unsigned int num_value_nok,
                                     const std::vector<double>& latencies)
:   ADSBLatencyBase(num_pos, num_no_data, num_pos_outside, num_pos_inside, num_value_ok, num_value_nok)
,   SingleProbabilityBase("SingleADSBLatency", result_id, requirement, sector_layer, utn, target, calculator, details)
{
    accumulator_.accumulate(latencies, true);

    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleADSBLatency::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedADSBLatency>(result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
boost::optional<double> SingleADSBLatency::computeResult_impl() const
{
    unsigned int num_values = num_value_ok_ + num_value_nok_;

    if (!num_values)
        return {};

    return (double)num_value_ok_ / (double)num_values;
}

/**
*/
unsigned int SingleADSBLatency::numIssues() const
{
    return num_value_nok_;
}

/**
*/
std::vector<std::string> SingleADSBLatency::targetTableHeadersCustom() const
{
    return { "LTMin", "LTMax", "LTAvg", "LTSDev", "#LTOK", "#LTNOK" };
}

/**
*/
nlohmann::json::array_t SingleADSBLatency::targetTableValuesCustom() const
{
    return { formatValue(accumulator_.min()),
             formatValue(accumulator_.max()),
             formatValue(accumulator_.mean()),
             formatValue(accumulator_.stddev()),
             num_value_ok_,
             num_value_nok_ };
}

/**
*/
std::vector<Single::TargetInfo> SingleADSBLatency::targetInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                          , num_pos_         },
             { "#NoData [1]"    , "Number of updates w/o TOMR and/or TORT"     , num_no_data_     },
             { "#PosOutside [1]", "Number of updates outside sector"           , num_pos_outside_ },
             { "#PosInside [1]" , "Number of updates inside sector"            , num_pos_inside_  },
             { "LTMin [s]"      , "Minimum of latency"                         , formatValue(accumulator_.min())    },
             { "LTMax [s]"      , "Maximum of latency"                         , formatValue(accumulator_.max())    },
             { "LTAvg [s]"      , "Average of latency"                         , formatValue(accumulator_.mean())   },
             { "LTSDev [s]"     , "Standard Deviation of latency"              , formatValue(accumulator_.stddev()) },
             { "#LTOK [1]"      , "Number of updates with acceptable latency"  , num_value_ok_    },
             { "#LTNOK [1]"     , "Number of updates with unacceptable latency", num_value_nok_   } };
}

/**
*/
std::vector<std::string> SingleADSBLatency::detailHeaders() const
{
    return { "ToD", "PosInside", "Latency", "LatencyOK", "#LTOK", "#LTNOK", "Comment" };
}

/**
*/
nlohmann::json::array_t SingleADSBLatency::detailValues(const EvaluationDetail& detail,
                                                        const EvaluationDetail* parent_detail) const
{
    auto value = detail.getValue(DetailKey::Value);

    return { Utils::Time::toString(detail.timestamp()),
             detail.getValue(DetailKey::PosInside).toBool(),
             value.isValid() ? nlohmann::json(value.toFloat()) : nlohmann::json(),
             detail.getValue(DetailKey::CheckPassed).toBool(),
             detail.getValue(DetailKey::NumCheckPassed).toUInt(),
             detail.getValue(DetailKey::NumCheckFailed).toUInt(),
             detail.comments().generalComment() };
}

/**
*/
bool SingleADSBLatency::detailIsOk(const EvaluationDetail& detail) const
{
    auto check_passed = detail.getValue(DetailKey::CheckPassed);

    return check_passed.isValid() && check_passed.toBool();
}

/**
*/
void SingleADSBLatency::addAnnotationForDetail(nlohmann::json& annotations_json,
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
 * JoinedADSBLatency
 **********************************************************************************************/

/**
*/
JoinedADSBLatency::JoinedADSBLatency(const std::string& result_id,
                                     std::shared_ptr<EvaluationRequirement::Base> requirement,
                                     const SectorLayer& sector_layer,
                                     EvaluationCalculator& calculator)
:   ADSBLatencyBase()
,   JoinedProbabilityBase("JoinedADSBLatency", result_id, requirement, sector_layer, calculator)
{
}

/**
*/
unsigned int JoinedADSBLatency::numIssues() const
{
    return num_value_nok_;
}

/**
*/
unsigned int JoinedADSBLatency::numUpdates() const
{
    return num_value_ok_ + num_value_nok_;
}

/**
*/
void JoinedADSBLatency::clearResults_impl()
{
    num_pos_         = 0;
    num_no_data_     = 0;
    num_pos_outside_ = 0;
    num_pos_inside_  = 0;
    num_value_ok_    = 0;
    num_value_nok_   = 0;

    accumulator_.reset();
}

/**
*/
void JoinedADSBLatency::accumulateSingleResult(const std::shared_ptr<Single>& single_result, bool first, bool last)
{
    std::shared_ptr<SingleADSBLatency> single = std::static_pointer_cast<SingleADSBLatency>(single_result);

    assert (single->resultUsable());

    num_pos_         += single->numPos();
    num_no_data_     += single->numNoData();
    num_pos_outside_ += single->numPosOutside();
    num_pos_inside_  += single->numPosInside();
    num_value_ok_    += single->numValueOk();
    num_value_nok_   += single->numValueNok();

    accumulator_.join(single->accumulator(), last);
}

/**
*/
boost::optional<double> JoinedADSBLatency::computeResult_impl() const
{
    loginf << "start"
           << " num_value_ok " << num_value_ok_
           << " num_value_nok " << num_value_nok_;

    unsigned int num_values = num_value_ok_ + num_value_nok_;

    if (!num_values)
        return {};

    return (double)num_value_ok_ / (double)num_values;
}

/**
*/
std::vector<Joined::SectorInfo> JoinedADSBLatency::sectorInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                          , num_pos_         },
             { "#NoData [1]"    , "Number of updates w/o TOMR and/or TORT"     , num_no_data_     },
             { "#PosOutside [1]", "Number of updates outside sector"           , num_pos_outside_ },
             { "#PosInside [1]" , "Number of updates inside sector"            , num_pos_inside_  },
             { "LTMin [s]"      , "Minimum of latency"                         , formatValue(accumulator_.min())    },
             { "LTMax [s]"      , "Maximum of latency"                         , formatValue(accumulator_.max())    },
             { "LTAvg [s]"      , "Average of latency"                         , formatValue(accumulator_.mean())   },
             { "LTSDev [s]"     , "Standard Deviation of latency"              , formatValue(accumulator_.stddev()) },
             { "#LTOK [1]"      , "Number of updates with acceptable latency"  , num_value_ok_    },
             { "#LTNOK [1]"     , "Number of updates with unacceptable latency", num_value_nok_   } };
}

}
