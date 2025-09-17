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

#include "eval/results/position/latency.h"
#include "stringconv.h"

#include "logger.h"

using namespace Utils;

namespace EvaluationRequirementResult
{

/**********************************************************************************************
 * SinglePositionLatency
 **********************************************************************************************/

/**
*/
SinglePositionLatency::SinglePositionLatency(const std::string& result_id, 
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
                                             unsigned int num_value_ok,
                                             unsigned int num_value_nok)
:   SinglePositionProbabilityBase("SinglePositionLatency", result_id, requirement, sector_layer, utn, target, calculator, details,
                                  num_pos, num_no_ref,num_pos_outside, num_pos_inside, num_ref_inaccurate, 
                                  num_value_ok, num_value_nok)
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SinglePositionLatency::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedPositionLatency> (result_id, requirement_, sector_layer_, calculator_);
}

/**
*/
std::vector<std::string> SinglePositionLatency::targetTableHeadersCustom() const
{
    return { "LTMin", "LTMax", "LTAvg", "LTSDev", "#LTOK", "#LTNOK" };
}

/**
*/
nlohmann::json::array_t SinglePositionLatency::targetTableValuesCustom() const
{
    return { formatValue(accumulator_.min()),
             formatValue(accumulator_.max()), 
             formatValue(accumulator_.mean()),
             formatValue(accumulator_.stddev()),
             num_passed_, 
             num_failed_ }; 
}

/**
*/
std::vector<Single::TargetInfo> SinglePositionLatency::targetInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                          , num_pos_                           }, 
             { "#NoRef [1]"     , "Number of updates w/o reference positions"  , num_no_ref_                        },
             { "#PosOutside [1]", "Number of updates outside sector"           , num_pos_outside_                   }, 
             { "#PosInside [1]" , "Number of updates inside sector"            , num_pos_inside_                    },
             { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_     },
             { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
             { "LTMin [m]"      , "Minimum of latency"                         , formatValue(accumulator_.min())    }, 
             { "LTMax [m]"      , "Maximum of latency"                         , formatValue(accumulator_.max())    },
             { "LTAvg [m]"      , "Average of latency"                         , formatValue(accumulator_.mean())   }, 
             { "LTSDev [m]"     , "Standard Deviation of latency"              , formatValue(accumulator_.stddev()) }, 
             { "LTVar [m^2]"    , "Variance of latency"                        , formatValue(accumulator_.var())    }, 
             { "#LTOK [1]"      , "Number of updates with acceptable latency"  , num_passed_                        }, 
             { "#LTNOK [1]"     , "Number of updates with unacceptable latency", num_failed_                        } };
}

/**
*/
std::vector<std::string> SinglePositionLatency::detailHeaders() const
{
    return { "ToD", "NoRef", "PosInside", "#RefPosIn", "DLatency", "DLatencyOK", "#LTOK", "#LTNOK", "Comment" };
}

/**
*/
nlohmann::json::array_t SinglePositionLatency::detailValues(const EvaluationDetail& detail,
                                                            const EvaluationDetail* parent_detail) const
{
    bool has_ref_pos = detail.numPositions() >= 2;

    return { Utils::Time::toString(detail.timestamp()),
            !has_ref_pos,
             detail.getValue(SinglePositionBaseCommon::DetailKey::PosInside).toBool(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumRefInaccurate).toUInt(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::Value).toFloat(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::CheckPassed).toBool(), 
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumCheckPassed).toUInt(), 
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumCheckFailed).toUInt(), 
             detail.comments().generalComment() }; 
}

/**********************************************************************************************
 * JoinedPositionLatency
 **********************************************************************************************/

/**
*/
JoinedPositionLatency::JoinedPositionLatency(const std::string& result_id, 
                                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                                             const SectorLayer& sector_layer, 
                                             EvaluationCalculator& calculator)
:   JoinedPositionProbabilityBase("JoinedPositionLatency", result_id, requirement, sector_layer, calculator, "latency")
{
}

/**
*/
std::vector<Joined::SectorInfo> JoinedPositionLatency::sectorInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                          , num_pos_                           }, 
             { "#NoRef [1]"     , "Number of updates w/o reference positions"  , num_no_ref_                        },
             { "#PosOutside [1]", "Number of updates outside sector"           , num_pos_outside_                   }, 
             { "#PosInside [1]" , "Number of updates inside sector"            , num_pos_inside_                    },
             { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
             { "#RefPosIn [%]"  , "Percentage of updates with inaccurate reference position"  , String::percentToStringProtected(num_ref_inaccurate_, num_pos_inside_, 2).c_str()},
             { "LTMin [m]"      , "Minimum of latency"                         , formatValue(accumulator_.min())    }, 
             { "LTMax [m]"      , "Maximum of latency"                         , formatValue(accumulator_.max())    },
             { "LTAvg [m]"      , "Average of latency"                         , formatValue(accumulator_.mean())   }, 
             { "LTSDev [m]"     , "Standard Deviation of latency"              , formatValue(accumulator_.stddev()) }, 
             { "LTVar [m^2]"    , "Variance of latency"                        , formatValue(accumulator_.var())    }, 
             { "#LTOK [1]"      , "Number of updates with acceptable latency"  , num_passed_                        }, 
             { "#LTNOK [1]"     , "Number of updates with unacceptable latency", num_failed_                        } };
}

}
