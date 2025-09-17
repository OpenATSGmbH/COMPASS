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

#include "eval/results/position/distance.h"

#include "logger.h"

namespace EvaluationRequirementResult
{

/**
*/
SinglePositionDistance::SinglePositionDistance(const std::string& result_id, 
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
                                               unsigned int num_comp_passed,
                                               unsigned int num_comp_failed)
:   SinglePositionProbabilityBase("SinglePositionDistance", result_id, requirement, sector_layer, utn, target, calculator, details,
                                  num_pos, num_no_ref,num_pos_outside, num_pos_inside, num_ref_inaccurate, num_comp_passed, num_comp_failed)
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SinglePositionDistance::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedPositionDistance> (result_id, requirement_, sector_layer_, calculator_);
}
/**
*/
std::vector<std::string> SinglePositionDistance::targetTableHeadersCustom() const
{
    return { "DMin", "DMax", "DAvg", "DSDev", "#CF", "#CP" };
}

/**
*/
nlohmann::json::array_t SinglePositionDistance::targetTableValuesCustom() const
{
    return { formatValue(accumulator_.min()),
             formatValue(accumulator_.max()), 
             formatValue(accumulator_.mean()),
             formatValue(accumulator_.stddev()),
             num_failed_,
             num_passed_ }; 
}

/**
*/
std::vector<Single::TargetInfo> SinglePositionDistance::targetInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           }, 
             { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
             { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   }, 
             { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
             { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
             { "DMin [m]"       , "Minimum of distance"                      , formatValue(accumulator_.min())    }, 
             { "DMax [m]"       , "Maximum of distance"                      , formatValue(accumulator_.max())    },
             { "DAvg [m]"       , "Average of distance"                      , formatValue(accumulator_.mean())   }, 
             { "DSDev [m]"      , "Standard Deviation of distance"           , formatValue(accumulator_.stddev()) }, 
             { "DVar [m^2]"     , "Variance of distance"                     , formatValue(accumulator_.var())    }, 
             { "#CF [1]"        , "Number of updates with failed comparison" , num_failed_                        }, 
             { "#CP [1]"        , "Number of updates with passed comparison" , num_passed_                        } };
}

/**
*/
std::vector<std::string> SinglePositionDistance::detailHeaders() const
{
    return { "ToD", "NoRef", "PosInside", "#RefPosIn", "Distance", "CP", "#CF", "#CP", "Comment" };
}

/**
*/
nlohmann::json::array_t SinglePositionDistance::detailValues(const EvaluationDetail& detail,
                                                             const EvaluationDetail* parent_detail) const
{
    bool has_ref_pos = detail.numPositions() >= 2;

    return { Utils::Time::toString(detail.timestamp()),
            !has_ref_pos,
             detail.getValue(SinglePositionBaseCommon::DetailKey::PosInside).toBool(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumRefInaccurate).toUInt(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::Value).toFloat(),
             detail.getValue(SinglePositionBaseCommon::DetailKey::CheckPassed).toBool(), 
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumCheckFailed).toUInt(), 
             detail.getValue(SinglePositionBaseCommon::DetailKey::NumCheckPassed).toUInt(), 
             detail.comments().generalComment() }; 
}

/**********************************************************************************************
 * JoinedPositionDistance
 **********************************************************************************************/

/**
*/
JoinedPositionDistance::JoinedPositionDistance(const std::string& result_id, 
                                               std::shared_ptr<EvaluationRequirement::Base> requirement,
                                               const SectorLayer& sector_layer, 
                                               EvaluationCalculator& calculator)
:   JoinedPositionProbabilityBase("JoinedPositionDistance", result_id, requirement, sector_layer, calculator, "distance")
{
}

/**
*/
std::vector<Joined::SectorInfo> JoinedPositionDistance::sectorInfos() const
{
    return { { "#Pos [1]"       , "Number of updates"                        , num_pos_                           }, 
             { "#NoRef [1]"     , "Number of updates w/o reference positions", num_no_ref_                        },
             { "#PosOutside [1]", "Number of updates outside sector"         , num_pos_outside_                   }, 
             { "#PosInside [1]" , "Number of updates inside sector"          , num_pos_inside_                    },
             { "#RefPosIn [1]"  , "Number of updates with inaccurate reference position"  , num_ref_inaccurate_   },
             { "DMin [m]"       , "Minimum of distance"                      , formatValue(accumulator_.min())    }, 
             { "DMax [m]"       , "Maximum of distance"                      , formatValue(accumulator_.max())    },
             { "DAvg [m]"       , "Average of distance"                      , formatValue(accumulator_.mean())   }, 
             { "DSDev [m]"      , "Standard Deviation of distance"           , formatValue(accumulator_.stddev()) }, 
             { "DVar [m^2]"     , "Variance of distance"                     , formatValue(accumulator_.var())    }, 
             { "#CF [1]"        , "Number of updates with failed comparison" , num_failed_                        }, 
             { "#CP [1]"        , "Number of updates with passed comparison" , num_passed_                        } };
}

}
