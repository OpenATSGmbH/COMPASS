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

#include "results/mode_c/present.h"

namespace EvaluationRequirementResult
{

/********************************************************************************
 * SingleModeCPresent
 ********************************************************************************/

/**
*/
SingleModeCPresent::SingleModeCPresent(const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer,
                                       unsigned int utn,
                                       const EvaluationTargetData* target,
                                       EvaluationCalculator& calculator,
                                       const EvaluationDetails& details,
                                       int num_updates,
                                       int num_no_ref_pos,
                                       int num_pos_outside,
                                       int num_pos_inside,
                                       int num_no_ref_id,
                                       int num_present_id,
                                       int num_missing_id)
:   SinglePresentBase("SingleModeAPresent", result_id, requirement, sector_layer, utn, target, calculator, details,
                        num_updates, num_no_ref_pos, num_pos_outside, num_pos_inside, num_no_ref_id, num_present_id, num_missing_id, "#NoRefC")
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleModeCPresent::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedModeCPresent> (result_id, requirement_, sector_layer_, calculator_);
}

/********************************************************************************
 * JoinedModeCPresent
 ********************************************************************************/

/**
*/
JoinedModeCPresent::JoinedModeCPresent(const std::string& result_id, 
                                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                                       const SectorLayer& sector_layer, 
                                       EvaluationCalculator& calculator)
:   JoinedPresentBase("JoinedModeCPresent", result_id, requirement, sector_layer, calculator, "#NoRefC")
{
}

}
