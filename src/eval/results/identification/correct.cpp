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

#include "results/identification/correct.h"

namespace EvaluationRequirementResult
{

/********************************************************************************
 * SingleIdentificationCorrect
 ********************************************************************************/

/**
*/
SingleIdentificationCorrect::SingleIdentificationCorrect(const std::string& result_id, 
                                                         std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                         const SectorLayer& sector_layer,
                                                         unsigned int utn,
                                                         const EvaluationTargetData* target,
                                                         EvaluationCalculator& calculator,
                                                         const EvaluationDetails& details,
                                                         unsigned int num_updates,
                                                         unsigned int num_no_ref_pos,
                                                         unsigned int num_no_ref_id,
                                                         unsigned int num_pos_outside,
                                                         unsigned int num_pos_inside,
                                                         unsigned int num_correct,
                                                         unsigned int num_not_correct)
:   SingleCorrectBase("SingleIdentificationCorrect", 
                      result_id, 
                      requirement, 
                      sector_layer, 
                      utn, 
                      target, 
                      calculator, 
                      details,
                      num_updates,
                      num_no_ref_pos,
                      num_no_ref_id,
                      num_pos_outside,
                      num_pos_inside,
                      num_correct,
                      num_not_correct,
                      "identification",
                      "#CID",
                      "#NCID")
{
    updateResult();
}

/**
*/
std::shared_ptr<Joined> SingleIdentificationCorrect::createEmptyJoined(const std::string& result_id)
{
    return std::make_shared<JoinedIdentificationCorrect> (result_id, requirement_, sector_layer_, calculator_);
}

/********************************************************************************
 * JoinedIdentificationCorrect
 ********************************************************************************/

/**
*/
JoinedIdentificationCorrect::JoinedIdentificationCorrect(const std::string& result_id, 
                                                         std::shared_ptr<EvaluationRequirement::Base> requirement,
                                                         const SectorLayer& sector_layer, 
                                                         EvaluationCalculator& calculator)
:   JoinedCorrectBase("JoinedIdentificationCorrect", 
                      result_id, 
                      requirement, 
                      sector_layer, 
                      calculator, 
                      "identification",
                      "#CID",
                      "#NCID")
{
}

}
