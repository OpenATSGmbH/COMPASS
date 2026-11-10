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

#pragma once

#include "results/base/presentbase.h"

namespace EvaluationRequirementResult
{

/**
*/
class SingleModeCPresent : public SinglePresentBase
{
public:
    SingleModeCPresent(const std::string& result_id, 
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
                       int num_no_ref_val, 
                       int num_present, 
                       int num_missing);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;
};

/**
*/
class JoinedModeCPresent : public JoinedPresentBase
{
public:
    JoinedModeCPresent(const std::string& result_id, 
                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                       const SectorLayer& sector_layer, 
                       EvaluationCalculator& calculator);
};

}
