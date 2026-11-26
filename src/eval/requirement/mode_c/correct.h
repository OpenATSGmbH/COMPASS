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

#include "requirement/base/probabilitybase.h"

namespace EvaluationRequirement
{

/**
*/
class ModeCCorrect : public ProbabilityBase
{
public:
    ModeCCorrect(
            const std::string& name, const std::string& short_name, const std::string& group_name,
            double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator, float max_distance_ft);

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
            const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
            const SectorLayer& sector_layer) override;

    std::string probabilityNameShort() const override final { return "PC"; }
    std::string probabilityName() const override final { return "Probability of correct Mode C"; }

    float maxDistanceFt() const;

protected:
    float max_distance_ft_ {300};
};

}
