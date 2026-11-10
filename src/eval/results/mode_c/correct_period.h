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

#include "results/base/intervalbase.h"

namespace EvaluationRequirementResult
{

/**
*/
class SingleModeCCorrectPeriod : public SingleIntervalBase
{
public:
    SingleModeCCorrectPeriod(const std::string& result_type, 
                             const std::string& result_id, 
                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                             const SectorLayer& sector_layer, 
                             unsigned int utn, 
                             const EvaluationTargetData* target,
                             EvaluationCalculator& calculator,
                             const EvaluationDetails& details,
                             int sum_uis, 
                             int missed_uis, 
                             TimePeriodCollection ref_periods);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;
};

/**
*/
class JoinedModeCCorrectPeriod : public JoinedIntervalBase
{
public:
    JoinedModeCCorrectPeriod(const std::string& result_type, 
                             const std::string& result_id, 
                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                             const SectorLayer& sector_layer, 
                             EvaluationCalculator& calculator);
};

} // namespace EvaluationRequirementResult
