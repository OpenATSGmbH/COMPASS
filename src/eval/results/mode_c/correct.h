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

#include "eval/results/base/correctbase.h"

namespace EvaluationRequirementResult
{

/**
*/
class SingleModeCCorrect : public SingleCorrectBase
{
public:
    SingleModeCCorrect(const std::string& result_id,
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
                       unsigned int num_not_correct);

    virtual std::shared_ptr<Joined> createEmptyJoined(const std::string& result_id) override;

protected:
    virtual std::string targetTableCustomSortColumn() const override { return "#NCMC"; }
    virtual Qt::SortOrder targetTableSortOrder() const override { return Qt::SortOrder::DescendingOrder; }
    virtual std::vector<TargetInfo> additionalTargetInfos() const override;
};

/**
*/
class JoinedModeCCorrect : public JoinedCorrectBase
{
public:
    JoinedModeCCorrect(const std::string& result_id,
                       std::shared_ptr<EvaluationRequirement::Base> requirement,
                       const SectorLayer& sector_layer, 
                       EvaluationCalculator& calculator);
};

}
