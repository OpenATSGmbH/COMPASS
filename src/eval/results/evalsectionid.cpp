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

#include "evalsectionid.h"

#include "sectorlayer.h"
#include "eval/requirement/group.h"
#include "eval/requirement/base/base.h"
#include "eval/results/base/base.h"
#include "evaluationcalculator.h"

#include <QString>

const std::string EvalSectionID::SectionTargets = "Targets";
const std::string EvalSectionID::SectionUTN     = "UTN ";
const std::string EvalSectionID::SectionSum     = "Sum";
const std::string EvalSectionID::SectionSectors = "Sectors";

/**
*/
std::string EvalSectionID::reqNameFromReqResultID(const std::string& req_result_id)
{
    auto strings = Utils::String::split(req_result_id, Sep[ 0 ]);
    if (strings.size() < 2)
        return "";

    return strings.back();
}

/**
*/
std::string EvalSectionID::targetResultsID()
{
    return SectionResults + Sep + SectionTargets;
}

/**
*/
std::string EvalSectionID::targetID()
{
    return SectionTargets;
}

/**
*/
std::string EvalSectionID::targetID(unsigned int utn)
{
    return targetID() + Sep + SectionUTN + std::to_string(utn);
}

/**
*/
std::string EvalSectionID::sectorLayerID(const SectorLayer& sector_layer)
{
    return sector_layer.name();
}

/**
*/
std::string EvalSectionID::sectorLayerID(const Evaluation::RequirementResultID& id)
{
    return id.sec_layer_name;
}

/**
*/
std::string EvalSectionID::requirementID(const EvaluationRequirement::Base& requirement)
{
    return requirement.name();
}

/**
*/
std::string EvalSectionID::requirementID(const Evaluation::RequirementResultID& id)
{
    return id.req_name;
}

/**
*/
std::string EvalSectionID::requirementGroupID(const EvaluationRequirement::Base& requirement)
{
    return requirement.groupName();
}

/**
*/
std::string EvalSectionID::requirementGroupID(const Evaluation::RequirementResultID& id)
{
    return id.req_group_name;
}

/**
*/
std::string EvalSectionID::sumID(const EvaluationRequirementResult::Base& result)
{
    return result.sumSectionName();
}

/**
*/
std::string EvalSectionID::sumID(const Evaluation::RequirementSumResultID& id)
{
    return id.req_sum_name;
}

/**
*/
std::string EvalSectionID::requirementGroupResultID(const EvaluationRequirementResult::Base& result)
{
    return sectorLayerID(result.sectorLayer()) + Sep + 
           requirementGroupID(*result.requirement()) + Sep + 
           requirementID(*result.requirement());
}

/**
*/
std::string EvalSectionID::requirementGroupResultID(const Evaluation::RequirementResultID& id)
{
    return sectorLayerID(id) + Sep + 
           requirementGroupID(id) + Sep + 
           requirementID(id);
}

/**
*/
std::string EvalSectionID::requirementGroupSectorID(const EvaluationRequirement::Base& requirement,
                                                    const SectorLayer& sector_layer)
{
    return requirementGroupID(requirement) + " " + sectorLayerID(sector_layer);
}

/**
*/
std::string EvalSectionID::requirementGroupSectorID(const Evaluation::RequirementResultID& id)
{
    return requirementGroupID(id) + " " + sectorLayerID(id);
}

/**
*/
std::string EvalSectionID::resultID(const EvaluationRequirementResult::Base& result)
{
    return result.resultId();
}

/**
*/
std::string EvalSectionID::targetResultID(unsigned int utn, const EvaluationRequirementResult::Base& result)
{
    traced_assert(result.requirement());
    return targetID(utn) + Sep + requirementGroupResultID(result);
}

/**
*/
std::string EvalSectionID::targetResultID(unsigned int utn, const Evaluation::RequirementResultID& id)
{
    return targetID(utn) + Sep + requirementGroupResultID(id);
}

/**
*/
std::string EvalSectionID::requirementResultID(const EvaluationRequirementResult::Base& result)
{
    traced_assert(result.requirement());
    return SectionSectors + Sep + 
           requirementGroupSectorID(*result.requirement(), result.sectorLayer()) + Sep + 
           resultID(result) + Sep + 
           requirementID(*result.requirement());
}

/**
*/
std::string EvalSectionID::requirementResultSumID(const EvaluationRequirementResult::Base& result)
{
    traced_assert(result.requirement());
    return SectionSectors + Sep + 
           requirementGroupSectorID(*result.requirement(), result.sectorLayer()) + Sep + 
           sumID(result) + Sep + 
           requirementID(*result.requirement());
}

/**
*/
std::string EvalSectionID::requirementResultSumID(const Evaluation::RequirementSumResultID& id)
{
    return SectionSectors + Sep + 
           requirementGroupSectorID(id) + Sep + 
           sumID(id) + Sep + 
           requirementID(id);
}

/**
*/
std::string EvalSectionID::createForTargetResult(unsigned int utn,
                                                 const EvaluationRequirementResult::Base& result)
{
    return reportResultID() + Sep + 
           targetResultID(utn, result);
}

/**
*/
std::string EvalSectionID::createForTargetResult(unsigned int utn,
                                                 const Evaluation::RequirementResultID& id)
{
    return reportResultID() + Sep + 
           targetResultID(utn, id);
}

/**
*/
std::string EvalSectionID::createForRequirementResult(const EvaluationRequirementResult::Base& result)
{
    return reportResultID() + Sep + 
           requirementResultID(result);
}

/**
*/
std::string EvalSectionID::createForRequirementResultSum(const EvaluationRequirementResult::Base& result)
{
    return reportResultID() + Sep + 
           requirementResultSumID(result);
}

/**
*/
std::string EvalSectionID::createForRequirementResultSum(const Evaluation::RequirementSumResultID& id)
{
    return reportResultID() + Sep + 
           requirementResultSumID(id);
}
