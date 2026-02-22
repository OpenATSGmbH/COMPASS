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

#include "task/result/report/sectionid.h"
#include "evaluationdefs.h"

#include <string>
#include <vector>

class SectorLayer;

class EvaluationCalculator;

namespace EvaluationRequirement
{
    class Base;
}

namespace EvaluationRequirementResult
{
    class Base;
}

namespace Evaluation
{
    struct RequirementResultID;
    struct RequirementSumResultID;
}

/**
 * Functionality for creating unique requirement result section ids of various types.
 *   - ids for target-specific result sections
 *   - ids for sum result sections
 *   - conversion between result section types
 *   - definition of all needed strings in one place
 */
class EvalSectionID : public ResultReport::SectionID
{
public:
    //creation of requirement result section ids
    static std::string createForTargetResult(unsigned int utn, const EvaluationRequirementResult::Base& result);
    static std::string createForTargetResult(unsigned int utn, const Evaluation::RequirementResultID& id);
    static std::string createForRequirementResult(const EvaluationRequirementResult::Base& result);
    static std::string createForRequirementResultSum(const EvaluationRequirementResult::Base& result);
    static std::string createForRequirementResultSum(const Evaluation::RequirementSumResultID& id);

    //conversion
    static std::string sumResult2Target(const std::string& sum_result_id, 
                                        unsigned int utn,
                                        const EvaluationCalculator& eval_calc);

    static std::string reqNameFromReqResultID(const std::string& req_result_id);

    //needed section id parts
    static std::string targetResultsID();
    static std::string targetID();
    static std::string targetID(unsigned int utn);
    static std::string sectorLayerID(const SectorLayer& sector_layer);
    static std::string sectorLayerID(const Evaluation::RequirementResultID& id);
    static std::string requirementID(const EvaluationRequirement::Base& requirement);
    static std::string requirementID(const Evaluation::RequirementResultID& id);
    static std::string requirementGroupID(const EvaluationRequirement::Base& requirement);
    static std::string requirementGroupID(const Evaluation::RequirementResultID& id);
    static std::string sumID(const EvaluationRequirementResult::Base& result);
    static std::string sumID(const Evaluation::RequirementSumResultID& id);

    static std::string requirementGroupResultID(const EvaluationRequirementResult::Base& result);
    static std::string requirementGroupResultID(const Evaluation::RequirementResultID& id);

    static std::string requirementGroupSectorID(const EvaluationRequirement::Base& requirement,
                                                const SectorLayer& sector_layer);
    static std::string requirementGroupSectorID(const Evaluation::RequirementResultID& id);
    static std::string resultID(const EvaluationRequirementResult::Base& result);
    static std::string targetResultID(unsigned int utn, const EvaluationRequirementResult::Base& result);
    static std::string targetResultID(unsigned int utn,
                                      const Evaluation::RequirementResultID& id);
    static std::string requirementResultID(const EvaluationRequirementResult::Base& result);
    static std::string requirementResultSumID(const EvaluationRequirementResult::Base& result);
    static std::string requirementResultSumID(const Evaluation::RequirementSumResultID& id);

    //section name definitions
    static const std::string SectionTargets;
    static const std::string SectionUTN;
    static const std::string SectionSum;
    static const std::string SectionSectors;
};
