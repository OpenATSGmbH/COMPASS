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

#include "evaluationdefs.h"
#include "evalsectionid.h"

#include "results/base/base.h"


namespace Evaluation
{

const std::string RequirementResultID::FieldLayerName = "sec_layer_name";
const std::string RequirementResultID::FieldGroupName = "req_group_name";
const std::string RequirementResultID::FieldReqName   = "req_name";

/**
 */
RequirementResultID::RequirementResultID(const std::string& r_sec_layer_name,
                                         const std::string& r_group_name,
                                         const std::string& r_name)
:   sec_layer_name(r_sec_layer_name)
,   req_group_name(r_group_name)
,   req_name      (r_name) 
{
}

/**
 */
bool RequirementResultID::valid() const
{
    return !sec_layer_name.empty() &&
           !req_group_name.empty() &&
           !req_name.empty();
}

/**
 */
void RequirementResultID::fromResult(const EvaluationRequirementResult::Base& result)
{
    sec_layer_name = EvalSectionID::sectorLayerID(result.sectorLayer());
    req_group_name = EvalSectionID::requirementGroupID(*result.requirement());
    req_name       = EvalSectionID::requirementID(*result.requirement());
}

/**
 */
nlohmann::json RequirementResultID::toJSON() const
{
    nlohmann::json j;

    j[ FieldLayerName ] = sec_layer_name;
    j[ FieldGroupName ] = req_group_name;
    j[ FieldReqName   ] = req_name;

    return j;
}

/**
 */
bool RequirementResultID::fromJSON(const nlohmann::json& j)
{
    if (!j.is_object()              ||
        !j.contains(FieldLayerName) ||
        !j.contains(FieldGroupName) ||
        !j.contains(FieldReqName))
        return false;

    sec_layer_name = j[ FieldLayerName ];
    req_group_name = j[ FieldGroupName ];
    req_name       = j[ FieldReqName   ];

    return true;
}

const std::string RequirementSumResultID::FieldSumName = "req_sum_name";

/**
 */
RequirementSumResultID::RequirementSumResultID(const std::string& r_sec_layer_name,
                                               const std::string& r_group_name,
                                               const std::string& r_name,
                                               const std::string& r_sum_name)
:   RequirementResultID(r_sec_layer_name, r_group_name, r_name)
,   req_sum_name       (r_sum_name) 
{
}

/**
 */
bool RequirementSumResultID::valid() const
{
    return RequirementResultID::valid() && !req_sum_name.empty();
}

/**
 */
void RequirementSumResultID::fromResult(const EvaluationRequirementResult::Base& result)
{
    RequirementResultID::fromResult(result);

    req_sum_name = result.sumSectionName();
}

/**
 */
nlohmann::json RequirementSumResultID::toJSON() const
{
    nlohmann::json j = RequirementResultID::toJSON();

    j[ FieldSumName ] = req_sum_name;

    return j;
}

/**
 */
bool RequirementSumResultID::fromJSON(const nlohmann::json& j)
{
    if (!RequirementResultID::fromJSON(j) ||
        !j.contains(FieldSumName))
        return false;

    req_sum_name = j[ FieldSumName ];

    return true;
}

} // namespace Evaluation
