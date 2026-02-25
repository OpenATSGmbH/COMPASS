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

#include "eval/requirement/base/baseconfig.h"
#include "eval/requirement/base/comparisontype.h"

namespace EvaluationRequirement {

class ProbabilityBaseConfig : public BaseConfig
{
public:
    ProbabilityBaseConfig(nlohmann::json& config,
                          Group* parent);
    virtual ~ProbabilityBaseConfig() {}

    double prob() const;
    void prob(double value);

    COMPARISON_TYPE probCheckType() const;
    void probCheckType(const COMPARISON_TYPE& prob_type);

protected:
    double prob_ {0};
    COMPARISON_TYPE prob_check_type_ {COMPARISON_TYPE::GREATER_THAN_OR_EQUAL};
};

} // namespace EvaluationRequirement
