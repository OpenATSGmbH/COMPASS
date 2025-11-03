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

#include "eval/requirement/base/comparisontype.h"
#include "eval/requirement/base/base.h"

#include <string>
#include <memory>

class EvaluationCalculator;

namespace EvaluationRequirement 
{

/**
*/
class ProbabilityBase : public Base
{
public:
    ProbabilityBase(const std::string& name, 
                    const std::string& short_name, 
                    const std::string& group_name,
                    double prob_threshold, 
                    COMPARISON_TYPE prob_check_type, 
                    bool invert_prob,
                    EvaluationCalculator& calculator,
                    const boost::optional<bool>& must_hold_for_any_target = boost::optional<bool>());
    virtual ~ProbabilityBase();

    bool invertProb() const;

    std::string getConditionResultNameShort() const override final;
    std::string getConditionResultName() const override final;
    std::string getConditionUnits() const override;

    virtual std::string probabilityNameShort() const = 0;
    virtual std::string probabilityName() const = 0;

protected:
    double convertValueToResult(double value) const override final;

    bool invert_prob_ = false;
};

} // namespace EvaluationRequirement
