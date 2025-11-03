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

#include "eval/requirement/base/probabilitybase.h"

namespace EvaluationRequirement
{

/**
*/
class PositionBase : public ProbabilityBase
{
public:
  PositionBase(const std::string& name, const std::string& short_name,
               const std::string& group_name, double prob_threshold,
               COMPARISON_TYPE prob_check_type, float ref_min_accuracy,
               EvaluationCalculator& calculator);
  virtual ~PositionBase();            

protected:
    float ref_min_accuracy_ {0};
};

}
