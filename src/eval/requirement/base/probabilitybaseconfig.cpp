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

#include "probabilitybaseconfig.h"

namespace EvaluationRequirement 
{

ProbabilityBaseConfig::ProbabilityBaseConfig(
        nlohmann::json& config,
        Group* parent)
    : BaseConfig(config, parent)
{
    registerParameter("prob", &prob_, 0.9);
    registerParameter("prob_check_type", (unsigned int*)&prob_check_type_,
                      (unsigned int)COMPARISON_TYPE::GREATER_THAN_OR_EQUAL);
}

double ProbabilityBaseConfig::prob() const
{
    return prob_;
}

void ProbabilityBaseConfig::prob(double value)
{
    prob_ = value;
}

COMPARISON_TYPE ProbabilityBaseConfig::probCheckType() const
{
    return prob_check_type_;
}

void ProbabilityBaseConfig::probCheckType(const COMPARISON_TYPE& prob_type)
{
    prob_check_type_ = prob_type;
}

} // namespace EvaluationRequirement
