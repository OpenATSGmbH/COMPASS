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

#include "eval/results/base/probabilitybase.h"
#include "number.h"

using namespace Utils;

namespace EvaluationRequirementResult
{

namespace common
{
    /**
    */
    EvaluationRequirement::ProbabilityBase& castRequirement(const std::shared_ptr<EvaluationRequirement::Base>& base_req)
    {
        traced_assert(base_req);

        EvaluationRequirement::ProbabilityBase* req_ptr = dynamic_cast<EvaluationRequirement::ProbabilityBase*>(base_req.get());
        traced_assert(req_ptr);

        return *req_ptr;
    }

    /**
    */
    double invertProbability(double prob)
    {
        return 1.0 - prob;
    }
}

/****************************************************************************************
 * SingleProbabilityBase
 ****************************************************************************************/

/**
*/
const unsigned int ProbabilityBase::NumProbabilityDecimalsDefault = 2;

/**
*/
nlohmann::json ProbabilityBase::formatProbability(double prob, unsigned int decimals)
{
    //return Utils::String::percentToString(std::round(prob * 10000.0) / 100.0, 2).c_str();

    return Number::round(100.0 * prob, decimals);
}

/**
*/
nlohmann::json ProbabilityBase::formatProbabilityOptional(const boost::optional<double>& prob,
                                                          unsigned int decimals)
{
    if (!prob.has_value())
        return nlohmann::json();

    return SingleProbabilityBase::formatProbability(prob.value(), decimals);
}

/****************************************************************************************
 * SingleProbabilityBase
 ****************************************************************************************/

/**
*/
SingleProbabilityBase::SingleProbabilityBase(const std::string& type, 
                                             const std::string& result_id,
                                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                                             const SectorLayer& sector_layer,
                                             unsigned int utn,
                                             const EvaluationTargetData* target,
                                             EvaluationCalculator& calculator,
                                             const EvaluationDetails& details)
:   Single(type, result_id, requirement, sector_layer, utn, target, calculator, details)
{
}

/**
*/
SingleProbabilityBase::~SingleProbabilityBase() = default;

/**
*/
EvaluationRequirement::ProbabilityBase& SingleProbabilityBase::probRequirement() const
{
    return common::castRequirement(requirement_);
}

/**
*/
boost::optional<double> SingleProbabilityBase::computeResult() const
{
    auto result = computeResult_impl();

    //invert probability?
    if (result.has_value() && probRequirement().invertProb())
        result = invertProb(result.value());

    return result;
}

/**
*/
double SingleProbabilityBase::invertProb(double prob) const
{
    return common::invertProbability(prob);
}

/**
*/
nlohmann::json SingleProbabilityBase::resultValue(double value) const
{
    // follow the requirement threshold, so a value failing a fine threshold such
    // as the ED-117 false identification 1e-6 is not shown as 0.00
    return formatProbability(value, requirement_->getNumThresholdDecimals());
}

/****************************************************************************************
 * JoinedProbabilityBase
 ****************************************************************************************/

/**
*/
JoinedProbabilityBase::JoinedProbabilityBase(const std::string& type, 
                                             const std::string& result_id,
                                             std::shared_ptr<EvaluationRequirement::Base> requirement,
                                             const SectorLayer& sector_layer,
                                             EvaluationCalculator& calculator)
:   Joined(type, result_id, requirement, sector_layer, calculator)
{
}

/**
*/
JoinedProbabilityBase::~JoinedProbabilityBase() = default;

/**
*/
EvaluationRequirement::ProbabilityBase& JoinedProbabilityBase::probRequirement() const
{
    return common::castRequirement(requirement_);
}

/**
*/
boost::optional<double> JoinedProbabilityBase::computeResult() const
{
    auto result = computeResult_impl();

    //invert probability?
    if (result.has_value() && probRequirement().invertProb())
        result = invertProb(result.value());

    return result;
}

/**
*/
double JoinedProbabilityBase::invertProb(double prob) const
{
    return common::invertProbability(prob);
}

/**
*/
nlohmann::json JoinedProbabilityBase::resultValue(double value) const
{
    // follow the requirement threshold, see SingleProbabilityBase::resultValue
    return formatProbability(value, requirement_->getNumThresholdDecimals());
}

}
