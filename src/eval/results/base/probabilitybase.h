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

#include "eval/results/base/single.h"
#include "eval/results/base/joined.h"

#include "eval/requirement/base/probabilitybase.h"

namespace EvaluationRequirementResult
{

/**
*/
class SingleProbabilityBase : public Single
{
public:
    SingleProbabilityBase(const std::string& type, 
                          const std::string& result_id,
                          std::shared_ptr<EvaluationRequirement::Base> requirement, 
                          const SectorLayer& sector_layer,
                          unsigned int utn, 
                          const EvaluationTargetData* target, 
                          EvaluationCalculator& calculator,
                          const EvaluationDetails& details);
    virtual ~SingleProbabilityBase();

    nlohmann::json resultValue(double value) const override;

    static nlohmann::json formatProbability(double prob);
    static nlohmann::json formatProbabilityOptional(const boost::optional<double>& prob);

protected:
    EvaluationRequirement::ProbabilityBase& probRequirement() const;

    boost::optional<double> computeResult() const override final;

private:
    double invertProb(double prob) const;
};

/**
*/
class JoinedProbabilityBase : public Joined
{
public:
    JoinedProbabilityBase(const std::string& type, 
                          const std::string& result_id,
                          std::shared_ptr<EvaluationRequirement::Base> requirement,
                          const SectorLayer& sector_layer,
                          EvaluationCalculator& calculator);
    virtual ~JoinedProbabilityBase();

protected:
    EvaluationRequirement::ProbabilityBase& probRequirement() const;

    boost::optional<double> computeResult() const override final;

private:
    double invertProb(double prob) const;
};

}
