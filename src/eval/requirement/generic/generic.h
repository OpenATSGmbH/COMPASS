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

#include "requirement/base/probabilitybase.h"

namespace EvaluationRequirement
{

/**
*/
class GenericBase : public ProbabilityBase
{
  public:
    GenericBase(const std::string& name, 
                const std::string& short_name, 
                const std::string& group_name,
                double prob, 
                COMPARISON_TYPE prob_check_type, 
                EvaluationCalculator& calculator);

    std::string valueName() const;
    std::string valueNameShort() const;
    std::string valueNamePlural() const;

    std::string result_type_;       // e.g. FalseMode3A

    std::string value_name_;        // the thing, the whole thing and the essence of the thing, e.g. "Mode 3/A Code"
    std::string value_name_short_;  // e.g. "M3/A"
    std::string value_name_plural_; // plural, e.g.  "Mode 3/A Codes"
};

/**
*/
class GenericInteger : public GenericBase
{
public:
    GenericInteger(const std::string& name, 
                   const std::string& short_name, 
                   const std::string& group_name,
                   double prob, 
                   COMPARISON_TYPE prob_check_type, 
                   EvaluationCalculator& calculator);

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer) override;

protected:
    std::function <std::pair<ValueComparisonResult,std::string> (
        const dbContent::TargetReport::Chain::DataID& id,
        const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff)> value_compare_func_;
};

/**
*/
class GenericDouble : public GenericBase
{
public:
    GenericDouble(const std::string& name, 
                  const std::string& short_name, 
                  const std::string& group_name,
                  double prob, 
                  COMPARISON_TYPE prob_check_type, 
                  double threshold, 
                  EvaluationCalculator& calculator);

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer) override;

protected:
    double threshold_ {0};

    std::function <std::pair<ValueComparisonResult,std::string> (
        const dbContent::TargetReport::Chain::DataID& id,
        const EvaluationTargetData& target_data,
        boost::posix_time::time_duration max_ref_time_diff, double threshold)> value_compare_func_;
};

}
