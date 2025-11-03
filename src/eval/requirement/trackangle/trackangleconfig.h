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

#include "eval/requirement/base/probabilitybaseconfig.h"

class Group;
class EvaluationStandard;

namespace ResultReport
{
    class Report;
}

namespace EvaluationRequirement
{

class TrackAngleConfig : public ProbabilityBaseConfig
{
public:
    TrackAngleConfig(const std::string& class_id, const std::string& instance_id,
                Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator);
    virtual ~TrackAngleConfig();

    std::shared_ptr<Base> createRequirement() override;

    float threshold() const;
    void threshold(float value);

    bool useMinimumSpeed() const;
    void useMinimumSpeed(bool value);

    float minimumSpeed() const;
    void minimumSpeed(float value);

    COMPARISON_TYPE thresholdValueCheckType() const;
    void thresholdValueCheckType(const COMPARISON_TYPE& type);

    bool failedValuesOfInterest() const;
    void failedValuesOfInterest(bool value);

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

protected:
    float threshold_ {15.0}; // max angle degree difference

    bool use_minimum_speed_ {true};
    float minimum_speed_ {3.0}; // m/s

    COMPARISON_TYPE threshold_value_check_type_ {COMPARISON_TYPE::LESS_THAN_OR_EQUAL};

    bool failed_values_of_interest_ {true};

    virtual BaseConfigWidget* createWidget() override;
};

}
