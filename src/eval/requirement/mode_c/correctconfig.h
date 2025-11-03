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

#include <memory>

class Group;
class EvaluationStandard;

namespace ResultReport
{
    class Report;
}

namespace EvaluationRequirement
{

class ModeCCorrectConfig : public ProbabilityBaseConfig
{
public:
    ModeCCorrectConfig(const std::string& class_id, const std::string& instance_id,
                                Group& group, EvaluationStandard& standard,
                                EvaluationCalculator& calculator);

    std::shared_ptr<Base> createRequirement() override;

    float maxDistanceFt() const;
    void maxDistanceFt(float value);

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

protected:
    float max_distance_ft_ {300};

    virtual BaseConfigWidget* createWidget() override;
};

}

