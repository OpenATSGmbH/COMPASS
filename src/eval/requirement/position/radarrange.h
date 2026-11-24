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

#include "requirement/base/positionbase.h"
#include "requirement/position/radar_helpers.h"

namespace EvaluationRequirement
{

/**
*/
class PositionRadarRange : public PositionBaseValue
{
public:
    PositionRadarRange(
            const std::string& name, const std::string& short_name, const std::string& group_name, float ref_min_accuracy,
            EvaluationCalculator& calculator, double threshold_value,
            RadarHelpers::Statistic statistic);

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate (
            const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
            const SectorLayer& sector_layer) override;

    RadarHelpers::Statistic statistic() const { return statistic_; }
    const RadarHelpers::StatisticInfo& statisticInfo() const
    { return RadarHelpers::statisticInfo(RadarHelpers::rangeStatistics(), statistic_); }

    std::string getConditionResultNameShort() const override final { return statisticInfo().short_name; }
    std::string getConditionResultName() const override final { return statisticInfo().name; }
    std::string getConditionUnits() const override final { return statisticInfo().units; }

protected:
    RadarHelpers::Statistic statistic_;
};

}
