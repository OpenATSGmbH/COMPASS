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

class DetectionConfig : public ProbabilityBaseConfig
{
public:
    DetectionConfig(const std::string& class_id, const std::string& instance_id,
                    Group& group, EvaluationStandard& standard, EvaluationCalculator& calculator);
    virtual ~DetectionConfig();

    std::shared_ptr<Base> createRequirement() override;

    bool useMinGapLength() const;
    void useMinGapLength(bool value);

    float minGapLength() const;
    void minGapLength(float value);

    bool useMaxGapLength() const;
    void useMaxGapLength(bool value);

    float maxGapLength() const;
    void maxGapLength(float value);

    float updateInterval() const;
    void updateInterval(float value);

    bool invertProb() const;
    void invertProb(bool value);

    bool useMissTolerance() const;
    void useMissTolerance(bool value);

    float missTolerance() const;
    void missTolerance(float value);

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

    bool holdForAnyTarget() const;
    void holdForAnyTarget(bool value);

    bool ignorePrimaryOnly() const;
    void ignorePrimaryOnly(bool value);

  protected:
    float update_interval_s_{0};

    bool use_min_gap_length_ {false};
    float min_gap_length_s_{0};

    bool use_max_gap_length_ {false};
    float max_gap_length_s_{0};

    bool invert_prob_ {false};

    bool use_miss_tolerance_{false};
    float miss_tolerance_s_{0};

    bool hold_for_any_target_ {false}; // if requirement must hold for any target (all single targets)

    bool ignore_primary_only_ {false};

    virtual BaseConfigWidget* createWidget() override;
};

}
