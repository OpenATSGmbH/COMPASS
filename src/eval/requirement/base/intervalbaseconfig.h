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

#include <boost/optional.hpp>

namespace ResultReport
{
    class Report;
    class SectionContentTable;
}

namespace EvaluationRequirement
{

class BaseConfigWidget;

/**
*/
class IntervalBaseConfig : public ProbabilityBaseConfig
{
public:
    enum ConfigFlags
    {
        UseMinGapLen = 1 << 0,
        UseMaxGapLen = 1 << 1,
        UseMissTol   = 1 << 2,
        UseAnyTarget = 1 << 4
    };

    IntervalBaseConfig(nlohmann::json& config,
                       Group* parent);
    virtual ~IntervalBaseConfig() = default;

    float updateInterval() const;
    void updateInterval(float value);

    bool useMinGapLength() const;
    void useMinGapLength(bool value);

    float minGapLength() const;
    void minGapLength(float value);

    bool useMaxGapLength() const;
    void useMaxGapLength(bool value);

    float maxGapLength() const;
    void maxGapLength(float value);

    bool useMissTolerance() const;
    void useMissTolerance(bool value);

    float missTolerance() const;
    void missTolerance(float value);

    bool invertProb() const;
    void invertProb(bool value);

    bool holdForAnyTarget() const;
    void holdForAnyTarget(bool value);

    uint32_t configFlags() const;

    virtual void addToReport (std::shared_ptr<ResultReport::Report> report) override;

protected:
    void configure(uint32_t flags);

    virtual BaseConfigWidget* createWidget() override;

    virtual std::string probabilityDescription() const = 0;
    virtual BaseConfigWidget* createWidget_impl() = 0;
    virtual void addCustomTableEntries(ResultReport::SectionContentTable& table) const {};
    
    uint32_t config_flags_    = std::numeric_limits<uint32_t>::max();

    float update_interval_s_  = 0.0f;

    bool use_min_gap_length_  = false;
    float min_gap_length_s_   = 0.0f;

    bool use_max_gap_length_  = false;
    float max_gap_length_s_   = 0.0f;

    bool use_miss_tolerance_  = false;
    float miss_tolerance_s_   = 0.0f;

    bool hold_for_any_target_ = false; // if requirement must hold for any target (all single targets)
};

} // namespace EvaluationRequirement
