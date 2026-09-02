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
 * ADS-B Ground Station latency per EUROCAE ED-129C Section 3.3.4 (REQ 19,
 * REQ 498): the time from reception of an ADS-B horizontal position message
 * to the output of the target report, computed per CAT021 test report as
 * Time of Report Transmission (I021/077) minus Time of Message Reception
 * for Position (I021/073). No reference data is needed for the value.
 *
 * The percentile conditions of the standard map to the probability check:
 * e.g. 95% of reports with latency <= 0.6 s (REQ 19), 99.9% of reports
 * with latency <= 1.0 s (REQ 498), as separate requirement instances.
 */
class ADSBLatency : public ProbabilityBase
{
public:
    ADSBLatency(const std::string& name, const std::string& short_name, const std::string& group_name,
                double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
                float max_value_s);

    float maxValue() const;

    virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate(
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer) override;

    std::string probabilityNameShort() const override final { return "PALT"; }
    std::string probabilityName() const override final { return "Probability of Acceptable Latency"; }

protected:
    float max_value_s_ {0};
};

}
