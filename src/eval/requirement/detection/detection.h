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
#include "eval/requirement/detection/detection_pd_helpers.h"

namespace EvaluationRequirement
{

/**
*/
class Detection : public ProbabilityBase
{
public:
  Detection(const std::string& name, const std::string& short_name, const std::string& group_name,
            double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
            float update_interval_s, bool use_min_gap_length, float min_gap_length_s,
            bool use_max_gap_length, float max_gap_length_s, bool invert_prob,
            bool use_miss_tolerance, float miss_tolerance_s, bool use_time_ratio,
            bool use_stationary_ui, float stationary_ui_s, float stationary_speed_threshold_ms,
            bool hold_for_any_target, bool ignore_primary_only);

  float updateInterval() const;
  bool useMinGapLength() const;
  float minGapLength() const;
  bool useMaxGapLength() const;
  float maxGapLength() const;
  bool useMissTolerance() const;
  float missTolerance() const;
  float missThreshold() const;
  bool useTimeRatio() const;
  bool useStationaryUI() const;
  float stationaryUI() const;
  float stationarySpeedThreshold() const;

  bool ignorePrimaryOnly() const;

  virtual std::shared_ptr<EvaluationRequirementResult::Single> evaluate(
      const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
      const SectorLayer& sector_layer) override;

  std::string probabilityNameShort() const override final { return (invertProb() ? "PG" : "PD"); }
  std::string probabilityName() const override final
  {
      return (invertProb() ? "Probability of Gap" : "Probability of Detection");
  }

protected:
    PDHelpers::MissTestParams missTestParams(float update_interval_s) const;

    bool isMiss (float d_tod, float update_interval_s) const;
    unsigned int getNumMisses(float d_tod, float update_interval_s) const;
    float getMissedTime(float d_tod, float update_interval_s) const;
    double getMissed(float d_tod, float update_interval_s) const;

    // gap-specific update interval: the stationary UI when the reference
    // ground speed near the gap midpoint is below the speed threshold
    float updateIntervalFor(const EvaluationTargetData& target_data,
                            const boost::posix_time::ptime& gap_begin,
                            const boost::posix_time::ptime& gap_end) const;

    float update_interval_s_{0};

    bool  use_min_gap_length_{false};
    float min_gap_length_s_  {0};

    bool  use_max_gap_length_{false};
    float max_gap_length_s_  {0};

    bool  use_miss_tolerance_{false};
    float miss_tolerance_s_  {0};

    // time-ratio calculation mode (ED-129C Appendix C "Interarrivaltime" method)
    bool use_time_ratio_ {false};

    // speed-dependent update interval for surface targets (ED-129C ORQ 627)
    bool  use_stationary_ui_ {false};
    float stationary_ui_s_   {10.0f};
    float stationary_speed_threshold_ms_ {0.5f};

    bool ignore_primary_only_ {true};
};

}
