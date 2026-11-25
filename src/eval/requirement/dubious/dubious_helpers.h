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

#include "results/base/detailcode.h"

#include <boost/optional.hpp>

#include <cstdint>
#include <vector>

namespace EvaluationRequirement
{

/**
 * Checks of the Dubious Target and Dubious Track requirements, free of the evaluation data
 * access, so they can be unit tested on plain values.
 */
namespace DubiousHelpers
{

struct Params
{
    bool   use_max_groundspeed    = false;
    double max_groundspeed_kts    = 0.0;
    bool   check_calculated_speed = false; // also the speed between two positions (Dubious Target)
    bool   use_max_acceleration   = false;
    double max_acceleration       = 0.0;   // [m/s^2]
    bool   use_max_turnrate       = false;
    double max_turnrate           = 0.0;   // [deg/s]
    bool   use_max_rocd           = false;
    double max_rocd               = 0.0;   // [ft/s]
    double min_comparison_time    = 0.0;   // [s]
    double max_comparison_time    = 0.0;   // [s]
};

struct Update
{
    double                  time_s                 = 0.0;
    boost::optional<double> speed_ms;
    boost::optional<double> track_angle_deg;
    boost::optional<double> mode_c_ft;
    double                  distance_to_previous_m = 0.0;
};

/// type bits of the detail code of the Dubious Target and Dubious Track results, one per reason without
/// a value. A reason with a value (speed, acceleration, turn rate, ROCD) is the filled value column.
enum CodeBit : std::uint32_t
{
    PrimaryOnly     = EvaluationRequirementResult::DetailCode::typeBit(0), // period: no Mode A/C and no Mode S data
    FewUpdates      = EvaluationRequirementResult::DetailCode::typeBit(1), // period: fewer updates than the minimum
    ShortDuration   = EvaluationRequirementResult::DetailCode::typeBit(2), // period: shorter than the minimum duration

    AllReasons      = PrimaryOnly | FewUpdates | ShortDuration
};

/// the values that make an update dubious, unset if the check passed or did not apply
struct UpdateFlags
{
    boost::optional<double> reported_speed_ms;
    boost::optional<double> calculated_speed_kts;
    boost::optional<double> acceleration;
    boost::optional<double> turnrate;
    boost::optional<double> rocd;

    /// a value made the update dubious
    bool dubious() const;
};

/**
 * Checks each update against the previous one. The reported speed is checked on its own. If it
 * is not too high, the speed from the distance to the previous update is checked (if enabled).
 * Acceleration, turnrate and ROCD are checked only if the time to the previous update is within
 * the comparison times.
 */
std::vector<UpdateFlags> checkUpdates(const std::vector<Update>& updates, const Params& p);

/// dubious if the ratio of dubious updates is higher than dubious_prob
bool isDubious(unsigned int num_dubious, unsigned int num_updates, float dubious_prob);

} // namespace DubiousHelpers

} // namespace EvaluationRequirement
