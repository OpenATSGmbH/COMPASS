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

#include "requirement/base/comparisontype.h"

#include <boost/optional.hpp>

namespace EvaluationRequirement
{

/**
 * Check of one test report of the Track Angle requirement, free of the evaluation data access,
 * so it can be unit tested on plain values.
 */
namespace TrackAngleHelpers
{

struct Params
{
    float           threshold         = 0.0f; // maximum angle difference [deg]
    bool            use_minimum_speed = false;
    float           minimum_speed     = 0.0f; // minimum reference speed [m/s]
    COMPARISON_TYPE check_type        = COMPARISON_TYPE::LESS_THAN_OR_EQUAL;
};

enum class Outcome
{
    RefSpeedTooLow, // reference slower than the minimum speed: not counted in the probability
    NoTstValue,     // test report without track angle: not counted in the probability
    Passed,
    Failed
};

struct Check
{
    Outcome outcome = Outcome::NoTstValue;
    float   diff    = 0.0f; // smallest signed angle difference test - reference [deg], in [-180, 180)
};

/**
 * Compares the absolute smallest angle difference of the test and reference track angle with the
 * threshold. The minimum speed is checked first, since the track angle of a slow target is not
 * defined well.
 */
Check check(double ref_speed_ms, double ref_angle_deg, const boost::optional<double>& tst_angle_deg,
            const Params& p);

} // namespace TrackAngleHelpers

} // namespace EvaluationRequirement
