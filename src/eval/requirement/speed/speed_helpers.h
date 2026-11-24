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
 * Check of one test report of the Speed requirement, free of the evaluation data access, so it
 * can be unit tested on plain values.
 */
namespace SpeedHelpers
{

struct Params
{
    float           threshold_value       = 0.0f; // maximum speed difference [m/s]
    bool            use_percent_if_higher = false;
    float           threshold_percent     = 0.0f; // of the reference speed
    COMPARISON_TYPE check_type            = COMPARISON_TYPE::LESS_THAN_OR_EQUAL;
};

enum class Outcome
{
    NoTstValue, // test report without speed: not counted in the probability
    Passed,
    Failed
};

struct Check
{
    Outcome outcome   = Outcome::NoTstValue;
    float   diff      = 0.0f; // absolute speed difference [m/s]
    float   threshold = 0.0f; // threshold used [m/s]
};

/// the fixed threshold, or the percentage of the reference speed if enabled and higher
float threshold(double ref_speed_ms, const Params& p);

/// compares the absolute difference of the test and reference speed with the threshold
Check check(double ref_speed_ms, const boost::optional<float>& tst_speed_ms, const Params& p);

} // namespace SpeedHelpers

} // namespace EvaluationRequirement
