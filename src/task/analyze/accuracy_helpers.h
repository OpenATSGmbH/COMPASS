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

#include <optional>
#include <vector>

namespace analysis
{

/**
 * Rules shared by the accuracy inspectors (MLAT, ADS-B, SMR): reported accuracy, reference accuracy
 * gate, consistency ratio and offset statistics. Free of the dataset, so they can be unit tested on
 * plain values.
 */
namespace AccuracyHelpers
{

constexpr double MinStdDevForRatioM = 1e-6; // below, a reported accuracy gives no consistency ratio

/// position standard deviation sqrt(sx^2 + sy^2) of a reported accuracy, none unless both are
/// finite and at least one is > 0
std::optional<double> reportedStdDev(double sx_m, double sy_m);

/// position standard deviation sqrt(sx^2 + sy^2) of a reference accuracy, none if not finite
std::optional<double> referenceStdDev(double sx_m, double sy_m);

/**
 * Reference accuracy gate: true if the offset must not be measured, because the reference is not
 * gate_factor times more accurate than the claimed accuracy. A reference without accuracy cannot be
 * shown to be good enough and is gated. No gate for gate_factor <= 0 or without a claimed accuracy > 0.
 */
bool refGated(double gate_factor, std::optional<double> ref_std_dev_m, std::optional<double> claimed_std_dev_m);

/// offset / reported accuracy, none without a reported accuracy above MinStdDevForRatioM
std::optional<double> consistencyRatio(double offset_m, std::optional<double> reported_std_dev_m);

struct ValueStats
{
    std::size_t num    = 0;
    double      median = 0.0;
    double      mean   = 0.0;
    double      p95    = 0.0;
};

/// count, median, mean and 95th percentile, all 0 for no values
ValueStats valueStats(const std::vector<double>& values);

} // namespace AccuracyHelpers

} // namespace analysis
