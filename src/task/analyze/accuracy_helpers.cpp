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


#include "accuracy_helpers.h"

#include "number.h"

#include <cmath>

namespace analysis
{
namespace AccuracyHelpers
{

/**
 */
std::optional<double> reportedStdDev(double sx_m, double sy_m)
{
    if (!std::isfinite(sx_m) || !std::isfinite(sy_m) || (sx_m <= 0.0 && sy_m <= 0.0))
        return std::nullopt;

    return std::sqrt(sx_m * sx_m + sy_m * sy_m);
}

/**
 */
std::optional<double> referenceStdDev(double sx_m, double sy_m)
{
    const double value = std::sqrt(sx_m * sx_m + sy_m * sy_m);

    if (!std::isfinite(value))
        return std::nullopt;

    return value;
}

/**
 */
bool refGated(double gate_factor, std::optional<double> ref_std_dev_m, std::optional<double> claimed_std_dev_m)
{
    if (gate_factor <= 0.0 || !claimed_std_dev_m || !std::isfinite(*claimed_std_dev_m) || *claimed_std_dev_m <= 0.0)
        return false;

    return !ref_std_dev_m || gate_factor * *ref_std_dev_m >= *claimed_std_dev_m;
}

/**
 */
std::optional<double> consistencyRatio(double offset_m, std::optional<double> reported_std_dev_m)
{
    if (!reported_std_dev_m || *reported_std_dev_m <= MinStdDevForRatioM)
        return std::nullopt;

    return offset_m / *reported_std_dev_m;
}

/**
 */
ValueStats valueStats(const std::vector<double>& values)
{
    ValueStats stats;
    stats.num = values.size();

    if (values.empty())
        return stats;

    stats.median = Utils::Number::percentile(values, 0.5);
    stats.mean   = Utils::Number::mean(values);
    stats.p95    = Utils::Number::percentile(values, 0.95);

    return stats;
}

} // namespace AccuracyHelpers
} // namespace analysis
