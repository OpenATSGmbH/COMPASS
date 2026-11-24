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

#include "requirement/position/positionhelpers.h"

#include "timeconv.h"

#include <cmath>

namespace EvaluationRequirement
{
namespace PositionHelpers
{

/**
 */
std::vector<AveragingWindow> averagingWindows(const std::vector<ErrorSample>& samples, double window_s)
{
    std::vector<AveragingWindow> windows;

    const auto window_length = Utils::Time::partialSeconds(window_s);

    size_t sample_idx = 0;

    while (sample_idx < samples.size())
    {
        const auto window_begin = samples[ sample_idx ].timestamp;
        const auto window_end   = window_begin + window_length;

        double sum_dx = 0.0, sum_dy = 0.0;
        size_t next_idx = sample_idx;

        while (next_idx < samples.size() && samples[ next_idx ].timestamp < window_end)
        {
            sum_dx += samples[ next_idx ].dx;
            sum_dy += samples[ next_idx ].dy;
            ++next_idx;
        }

        AveragingWindow w;
        w.first = sample_idx;
        w.end   = next_idx;

        const double num = (double)(next_idx - sample_idx);
        const double mean_dx = sum_dx / num;
        const double mean_dy = sum_dy / num;

        w.mean_distance = std::sqrt(mean_dx * mean_dx + mean_dy * mean_dy);
        w.span_s        = Utils::Time::partialSeconds(samples[ next_idx - 1 ].timestamp - window_begin);

        windows.push_back(w);

        sample_idx = next_idx;
    }

    return windows;
}

} // namespace PositionHelpers
} // namespace EvaluationRequirement
