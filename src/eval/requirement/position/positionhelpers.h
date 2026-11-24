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

#include <vector>

#include <boost/date_time/posix_time/ptime.hpp>

namespace EvaluationRequirement
{

/**
 * Algorithms of the position requirements, free of the evaluation data access, so they can
 * be unit tested on plain inputs.
 */
namespace PositionHelpers
{

/// position error of a test report in the local Cartesian frame (test minus reference)
struct ErrorSample
{
    boost::posix_time::ptime timestamp;
    double                   dx = 0.0;
    double                   dy = 0.0;
};

/// averaging window over samples [first, end)
struct AveragingWindow
{
    size_t first         = 0;
    size_t end           = 0;
    double mean_distance = 0.0; // length of the mean error vector [m]
    double span_s        = 0.0; // time from the first to the last sample in the window [s]
};

/**
 * Averaging windows over the samples (sorted by time): a window starts at the first sample not
 * yet used and contains the samples before its start plus window_s. So a data gap does not
 * create empty windows.
 */
std::vector<AveragingWindow> averagingWindows(const std::vector<ErrorSample>& samples, double window_s);

} // namespace PositionHelpers

} // namespace EvaluationRequirement
