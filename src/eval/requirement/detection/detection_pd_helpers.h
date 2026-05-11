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

#include "boost/date_time/posix_time/ptime.hpp"
#include "boost/date_time/posix_time/posix_time_duration.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

namespace EvaluationRequirement
{
namespace PDHelpers
{

// Parameters of the time-difference miss test, as documented in
// src/eval/requirement/detection/readme_detection.md, section 3.4.
struct MissTestParams
{
    float update_interval_s = 1.0f;

    bool  use_miss_tolerance = false;
    float miss_tolerance_s   = 0.0f;

    bool  use_min_gap_length = false;
    float min_gap_length_s   = 0.0f;

    bool  use_max_gap_length = false;
    float max_gap_length_s   = 0.0f;
};

// Returns the gap length after applying miss tolerance (never negative).
inline float adjustedGap(float gap_s, const MissTestParams& p)
{
    float adj = gap_s;
    if (p.use_miss_tolerance)
        adj -= p.miss_tolerance_s;
    return adj;
}

// True iff `gap_s` counts as a miss under `p`. Matches the four-filter rule
// from readme_detection.md section 3.4 (tolerance, min, max, > UI).
inline bool isMiss(float gap_s, const MissTestParams& p)
{
    const float adj = adjustedGap(gap_s, p);

    if (p.use_min_gap_length && adj < p.min_gap_length_s)
        return false;
    if (p.use_max_gap_length && adj > p.max_gap_length_s)
        return false;

    return adj > p.update_interval_s;
}

// Number of missed update intervals attributed to `gap_s`. Caller must have
// established `isMiss(gap_s, p) == true`; otherwise the result is 0.
inline unsigned int numMisses(float gap_s, const MissTestParams& p)
{
    if (!isMiss(gap_s, p))
        return 0;
    if (p.update_interval_s <= 0.0f)
        return 0;

    const float adj = adjustedGap(gap_s, p);
    if (adj <= 0.0f)
        return 0;
    return static_cast<unsigned int>(std::floor(adj / p.update_interval_s));
}

// Contiguous reference-coverage window, as built by `buildReferencePeriods()`.
struct RefPeriod
{
    boost::posix_time::ptime begin;
    boost::posix_time::ptime end;

    bool empty() const { return end <= begin; }
};

// Splits a sorted set of reference timestamps into maximal contiguous
// periods. A new period starts whenever the gap between consecutive
// timestamps exceeds `max_gap`. Periods shorter than `min_duration` are
// dropped (matches detection.cpp's hardcoded 1 s floor; the caller chooses
// the threshold).
//
// Mirrors the reference-period construction in
// `Detection::evaluate()` (readme_detection.md section 3.2), minus the
// sector / exclusion-interval filter, which are evaluation-specific.
inline std::vector<RefPeriod> buildReferencePeriods(
    const std::set<boost::posix_time::ptime>& timestamps,
    boost::posix_time::time_duration          max_gap,
    boost::posix_time::time_duration          min_duration)
{
    std::vector<RefPeriod> periods;

    if (timestamps.empty())
        return periods;

    auto it = timestamps.begin();
    RefPeriod current{*it, *it};
    ++it;

    for (; it != timestamps.end(); ++it)
    {
        const auto ts = *it;
        if (ts - current.end > max_gap)
        {
            periods.push_back(current);
            current = RefPeriod{ts, ts};
        }
        else
        {
            current.end = ts;
        }
    }
    periods.push_back(current);

    periods.erase(std::remove_if(periods.begin(), periods.end(),
                                 [&](const RefPeriod& p) {
                                     return (p.end - p.begin) < min_duration;
                                 }),
                  periods.end());

    return periods;
}

} // namespace PDHelpers
} // namespace EvaluationRequirement
