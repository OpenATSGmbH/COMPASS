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

#include <boost/date_time/posix_time/ptime.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

class AnalysisDataset;

namespace dbContent { namespace TargetReport { class Chain; } }

namespace analysis
{

/// Time-sorted (timestamp, ground speed [m/s]) samples for one target. Speed is
/// NaN where the report carried no ground speed. Used to classify each PD slot
/// as standing or moving so the coverage walk can apply a separate, slower
/// nominal update interval to standing targets.
struct SpeedSamples
{
    std::vector<boost::posix_time::ptime> ts;
    std::vector<double>                   sp;  // m/s, NaN if unknown
};

/// Ground speed nearest to `t` within `window_s` seconds. Returns NaN if the
/// nearest in-window sample has no usable speed (caller may try another source).
inline double nearestSpeed(const SpeedSamples& s, boost::posix_time::ptime t, double window_s)
{
    if (s.ts.empty())
        return std::numeric_limits<double>::quiet_NaN();

    auto it = std::lower_bound(s.ts.begin(), s.ts.end(), t);

    double best_dt = std::numeric_limits<double>::max();
    double best_sp = std::numeric_limits<double>::quiet_NaN();

    auto consider = [&](std::size_t idx) {
        double dt = std::abs(static_cast<double>((s.ts[idx] - t).total_microseconds()) / 1.0e6);
        if (dt <= window_s && std::isfinite(s.sp[idx]) && dt < best_dt)
        {
            best_dt = dt;
            best_sp = s.sp[idx];
        }
    };
    if (it != s.ts.end())
        consider(static_cast<std::size_t>(it - s.ts.begin()));
    if (it != s.ts.begin())
        consider(static_cast<std::size_t>((it - 1) - s.ts.begin()));
    return best_sp;
}

/// Movement-aware update-interval selector. A slot/gap is "standing" when the
/// reported ground speed (test side where available, reference side as fallback)
/// is below `standing_max`; unknown speed counts as moving (matches
/// ReconstructorInfo::isMoving()). Standing slots use the slower `ui_standing`.
struct MovementUI
{
    const SpeedSamples* test         = nullptr;
    const SpeedSamples* ref          = nullptr;
    double              window_s      = 10.0;
    double              standing_max  = 0.5;
    double              ui_moving     = 1.0;
    double              ui_standing   = 5.0;

    double speedAt(boost::posix_time::ptime t) const
    {
        double s = test ? nearestSpeed(*test, t, window_s)
                        : std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(s))
            return s;
        return ref ? nearestSpeed(*ref, t, window_s)
                   : std::numeric_limits<double>::quiet_NaN();
    }

    bool standingAt(boost::posix_time::ptime t) const
    {
        double s = speedAt(t);
        return std::isfinite(s) && s < standing_max;
    }

    double uiAt(boost::posix_time::ptime t) const
    {
        return standingAt(t) ? ui_standing : ui_moving;
    }
};

/// Build time-sorted speed samples from a target's test chains (all test
/// dbcontents present), reading the ground speed of each report.
SpeedSamples gatherTestSpeeds(unsigned int utn, AnalysisDataset& dataset);

/// Build time-sorted speed samples from a target's reference (RefTraj) chain.
SpeedSamples gatherRefSpeeds(dbContent::TargetReport::Chain& ref_chain);

}  // namespace analysis
