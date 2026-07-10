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

#include "eval/requirement/detection/detection_pd_helpers.h"
#include "movementui.h"

#include "boost/date_time/posix_time/ptime.hpp"

#include <vector>

namespace mlatcoverage_internal
{

// One per status-message cycle that falls inside a reference period.
// `t_cycle` is also the timestamp at which the reference position is
// looked up for cell attribution. `is_miss` is true when no test report
// exists in the cycle window `[t_cycle, t_window_end)` (see
// `evaluateCyclesInPeriod`).
struct CycleEvent
{
    boost::posix_time::ptime t_cycle;
    bool                     is_miss = false;
};

// Pure helper: maps (period, cycle timestamps, test timestamps) to the
// per-cycle event sequence used by the status-message PD walker.
//
//   * `cycles_sorted` and `tst_ts_sorted` must be sorted ascending.
//   * Only cycles in `[period.begin, period.end]` are considered.
//   * The window for the i-th in-period cycle is
//     `[c_i, c_{i+1})` where `c_{i+1}` is the next in-period cycle, or
//     `period.end` for the last one.
//   * `is_miss` is true iff no test report timestamp lies in that window.
//
// When `mv` is given, standing targets are not expected every cycle: a standing
// cycle only opens an expected opportunity once `mv->ui_standing` has elapsed,
// and its window spans that interval (the intervening cycles are skipped, since
// a standing target legitimately reports less often than the cycle rate). When
// `mv` is null, or the target is moving, every in-period cycle is expected (the
// original per-cycle behavior).
std::vector<CycleEvent> evaluateCyclesInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod&     period,
    const std::vector<boost::posix_time::ptime>&           cycles_sorted,
    const std::vector<boost::posix_time::ptime>&           tst_ts_sorted,
    const analysis::MovementUI*                            mv = nullptr);

} // namespace mlatcoverage_internal
