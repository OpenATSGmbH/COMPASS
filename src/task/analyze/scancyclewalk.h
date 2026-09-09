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

#include <cstddef>
#include <vector>

namespace analysis
{

// One per scan cycle (or nominal slot) that falls inside a reference period.
// `t_cycle` is also the timestamp at which the reference position is looked
// up for cell attribution. `num_reports` is the number of test reports in the
// cycle window `[t_cycle, t_window_end)`, `is_miss` is true when it is zero.
struct CycleEvent
{
    boost::posix_time::ptime t_cycle;
    bool                     is_miss     = false;
    unsigned int             num_reports = 0;
};

// Pure helper: maps (period, cycle timestamps, test timestamps) to the
// per-cycle event sequence used by the status-message / scan-cycle PD walk.
//
//   * `cycles_sorted` and `tst_ts_sorted` must be sorted ascending.
//   * Only cycles in `[period.begin, period.end]` are considered.
//   * The window for the i-th in-period cycle is `[c_i, c_{i+1})` where
//     `c_{i+1}` is the next in-period cycle, or `period.end` for the last one.
//   * `num_reports` counts the test timestamps in that window.
//
// When `mv` is given, standing targets are not expected every cycle: a standing
// cycle only opens an expected opportunity once `mv->ui_standing` has elapsed,
// and its window spans that interval (the intervening cycles are skipped, since
// a standing target legitimately reports less often than the cycle rate). When
// `mv` is null, or the target is moving, every in-period cycle is expected.
std::vector<CycleEvent> evaluateCyclesInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<boost::posix_time::ptime>&       cycles_sorted,
    const std::vector<boost::posix_time::ptime>&       tst_ts_sorted,
    const MovementUI*                                  mv = nullptr);

// Nominal-period variant for sources without cycle messages: slots start at
// `period.begin + k * period_s` and span `[t, t + period_s)`. Only slots that
// lie completely inside the period are produced (floor semantics, as the
// detection requirement's expected update count). `num_reports` counts the
// test timestamps in each slot window. `max_slots` guards degenerate periods.
std::vector<CycleEvent> evaluateNominalSlotsInPeriod(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    double                                             period_s,
    const std::vector<boost::posix_time::ptime>&       tst_ts_sorted,
    std::size_t                                        max_slots = 50000000);

} // namespace analysis
