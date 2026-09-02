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
#include <functional>
#include <vector>

namespace analysis
{

/// Parameters of the time-difference coverage walk. `mv` supplies the update
/// interval per timestamp, so a standing target is expected less often than a
/// moving one. The miss tolerance is the slack granted at the end of a gap
/// before the last slot counts as missed, matching the detection requirement.
struct PDWalkParams
{
    const MovementUI* mv = nullptr;

    bool  use_miss_tolerance = false;
    float miss_tolerance_s   = 0.0f;

    /// runaway guard for degenerate update intervals
    std::size_t max_iterations = 50000000;
};

typedef std::function<void(const boost::posix_time::ptime&)> PDWalkSlotFunc;

/**
 * Time-difference coverage walk over one reference period, shared by the
 * coverage inspectors (see `src/task/analyze/`):
 *
 *   - Expected slots: starting at `period.begin`, step by the update interval
 *     valid at the current timestamp until `period.end`. Each slot is reported
 *     to `on_expected` (one #EUI).
 *   - Gaps: the test timestamps inside `[period.begin, period.end]` split the
 *     period into gaps (period start to first report, between reports, last
 *     report to period end; the whole period when no report is inside).
 *   - Missed slots: inside each gap, step by the update interval valid at the
 *     current timestamp. Each slot that lies more than the miss tolerance
 *     before the end of the gap is reported to `on_miss` (one #MUI).
 *
 * The update interval is re-evaluated at every step instead of being frozen at
 * the start of the gap. Freezing it over-counts a gap in which the target stops
 * moving, e.g. a standing hour after taxiing counted at the moving cadence,
 * which can produce more missed than expected slots and hence a negative
 * probability of detection.
 *
 * `tst_ts_sorted` must be sorted ascending. The callbacks receive the slot
 * timestamp; the caller resolves positions, cells and counters.
 */
void walkReferencePeriodTimeDifference(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<boost::posix_time::ptime>&       tst_ts_sorted,
    const PDWalkParams&                                params,
    const PDWalkSlotFunc&                              on_expected,
    const PDWalkSlotFunc&                              on_miss);

/// Runs `walkReferencePeriodTimeDifference()` for each period.
void walkReferencePeriodsTimeDifference(
    const std::vector<EvaluationRequirement::PDHelpers::RefPeriod>& periods,
    const std::vector<boost::posix_time::ptime>&                    tst_ts_sorted,
    const PDWalkParams&                                             params,
    const PDWalkSlotFunc&                                           on_expected,
    const PDWalkSlotFunc&                                           on_miss);

} // namespace analysis
