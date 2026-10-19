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

    /// sampling step along a piece when the caller splits it into cells
    double sample_step_s = 0.2;

    /// runaway guard for degenerate update intervals
    std::size_t max_iterations = 50000000;
};

/// Piece callback: a piece [begin, end) of the reference time line, carrying
/// `expected` expected slots (#EUI) and, if `is_miss`, the same number of
/// missed slots (#MUI). A missed piece always carries exactly 1 slot.
typedef std::function<void(const boost::posix_time::ptime&,
                           const boost::posix_time::ptime&,
                           double,
                           bool)> PDWalkPieceFunc;

/// Gap callback: begin and end of a gap with at least one missed slot, and the
/// number of missed slots inside it. Begin and end are the bounding test
/// timestamps, or the period bounds where the gap touches the period.
typedef std::function<void(const boost::posix_time::ptime&,
                           const boost::posix_time::ptime&,
                           unsigned int)> PDWalkGapFunc;

/// Sample callback: a time inside a piece and the fraction of the piece it
/// stands for. The fractions of one piece sum to 1.
typedef std::function<void(const boost::posix_time::ptime&, double)> PDWalkSampleFunc;

/**
 * Time-difference coverage walk over one reference period, shared by the
 * coverage inspectors (see `src/task/analyze/`).
 *
 * Everything is derived from the time line of the target only: the test
 * timestamps inside `[period.begin, period.end]` split the period into gaps
 * (period start to first report, between reports, last report to period end;
 * the whole period when no report is inside). For each gap [g0, g1):
 *
 *   - Missed slots: step by the update interval valid at the current
 *     timestamp, t_k = t_(k-1) + UI(t_(k-1)), starting at g0. Each t_k that
 *     lies more than the miss tolerance before g1 is one missed slot. This is
 *     the gap rule of the detection requirement.
 *   - Pieces: each missed slot k is the piece [t_k, t_(k+1)) (the last one
 *     ends at g1) with 1 expected and 1 missed slot. The piece [g0, t_1) (the
 *     whole gap without a miss) is detected and carries the remaining
 *     fraction, (g1 - t_m) / UI(t_m) after the last miss t_m, or
 *     (g1 - g0) / UI(g0) without a miss.
 *
 * So a gap carries (gap length) / UI expected slots, of which the gap rule
 * misses are missed, and the period (period length) / UI. Reports closer than
 * the update interval give less than one expected slot and no miss.
 *
 * The caller places each piece into cells, see `samplePiece()`. A missed slot
 * and its expected slot share the same piece and therefore the same cells, so
 * no cell or sector can get more missed than expected slots: the probability
 * of detection stays within [0, 1] everywhere.
 *
 * The update interval is re-evaluated at every step instead of being frozen at
 * the start of the gap, so a gap in which the target stops moving is counted
 * at the standing cadence from that point on.
 *
 * `tst_ts_sorted` must be sorted ascending. `on_gap` is called once per gap
 * with at least one missed slot, after its pieces.
 */
void walkReferencePeriodTimeDifference(
    const EvaluationRequirement::PDHelpers::RefPeriod& period,
    const std::vector<boost::posix_time::ptime>&       tst_ts_sorted,
    const PDWalkParams&                                params,
    const PDWalkPieceFunc&                             on_piece,
    const PDWalkGapFunc&                               on_gap = PDWalkGapFunc());

/// Runs `walkReferencePeriodTimeDifference()` for each period.
void walkReferencePeriodsTimeDifference(
    const std::vector<EvaluationRequirement::PDHelpers::RefPeriod>& periods,
    const std::vector<boost::posix_time::ptime>&                    tst_ts_sorted,
    const PDWalkParams&                                             params,
    const PDWalkPieceFunc&                                          on_piece,
    const PDWalkGapFunc&                                            on_gap = PDWalkGapFunc());

/// Splits the piece [begin, end) into n equal sub-intervals of at most
/// `max_step_s` and calls `on_sample` with the middle of each and 1 / n. The
/// caller puts each sample into the cell at the reference position at that
/// time, so a piece that crosses cells is split by the time spent in each.
void samplePiece(const boost::posix_time::ptime& begin,
                 const boost::posix_time::ptime& end,
                 double                          max_step_s,
                 const PDWalkSampleFunc&         on_sample);

} // namespace analysis
