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

#include "requirement/detection/detection_pd_helpers.h"

#include <functional>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>

class TimePeriodCollection;

namespace EvaluationRequirement
{

/**
 * Time-difference walk of the Detection requirement (probability of detection, long gaps),
 * free of the evaluation data access, so it can be unit tested on plain inputs.
 */
namespace DetectionWalk
{

/// one step of the walk, in walk order (the requirement creates its details from them)
struct Step
{
    enum class Type
    {
        PeriodWithoutTestData, // no test data at all: gap from period begin to period end
        PeriodEnd,             // period finished: gap from the last test report (or period begin) to period end
        OutsideRefPeriods,     // test report outside the reference periods
        NoRefPos,              // test report without mapped reference position
        OutsideSector,         // test report outside the sector (or in an excluded time)
        FirstAfterOutside,     // first test report after test reports outside the sector
        FirstInPeriod,         // first test report in a period: gap from period begin to the report
        Gap                    // gap from the previous test report to the report
    };

    Type                     type;
    int                      tst_index = -1;    // index of the test report the step belongs to, -1 if after the walk
    boost::posix_time::ptime t0;                // gap begin
    boost::posix_time::ptime t1;                // gap end
    bool                     t0_is_tst = false; // gap begins at a test report (else at the period begin)
    unsigned int             period    = 0;     // reference period index
    float                    t_diff    = 0.0f;  // gap length [s]
    bool                     has_gap   = false; // step has a gap (t0, t1, t_diff, miss)
    bool                     miss      = false; // gap is a miss
    double                   missed    = 0.0;   // missed amount of the gap
    double                   sum_missed = 0.0;  // running sum of the missed amount after this step
};

struct Params
{
    PDHelpers::MissTestParams miss_params;       // base miss test parameters (base update interval)
    bool                      use_gap_count  = false;
    bool                      use_time_ratio = false;
};

struct Result
{
    double            sum_expected = 0.0;
    double            sum_missed   = 0.0;
    std::vector<Step> steps;
};

typedef std::function<bool(size_t)> TstCheck; // check of the test report at an index
typedef std::function<float(const boost::posix_time::ptime&, const boost::posix_time::ptime&)> GapUIFunc;

/// reference periods from the reference times and their inside state, periods shorter than 1 s are removed
TimePeriodCollection referencePeriods(const std::vector<std::pair<boost::posix_time::ptime, bool>>& ref_inside,
                                      const boost::posix_time::time_duration& max_ref_time_diff);

/// missed amount of a gap that is a miss: 1 gap (gap count), missed seconds (time ratio) or missed update intervals
double missedAmount(float t_diff, const PDHelpers::MissTestParams& p, const Params& params);

/**
 * Walks the test reports (sorted by time) through the reference periods.
 * has_ref_pos and inside_sector are only called for test reports inside a reference period,
 * gap_ui gives the update interval of a gap (e.g. stationary update interval).
 */
Result walk(const TimePeriodCollection& ref_periods,
            const std::vector<boost::posix_time::ptime>& tst_times,
            const TstCheck& has_ref_pos,
            const TstCheck& inside_sector,
            const GapUIFunc& gap_ui,
            const Params& params);

/// update cycle figures of the status message method
struct CycleStats
{
    double       median_cycle_s    = 0.0;
    unsigned int cycles_per_period = 1;
    double       period_s          = 0.0; // length of an expected period: cycles_per_period * median_cycle_s
    unsigned int num_outages       = 0;   // cycle gaps longer than OutageFactor median cycles
    double       longest_outage_s  = 0.0;
};

/// a cycle gap longer than this factor times the median cycle length is an outage of the status messages
const double OutageFactor = 1.5;

/**
 * Update cycle figures. The cycles per period are the update interval over the median cycle
 * length, rounded, at least 1. Needs at least 2 cycles.
 */
CycleStats cycleStats(const std::vector<boost::posix_time::ptime>& cycles, float update_interval_s);

/// number of reported update cycles per expected period of the status message method, see cycleStats()
unsigned int cyclesPerPeriod(const std::vector<boost::posix_time::ptime>& cycles, float update_interval_s);

/// expected period of the status message walk without a detecting test report
struct CycleMiss
{
    boost::posix_time::ptime begin;
    boost::posix_time::ptime end;
    double                   sum_missed = 0.0; // running sum of the missed periods after this one
};

struct CycleResult
{
    double                 sum_expected = 0.0;
    double                 sum_missed   = 0.0;
    std::vector<CycleMiss> misses;
};

/**
 * Status message walk: in each reference period, expected periods of cycles_per_period reported
 * cycles, starting at the first cycle at or after the period begin. A period that ends after the
 * reference period or after the last cycle is not counted. An expected period is missed if no
 * test report (sorted by time) with detects(index) lies in [begin, end).
 */
CycleResult statusCycleWalk(const TimePeriodCollection& ref_periods,
                            const std::vector<boost::posix_time::ptime>& cycles,
                            const std::vector<boost::posix_time::ptime>& tst_times,
                            const TstCheck& detects,
                            unsigned int cycles_per_period);

} // namespace DetectionWalk

} // namespace EvaluationRequirement
