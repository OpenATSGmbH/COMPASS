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

#include "requirement/base/comparisontype.h"
#include "targetposition.h"
#include "dbcontent/target/targetreportmapping.h"
#include "dbcontent/target/targetvelocity.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/optional.hpp>

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

/**
 * Reference data of a test report. evaluatePositions() calls the functions in this order and
 * stops at the first that rules the report out, so the later (expensive) lookups only run when
 * needed. ref_speed is only called if set.
 */
struct ReferenceLookup
{
    std::function<bool(const dbContent::TargetReport::DataID&)>                                        has_ref_data;
    std::function<boost::optional<dbContent::TargetPosition>(const dbContent::TargetReport::DataID&)> ref_pos;
    std::function<boost::optional<dbContent::TargetVelocity>(const dbContent::TargetReport::DataID&)> ref_speed;
    std::function<bool(const dbContent::TargetReport::DataID&)>                                        ref_pos_inside;
    std::function<boost::optional<double>(const dbContent::TargetReport::DataID&)>                     ref_min_acc;
};

struct ReportSettings
{
    double ref_min_accuracy     = 0.0; // max. reference std. dev. [m]
    bool   skip_no_data_details = false;
};

/// test report with a usable reference: inside the sector, reference accurate enough
struct UsableReport
{
    boost::posix_time::ptime                   timestamp;
    dbContent::TargetPosition                  tst_pos;
    dbContent::TargetPosition                  ref_pos;
    boost::optional<dbContent::TargetVelocity> ref_spd; // set if ReferenceLookup::ref_speed is set
};

/// check of a usable test report by the requirement
struct ReportCheck
{
    enum class Outcome
    {
        Passed,
        Failed,
        Invalid, // no value, e.g. a failed calculation (counted as calculation error)
        Deferred // compared later by the caller, no detail (averaging)
    };

    Outcome outcome = Outcome::Invalid;
    double  value   = 0.0;
};

struct PositionCounts
{
    unsigned int num_pos             = 0;
    unsigned int num_no_ref          = 0;
    unsigned int num_pos_outside     = 0;
    unsigned int num_pos_inside      = 0; // includes the inaccurate reference and invalid value reports
    unsigned int num_ref_inaccurate  = 0;
    unsigned int num_pos_calc_errors = 0;
    unsigned int num_comp_passed     = 0;
    unsigned int num_comp_failed     = 0;
};

struct PositionDetail
{
    boost::posix_time::ptime                   timestamp;
    dbContent::TargetPosition                  tst_pos;
    boost::optional<dbContent::TargetPosition> ref_pos;
    std::uint32_t                              code = 0; // reason bits, see DetailCode
    boost::optional<bool>                      pos_inside;
    boost::optional<double>                    value;
    boost::optional<bool>                      check_passed; // none without a comparison
};

struct PositionEvaluation
{
    PositionCounts              counts;
    std::vector<PositionDetail> details;
};

using TstPosFunc      = std::function<dbContent::TargetPosition(const dbContent::TargetReport::DataID&)>;
using ReportCheckFunc = std::function<ReportCheck(const UsableReport&)>;

/**
 * Walk over the test reports of the position requirements: per report the reference checks of
 * the lookup, then the check of the requirement for the usable reports. One detail per report,
 * in time order (except the deferred ones). A report without a comparison gets a detail without
 * a value and without check_passed, so it is not an issue, with the reference position if known,
 * and its reason as DetailCode bit.
 */
PositionEvaluation evaluatePositions(const dbContent::TargetReport::IndexMap& tst_indexes,
                                     const TstPosFunc& tst_pos,
                                     const ReferenceLookup& ref,
                                     const ReportSettings& settings,
                                     const ReportCheckFunc& check);

struct DistanceSettings
{
    double          threshold          = 0.0;
    COMPARISON_TYPE check_type         = COMPARISON_TYPE::LESS_THAN_OR_EQUAL;
    bool            use_averaging      = false;
    double          averaging_window_s = 0.0;
};

/**
 * Position Distance: per report the distance to the mapped reference position, compared with
 * the threshold. With averaging, one comparison per averaging window on the mean position error
 * (see averagingWindows()), its details after the ones of the reports.
 */
PositionEvaluation evaluateDistances(const dbContent::TargetReport::IndexMap& tst_indexes,
                                     const TstPosFunc& tst_pos,
                                     const ReferenceLookup& ref,
                                     const ReportSettings& settings,
                                     const DistanceSettings& distance_settings);

/// Position Distance RMS: per report the distance to the mapped reference position, passed if <= threshold
PositionEvaluation evaluateDistancesRMS(const dbContent::TargetReport::IndexMap& tst_indexes,
                                        const TstPosFunc& tst_pos,
                                        const ReferenceLookup& ref,
                                        const ReportSettings& settings,
                                        double threshold);

enum class TrackOffsetComponent { Along, Across };

/**
 * Position Along / Across: per report the offset along or across the reference track (needs
 * ReferenceLookup::ref_speed), passed if its absolute value is <= max_abs_value.
 */
PositionEvaluation evaluateTrackOffsets(const dbContent::TargetReport::IndexMap& tst_indexes,
                                        const TstPosFunc& tst_pos,
                                        const ReferenceLookup& ref,
                                        const ReportSettings& settings,
                                        TrackOffsetComponent component,
                                        double max_abs_value);

} // namespace PositionHelpers

} // namespace EvaluationRequirement
