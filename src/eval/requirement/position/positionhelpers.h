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
 * Reference data of a test report. evaluateDistances() calls the functions in this order and
 * stops at the first that rules the report out, so the later (expensive) lookups only run when
 * needed.
 */
struct ReferenceLookup
{
    std::function<bool(const dbContent::TargetReport::DataID&)>                               has_ref_data;
    std::function<boost::optional<dbContent::TargetPosition>(const dbContent::TargetReport::DataID&)> ref_pos;
    std::function<bool(const dbContent::TargetReport::DataID&)>                               ref_pos_inside;
    std::function<boost::optional<double>(const dbContent::TargetReport::DataID&)>            ref_min_acc;
};

struct DistanceSettings
{
    double          threshold          = 0.0;
    COMPARISON_TYPE check_type         = COMPARISON_TYPE::LESS_THAN_OR_EQUAL;
    double          ref_min_accuracy   = 0.0;  // max. reference std. dev. [m]
    bool            use_averaging      = false;
    double          averaging_window_s = 0.0;
    bool            skip_no_data_details = false;
};

struct DistanceCounts
{
    unsigned int num_pos             = 0;
    unsigned int num_no_ref          = 0;
    unsigned int num_pos_outside     = 0;
    unsigned int num_pos_inside      = 0; // includes the inaccurate reference and invalid distance reports
    unsigned int num_ref_inaccurate  = 0;
    unsigned int num_pos_calc_errors = 0;
    unsigned int num_comp_passed     = 0;
    unsigned int num_comp_failed     = 0;
};

struct DistanceDetail
{
    boost::posix_time::ptime                   timestamp;
    dbContent::TargetPosition                  tst_pos;
    boost::optional<dbContent::TargetPosition> ref_pos;
    boost::optional<bool>                      pos_inside;
    boost::optional<double>                    value;
    bool                                       check_passed = false;
    std::string                                comment;
};

struct DistanceEvaluation
{
    DistanceCounts              counts;
    std::vector<DistanceDetail> details;
};

/**
 * Position Distance of the test reports: per report the distance to the mapped reference
 * position, compared with the threshold. With averaging, one comparison per averaging window
 * on the mean position error (see averagingWindows()).
 */
DistanceEvaluation evaluateDistances(
        const dbContent::TargetReport::IndexMap& tst_indexes,
        const std::function<dbContent::TargetPosition(const dbContent::TargetReport::DataID&)>& tst_pos,
        const ReferenceLookup& ref,
        const DistanceSettings& settings);

} // namespace PositionHelpers

} // namespace EvaluationRequirement
