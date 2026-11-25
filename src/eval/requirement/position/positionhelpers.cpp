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

#include "requirement/position/positionhelpers.h"

#include "projection/transformation.h"
#include "stringconv.h"
#include "timeconv.h"
#include "traced_assert.h"

#include <cmath>
#include <tuple>

namespace EvaluationRequirement
{
namespace PositionHelpers
{

/**
 */
std::vector<AveragingWindow> averagingWindows(const std::vector<ErrorSample>& samples, double window_s)
{
    std::vector<AveragingWindow> windows;

    const auto window_length = Utils::Time::partialSeconds(window_s);

    size_t sample_idx = 0;

    while (sample_idx < samples.size())
    {
        const auto window_begin = samples[ sample_idx ].timestamp;
        const auto window_end   = window_begin + window_length;

        double sum_dx = 0.0, sum_dy = 0.0;
        size_t next_idx = sample_idx;

        while (next_idx < samples.size() && samples[ next_idx ].timestamp < window_end)
        {
            sum_dx += samples[ next_idx ].dx;
            sum_dy += samples[ next_idx ].dy;
            ++next_idx;
        }

        AveragingWindow w;
        w.first = sample_idx;
        w.end   = next_idx;

        const double num = (double)(next_idx - sample_idx);
        const double mean_dx = sum_dx / num;
        const double mean_dy = sum_dy / num;

        w.mean_distance = std::sqrt(mean_dx * mean_dx + mean_dy * mean_dy);
        w.span_s        = Utils::Time::partialSeconds(samples[ next_idx - 1 ].timestamp - window_begin);

        windows.push_back(w);

        sample_idx = next_idx;
    }

    return windows;
}

/**
 */
DistanceEvaluation evaluateDistances(const dbContent::TargetReport::IndexMap& tst_indexes,
                                     const std::function<dbContent::TargetPosition(const dbContent::TargetReport::DataID&)>& tst_pos_of,
                                     const ReferenceLookup& ref,
                                     const DistanceSettings& settings)
{
    DistanceEvaluation result;
    DistanceCounts&    counts = result.counts;

    Transformation ogr_geo2cart;

    // averaged position mode: usable samples with their error vector in the local
    // Cartesian frame, compared per averaging window after the walk below
    struct AveragingSample
    {
        ErrorSample               error;
        dbContent::TargetPosition tst_pos;
        dbContent::TargetPosition ref_pos;
    };
    std::vector<AveragingSample> samples;

    auto addDetail = [ & ] (const boost::posix_time::ptime& timestamp,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            const boost::optional<bool>& pos_inside,
                            const boost::optional<double>& value,
                            bool check_passed,
                            const std::string& comment)
    {
        result.details.push_back({ timestamp, tst_pos, ref_pos, pos_inside, value, check_passed, comment });
    };

    for (const auto& tst_index : tst_indexes)
    {
        const dbContent::TargetReport::DataID tst_id(tst_index);
        const auto& timestamp = tst_index.first;

        ++counts.num_pos;

        const dbContent::TargetPosition tst_pos = tst_pos_of(tst_id);

        if (!ref.has_ref_data(tst_id))
        {
            if (!settings.skip_no_data_details)
                addDetail(timestamp, tst_pos, {}, {}, {}, false, "No reference data");

            ++counts.num_no_ref;
            continue;
        }

        const auto ref_pos = ref.ref_pos(tst_id);

        if (!ref_pos.has_value())
        {
            if (!settings.skip_no_data_details)
                addDetail(timestamp, tst_pos, {}, {}, {}, false, "No reference position");

            ++counts.num_no_ref;
            continue;
        }

        if (!ref.ref_pos_inside(tst_id))
        {
            if (!settings.skip_no_data_details)
                addDetail(timestamp, tst_pos, ref_pos, false, {}, false, "Outside sector");

            ++counts.num_pos_outside;
            continue;
        }
        ++counts.num_pos_inside;

        const auto ref_pos_acc = ref.ref_min_acc(tst_id);

        if (ref_pos_acc && *ref_pos_acc > settings.ref_min_accuracy)
        {
            if (!settings.skip_no_data_details)
                addDetail(timestamp, tst_pos, {}, {}, {}, false, "Inaccurate reference position");

            ++counts.num_ref_inaccurate;
            continue;
        }

        bool transform_ok;

        if (settings.use_averaging)
        {
            // collect the error vector, one comparison per averaging window follows below
            double sample_distance, sample_angle;

            std::tie(transform_ok, sample_distance, sample_angle) = ogr_geo2cart.distanceAngleCart(
                        ref_pos->latitude_, ref_pos->longitude_, tst_pos.latitude_, tst_pos.longitude_);
            traced_assert(transform_ok);

            if (std::isnan(sample_distance) || std::isinf(sample_distance)
                    || std::isnan(sample_angle) || std::isinf(sample_angle))
            {
                addDetail(timestamp, tst_pos, ref_pos, true, {}, false, "Distance Invalid");
                ++counts.num_pos_calc_errors;
                continue;
            }

            samples.push_back({ { timestamp,
                                  sample_distance * std::cos(sample_angle),
                                  sample_distance * std::sin(sample_angle) },
                                tst_pos, *ref_pos });
            continue;
        }

        double distance;

        std::tie(transform_ok, distance) = ogr_geo2cart.distanceL2Cart(
                    ref_pos->latitude_, ref_pos->longitude_, tst_pos.latitude_, tst_pos.longitude_);
        traced_assert(transform_ok);

        if (std::isnan(distance) || std::isinf(distance))
        {
            addDetail(timestamp, tst_pos, ref_pos, true, {}, false, "Distance Invalid");
            ++counts.num_pos_calc_errors;
            continue;
        }

        const bool passed = compareValue(std::fabs(distance), settings.threshold, settings.check_type);

        if (passed)
            ++counts.num_comp_passed;
        else
            ++counts.num_comp_failed;

        addDetail(timestamp, tst_pos, ref_pos, true, distance, passed, passed ? "Passed" : "Failed");
    }

    // averaged position mode: one comparison per averaging window, on the mean
    // position error of the test reports inside it. A window is anchored at the
    // first sample not yet consumed, so a data gap does not create empty windows.
    if (settings.use_averaging)
    {
        std::vector<ErrorSample> error_samples;
        error_samples.reserve(samples.size());

        for (const auto& sample : samples)
            error_samples.push_back(sample.error);

        for (const auto& window : averagingWindows(error_samples, settings.averaging_window_s))
        {
            const size_t num_in_window = window.end - window.first;
            traced_assert(num_in_window > 0);

            const bool passed = compareValue(window.mean_distance, settings.threshold, settings.check_type);

            if (passed)
                ++counts.num_comp_passed;
            else
                ++counts.num_comp_failed;

            const std::string comment = std::string(passed ? "Passed" : "Failed")
                    + " (mean position error of " + std::to_string(num_in_window) + " reports over "
                    + Utils::String::doubleToStringPrecision(window.span_s, 1) + " s)";

            const auto& first = samples[ window.first ];

            addDetail(first.error.timestamp, first.tst_pos, first.ref_pos,
                      true, window.mean_distance, passed, comment);
        }
    }

    traced_assert(counts.num_pos - counts.num_no_ref == counts.num_pos_inside + counts.num_pos_outside);

    return result;
}

} // namespace PositionHelpers
} // namespace EvaluationRequirement
