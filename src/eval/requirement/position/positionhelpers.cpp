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

#include "requirement/base/trackoffsethelpers.h"
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
PositionEvaluation evaluatePositions(const dbContent::TargetReport::IndexMap& tst_indexes,
                                     const TstPosFunc& tst_pos_of,
                                     const ReferenceLookup& ref,
                                     const ReportSettings& settings,
                                     const ReportCheckFunc& check)
{
    PositionEvaluation result;
    PositionCounts&    counts = result.counts;

    auto addDetail = [ & ] (const boost::posix_time::ptime& timestamp,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            const boost::optional<bool>& pos_inside,
                            const boost::optional<double>& value,
                            const boost::optional<bool>& check_passed,
                            const std::string& comment)
    {
        result.details.push_back({ timestamp, tst_pos, ref_pos, pos_inside, value, check_passed, comment });
    };

    auto addNoDataDetail = [ & ] (const boost::posix_time::ptime& timestamp,
                                  const dbContent::TargetPosition& tst_pos,
                                  const boost::optional<dbContent::TargetPosition>& ref_pos,
                                  const boost::optional<bool>& pos_inside,
                                  const std::string& comment)
    {
        if (!settings.skip_no_data_details)
            addDetail(timestamp, tst_pos, ref_pos, pos_inside, {}, {}, comment);
    };

    for (const auto& tst_index : tst_indexes)
    {
        const dbContent::TargetReport::DataID tst_id(tst_index);
        const auto& timestamp = tst_index.first;

        ++counts.num_pos;

        const dbContent::TargetPosition tst_pos = tst_pos_of(tst_id);

        if (!ref.has_ref_data(tst_id))
        {
            addNoDataDetail(timestamp, tst_pos, {}, {}, "No reference data");
            ++counts.num_no_ref;
            continue;
        }

        const auto ref_pos = ref.ref_pos(tst_id);

        if (!ref_pos.has_value())
        {
            addNoDataDetail(timestamp, tst_pos, {}, {}, "No reference position");
            ++counts.num_no_ref;
            continue;
        }

        boost::optional<dbContent::TargetVelocity> ref_spd;

        if (ref.ref_speed)
        {
            ref_spd = ref.ref_speed(tst_id);

            if (!ref_spd.has_value())
            {
                addNoDataDetail(timestamp, tst_pos, ref_pos, {}, "No reference speed");
                ++counts.num_no_ref;
                continue;
            }
        }

        if (!ref.ref_pos_inside(tst_id))
        {
            addNoDataDetail(timestamp, tst_pos, ref_pos, false, "Outside sector");
            ++counts.num_pos_outside;
            continue;
        }
        ++counts.num_pos_inside;

        const auto ref_pos_acc = ref.ref_min_acc(tst_id);

        if (ref_pos_acc && *ref_pos_acc > settings.ref_min_accuracy)
        {
            addNoDataDetail(timestamp, tst_pos, ref_pos, true, "Inaccurate reference position");

            ++counts.num_ref_inaccurate;
            continue;
        }

        const ReportCheck report_check = check({ timestamp, tst_pos, *ref_pos, ref_spd });

        switch (report_check.outcome)
        {
            case ReportCheck::Outcome::Passed:
            case ReportCheck::Outcome::Failed:
            {
                const bool passed = report_check.outcome == ReportCheck::Outcome::Passed;

                if (passed)
                    ++counts.num_comp_passed;
                else
                    ++counts.num_comp_failed;

                addDetail(timestamp, tst_pos, ref_pos, true, report_check.value, passed, report_check.comment);
                break;
            }
            case ReportCheck::Outcome::Invalid:
                addDetail(timestamp, tst_pos, ref_pos, true, {}, {}, report_check.comment);
                ++counts.num_pos_calc_errors;
                break;
            case ReportCheck::Outcome::Deferred:
                break;
        }
    }

    traced_assert(counts.num_pos - counts.num_no_ref == counts.num_pos_inside + counts.num_pos_outside);

    return result;
}

namespace
{

ReportCheck compared(bool passed, double value, const std::string& comment)
{
    return { passed ? ReportCheck::Outcome::Passed : ReportCheck::Outcome::Failed, value, comment };
}

ReportCheck invalid(const std::string& comment)
{
    return { ReportCheck::Outcome::Invalid, 0.0, comment };
}

/// distance from the reference to the test position, none if not finite
boost::optional<double> refDistance(Transformation& trafo, const UsableReport& report)
{
    bool   transform_ok;
    double distance;

    std::tie(transform_ok, distance) = trafo.distanceL2Cart(
                report.ref_pos.latitude_, report.ref_pos.longitude_, report.tst_pos.latitude_, report.tst_pos.longitude_);
    traced_assert(transform_ok);

    if (std::isnan(distance) || std::isinf(distance))
        return boost::none;

    return distance;
}

} // namespace

/**
 */
PositionEvaluation evaluateDistances(const dbContent::TargetReport::IndexMap& tst_indexes,
                                     const TstPosFunc& tst_pos,
                                     const ReferenceLookup& ref,
                                     const ReportSettings& settings,
                                     const DistanceSettings& distance_settings)
{
    Transformation trafo;

    // averaged position mode: usable samples with their error vector in the local
    // Cartesian frame, compared per averaging window after the walk
    struct AveragingSample
    {
        ErrorSample               error;
        dbContent::TargetPosition tst_pos;
        dbContent::TargetPosition ref_pos;
    };
    std::vector<AveragingSample> samples;

    auto check = [ & ] (const UsableReport& report) -> ReportCheck
    {
        if (distance_settings.use_averaging)
        {
            bool   transform_ok;
            double sample_distance, sample_angle;

            std::tie(transform_ok, sample_distance, sample_angle) = trafo.distanceAngleCart(
                        report.ref_pos.latitude_, report.ref_pos.longitude_,
                        report.tst_pos.latitude_, report.tst_pos.longitude_);
            traced_assert(transform_ok);

            if (std::isnan(sample_distance) || std::isinf(sample_distance)
                    || std::isnan(sample_angle) || std::isinf(sample_angle))
                return invalid("Distance Invalid");

            samples.push_back({ { report.timestamp,
                                  sample_distance * std::cos(sample_angle),
                                  sample_distance * std::sin(sample_angle) },
                                report.tst_pos, report.ref_pos });

            return { ReportCheck::Outcome::Deferred, 0.0, "" };
        }

        const auto d = refDistance(trafo, report);

        if (!d)
            return invalid("Distance Invalid");

        const bool passed = compareValue(std::fabs(*d), distance_settings.threshold, distance_settings.check_type);

        return compared(passed, *d, passed ? "Passed" : "Failed");
    };

    PositionEvaluation result = evaluatePositions(tst_indexes, tst_pos, ref, settings, check);

    if (!distance_settings.use_averaging)
        return result;

    // one comparison per averaging window, on the mean position error of the test reports
    // inside it. A window is anchored at the first sample not yet consumed, so a data gap
    // does not create empty windows.
    std::vector<ErrorSample> error_samples;
    error_samples.reserve(samples.size());

    for (const auto& sample : samples)
        error_samples.push_back(sample.error);

    for (const auto& window : averagingWindows(error_samples, distance_settings.averaging_window_s))
    {
        const size_t num_in_window = window.end - window.first;
        traced_assert(num_in_window > 0);

        const bool passed = compareValue(window.mean_distance, distance_settings.threshold, distance_settings.check_type);

        if (passed)
            ++result.counts.num_comp_passed;
        else
            ++result.counts.num_comp_failed;

        const std::string comment = std::string(passed ? "Passed" : "Failed")
                + " (mean position error of " + std::to_string(num_in_window) + " reports over "
                + Utils::String::doubleToStringPrecision(window.span_s, 1) + " s)";

        const auto& first = samples[ window.first ];

        result.details.push_back({ first.error.timestamp, first.tst_pos, first.ref_pos,
                                   true, window.mean_distance, passed, comment });
    }

    return result;
}

/**
 */
PositionEvaluation evaluateDistancesRMS(const dbContent::TargetReport::IndexMap& tst_indexes,
                                        const TstPosFunc& tst_pos,
                                        const ReferenceLookup& ref,
                                        const ReportSettings& settings,
                                        double threshold)
{
    Transformation trafo;

    auto check = [ & ] (const UsableReport& report) -> ReportCheck
    {
        const auto d = refDistance(trafo, report);

        if (!d)
            return invalid("DistanceRMS Invalid");

        const bool passed = std::fabs(*d) <= threshold;

        return compared(passed, *d, passed ? "Passed" : "Failed");
    };

    return evaluatePositions(tst_indexes, tst_pos, ref, settings, check);
}

/**
 */
PositionEvaluation evaluateTrackOffsets(const dbContent::TargetReport::IndexMap& tst_indexes,
                                        const TstPosFunc& tst_pos,
                                        const ReferenceLookup& ref,
                                        const ReportSettings& settings,
                                        TrackOffsetComponent component,
                                        double max_abs_value)
{
    traced_assert(ref.ref_speed);

    const bool along = component == TrackOffsetComponent::Along;

    Transformation trafo;

    auto check = [ & ] (const UsableReport& report) -> ReportCheck
    {
        bool   transform_ok;
        double distance, angle;

        std::tie(transform_ok, distance, angle) = trafo.distanceAngleCart(
                    report.ref_pos.latitude_, report.ref_pos.longitude_,
                    report.tst_pos.latitude_, report.tst_pos.longitude_);
        traced_assert(transform_ok);

        if (std::isnan(distance) || std::isinf(distance))
            return invalid("Distance Invalid");

        if (!std::isfinite(angle) || !std::isfinite(report.ref_spd->track_angle_))
            return invalid("Angle Invalid");

        const auto offset = TrackOffsetHelpers::trackOffset(distance, angle, report.ref_spd->track_angle_);
        const double value = along ? offset.along : offset.across;
        traced_assert(std::isfinite(value));

        const bool passed = std::fabs(value) <= max_abs_value;

        return compared(passed, value, passed ? "" : (along ? "Along-track not OK" : "Across-track not OK"));
    };

    return evaluatePositions(tst_indexes, tst_pos, ref, settings, check);
}

} // namespace PositionHelpers
} // namespace EvaluationRequirement
