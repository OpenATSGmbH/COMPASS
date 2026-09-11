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

#include "eval/requirement/position/distance.h"
#include "eval/results/position/distance.h"
#include "eval/standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"
#include "util/timeconv.h"
#include "sectorlayer.h"
#include "stringconv.h"

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

PositionDistance::PositionDistance(
        const std::string& name, const std::string& short_name, const std::string& group_name,
            double prob, COMPARISON_TYPE prob_check_type, float ref_min_accuracy, EvaluationCalculator& calculator,
            float threshold_value, COMPARISON_TYPE threshold_value_check_type,
            bool failed_values_of_interest,
            bool use_averaging, float averaging_window_s)
    : PositionBaseProb(name, short_name, group_name, prob, prob_check_type, ref_min_accuracy, calculator),
      threshold_value_(threshold_value), threshold_value_check_type_(threshold_value_check_type),
      failed_values_of_interest_(failed_values_of_interest),
      use_averaging_(use_averaging), averaging_window_s_(averaging_window_s)
{
}

float PositionDistance::thresholdValue() const
{
    return threshold_value_;
}

COMPARISON_TYPE PositionDistance::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

bool PositionDistance::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

bool PositionDistance::useAveraging() const
{
    return use_averaging_;
}

float PositionDistance::averagingWindow() const
{
    return averaging_window_s_;
}

std::shared_ptr<EvaluationRequirementResult::Single> PositionDistance::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " threshold_value " << threshold_value_ << " threshold_value_check_type " << threshold_value_check_type_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    logdbg << "'" << name_ << "': utn " << target_data.utn_ << " test data size " << tst_data.size();

    unsigned int num_pos {0};
    unsigned int num_no_ref {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_ref_inaccurate {0};
    unsigned int num_pos_calc_errors {0};
    unsigned int num_comp_failed {0};
    unsigned int num_comp_passed {0};

    typedef EvaluationRequirementResult::SinglePositionDistance Result;
    typedef EvaluationDetail                                    Detail;
    typedef Result::EvaluationDetails                           Details;
    Details details;

    ptime timestamp;

    Transformation ogr_geo2cart;

    dbContent::TargetPosition tst_pos;

    bool is_inside;
    boost::optional<dbContent::TargetPosition> ref_pos;

    bool comp_passed;

    unsigned int num_distances {0};
    string comment;

    // averaged position mode: usable samples with their error vector in the local
    // Cartesian frame, compared per averaging window after the walk below
    struct AveragingSample
    {
        ptime                     timestamp;
        dbContent::TargetPosition tst_pos;
        dbContent::TargetPosition ref_pos;
        double                    dx {0};
        double                    dy {0};
    };
    std::vector<AveragingSample> samples;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            const QVariant& pos_inside,
                            const QVariant& offset,
                            const QVariant& check_passed,
                            const QVariant& num_pos,
                            const QVariant& num_no_ref,
                            const QVariant& num_pos_inside,
                            const QVariant& num_pos_outside,
                            const QVariant& num_comp_passed,
                            const QVariant& num_comp_failed,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                                             .setValue(Result::DetailKey::Value, offset)
                                             .setValue(Result::DetailKey::CheckPassed, check_passed)
                                             .addPosition(ref_pos)
                                             .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        ++num_pos;

        timestamp = tst_id.first;
        tst_pos = target_data.tstChain().pos(tst_id);

        comp_passed = false;

        if (!target_data.hasMappedRefData(tst_id, max_ref_time_diff))
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            {}, // ref_pos
                            {}, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_passed, num_comp_failed,
                            "No reference data");

            ++num_no_ref;
            continue;
        }

        ref_pos = target_data.mappedRefPos(tst_id, max_ref_time_diff);

        if (!ref_pos.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            {}, // ref_pos
                            {}, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_passed, num_comp_failed,
                            "No reference position");

            ++num_no_ref;
            continue;
        }

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.mappedRefPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            ref_pos, // ref_pos
                            is_inside, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_passed, num_comp_failed, 
                            "Outside sector");
            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        auto ref_pos_acc = target_data.mappedRefMinAcc(tst_id, max_ref_time_diff, false); // max std dev

        logdbg << "utn " << target_data.utn_ << " ref_pos_acc " << (bool) ref_pos_acc << " acc " << (ref_pos_acc ? *ref_pos_acc : 666.0) << " ref_min_accuracy " << ref_min_accuracy_;

        if (ref_pos_acc && *ref_pos_acc > ref_min_accuracy_)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            {}, // ref_pos
                            {}, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_passed, num_comp_failed,
                            "Inaccurate reference position");

            ++num_ref_inaccurate;

            continue;            
        }

        bool   transform_ok;
        double distance;

        if (use_averaging_)
        {
            // collect the error vector, one comparison per averaging window follows below
            double sample_distance, sample_angle;

            std::tie(transform_ok, sample_distance, sample_angle) = ogr_geo2cart.distanceAngleCart(
                        ref_pos->latitude_, ref_pos->longitude_, tst_pos.latitude_, tst_pos.longitude_);
            traced_assert(transform_ok);

            if (std::isnan(sample_distance) || std::isinf(sample_distance)
                    || std::isnan(sample_angle) || std::isinf(sample_angle))
            {
                addDetail(timestamp, tst_pos,
                            ref_pos, // ref_pos
                            is_inside, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_passed, num_comp_failed,
                            "Distance Invalid");
                ++num_pos_calc_errors;
                continue;
            }

            samples.push_back({timestamp, tst_pos, *ref_pos,
                               sample_distance * std::cos(sample_angle),
                               sample_distance * std::sin(sample_angle)});
            continue;
        }

        std::tie(transform_ok, distance) = ogr_geo2cart.distanceL2Cart(ref_pos->latitude_, ref_pos->longitude_, tst_pos.latitude_, tst_pos.longitude_);
        traced_assert(transform_ok);

        if (std::isnan(distance) || std::isinf(distance))
        {
            addDetail(timestamp, tst_pos,
                        ref_pos, // ref_pos
                        is_inside, {}, comp_passed, // pos_inside, value, check_passed
                        num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                        num_comp_passed, num_comp_failed, 
                        "Distance Invalid");
            ++num_pos_calc_errors;
            continue;
        }

        ++num_distances;

        if (compareValue(fabs(distance), threshold_value_, threshold_value_check_type_))
        {
            comp_passed = true;
            ++num_comp_passed;
            comment = "Passed";
        }
        else
        {
            ++num_comp_failed;
            comment = "Failed";
        }

        addDetail(timestamp, tst_pos,
                    ref_pos,
                    is_inside, distance, comp_passed, // pos_inside, value, check_passed
                    num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                    num_comp_passed, num_comp_failed,
                    comment);
    }

    // averaged position mode: one comparison per averaging window, on the mean
    // position error of the test reports inside it. A window is anchored at the
    // first sample not yet consumed, so a data gap does not create empty windows.
    if (use_averaging_)
    {
        const time_duration averaging_window = Time::partialSeconds(averaging_window_s_);

        size_t sample_idx = 0;

        while (sample_idx < samples.size())
        {
            const ptime window_begin = samples[sample_idx].timestamp;
            const ptime window_end   = window_begin + averaging_window;

            double sum_dx {0}, sum_dy {0};
            size_t next_idx = sample_idx;

            while (next_idx < samples.size() && samples[next_idx].timestamp < window_end)
            {
                sum_dx += samples[next_idx].dx;
                sum_dy += samples[next_idx].dy;
                ++next_idx;
            }

            const size_t num_in_window = next_idx - sample_idx;
            traced_assert(num_in_window > 0);

            const double mean_dx = sum_dx / (double) num_in_window;
            const double mean_dy = sum_dy / (double) num_in_window;
            const double mean_distance = std::sqrt(mean_dx * mean_dx + mean_dy * mean_dy);

            ++num_distances;

            const bool window_passed = compareValue(mean_distance, threshold_value_, threshold_value_check_type_);

            if (window_passed)
            {
                ++num_comp_passed;
                comment = "Passed";
            }
            else
            {
                ++num_comp_failed;
                comment = "Failed";
            }

            comment += " (mean position error of " + to_string(num_in_window) + " reports over "
                    + String::doubleToStringPrecision(
                           Time::partialSeconds(samples[next_idx - 1].timestamp - window_begin), 1) + " s)";

            addDetail(window_begin, samples[sample_idx].tst_pos,
                        samples[sample_idx].ref_pos,
                        true, mean_distance, window_passed, // pos_inside, value, check_passed
                        num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                        num_comp_passed, num_comp_failed,
                        comment);

            sample_idx = next_idx;
        }
    }

    //        logdbg << "'" << name_ << "': utn " << target_data.utn_
    //               << " num_pos " << num_pos << " num_no_ref " <<  num_no_ref
    //               << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside
    //               << " num_pos_ok " << num_pos_ok << " num_pos_nok " << num_pos_nok
    //               << " num_distances " << num_distances;

    traced_assert(num_no_ref <= num_pos);

    if (num_pos - num_no_ref != num_pos_inside + num_pos_outside)
        loginf << "'" << name_ << "': utn " << target_data.utn_
               << " num_pos " << num_pos << " num_no_ref " <<  num_no_ref
               << " num_pos_outside " << num_pos_outside 
               << " num_pos_inside " << num_pos_inside
               << " num_pos_calc_errors " << num_pos_calc_errors
               << " num_distances " << num_distances;

    traced_assert(num_pos - num_no_ref == num_pos_inside + num_pos_outside);

    traced_assert(num_distances == num_comp_failed + num_comp_passed);

    //assert (details.size() == num_pos);

    return make_shared<EvaluationRequirementResult::SinglePositionDistance>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_ref_inaccurate,
                 num_comp_passed, num_comp_failed);
}

}
