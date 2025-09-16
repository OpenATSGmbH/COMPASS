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

#include "eval/requirement/speed/speed.h"
#include "eval/results/speed/speed.h"
#include "eval/standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"

#include "util/timeconv.h"
#include "sectorlayer.h"

#include <ogr_spatialref.h>

#include <algorithm>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

Speed::Speed(
        const std::string& name, const std::string& short_name, const std::string& group_name,
        double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
        float threshold_value, bool use_percent_if_higher, float threshold_percent,
        COMPARISON_TYPE threshold_value_check_type, bool failed_values_of_interest)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      threshold_value_(threshold_value),
      use_percent_if_higher_(use_percent_if_higher), threshold_percent_(threshold_percent),
      threshold_value_check_type_(threshold_value_check_type),
      failed_values_of_interest_(failed_values_of_interest)
{
}

float Speed::thresholdValue() const
{
    return threshold_value_;
}

bool Speed::usePercentIfHigher() const
{
    return use_percent_if_higher_;
}

float Speed::thresholdPercent() const
{
    return threshold_percent_;
}

COMPARISON_TYPE Speed::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

bool Speed::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

std::shared_ptr<EvaluationRequirementResult::Single> Speed::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " threshold_value " << threshold_value_ << " threshold_value_check_type " << threshold_value_check_type_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    unsigned int num_pos {0};
    unsigned int num_no_ref {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_no_tst_value {0};
    unsigned int num_comp_failed {0};
    unsigned int num_comp_passed {0};

    float tmp_threshold_value;

    typedef EvaluationRequirementResult::SingleSpeed Result;
    typedef EvaluationDetail                         Detail;
    typedef Result::EvaluationDetails                Details;
    Details details;

    ptime timestamp;

    dbContent::TargetPosition tst_pos;

    bool is_inside;
    boost::optional<dbContent::TargetPosition> ref_pos;

    boost::optional<dbContent::TargetVelocity> ref_spd;
    boost::optional<float> tst_spd_ms;
    float spd_diff;

    bool comp_passed;

    unsigned int num_speeds {0};
    string comment;

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
                            const QVariant& num_comp_failed,
                            const QVariant& num_comp_passed,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::PosInside, pos_inside.isValid() ? pos_inside : "false")
                                             .setValue(Result::DetailKey::Offset, offset)
                                             .setValue(Result::DetailKey::CheckPassed, check_passed)
                                             .setValue(Result::DetailKey::NumPos, num_pos)
                                             .setValue(Result::DetailKey::NumNoRef, num_no_ref)
                                             .setValue(Result::DetailKey::NumInside, num_pos_inside)
                                             .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                                             .setValue(Result::DetailKey::NumCheckFailed, num_comp_failed)
                                             .setValue(Result::DetailKey::NumCheckPassed, num_comp_passed)
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
                            num_comp_failed, num_comp_passed,
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
                            num_comp_failed, num_comp_passed,
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
                            num_comp_failed, num_comp_passed,
                            "Outside sector");
            ++num_pos_outside;
            continue;
        }

        ref_spd = target_data.mappedRefSpeed(tst_id, max_ref_time_diff);

        if (!ref_spd.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            {}, // ref_pos
                            {}, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_failed, num_comp_passed,
                            "No reference speed");

            ++num_no_ref;
            continue;
        }

        ++num_pos_inside;

        // ref_spd ok
        tst_spd_ms = target_data.tstChain().groundSpeed(tst_id);

        if (!tst_spd_ms.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                            {}, // ref_pos
                            {}, {}, comp_passed, // pos_inside, value, check_passed
                            num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                            num_comp_failed, num_comp_passed,
                            "No tst speed");

            ++num_no_tst_value;
            continue;
        }

        spd_diff = fabs(ref_spd->speed_ - *tst_spd_ms);

        if (use_percent_if_higher_ && *tst_spd_ms * threshold_percent_ > threshold_value_) // use percent based threshold
            tmp_threshold_value = *tst_spd_ms * threshold_percent_;
        else
            tmp_threshold_value = threshold_value_;

        ++num_speeds;

        if (compareValue(spd_diff, tmp_threshold_value, threshold_value_check_type_))
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
                    is_inside, spd_diff, comp_passed, // pos_inside, value, check_passed
                    num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                    num_comp_failed, num_comp_passed,
                    comment);
    }

    //        logdbg << "" << name_ << "': utn " << target_data.utn_
    //               << " num_pos " << num_pos << " num_no_ref " <<  num_no_ref
    //               << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside
    //               << " num_pos_ok " << num_pos_ok << " num_pos_nok " << num_pos_nok
    //               << " num_speeds " << num_speeds;

    traced_assert(num_no_ref <= num_pos);

    if (num_pos - num_no_ref != num_pos_inside + num_pos_outside)
        logwrn << "'" << name_ << "': utn " << target_data.utn_
               << " num_pos " << num_pos << " num_no_ref " <<  num_no_ref
               << " num_pos_outside " << num_pos_outside << " num_pos_inside " << num_pos_inside;
    traced_assert(num_pos - num_no_ref == num_pos_inside + num_pos_outside);


    if (num_speeds != num_comp_failed + num_comp_passed)
        logwrn << "'" << name_ << "': utn " << target_data.utn_
               << " num_speeds " << num_speeds << " num_comp_failed " <<  num_comp_failed
               << " num_comp_passed " << num_comp_passed;

    traced_assert(num_speeds == num_comp_failed + num_comp_passed);

    return make_shared<EvaluationRequirementResult::SingleSpeed>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_no_tst_value,
                num_comp_failed, num_comp_passed);
}

}
