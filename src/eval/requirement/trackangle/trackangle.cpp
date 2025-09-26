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

#include "eval/requirement/trackangle/trackangle.h"
#include "eval/results/trackangle/trackangle.h"
#include "eval/standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"

#include "util/timeconv.h"
#include "sectorlayer.h"
#include "util/number.h"
#include "global.h"

#include <ogr_spatialref.h>

#include <algorithm>
#include <cmath>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

TrackAngle::TrackAngle(
        const std::string& name, const std::string& short_name, const std::string& group_name,
        double prob, COMPARISON_TYPE prob_check_type, EvaluationCalculator& calculator,
                    float threshold, bool use_minimum_speed, float minimum_speed,
        COMPARISON_TYPE threshold_value_check_type, bool failed_values_of_interest)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      threshold_(threshold),
      use_minimum_speed_(use_minimum_speed), minimum_speed_(minimum_speed),
      threshold_value_check_type_(threshold_value_check_type),
      failed_values_of_interest_(failed_values_of_interest)
{
}

float TrackAngle::threshold() const
{
    return threshold_;
}

bool TrackAngle::useMinimumSpeed() const
{
    return use_minimum_speed_;
}

float TrackAngle::minimumSpeed() const
{
    return minimum_speed_;
}

COMPARISON_TYPE TrackAngle::thresholdValueCheckType() const
{
    return threshold_value_check_type_;
}

bool TrackAngle::failedValuesOfInterest() const
{
    return failed_values_of_interest_;
}

std::shared_ptr<EvaluationRequirementResult::Single> TrackAngle::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " threshold_percent " << threshold_
           << " threshold_value_check_type " << threshold_value_check_type_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    unsigned int num_pos {0};
    unsigned int num_no_ref {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_pos_ref_spd_low {0};
    unsigned int num_no_tst_value {0};
    unsigned int num_comp_failed {0};
    unsigned int num_comp_passed {0};

    typedef EvaluationRequirementResult::SingleTrackAngle Result;
    typedef EvaluationDetail                              Detail;
    typedef Result::EvaluationDetails                     Details;
    Details details;

    ptime timestamp;

    dbContent::TargetPosition tst_pos;

    bool is_inside;
    boost::optional<dbContent::TargetPosition> ref_pos;

    boost::optional<dbContent::TargetVelocity> ref_spd;
    double ref_trackangle_deg;
    boost::optional<double> tst_trackangle_deg;
    float trackangle_min_diff;

    bool comp_passed;

    unsigned int num_trackangle_comp {0};
    string comment;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            //const std::vector<EvaluationDetail::Line>& lines,
                            const QVariant& pos_inside,
                            const QVariant& offset,
                            const QVariant& check_passed,
                            const QVariant& value_ref,
                            const QVariant& value_tst,
                            const QVariant& speed_ref,
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
                                             .setValue(Result::DetailKey::ValueRef, value_ref)
                                             .setValue(Result::DetailKey::ValueTst, value_tst)
                                             .setValue(Result::DetailKey::SpeedRef, speed_ref)
                                             .setValue(Result::DetailKey::NumPos, num_pos)
                                             .setValue(Result::DetailKey::NumNoRef, num_no_ref)
                                             .setValue(Result::DetailKey::NumInside, num_pos_inside)
                                             .setValue(Result::DetailKey::NumOutside, num_pos_outside)
                                             .setValue(Result::DetailKey::NumCheckFailed, num_comp_failed)
                                             .setValue(Result::DetailKey::NumCheckPassed, num_comp_passed)
                                             .addPosition(ref_pos)
                                             //.addLines(lines)
                                             .generalComment(comment));
    };

//    QColor ref_line_color = QColor("#ffa500");
//    QColor tst_line_color = QColor("#00BBBB");

//    std::vector<EvaluationDetail::Line> lines;

    for (const auto& tst_id : tst_data)
    {
        ++num_pos;

        timestamp = tst_id.first;
        tst_pos = target_data.tstChain().pos(tst_id);

        comp_passed = false;

        //lines.clear();

        if (!target_data.hasMappedRefData(tst_id, max_ref_time_diff))
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                          {}, //lines, // ref_pos, lines
                          {}, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, {}, // value_ref, value_tst, speed_ref
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
                          {}, //lines, // ref_pos, lines
                          {}, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, {}, // value_ref, value_tst, speed_ref
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
                          {}, //lines, // ref_pos, lines
                          is_inside, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, {}, // value_ref, value_tst, speed_ref
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
                          {}, //lines, // ref_pos, lines
                          {}, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, {}, // value_ref, value_tst, speed_ref
                          num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                          num_comp_failed, num_comp_passed,
                          "No reference speed");

            ++num_no_ref;
            continue;
        }

        ++num_pos_inside;

        // ref_spd ok

        if (use_minimum_speed_ && ref_spd->speed_ < minimum_speed_)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                          {}, //lines, // ref_pos, lines
                          {}, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, ref_spd->speed_, // value_ref, value_tst, speed_ref
                          num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                          num_comp_failed, num_comp_passed,
                          "Reference speed too low");

            ++num_pos_ref_spd_low;

            //loginf << Time::toString(timestamp) << " ref spd low "  << ref_spd.speed_;

            continue;
        }

        ref_trackangle_deg = ref_spd->track_angle_;
        tst_trackangle_deg = target_data.tstChain().trackAngle(tst_id);

        if (!tst_trackangle_deg.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos,
                          {}, //lines, // ref_pos, lines
                          {}, {}, comp_passed, // pos_inside, value, check_passed
                          {}, {}, ref_spd->speed_, // value_ref, value_tst, speed_ref
                          num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                          num_comp_failed, num_comp_passed,
                          "No tst speed");

            ++num_no_tst_value;
            continue;
        }

        trackangle_min_diff = Number::calculateMinAngleDifference(ref_trackangle_deg, *tst_trackangle_deg);

        // tst vector
        //lines.emplace_back(tst_pos, getPositionAtAngle(tst_pos, tst_trackangle_deg, ref_spd.speed_), tst_line_color);

        // ref vector
        //lines.emplace_back(tst_pos, getPositionAtAngle(tst_pos, ref_trackangle_deg, ref_spd.speed_), ref_line_color);

//        loginf << Time::toString(timestamp) << " tst_track ref " << ref_trackangle_deg
//               << " tst " << tst_trackangle_deg << " diff " << trackangle_min_diff;

        ++num_trackangle_comp;

        if (compareValue(fabs(trackangle_min_diff), threshold_, threshold_value_check_type_))
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

//        comment += " tst_track ref " +to_string(ref_trackangle_deg)
//                + " tst " + to_string(tst_trackangle_deg) + " diff " + to_string(trackangle_min_diff);

        addDetail(timestamp, tst_pos,
                  ref_pos, //lines, // ref_pos, lines
                  is_inside, trackangle_min_diff, comp_passed, // pos_inside, value, check_passed
                  ref_trackangle_deg, *tst_trackangle_deg, ref_spd->speed_, // value_ref, value_tst, speed_ref
                  num_pos, num_no_ref, num_pos_inside, num_pos_outside,
                  num_comp_failed, num_comp_passed,
                  comment);
    }

    //        logdbg << "'" << name_ << "': utn " << target_data.utn_
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


    if (num_trackangle_comp != num_comp_failed + num_comp_passed)
        logwrn << "'" << name_ << "': utn " << target_data.utn_
               << " num_speeds " << num_trackangle_comp
               << " num_comp_failed " <<  num_comp_failed << " num_comp_passed " << num_comp_passed;

    traced_assert(num_trackangle_comp == num_comp_failed + num_comp_passed);

    return make_shared<EvaluationRequirementResult::SingleTrackAngle>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_no_tst_value,
                num_comp_failed, num_comp_passed);
}

// deg, m/s
dbContent::TargetPosition TrackAngle::getPositionAtAngle(
        const dbContent::TargetPosition& org, double track_angle, double speed)
{
    dbContent::TargetPosition new_pos;

    double track_angle_math =  DEG2RAD * (90 - track_angle);
    double x = speed * cos(track_angle_math); // 2sec
    double y = speed * sin(track_angle_math);

    bool ok;
    new_pos = org;

    tie (ok, new_pos.latitude_, new_pos.longitude_) = trafo_.wgsAddCartOffset(org.latitude_, org.longitude_, x, y);
    traced_assert(ok);

    return new_pos;
}

}
