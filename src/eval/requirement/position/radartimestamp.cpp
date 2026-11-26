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


#include "requirement/position/radartimestamp.h"
#include "requirement/base/trackoffsethelpers.h"
#include "results/position/radartimestamp.h"
#include "results/base/detailcode.h"
#include "standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"
#include "timeconv.h"
#include "sectorlayer.h"

#include <cmath>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

namespace DetailCode = EvaluationRequirementResult::DetailCode;

PositionRadarTimeStamp::PositionRadarTimeStamp(
        const std::string& name, const std::string& short_name, const std::string& group_name, float ref_min_accuracy,
        EvaluationCalculator& calculator, double threshold_value)
    : PositionBaseValue(name, short_name, group_name, threshold_value, COMPARISON_TYPE::LESS_THAN,
                        ref_min_accuracy, calculator)
{
}

std::shared_ptr<EvaluationRequirementResult::Single> PositionRadarTimeStamp::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_ << " threshold_value " << threshold();

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    unsigned int num_pos {0};
    unsigned int num_no_ref {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_ref_inaccurate {0};
    unsigned int num_pos_calc_errors {0};
    unsigned int num_comp_passed {0};

    typedef EvaluationRequirementResult::SinglePositionRadarTimeStamp Result;
    typedef EvaluationDetail                                          Detail;
    typedef Result::EvaluationDetails                                 Details;
    Details details;

    std::vector<double> along_values;
    std::vector<double> speed_values;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const boost::optional<dbContent::TargetPosition>& ref_pos,
                            const QVariant& pos_inside,
                            const QVariant& offset,
                            std::uint32_t code)
    {
        // the time stamp error is a statistic of all updates, single updates are not checked
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::PosInside, pos_inside)
                                             .setValue(Result::DetailKey::Value, offset)
                                             .setValue(Result::DetailKey::CheckPassed, true)
                                             .addPosition(ref_pos)
                                             .addCode(code));
    };

    Transformation ogr_geo2cart;

    for (const auto& tst_id : tst_data)
    {
        ++num_pos;

        const ptime timestamp = tst_id.first;
        const dbContent::TargetPosition tst_pos = target_data.tstChain().pos(tst_id);

        if (!target_data.hasMappedRefData(tst_id, max_ref_time_diff))
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, {}, {}, {}, DetailCode::NoRefData);

            ++num_no_ref;
            continue;
        }

        auto ref_pos = target_data.mappedRefPos(tst_id, max_ref_time_diff);

        if (!ref_pos.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, {}, {}, {}, DetailCode::NoRefPosition);

            ++num_no_ref;
            continue;
        }

        auto ref_spd = target_data.mappedRefSpeed(tst_id, max_ref_time_diff);

        if (!ref_spd.has_value())
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, ref_pos, {}, {}, DetailCode::NoRefSpeed);

            ++num_no_ref;
            continue;
        }

        bool is_inside = target_data.isTimeStampNotExcluded(timestamp)
                         && target_data.mappedRefPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, ref_pos, is_inside, {}, 0);

            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        auto ref_pos_acc = target_data.mappedRefMinAcc(tst_id, max_ref_time_diff); // max std dev

        if (ref_pos_acc && *ref_pos_acc > ref_min_accuracy_)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, ref_pos, is_inside, {}, DetailCode::RefInaccurate);

            ++num_ref_inaccurate;
            continue;
        }

        bool   transform_ok;
        double distance, angle;

        std::tie(transform_ok, distance, angle) = ogr_geo2cart.distanceAngleCart(
            ref_pos->latitude_, ref_pos->longitude_, tst_pos.latitude_, tst_pos.longitude_);

        if (!transform_ok || !std::isfinite(distance) || !std::isfinite(angle)
            || !std::isfinite(ref_spd->track_angle_))
        {
            addDetail(timestamp, tst_pos, ref_pos, is_inside, {}, Result::OffsetInvalid);
            ++num_pos_calc_errors;
            continue;
        }

        if (ref_spd->speed_ <= 0 || !std::isfinite(ref_spd->speed_))
        {
            addDetail(timestamp, tst_pos, ref_pos, is_inside, {}, Result::SpeedInvalid);
            ++num_pos_calc_errors;
            continue;
        }

        const double d_along = TrackOffsetHelpers::trackOffset(distance, angle, ref_spd->track_angle_).along;

        along_values.push_back(d_along);
        speed_values.push_back(ref_spd->speed_);

        ++num_comp_passed;

        addDetail(timestamp, tst_pos, ref_pos, is_inside, -d_along / ref_spd->speed_, 0);
    }

    traced_assert(num_no_ref <= num_pos);
    traced_assert(num_pos - num_no_ref == num_pos_inside + num_pos_outside);

    return make_shared<EvaluationRequirementResult::SinglePositionRadarTimeStamp>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos, num_no_ref, num_pos_outside, num_pos_inside, num_ref_inaccurate,
                num_comp_passed, 0, along_values, speed_values);
}

}
