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

#include "requirement/position/distance.h"
#include "requirement/position/positionhelpers.h"
#include "results/position/distance.h"
#include "standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"
#include "timeconv.h"
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

    const time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    using dbContent::TargetReport::DataID;

    PositionHelpers::ReferenceLookup ref;
    ref.has_ref_data   = [ & ] (const DataID& id) { return target_data.hasMappedRefData(id, max_ref_time_diff); };
    ref.ref_pos        = [ & ] (const DataID& id) { return target_data.mappedRefPos(id, max_ref_time_diff); };
    ref.ref_pos_inside = [ & ] (const DataID& id) { return target_data.isTimeStampNotExcluded(id.timestamp())
                                                           && target_data.mappedRefPosInside(sector_layer, id); };
    ref.ref_min_acc    = [ & ] (const DataID& id) { return target_data.mappedRefMinAcc(id, max_ref_time_diff, false); }; // max std dev

    PositionHelpers::DistanceSettings settings;
    settings.threshold            = threshold_value_;
    settings.check_type           = threshold_value_check_type_;
    settings.ref_min_accuracy     = ref_min_accuracy_;
    settings.use_averaging        = use_averaging_;
    settings.averaging_window_s   = averaging_window_s_;
    settings.skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    const auto evaluation = PositionHelpers::evaluateDistances(
                target_data.tstChain().timestampIndexes(),
                [ & ] (const DataID& id) { return target_data.tstChain().pos(id); },
                ref, settings);

    typedef EvaluationRequirementResult::SinglePositionDistance Result;

    Result::EvaluationDetails details;
    details.reserve(evaluation.details.size());

    for (const auto& d : evaluation.details)
        details.push_back(EvaluationDetail(d.timestamp, d.tst_pos)
                          .setValue(Result::DetailKey::PosInside, d.pos_inside ? QVariant(*d.pos_inside) : QVariant("false"))
                          .setValue(Result::DetailKey::Value, d.value ? QVariant(*d.value) : QVariant())
                          .setValue(Result::DetailKey::CheckPassed, d.check_passed)
                          .addPosition(d.ref_pos)
                          .generalComment(d.comment));

    const auto& c = evaluation.counts;

    return make_shared<EvaluationRequirementResult::SinglePositionDistance>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, c.num_pos, c.num_no_ref, c.num_pos_outside, c.num_pos_inside, c.num_ref_inaccurate,
                c.num_comp_passed, c.num_comp_failed);
}

}
