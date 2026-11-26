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

#include "requirement/position/along.h"
#include "requirement/position/positionhelpers.h"
#include "requirement/position/positiontargetdata.h"
#include "results/position/along.h"
#include "standard/evaluationstandard.h"
#include "evaluationmanager.h"
#include "logger.h"
#include "timeconv.h"
#include "sectorlayer.h"

#include <algorithm>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

PositionAlong::PositionAlong(const std::string& name, const std::string& short_name, const std::string& group_name,
                             double prob, COMPARISON_TYPE prob_check_type, float ref_min_accuracy, 
                             EvaluationCalculator& calculator, float max_abs_value)
    : PositionBaseProb(name, short_name, group_name, prob, prob_check_type, ref_min_accuracy, calculator), 
      max_abs_value_(max_abs_value)
{
}

float PositionAlong::maxAbsValue() const
{
    return max_abs_value_;
}

std::shared_ptr<EvaluationRequirementResult::Single> PositionAlong::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " max_abs_value " << max_abs_value_;

    const time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    PositionHelpers::ReportSettings settings;
    settings.ref_min_accuracy     = ref_min_accuracy_;
    settings.skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    const auto evaluation = PositionHelpers::evaluateTrackOffsets(
                target_data.tstChain().timestampIndexes(),
                PositionTargetData::tstPositions(target_data),
                PositionTargetData::referenceLookup(target_data, sector_layer, max_ref_time_diff, true),
                settings, PositionHelpers::TrackOffsetComponent::Along, max_abs_value_);

    const auto& c = evaluation.counts;

    return std::make_shared<EvaluationRequirementResult::SinglePositionAlong>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, PositionTargetData::details(evaluation),
                c.num_pos, c.num_no_ref, c.num_pos_outside, c.num_pos_inside, c.num_ref_inaccurate,
                c.num_comp_passed, c.num_comp_failed);
}

}
