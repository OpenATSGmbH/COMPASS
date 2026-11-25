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

#include "requirement/position/distancerms.h"
#include "requirement/position/positionhelpers.h"
#include "requirement/position/positiontargetdata.h"
#include "results/position/distancerms.h"
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

PositionDistanceRMS::PositionDistanceRMS(
        const std::string& name, const std::string& short_name, const std::string& group_name, float ref_min_accuracy,
        EvaluationCalculator& calculator, double threshold_value)
    : PositionBaseValue(name, short_name, group_name, threshold_value, COMPARISON_TYPE::LESS_THAN_OR_EQUAL, 
        ref_min_accuracy, calculator)
{
}

std::shared_ptr<EvaluationRequirementResult::Single> PositionDistanceRMS::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " threshold_value " << threshold();

    const time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    PositionHelpers::ReportSettings settings;
    settings.ref_min_accuracy     = ref_min_accuracy_;
    settings.skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    const auto evaluation = PositionHelpers::evaluateDistancesRMS(
                target_data.tstChain().timestampIndexes(),
                PositionTargetData::tstPositions(target_data),
                PositionTargetData::referenceLookup(target_data, sector_layer, max_ref_time_diff, false),
                settings, threshold());

    const auto& c = evaluation.counts;

    return std::make_shared<EvaluationRequirementResult::SinglePositionDistanceRMS>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, PositionTargetData::details(evaluation),
                c.num_pos, c.num_no_ref, c.num_pos_outside, c.num_pos_inside, c.num_ref_inaccurate,
                c.num_comp_passed, c.num_comp_failed);
}

}
