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

#include "requirement/position/positionhelpers.h"
#include "evaluationdetail.h"

#include <vector>

#include <boost/date_time/posix_time/posix_time_duration.hpp>

class EvaluationTargetData;
class SectorLayer;

namespace EvaluationRequirement
{

/**
 * Connects the position requirements to the evaluation target data: the inputs of the
 * PositionHelpers walks and the report details of their results.
 */
namespace PositionTargetData
{

/// reference lookups of the target, with the reference speed if with_speed
PositionHelpers::ReferenceLookup referenceLookup(const EvaluationTargetData& target_data,
                                                 const SectorLayer& sector_layer,
                                                 boost::posix_time::time_duration max_ref_time_diff,
                                                 bool with_speed);

/// test report positions of the target
PositionHelpers::TstPosFunc tstPositions(const EvaluationTargetData& target_data);

std::vector<EvaluationDetail> details(const PositionHelpers::PositionEvaluation& evaluation);

} // namespace PositionTargetData

} // namespace EvaluationRequirement
