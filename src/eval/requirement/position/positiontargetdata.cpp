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

#include "requirement/position/positiontargetdata.h"
#include "results/position/positionbase.h"
#include "evaluationtargetdata.h"

#include <QVariant>

namespace EvaluationRequirement
{
namespace PositionTargetData
{

using dbContent::TargetReport::DataID;

/**
 */
PositionHelpers::ReferenceLookup referenceLookup(const EvaluationTargetData& target_data,
                                                 const SectorLayer& sector_layer,
                                                 boost::posix_time::time_duration max_ref_time_diff,
                                                 bool with_speed)
{
    PositionHelpers::ReferenceLookup ref;

    ref.has_ref_data   = [ &target_data, max_ref_time_diff ] (const DataID& id)
                         { return target_data.hasMappedRefData(id, max_ref_time_diff); };
    ref.ref_pos        = [ &target_data, max_ref_time_diff ] (const DataID& id)
                         { return target_data.mappedRefPos(id, max_ref_time_diff); };
    ref.ref_pos_inside = [ &target_data, &sector_layer ] (const DataID& id)
                         { return target_data.isTimeStampNotExcluded(id.timestamp())
                                  && target_data.mappedRefPosInside(sector_layer, id); };
    ref.ref_min_acc    = [ &target_data, max_ref_time_diff ] (const DataID& id)
                         { return target_data.mappedRefMinAcc(id, max_ref_time_diff); }; // max std dev

    if (with_speed)
        ref.ref_speed  = [ &target_data, max_ref_time_diff ] (const DataID& id)
                         { return target_data.mappedRefSpeed(id, max_ref_time_diff); };

    return ref;
}

/**
 */
PositionHelpers::TstPosFunc tstPositions(const EvaluationTargetData& target_data)
{
    return [ &target_data ] (const DataID& id) { return target_data.tstChain().pos(id); };
}

/**
 */
std::vector<EvaluationDetail> details(const PositionHelpers::PositionEvaluation& evaluation)
{
    typedef EvaluationRequirementResult::SinglePositionBaseCommon::DetailKey DetailKey;

    std::vector<EvaluationDetail> details;
    details.reserve(evaluation.details.size());

    for (const auto& d : evaluation.details)
        details.push_back(EvaluationDetail(d.timestamp, d.tst_pos)
                          .setValue(DetailKey::PosInside, d.pos_inside ? QVariant(*d.pos_inside) : QVariant("false"))
                          .setValue(DetailKey::Value, d.value ? QVariant(*d.value) : QVariant())
                          .setValue(DetailKey::CheckPassed, d.check_passed ? QVariant(*d.check_passed) : QVariant())
                          .addPosition(d.ref_pos)
                          .generalComment(d.comment));

    return details;
}

} // namespace PositionTargetData
} // namespace EvaluationRequirement
