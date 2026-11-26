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

#include "intervalbase.h"
#include "intervalhelpers.h"
#include "timeperiod.h"
#include "evaluationcalculator.h"
#include "timeconv.h"

#include "results/base/intervalbase.h"
#include "standard/evaluationstandard.h"

#include "timeconv.h"


namespace EvaluationRequirement
{

/**
*/
IntervalBase::IntervalBase(const std::string& name, 
                           const std::string& short_name, 
                           const std::string& group_name,
                           double prob, 
                           COMPARISON_TYPE prob_check_type, 
                           EvaluationCalculator& calculator,
                           float update_interval_s, 
                           const boost::optional<float>& min_gap_length_s,
                           const boost::optional<float>& max_gap_length_s,
                           const boost::optional<float>& miss_tolerance_s,
                           const boost::optional<float>& min_ref_period_s,
                           const boost::optional<bool>& must_hold_for_any_target)
:   ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator, must_hold_for_any_target)
,   update_interval_s_       (update_interval_s       )
,   min_gap_length_s_        (min_gap_length_s        )
,   max_gap_length_s_        (max_gap_length_s        )
,   miss_tolerance_s_        (miss_tolerance_s        )
,   min_ref_period_s_        (min_ref_period_s        )
{
}

/**
*/
float IntervalBase::missThreshold() const
{
    return miss_tolerance_s_.has_value() ? update_interval_s_ + miss_tolerance_s_.value() : update_interval_s_;
}

/**
*/
IntervalHelpers::MissParams IntervalBase::missParams() const
{
    IntervalHelpers::MissParams p;
    p.update_interval_s = update_interval_s_;
    p.min_gap_length_s  = min_gap_length_s_;
    p.max_gap_length_s  = max_gap_length_s_;
    p.miss_tolerance_s  = miss_tolerance_s_;

    return p;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> IntervalBase::evaluate(const EvaluationTargetData& target_data, 
                                                                            std::shared_ptr<Base> instance,
                                                                            const SectorLayer& sector_layer)
{
    auto max_ref_time_diff    = Utils::Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());
    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    typedef EvaluationRequirementResult::SingleIntervalBase Result;
    typedef EvaluationDetail                                Detail;
    typedef Result::EvaluationDetails                       Details;

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    //create inside/outside sector reference periods and extend to test data min max times
    TimePeriodCollection periods;
    periods.createFromReference(target_data, sector_layer, max_ref_time_diff,
                                [ & ] (const dbContent::TargetReport::DataID& ref_id)
                                { return this->useReferenceUpdate(target_data, ref_id); });

    if (min_ref_period_s_.has_value() && min_ref_period_s_.value() > 0)
        periods.removeSmallPeriods(Utils::Time::partialSeconds(min_ref_period_s_.value()));
    
    //extend to test data min max times and sort test updates into ref periods
    std::vector<dbContent::TargetReport::DataID> tst_updates;
    tst_updates.reserve(tst_data.size());

    for (const auto& tst_it : tst_data)
        tst_updates.emplace_back(tst_it);

    IntervalHelpers::addTestUpdates(periods, tst_updates);

    //create events from periods
    auto validity = [ & ] (const dbContent::TargetReport::DataID& data_id)
    {
        return isValid(data_id, target_data, sector_layer, max_ref_time_diff);
    };

    auto events = IntervalHelpers::periodEvents(periods, validity, missParams(), skip_no_data_details);

    auto counts = IntervalHelpers::counts(periods, events, missParams(), exclude_detection_misses_);

    // the details show the same misses as the counts
    if (exclude_detection_misses_)
        IntervalHelpers::excludeDetectionMisses(periods, missParams(), events);

    //process events and create details
    Details details;

    auto addDetail = [ & ] (const boost::posix_time::ptime& timestamp,
                            const std::vector<dbContent::TargetPosition>& positions,
                            const QVariant& d_tod,
                            const QVariant& miss_occurred,
                            const QVariant& missed_uis,
                            const QVariant& invalid_updates,
                            std::uint32_t code)
    {
        details.push_back(Detail(timestamp, positions).setValue(Result::DetailKey::DiffTOD       , d_tod          )
                                                      .setValue(Result::DetailKey::MissOccurred  , miss_occurred  )
                                                      .setValue(Result::DetailKey::MissedUIs     , missed_uis     )
                                                      .setValue(Result::DetailKey::InvalidUpdates, invalid_updates)
                                                      .addCode(code));
    };

    size_t   ne           = events.size();
    uint32_t misses_total = 0;

    std::vector<dbContent::TargetPosition> ref_updates;

    for (size_t i = 0; i < ne; ++i)
    {
        const auto& event = events[ i ];

        //accumulate misses up to the current event
        misses_total += event.misses;

        //generate detail info for event and skip if no detail shall be generated
        auto detail_info = eventDetailInfo(target_data, event, ref_updates);
        if (!detail_info.generate_detail)
            continue;

        std::vector<dbContent::TargetPosition> positions;
        if (detail_info.evt_ref_updates_idx0.has_value() && detail_info.evt_ref_updates_idx1.has_value())
        {
            //add collected reference updates of interval
            positions.assign(ref_updates.begin() + detail_info.evt_ref_updates_idx0.value(),
                             ref_updates.begin() + detail_info.evt_ref_updates_idx1.value() + 1);

            //invert vector (so that the first position is the tested location and the last one is the reference position the misses are measured to)
            std::reverse(positions.begin(), positions.end());
        }
        else
        {
            //add event position and any existing interval reference position
            positions.push_back(detail_info.evt_position);

            if (detail_info.evt_position_ref.has_value())
                positions.push_back(detail_info.evt_position_ref.value());
        }

        //add detail for event
        addDetail(detail_info.evt_time, 
                  positions, 
                  detail_info.evt_has_dt ? detail_info.evt_dt : QVariant(), 
                  detail_info.evt_has_misses, 
                  misses_total,
                  detail_info.evt_has_misses ? QVariant(event.invalid_updates) : QVariant(),
                  event.code);
    }

    //create derived-specific result
    return createResult("UTN:" + std::to_string(target_data.utn_),
                        instance,
                        target_data,
                        sector_layer,
                        details,
                        periods,
                        counts.sum_uis,
                        counts.misses);
}

/**
*/
IntervalBase::DetailInfo IntervalBase::eventDetailInfo(const EvaluationTargetData& target_data, 
                                                       const Event& event,
                                                       std::vector<dbContent::TargetPosition>& ref_updates) const
{
    bool has_miss = event.misses > 0;

    IntervalBase::DetailInfo dinfo;
    dinfo.evt_has_misses = has_miss;
    dinfo.evt_dt         = event.dtSeconds();
    dinfo.evt_has_dt     = false;

    //stores a single reference update given a test data id
    auto storeRefUpdate = [ & ] (const dbContent::TargetReport::DataID& id_tst)
    {
        //map to ref pos
        auto ref_pos = target_data.mappedRefPos(id_tst);
        traced_assert(ref_pos.has_value());

        //store index range into detail info
        dinfo.evt_ref_updates_idx0 = ref_updates.size();
        dinfo.evt_ref_updates_idx1 = ref_updates.size();

        //collect
        ref_updates.push_back(ref_pos.value());
    };

    //stores all reference updates in the timespan of an interval (plus the reference interpolated end positions of the interval)
    auto storeRefUpdates = [ & ] (IntervalBase::DetailInfo& dinfo,
                                  const dbContent::TargetReport::DataID& id_tst0,
                                  const dbContent::TargetReport::DataID& id_tst1,
                                  const boost::optional<dbContent::TargetPosition>& ref_pos0,
                                  const boost::optional<dbContent::TargetPosition>& ref_pos1)
    {
        //reference-interpolated interval end positions
        boost::optional<dbContent::TargetPosition> pos0 = ref_pos0;
        boost::optional<dbContent::TargetPosition> pos1 = ref_pos1;

        //not passed directly => interpolate now
        if (!pos0.has_value()) pos0 = target_data.mappedRefPos(id_tst0);
        if (!pos1.has_value()) pos1 = target_data.mappedRefPos(id_tst1);

        //interpolation of ref should always be possible, since the period is inside a valid reference period
        traced_assert(pos0.has_value() && pos1.has_value());

        //retrieve all ref updates inside the interval
        auto positions = target_data.refChain().positionsBetween(id_tst0.timestamp(), 
                                                                    id_tst1.timestamp(), 
                                                                    false, 
                                                                    false);
        unsigned int idx0 = ref_updates.size(); //from idx

        //collect all updates
        ref_updates.push_back(pos0.value());
        ref_updates.insert(ref_updates.end(), positions.begin(), positions.end());
        ref_updates.push_back(pos1.value());

        unsigned int idx1 = ref_updates.size() - 1; //to idx (inclusive)

        //store index range into detail info
        dinfo.evt_ref_updates_idx0 = idx0;
        dinfo.evt_ref_updates_idx1 = idx1;
    };

    switch (event.type)
    {
        case Event::TypeNoReference:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_position    = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref     = false;
            dinfo.generate_detail = true;

            //no ref updates to store

            break;
        }
        case Event::TypeEnterPeriod:
        {
            traced_assert(target_data.refChain().posOpt(event.interval_time0).has_value());

            dinfo.evt_time        = event.interval_time0;
            dinfo.evt_position    = target_data.refChain().pos(event.interval_time0);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            //no other ref updates to store

            break;
        }
        case Event::TypeFirstValidUpdateInPeriod:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_position    = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            storeRefUpdate(event.data_id);

            break;
        }
        case Event::TypeValidFirst:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());
            traced_assert(target_data.refChain().posOpt(event.interval_time0).has_value());

            dinfo.evt_time         = event.data_id.timestamp();
            dinfo.evt_position     = target_data.tstChain().pos(event.data_id);
            dinfo.evt_position_ref = target_data.refChain().pos(event.interval_time0);
            dinfo.evt_has_ref      = true;
            dinfo.evt_has_dt       = true;
            dinfo.generate_detail  = true;

            storeRefUpdates(dinfo, 
                            event.interval_time0, 
                            event.data_id, 
                            dinfo.evt_position_ref,
                            {});
            break;
        }
        case Event::TypeValid:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());
            traced_assert(target_data.tstChain().posOpt(event.interval_time0).has_value());

            dinfo.evt_time         = event.data_id.timestamp();
            dinfo.evt_position     = target_data.tstChain().pos(event.data_id);
            dinfo.evt_position_ref = target_data.tstChain().pos(event.interval_time0);
            dinfo.evt_has_ref      = true;
            dinfo.evt_has_dt       = true;
            dinfo.generate_detail  = true;

            target_data.mappedRefPos(event.data_id);

            storeRefUpdates(dinfo, 
                            event.interval_time0, 
                            event.data_id,
                            {},
                            {});
            break;
        }
        case Event::TypeValidLast:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());
            traced_assert(target_data.refChain().posOpt(event.interval_time1).has_value());

            // the gap runs from the last valid update to the period end, like all gap details
            // the detail time is the gap end
            dinfo.evt_time         = event.interval_time1;
            dinfo.evt_position     = target_data.refChain().pos(event.interval_time1);
            dinfo.evt_position_ref = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref      = true;
            dinfo.evt_has_dt       = true;
            dinfo.generate_detail  = true;

            storeRefUpdates(dinfo,
                            event.data_id,
                            event.interval_time1,
                            {},
                            dinfo.evt_position);
            break;
        }
        case Event::TypeInvalid:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_position    = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            storeRefUpdate(event.data_id);

            break;
        }
        case Event::TypeDataMissing:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_position    = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            storeRefUpdate(event.data_id);

            break;
        }
        case Event::TypeLastValidUpdateInPeriod:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_position    = target_data.tstChain().pos(event.data_id);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            storeRefUpdate(event.data_id);

            break;
        }
        case Event::TypeLeavePeriod:
        {
            traced_assert(target_data.refChain().posOpt(event.interval_time1).has_value());

            dinfo.evt_time        = event.interval_time1;
            dinfo.evt_position    = target_data.refChain().pos(event.interval_time1);
            dinfo.evt_has_ref     = true;
            dinfo.generate_detail = true;

            //no other ref updates to store

            break;
        }
        case Event::TypeEmptyPeriod:
        {
            traced_assert(target_data.refChain().posOpt(event.interval_time0).has_value());
            traced_assert(target_data.refChain().posOpt(event.interval_time1).has_value());

            dinfo.evt_time         = event.interval_time1;
            dinfo.evt_position     = target_data.refChain().pos(event.interval_time1);
            dinfo.evt_position_ref = target_data.refChain().pos(event.interval_time0);
            dinfo.evt_has_ref      = true;
            dinfo.evt_has_dt       = true;
            dinfo.generate_detail  = has_miss;

            storeRefUpdates(dinfo, 
                            event.interval_time0, 
                            event.interval_time1,
                            dinfo.evt_position_ref,
                            dinfo.evt_position);
            break;
        }
    }

    return dinfo;
}

} // namespace EvaluationRequirement
