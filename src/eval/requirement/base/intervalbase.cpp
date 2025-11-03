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
#include "timeperiod.h"
#include "evaluationcalculator.h"
#include "timeconv.h"

#include "eval/results/base/intervalbase.h"
#include "eval/standard/evaluationstandard.h"

#include "util/timeconv.h"


namespace EvaluationRequirement 
{

/**
*/
boost::posix_time::time_duration Event::dt() const
{
    return interval_time1 - interval_time0;
}

/**
*/
double Event::dtSeconds() const
{
    return Utils::Time::partialSeconds(dt());
}

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

    //determine min max test times
    boost::optional<boost::posix_time::ptime> tmin_tst;
    boost::optional<boost::posix_time::ptime> tmax_tst;
    if (!tst_data.empty())
    {
        tmin_tst = tst_data.begin()->first;
        tmax_tst = tst_data.rbegin()->first;
    }

    //create inside/outside sector reference periods and extend to test data min max times
    TimePeriodCollection periods;
    periods.createFromReference(target_data, sector_layer, max_ref_time_diff);

    if (min_ref_period_s_.has_value() && min_ref_period_s_.value() > 0)
        periods.removeSmallPeriods(Utils::Time::partialSeconds(min_ref_period_s_.value()));
    
    periods.fillInOutsidePeriods(tmin_tst, tmax_tst);

    uint32_t sum_uis = periods.getUIs(update_interval_s_);

    //sort test updates into ref periods
    for (const auto& tst_it : tst_data)
    {
        auto timestamp = tst_it.first;

        bool inside_period = periods.isInside(timestamp);
        traced_assert(inside_period); //!should _always_ land in a period!

        TimePeriodUpdate update;
        update.data_id = dbContent::TargetReport::DataID(tst_it);

        periods.addUpdate(update);
    }

    //create events from periods
    auto events = periodEvents(periods, target_data, sector_layer, skip_no_data_details);

    //process events and create details
    Details details;

    auto addDetail = [ & ] (const boost::posix_time::ptime& timestamp,
                            const std::vector<dbContent::TargetPosition>& positions,
                            const QVariant& d_tod,
                            const QVariant& miss_occurred,
                            const QVariant& ref_exists,
                            const QVariant& missed_uis,
                            const std::string& comment)
    {
        details.push_back(Detail(timestamp, positions).setValue(Result::DetailKey::DiffTOD     , d_tod        )
                                                      .setValue(Result::DetailKey::MissOccurred, miss_occurred)
                                                      .setValue(Result::DetailKey::RefExists   , ref_exists   )
                                                      .setValue(Result::DetailKey::MissedUIs   , missed_uis   )
                                                      .generalComment(comment));
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

        //!events which cause misses should generate comments!
        traced_assert(event.misses == 0 || !detail_info.evt_comment.empty());

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
                  detail_info.evt_has_ref,
                  misses_total, 
                  detail_info.evt_comment);
    }

    //create derived-specific result
    return createResult("UTN:" + std::to_string(target_data.utn_),
                        instance,
                        target_data,
                        sector_layer,
                        details,
                        periods,
                        sum_uis,
                        misses_total);
}

/**
*/
std::vector<Event> IntervalBase::periodEvents(const TimePeriodCollection& periods,
                                              const EvaluationTargetData& target_data,
                                              const SectorLayer& sector_layer,
                                              bool skip_no_data_details) const
{
    std::vector<Event> events;

    uint32_t inside_period_cnt = 0;

    for (size_t i = 0; i < periods.size(); ++i)
    {
        const auto& period = periods.period(i);

        //create events for period
        auto events_period = periodEvents(period, target_data, sector_layer, skip_no_data_details);

        //period is inside period?
        if (period.type() == TimePeriod::Type::InsideSector)
        {
            //add inside period index to events
            for (auto& e : events_period)
                e.period = inside_period_cnt;

            ++inside_period_cnt;
        }

        //collect
        if (!events_period.empty())
            events.insert(events.end(), events_period.begin(), events_period.end());
    }

    return events;
}

/**
*/
std::vector<Event> IntervalBase::periodEvents(const TimePeriod& period,
                                              const EvaluationTargetData& target_data,
                                              const SectorLayer& sector_layer,
                                              bool skip_no_data_details) const
{
    std::vector<Event> events;

    auto logEvent = [ & ] (std::vector<Event>& events,
                           Event::Type type, 
                           const TimePeriodUpdate* update,
                           const boost::posix_time::ptime& time0,
                           const boost::posix_time::ptime& time1,
                           const std::string& error,
                           bool has_ref_data,
                           bool compute_misses)
    {
        traced_assert(time1 >= time0);

        Event e;
        e.type           = type;
        e.error          = error;
        e.interval_time0 = time0;
        e.interval_time1 = time1;

        if (update)
            e.data_id = update->data_id;

        if (compute_misses)
        {
            double dt = Utils::Time::partialSeconds(time1 - time0);

            e.misses = numMisses(dt);
        }

        e.had_ref_data = has_ref_data;

        events.push_back(e);
    };

    //outside sector period => add all updates as OutsideSector
    if (period.type() == TimePeriod::Type::OutsideSector)
    {
        if (!skip_no_data_details)
        {
            for (const auto& update : period.getUpdates())
            {
                logEvent(events, Event::TypeNoReference, &update, update.data_id.timestamp(), update.data_id.timestamp(), "", true, false);
            }
        }

        return events;
    }

    //log period entered
    if (!skip_no_data_details)
        logEvent(events, Event::TypeEnterPeriod, nullptr, period.begin(), period.end(), "", true, false);

    //inside sector period
    const auto& updates = period.getUpdates();

    if (updates.empty())
    {
        //no updates in period => add single event for this case
        if (!skip_no_data_details)
            logEvent(events, Event::TypeEmptyPeriod, nullptr, period.begin(), period.end(), "", true, false);
    }
    else
    {
        //period obtains updates
        auto max_ref_time_diff = Utils::Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

        size_t n = updates.size();

        const TimePeriodUpdate* valid_last     = nullptr;
        bool                    had_ref_data   = true;
        int                     valid_last_idx = -1;

        for (size_t i = 0; i < n; ++i)
        {
            const auto& update = updates[ i ];

            //check if update is valid
            auto validity = isValid(update.data_id, target_data, sector_layer, max_ref_time_diff);

            //invalid update => log and skip
            if (validity.value == Validity::Value::Invalid)
            {
                logEvent(events, Event::TypeInvalid, &update, update.data_id.timestamp(), update.data_id.timestamp(), validity.comment, true, false);
                continue;
            }

            //was reference data missing? => yields a valid update (in dubio pro reo)
            had_ref_data = validity.value != Validity::Value::RefDataMissing;

            //first valid update? => log
            if (!valid_last && !skip_no_data_details)
                logEvent(events, Event::TypeFirstValidUpdateInPeriod, &update, update.data_id.timestamp(), update.data_id.timestamp(), "", had_ref_data, false);

            //valid update => check for misses
            logEvent(events, 
                     valid_last ? Event::TypeValid : Event::Type::TypeValidFirst,
                     &update,
                     valid_last ? valid_last->data_id.timestamp() : period.begin(),      // time0 is either the last valid update or period begin
                     update.data_id.timestamp(),                                         // time1 always update time
                     "",
                     had_ref_data,
                     true);                                                              // check misses in both cases

            valid_last     = &updates[ i ];
            valid_last_idx = (int)events.size();
        }

        //add event for last valid update
        if (valid_last)
        {
            std::vector<Event> last_events;

            if (!skip_no_data_details)
                logEvent(last_events, Event::TypeLastValidUpdateInPeriod, valid_last, valid_last->data_id.timestamp(), valid_last->data_id.timestamp(), "", had_ref_data, false);
            logEvent(last_events, Event::TypeValidLast, valid_last, valid_last->data_id.timestamp(), period.end(), "", had_ref_data, true);

            events.insert(events.begin() + valid_last_idx, last_events.begin(), last_events.end());
        }

        //no valid updates in period => add single event for this case
        if (!valid_last)
        {
            if (!skip_no_data_details)
                logEvent(events, Event::TypeEmptyPeriod, nullptr, period.begin(), period.end(), "", true, false);
        }
    }

    //log period left
    if (!skip_no_data_details)
        logEvent(events, Event::TypeLeavePeriod, nullptr, period.begin(), period.end(), "", true, false);

    return events;
}

/**
*/
uint32_t IntervalBase::numMisses(double dt) const
{
    if (miss_tolerance_s_.has_value())
        dt -= miss_tolerance_s_.value();

    if (min_gap_length_s_.has_value() && dt < min_gap_length_s_.value()) // supress gaps smaller as min gap length
        return 0;

    if (max_gap_length_s_.has_value() && dt > max_gap_length_s_.value()) // supress gaps larger as max gap length
        return 0;

    if (dt <= update_interval_s_)
        return 0;

    return (uint32_t)std::floor(dt / update_interval_s_);
}

/**
*/
IntervalBase::DetailInfo IntervalBase::eventDetailInfo(const EvaluationTargetData& target_data, 
                                                       const Event& event,
                                                       std::vector<dbContent::TargetPosition>& ref_updates) const
{
    std::string thres        = Utils::String::doubleToStringPrecision(missThreshold(), 2);
    std::string period       = "period " + (event.period.has_value() ? std::to_string(event.period.value()) : "");
    std::string miss         = "(DToD > " + thres + ")";
    std::string hit          = "(DToD <= " + thres + ")";
    std::string evt_ts       = Utils::Time::toString(event.interval_time1);
    std::string evt_interval = "[" + Utils::Time::toString(event.interval_time0) + "," + evt_ts + "]";
    std::string error        = event.error.empty() ? "Unknown" : event.error;

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
            dinfo.evt_comment     = "Outside sector/reference period";
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
            dinfo.evt_comment     = "Entering " + period;
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
            dinfo.evt_comment     = "First valid target report of " + period;
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
            dinfo.evt_comment      = has_miss ? "Miss detected at start of current " + period + " " + miss + ", between " + evt_interval : 
                                                "Begin of current " + period + " OK " + hit;
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
            dinfo.evt_comment      = has_miss ? "Miss detected in current " + period + " " + miss + ", between " + evt_interval : "OK " + hit;
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

            dinfo.evt_time         = event.data_id.timestamp();
            dinfo.evt_comment      = has_miss ? "Miss detected at end of current " + period + " " + miss + ", between " + evt_interval : 
                                                "End of current " + period + " OK " + hit;
            dinfo.evt_position     = target_data.tstChain().pos(event.data_id);
            dinfo.evt_position_ref = target_data.refChain().pos(event.interval_time1);
            dinfo.evt_has_ref      = true;
            dinfo.evt_has_dt       = true;
            dinfo.generate_detail  = true;

            storeRefUpdates(dinfo, 
                            event.data_id, 
                            event.interval_time1, 
                            {},
                            dinfo.evt_position_ref);
            break;
        }
        case Event::TypeInvalid:
        {
            traced_assert(target_data.tstChain().posOpt(event.data_id).has_value());

            dinfo.evt_time        = event.data_id.timestamp();
            dinfo.evt_comment     = "Invalid target report: " + error;
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
            dinfo.evt_comment     = "Invalid target report: " + error;
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
            dinfo.evt_comment     = "Last valid target report of " + period;
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
            dinfo.evt_comment     = "Leaving " + period;
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
            dinfo.evt_comment      = has_miss ? "Miss detected in empty period " + period + " " + miss + ", between " + evt_interval : "";
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

    //add remark that reference data was missing
    if (!event.had_ref_data)
        dinfo.evt_comment += " (no ref info)";

    return dinfo;
}

} // namespace EvaluationRequirement
