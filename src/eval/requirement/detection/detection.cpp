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

#include "requirement/detection/detection.h"
#include "requirement/detection/detection_pd_helpers.h"
#include "requirement/detection/detection_walk.h"
#include "results/detection/detection.h"
#include "standard/evaluationstandard.h"

#include "evaluationmanager.h"
#include "evaluationdetail.h"

#include "logger.h"
#include "stringconv.h"
#include "sectorlayer.h"
#include "timeperiod.h"

#include "boost/date_time/posix_time/ptime.hpp"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;


namespace EvaluationRequirement
{

/**
*/
Detection::Detection(const std::string& name, 
                     const std::string& short_name, 
                     const std::string& group_name,
                     double prob, 
                     COMPARISON_TYPE prob_check_type, 
                     EvaluationCalculator& calculator,
                     float update_interval_s, 
                     bool use_min_gap_length, 
                     float min_gap_length_s,
                     bool use_max_gap_length, 
                     float max_gap_length_s, 
                     bool invert_prob,
                     bool use_miss_tolerance,
                     float miss_tolerance_s,
                     bool use_time_ratio,
                     bool use_gap_count,
                     bool use_stationary_ui,
                     float stationary_ui_s,
                     float stationary_speed_threshold_ms,
                     bool hold_for_any_target,
                     const std::string& pd_calculation_method)
    : ProbabilityBase     (name, short_name, group_name, prob, prob_check_type, invert_prob, calculator, hold_for_any_target),
    update_interval_s_  (update_interval_s),
    use_min_gap_length_ (use_min_gap_length),
    min_gap_length_s_   (min_gap_length_s),
    use_max_gap_length_ (use_max_gap_length),
    max_gap_length_s_   (max_gap_length_s),
    use_miss_tolerance_ (use_miss_tolerance),
    miss_tolerance_s_   (miss_tolerance_s),
    use_time_ratio_     (use_time_ratio),
    use_gap_count_      (use_gap_count),
    use_stationary_ui_  (use_stationary_ui),
    stationary_ui_s_    (stationary_ui_s),
    stationary_speed_threshold_ms_(stationary_speed_threshold_ms),
    pd_calculation_method_(pd_calculation_method)
{
}

/**
*/
float Detection::updateInterval() const
{
    return update_interval_s_;
}

/**
*/
bool Detection::useMinGapLength() const
{
    return use_min_gap_length_;
}

/**
*/
float Detection::minGapLength() const
{
    return min_gap_length_s_;
}

/**
*/
bool Detection::useMaxGapLength() const
{
    return use_max_gap_length_;
}

/**
*/
float Detection::maxGapLength() const
{
    return max_gap_length_s_;
}

/**
*/
bool Detection::useMissTolerance() const
{
    return use_miss_tolerance_;
}

/**
*/
float Detection::missTolerance() const
{
    return miss_tolerance_s_;
}

/**
*/
float Detection::missThreshold() const
{
    return use_miss_tolerance_ ? update_interval_s_+miss_tolerance_s_ : update_interval_s_;
}

/**
*/
const std::string& Detection::pdCalculationMethod() const
{
    return pd_calculation_method_;
}

/**
 * Status-message method: the expected periods are taken from the update cycles the
 * test data source reports (CAT019 / CAT010 start of update cycle) instead of a
 * configured nominal update interval. `cycles_per_period` consecutive reported
 * cycles form one expected period, so a requirement asking for detection within
 * 2 s on a 1 s cycle source groups 2 cycles. A period counts as expected when it
 * lies fully inside a reference period, and as a miss when the target has no test
 * report inside it. Miss tolerance and gap length filters do not apply here, the
 * period boundaries come from the source.
*/
std::shared_ptr<EvaluationRequirementResult::Single> Detection::evaluateStatusCycles(
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer, TimePeriodCollection& ref_periods,
        const std::vector<ptime>& cycles)
{
    typedef EvaluationRequirementResult::SingleDetection Result;
    typedef EvaluationDetail                             Detail;
    typedef Result::EvaluationDetails                    Details;
    Details details;

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    std::vector<ptime> tst_times;
    tst_times.reserve(tst_data.size());

    for (const auto& tst_it : tst_data)
        tst_times.push_back(tst_it.first);

    auto detects = [ & ] (size_t i) { return target_data.isTimeStampNotExcluded(tst_times[ i ]); };

    auto walk = DetectionWalk::statusCycleWalk(ref_periods, cycles, tst_times, detects,
                                               DetectionWalk::cyclesPerPeriod(cycles, update_interval_s_));

    for (const auto& miss : walk.misses)
    {
        // the period boundaries are cycle timestamps, not reference report
        // timestamps, so the reference reports inside the period carry the
        // position of the miss
        auto ref_updates = target_data.refChain().positionsBetween(miss.begin, miss.end, false, false);

        if (!ref_updates.empty())
            details.push_back(Detail(miss.end, ref_updates)
                                  .setValue(Result::DetailKey::DiffTOD,
                                            Time::partialSeconds(miss.end - miss.begin))
                                  .setValue(Result::DetailKey::MissOccurred, true)
                                  .setValue(Result::DetailKey::MissedUIs, miss.sum_missed)
                                  .generalComment("Miss detected, no target report in reported"
                                                  " update cycle [" + Time::toString(miss.begin)
                                                  + ", " + Time::toString(miss.end) + "]"));
    }

    auto ret = make_shared<EvaluationRequirementResult::SingleDetection>(
        "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
        calculator_, details, walk.sum_expected, walk.sum_missed, ref_periods);

    return ret;
}

/**
*/
bool Detection::useTimeRatio() const
{
    return use_time_ratio_;
}

/**
*/
bool Detection::useGapCount() const
{
    return use_gap_count_;
}

/**
*/
bool Detection::useStationaryUI() const
{
    return use_stationary_ui_;
}

/**
*/
float Detection::stationaryUI() const
{
    return stationary_ui_s_;
}

/**
*/
float Detection::stationarySpeedThreshold() const
{
    return stationary_speed_threshold_ms_;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> Detection::evaluate (const EvaluationTargetData& target_data, 
                                                                         std::shared_ptr<Base> instance,
                                                                         const SectorLayer& sector_layer)
{
    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.currentStandard().referenceMaxTimeDiff());

    // create ref time periods
    std::vector<std::pair<ptime, bool>> ref_inside;
    {
        const auto& ref_data = target_data.refChain().timestampIndexes();
        ref_inside.reserve(ref_data.size());

        for (auto& ref_it : ref_data)
            ref_inside.emplace_back(ref_it.first, target_data.isTimeStampNotExcluded(ref_it.first)
                                                  && target_data.refPosInside(sector_layer, ref_it));
    }

    TimePeriodCollection ref_periods = DetectionWalk::referencePeriods(ref_inside, max_ref_time_diff);

    // status-message method, when the test data source reports its update cycles.
    // Without cycles the time-difference method below applies. The gap count mode
    // always walks the time differences, so a status message setting on a gap count
    // instance cannot turn it into a detection requirement.
    if (pd_calculation_method_ == "status_message" && !use_gap_count_)
    {
        const auto& cycles = calculator_.testStatusCycles();

        if (cycles.size() >= 2)
            return evaluateStatusCycles(target_data, instance, sector_layer, ref_periods, cycles);
    }

    // walk the test data through the reference periods
    const auto& tst_data = target_data.tstChain().timestampIndexes();

    std::vector<ptime>                           tst_times;
    std::vector<dbContent::TargetReport::DataID> tst_ids;
    tst_times.reserve(tst_data.size());
    tst_ids.reserve(tst_data.size());

    for (const auto& tst_it : tst_data)
    {
        tst_times.push_back(tst_it.first);
        tst_ids.emplace_back(tst_it);
    }

    DetectionWalk::Params params;
    params.miss_params    = missTestParams(update_interval_s_);
    params.use_gap_count  = use_gap_count_;
    params.use_time_ratio = use_time_ratio_;

    auto has_ref_pos = [ & ] (size_t i)
    {
        return target_data.mappedRefPos(tst_ids[ i ], max_ref_time_diff).has_value();
    };

    auto inside_sector = [ & ] (size_t i)
    {
        return target_data.isTimeStampNotExcluded(tst_times[ i ])
            && target_data.mappedRefPosInside(sector_layer, tst_ids[ i ]);
    };

    auto gap_ui = [ & ] (const ptime& t0, const ptime& t1)
    {
        return updateIntervalFor(target_data, t0, t1);
    };

    auto walk = DetectionWalk::walk(ref_periods, tst_times, has_ref_pos, inside_sector, gap_ui, params);

    // create the details from the walk steps
    typedef EvaluationRequirementResult::SingleDetection Result;
    typedef EvaluationDetail                             Detail;
    typedef Result::EvaluationDetails                    Details;
    typedef boost::optional<dbContent::TargetPosition>   OptPos;
    typedef DetectionWalk::Step                          Step;

    Details details;

    auto addDetail = [ & ] (const ptime& ts,
                         const std::vector<dbContent::TargetPosition>& positions,
                         const QVariant& d_tod,
                         const QVariant& miss_occurred,
                         const QVariant& missed_uis,
                         const std::string& comment)
    {
        details.push_back(Detail(ts, positions).setValue(Result::DetailKey::DiffTOD, d_tod)
                              .setValue(Result::DetailKey::MissOccurred, miss_occurred)
                              .setValue(Result::DetailKey::MissedUIs, missed_uis)
                              .generalComment(comment));
    };

    std::vector<dbContent::TargetPosition> ref_updates;

    //stores a single reference update given a test data id
    auto storeRefUpdate = [ & ] (std::vector<dbContent::TargetPosition>& ref_updates,
                              const dbContent::TargetReport::DataID& id_tst)
    {
        //map to ref pos
        auto ref_pos = target_data.mappedRefPos(id_tst);
        traced_assert(ref_pos.has_value());

        ref_updates = { ref_pos.value() };
    };

    //stores all reference updates in the timespan of an interval (plus the reference interpolated end positions of the interval)
    auto storeRefUpdates = [ & ] (std::vector<dbContent::TargetPosition>& ref_updates,
                               const dbContent::TargetReport::DataID& id0,
                               const dbContent::TargetReport::DataID& id1,
                               const boost::optional<dbContent::TargetPosition>& ref_pos0,
                               const boost::optional<dbContent::TargetPosition>& ref_pos1)
    {
        ref_updates.clear();

        //reference-interpolated interval end positions
        boost::optional<dbContent::TargetPosition> pos0 = ref_pos0;
        boost::optional<dbContent::TargetPosition> pos1 = ref_pos1;

        //not passed directly => interpolate now
        if (!pos0.has_value()) pos0 = target_data.mappedRefPos(id0);
        if (!pos1.has_value()) pos1 = target_data.mappedRefPos(id1);

        //interpolation of ref should always be possible, since the period is inside a valid reference period
        traced_assert(pos0.has_value() && pos1.has_value());

        //retrieve all ref updates inside the interval
        auto positions = target_data.refChain().positionsBetween(id0.timestamp(), 
                                                                 id1.timestamp(), 
                                                                 false, 
                                                                 false);
        //collect all updates
        ref_updates.push_back(pos0.value());
        ref_updates.insert(ref_updates.end(), positions.begin(), positions.end());
        ref_updates.push_back(pos1.value());
    };

    std::string thres = String::doubleToStringPrecision(missThreshold(), 2);

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    dbContent::TargetPosition pos_current;
    int   last_tst_index = -1;
    ptime timestamp; // time of the current test report

    for (const auto& step : walk.steps)
    {
        // new test report: its position is the current one (finished periods overwrite it)
        if (step.tst_index >= 0 && step.tst_index != last_tst_index)
        {
            last_tst_index = step.tst_index;
            timestamp      = tst_times[ step.tst_index ];
            pos_current    = target_data.tstChain().pos(tst_ids[ step.tst_index ]);
        }

        double sum_missed_before = step.sum_missed - step.missed;

        switch (step.type)
        {
            case Step::Type::PeriodWithoutTestData:
            {
                if (!step.miss)
                    break;

                pos_current   = target_data.refChain().pos(step.t1);
                auto last_pos = target_data.refChain().pos(step.t0);

                std::string comment = "Miss detected (DToD > " + thres + "), last was " + Time::toString(step.t0);

                storeRefUpdates(ref_updates, step.t0, step.t1, last_pos, pos_current);
                addDetail(step.t1, ref_updates, step.t_diff, true, step.sum_missed, comment);
                break;
            }
            case Step::Type::PeriodEnd:
            {
                pos_current = target_data.refChain().pos(step.t1);

                OptPos last_pos;
                if (!step.t0_is_tst)
                    last_pos = target_data.refChain().pos(step.t0);

                std::string comment;
                if (step.miss)
                    comment = "Miss detected in previous period " + to_string(step.period)
                              + " (DToD > " + thres + "), between [" + Time::toString(step.t0) + ", "
                              + Time::toString(step.t1) + "]\n";
                else
                    comment = "Previous period " + to_string(step.period) + " OK (DToD <= " + thres + ")\n";

                storeRefUpdates(ref_updates, step.t0, step.t1, last_pos, pos_current);
                addDetail(timestamp, ref_updates, step.t_diff, step.miss, step.sum_missed, comment);
                break;
            }
            case Step::Type::OutsideRefPeriods:
            {
                if (!skip_no_data_details)
                    addDetail(timestamp, { pos_current }, {}, false, step.sum_missed,
                              "Outside of reference time periods");
                break;
            }
            case Step::Type::NoRefPos:
            {
                if (!skip_no_data_details)
                    addDetail(timestamp, { pos_current }, {}, false, step.sum_missed,
                              "At exact beginning of reference time period");
                break;
            }
            case Step::Type::OutsideSector:
            {
                if (!skip_no_data_details)
                    addDetail(timestamp, { pos_current }, {}, false, step.sum_missed, "Outside sector");
                break;
            }
            case Step::Type::FirstAfterOutside:
            {
                storeRefUpdate(ref_updates, tst_ids[ step.tst_index ]);
                addDetail(timestamp, ref_updates, {}, false, step.sum_missed, "First target report after outside sector");
                break;
            }
            case Step::Type::FirstInPeriod:
            {
                storeRefUpdate(ref_updates, tst_ids[ step.tst_index ]);
                addDetail(timestamp, ref_updates, {}, false, sum_missed_before,
                          "First target report in period " + to_string(step.period));

                // gap from the period begin
                auto last_pos = target_data.refChain().pos(step.t0);

                std::string comment;
                if (step.miss)
                    comment = "Miss detected in current period " + to_string(step.period)
                              + " (DToD > " + thres + "), between [" + Time::toString(step.t0)
                              + ", " + Time::toString(step.t1) + "]\n";
                else
                    comment = "Current period " + to_string(step.period) + " OK (DToD <= " + thres + ")\n";

                storeRefUpdates(ref_updates, step.t0, step.t1, last_pos, {});
                addDetail(timestamp, ref_updates, step.t_diff, step.miss, step.sum_missed, comment);
                break;
            }
            case Step::Type::Gap:
            {
                std::string comment = step.miss ? "Miss detected (DToD > " + thres + "), last was " + Time::toString(step.t0)
                                                : "OK (DToD <= " + thres + ")";

                storeRefUpdates(ref_updates, step.t0, step.t1, {}, {});
                addDetail(timestamp, ref_updates, step.t_diff, step.miss, step.sum_missed, comment);
                break;
            }
        }
    }

    return make_shared<EvaluationRequirementResult::SingleDetection>(
        "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
        calculator_, details, walk.sum_expected, walk.sum_missed, ref_periods);
}


/**
*/
PDHelpers::MissTestParams Detection::missTestParams(float update_interval_s) const
{
    PDHelpers::MissTestParams p;
    p.update_interval_s  = update_interval_s;
    p.use_miss_tolerance = use_miss_tolerance_;
    p.miss_tolerance_s   = miss_tolerance_s_;
    p.use_min_gap_length = use_min_gap_length_;
    p.min_gap_length_s   = min_gap_length_s_;
    p.use_max_gap_length = use_max_gap_length_;
    p.max_gap_length_s   = max_gap_length_s_;
    return p;
}

/**
 * Gap-specific update interval (ED-129C ORQ 627, APT services): the
 * stationary update interval when the reference ground speed of the report
 * nearest to the gap midpoint is below the speed threshold. Without a
 * usable reference ground speed the normal update interval applies.
*/
float Detection::updateIntervalFor(const EvaluationTargetData& target_data,
                                   const boost::posix_time::ptime& gap_begin,
                                   const boost::posix_time::ptime& gap_end) const
{
    if (!use_stationary_ui_)
        return update_interval_s_;

    ptime mid = gap_begin + (gap_end - gap_begin) / 2;

    auto times = target_data.refChain().findDataMappingTimes(mid);

    boost::optional<float> speed;

    // nearest surrounding reference report with a ground speed
    bool other1_nearer = times.has_other1_ && (!times.has_other2_
                         || (mid - times.timestamp_other1_) <= (times.timestamp_other2_ - mid));

    if (other1_nearer)
    {
        speed = target_data.refChain().groundSpeed(times.dataid_other1_);
        if (!speed.has_value() && times.has_other2_)
            speed = target_data.refChain().groundSpeed(times.dataid_other2_);
    }
    else if (times.has_other2_)
    {
        speed = target_data.refChain().groundSpeed(times.dataid_other2_);
        if (!speed.has_value() && times.has_other1_)
            speed = target_data.refChain().groundSpeed(times.dataid_other1_);
    }

    if (!speed.has_value())
        return update_interval_s_;

    return speed.value() < stationary_speed_threshold_ms_ ? stationary_ui_s_ : update_interval_s_;
}

}
