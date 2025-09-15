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

#include "eval/requirement/detection/detection.h"

#include "eval/results/detection/detection.h"

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
                     bool hold_for_any_target, bool ignore_primary_only)
    : ProbabilityBase     (name, short_name, group_name, prob, prob_check_type, invert_prob, calculator, hold_for_any_target),
    update_interval_s_  (update_interval_s),
    use_min_gap_length_ (use_min_gap_length),
    min_gap_length_s_   (min_gap_length_s),
    use_max_gap_length_ (use_max_gap_length),
    max_gap_length_s_   (max_gap_length_s),
    use_miss_tolerance_ (use_miss_tolerance),
    miss_tolerance_s_   (miss_tolerance_s),
    ignore_primary_only_(ignore_primary_only)
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

bool Detection::ignorePrimaryOnly() const
{
    return ignore_primary_only_;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> Detection::evaluate (const EvaluationTargetData& target_data, 
                                                                         std::shared_ptr<Base> instance,
                                                                         const SectorLayer& sector_layer)
{
    bool debug = false; //target_data.utn_ == 613;

    if (debug)
        loginf << "'" << name_ << ": utn " << target_data.utn_
               << " update_interval " << update_interval_s_ << " prob " << threshold()
               << " use_miss_tolerance " << use_miss_tolerance_ << " miss_tolerance " << miss_tolerance_s_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.settings().max_ref_time_diff_);

    // create ref time periods
    TimePeriodCollection ref_periods;

    boost::optional<dbContent::TargetPosition> ref_pos;
    ptime timestamp, last_ts;

    bool is_inside, was_inside;
    {
        const auto& ref_data = target_data.refChain().timestampIndexes();
        bool first {true};

        for (auto& ref_it : ref_data)
        {
            timestamp = ref_it.first;
            was_inside = is_inside;

            is_inside = target_data.isTimeStampNotExcluded(timestamp)
                     && target_data.refPosInside(sector_layer, ref_it);

            if (first)
            {
                if (is_inside) // create time period
                    ref_periods.add({timestamp, timestamp});

                first = false;

                continue;
            }

            // not first, was_inside is valid

            if (was_inside)
            {
                if (is_inside)
                {
                    // extend last time period, if possible, or finish last and create new one
                    if (ref_periods.lastPeriod().isCloseToEnd(timestamp, max_ref_time_diff)) // 4.9
                        ref_periods.lastPeriod().extend(timestamp);
                    else
                        ref_periods.add({timestamp, timestamp});
                }
            }
            else if (is_inside) // was not inside and is now inside
                ref_periods.add({timestamp, timestamp}); // create new time period
        }
    }
    ref_periods.removeSmallPeriods(Time::partialSeconds(1));

    if (debug)
        loginf << "'" << name_ << ": utn " << target_data.utn_
               << " periods '" << ref_periods.print() << "'";

    timestamp = {};
    last_ts   = {};

    // evaluate test data
    const auto& tst_data = target_data.tstChain().timestampIndexes();

    int sum_uis = ref_periods.getUIs(update_interval_s_);

    float t_diff;
    int sum_missed_uis {0};
    bool was_outside {false};
    is_inside = false;
    //bool ok;

    typedef EvaluationRequirementResult::SingleDetection Result;
    typedef EvaluationDetail                             Detail;
    typedef Result::EvaluationDetails                    Details;
    Details details;

    dbContent::TargetPosition pos_current;

    unsigned int tst_data_size = tst_data.size();

    auto addDetail = [ & ] (const ptime& ts,
                         const std::vector<dbContent::TargetPosition>& positions,
                         const QVariant& d_tod,
                         const QVariant& miss_occurred,
                         const QVariant& ref_exists,
                         const QVariant& missed_uis,
                         const QVariant& max_gap_uis,
                         const QVariant& no_ref_uis,
                         const std::string& comment)
    {
        details.push_back(Detail(ts, positions).setValue(Result::DetailKey::DiffTOD, d_tod)
                              .setValue(Result::DetailKey::MissOccurred, miss_occurred)
                              .setValue(Result::DetailKey::RefExists, ref_exists)
                              .setValue(Result::DetailKey::MissedUIs, missed_uis)
                              .setValue(Result::DetailKey::MaxGapUIs, max_gap_uis)
                              .setValue(Result::DetailKey::NoRefUIs, no_ref_uis)
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

    if (!tst_data_size) // if not test data, add everything as misses
    {
        if (debug)
            loginf << "'" << name_ << ": utn " << target_data.utn_
                   << " no tst_data_size";

        for (auto& period_it : ref_periods)
        {
            last_ts = period_it.begin();
            timestamp = period_it.end();

            t_diff = Time::partialSeconds(timestamp - last_ts);

            if (isMiss(t_diff))
            {
                sum_missed_uis += getNumMisses(t_diff);

                pos_current = target_data.refChain().pos(timestamp);

                if (debug)
                    loginf << "'" << name_ << ": utn " << target_data.utn_
                           << " miss of " << String::timeStringFromDouble(t_diff)
                           << " uis " << getNumMisses(t_diff)
                           << " at [" << Time::toString(last_ts)
                           << "," << Time::toString(timestamp)
                           << "] sum_missed_uis " << sum_missed_uis;

                string comment = "Miss detected (DToD > "
                                 +String::doubleToStringPrecision(missThreshold(), 2)
                                 +"), last was "+Time::toString(last_ts);

                auto last_pos = target_data.refChain().pos(last_ts);

                storeRefUpdates(ref_updates, last_ts, timestamp, last_pos, pos_current);
                addDetail(timestamp, ref_updates, t_diff, true, true, sum_missed_uis, 0, 0, comment); 
            }
        }

        auto ret = make_shared<EvaluationRequirementResult::SingleDetection>(
            "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
            calculator_, details, sum_uis, sum_missed_uis, ref_periods);

        if (ignore_primary_only_ && target_data.isPrimaryOnly())
            ret->setIgnoreResult("Primary-only");

        return ret;
    }

    // collect test times in ref periods
    map<unsigned int, ptime> period_last_tst_times; // period number -> lst tst times
    set<unsigned int> finalized_periods;

    bool is_inside_ref_time_period {false};//, was_inside_ref_time_period {false};
    unsigned int period_index {0};

    set<unsigned int> finished_periods;

    int period_max_index_before_ts;

    typedef boost::optional<dbContent::TargetPosition> OptPos;

    string comment;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    for (const auto& tst_it : tst_data)
    {
        comment = "";

        last_ts = timestamp;
        timestamp = tst_it.first;
        pos_current = target_data.tstChain().pos(tst_it);

        if (debug)
            loginf << "'" << name_ << ": utn " << target_data.utn_ << " tod " << timestamp;

        //was_inside_ref_time_period = is_inside_ref_time_period;
        is_inside_ref_time_period = ref_periods.isInside(timestamp);

        // check if previous periods need to be finalized
        period_max_index_before_ts = ref_periods.getPeriodMaxIndexBefore(timestamp);

        if (debug)
            loginf << "'" << name_ << ": utn " << target_data.utn_
                   << " is_inside_ref_time_period " << is_inside_ref_time_period
                   << " period_max_index_before_ts " << period_max_index_before_ts;

        if (period_max_index_before_ts != -1)
        {
            traced_assert(period_max_index_before_ts >= 0);

            for (unsigned int period_cnt=0; period_cnt <= static_cast<unsigned int>(period_max_index_before_ts); ++period_cnt)
            {
                if (!finished_periods.count(period_cnt)) // previous not finalized
                {
                    ptime last_period_ts = ref_periods.period(period_cnt).begin();
                    bool tst_time_found = false;
                    ptime last_period_ts_end = ref_periods.period(period_cnt).end();

                    if (period_last_tst_times.count(period_cnt))
                    {
                        last_period_ts = period_last_tst_times.at(period_cnt);
                        tst_time_found = true;
                    }

                    traced_assert(last_period_ts_end >= last_period_ts);

                    t_diff = Time::partialSeconds(last_period_ts_end - last_period_ts);

                    pos_current = target_data.refChain().pos(last_period_ts_end);

                    boost::optional<dbContent::TargetPosition> last_pos;

                    if (tst_time_found)
                        last_pos = target_data.tstChain().pos(last_period_ts);
                    else
                        last_pos = target_data.refChain().pos(last_period_ts);
                    
                    if (isMiss(t_diff))
                    {
                        sum_missed_uis += getNumMisses(t_diff);

                        if (debug)
                            loginf << "'" << name_ << ": utn " << target_data.utn_
                                   << " miss of " << String::timeStringFromDouble(t_diff)
                                   << " uis " << getNumMisses(t_diff)
                                   << " at [" << Time::toString(last_period_ts)
                                   << "," << Time::toString(ref_periods.period(period_cnt).end())
                                   << "] missed_uis " << sum_missed_uis;

                        comment = "Miss detected in previous period "+to_string(period_cnt)
                                  +" (DToD > " +String::doubleToStringPrecision(missThreshold(), 2)
                                  +"), between ["+Time::toString(last_period_ts)+", "
                                  +Time::toString(ref_periods.period(period_cnt).end())+"]\n";

                        storeRefUpdates(ref_updates, last_period_ts, last_period_ts_end, tst_time_found ? OptPos() : OptPos(last_pos), pos_current);
                        addDetail(timestamp, ref_updates, t_diff, true, true, sum_missed_uis, 0, 0, comment);
                    }
                    else
                    {
                        comment = "Previous period "+to_string(period_cnt)
                                  +" OK (DToD <= "+String::doubleToStringPrecision(missThreshold(), 2)+")\n";

                        storeRefUpdates(ref_updates, last_period_ts, last_period_ts_end,
                                        tst_time_found ? OptPos() : OptPos(last_pos), pos_current);
                        addDetail(timestamp, ref_updates, t_diff, false, true, sum_missed_uis, 0, 0, comment);
                    }

                    finished_periods.insert(period_cnt);
                }
            }
        }

        if (!is_inside_ref_time_period)
        {
            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_ << " outside ref time perionds";

            if (!skip_no_data_details)
                addDetail(timestamp, { pos_current }, {}, false, false, sum_missed_uis, 0, 0,
                          "Outside of reference time periods");

            // TODO undetected previous miss possible
            continue;
        }

        // is inside period
        period_index = ref_periods.getPeriodIndex(timestamp);

        ref_pos = target_data.mappedRefPos(tst_it, max_ref_time_diff, debug);

        //        ref_pos = ret_pos.first;
        //        ok = ret_pos.second;
        //assert (ok); // must be since inside ref time interval

        if (!ref_pos.has_value()) // only happens if test time is exact beginning of reference interval
        {
            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_ << " no ref_pos value";

            if (!skip_no_data_details)
                addDetail(timestamp, { pos_current }, {}, false, true, sum_missed_uis, 0, 0,
                          "At exact beginning of reference time period");

            continue;
        }

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.mappedRefPosInside(sector_layer, tst_it);

        if (!is_inside)
        {
            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_
                       << " outside";

            if (!skip_no_data_details)
                addDetail(timestamp, { pos_current }, {}, false, true, sum_missed_uis, 0, 0, "Outside sector");

            was_outside = true;

            //++num_pos_outside;
            continue;
        }

        if (!period_last_tst_times.count(period_index) || was_outside) // first in period or was outside
        {
            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_
                       << " first in period " << period_index <<" (" << period_last_tst_times.count(period_index)
                       << ") or was_outside " << was_outside;

            if (was_outside)
            {
                storeRefUpdate(ref_updates, tst_it);
                addDetail(timestamp, ref_updates, {}, false, true, sum_missed_uis, 0, 0, "First target report after outside sector");
            }
            else // first in period
            {
                storeRefUpdate(ref_updates, tst_it);
                addDetail(timestamp, ref_updates, {}, false, true, sum_missed_uis, 0, 0, "First target report in period " + to_string(period_index));

                // check if begin time in period is miss

                t_diff = Time::partialSeconds(timestamp - ref_periods.period(period_index).begin());

                auto last_pos = target_data.refChain().pos(ref_periods.period(period_index).begin());

                if (isMiss(t_diff))
                {
                    sum_missed_uis += getNumMisses(t_diff);

                    if (debug)
                       loginf << "'" << name_ << ": utn " << target_data.utn_
                               << " miss of " << String::timeStringFromDouble(t_diff)
                               << " uis " << getNumMisses(t_diff)
                               << " at [" << Time::toString(ref_periods.period(period_index).begin())
                               << "," << Time::toString(timestamp)
                               << "] missed_uis " << sum_missed_uis;

                    comment = "Miss detected in current period " + to_string(period_index)
                              +" (DToD > " +String::doubleToStringPrecision(missThreshold(), 2)
                              +"), between ["+Time::toString(ref_periods.period(period_index).begin())
                              +", "+Time::toString(timestamp)+"]\n";

                    storeRefUpdates(ref_updates, ref_periods.period(period_index).begin(), timestamp, last_pos, {});
                    addDetail(timestamp, ref_updates, t_diff, true, false, sum_missed_uis, 0, 0, comment);
                }
                else
                {
                    std::string comment = "Current period " + to_string(period_index)
                                          + " OK (DToD <= " + String::doubleToStringPrecision(missThreshold(), 2) + ")\n";

                    storeRefUpdates(ref_updates, ref_periods.period(period_index).begin(), timestamp, last_pos, {});
                    addDetail(timestamp, ref_updates, t_diff, false, false, sum_missed_uis, 0, 0, comment);
                }
            }

            was_outside = false;
            period_last_tst_times[period_index] = timestamp;

            continue;
        }

        traced_assert(timestamp >= last_ts);
        t_diff = Time::partialSeconds(timestamp - last_ts);

        if (debug)
            loginf << "'" << name_ << ": utn " << target_data.utn_
                   << " ts " << Time::toString(timestamp) << " d_tod " << String::timeStringFromDouble(t_diff);

        if (isMiss(t_diff))
        {
            sum_missed_uis += getNumMisses(t_diff);

            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_
                       << " miss of " << String::timeStringFromDouble(t_diff)
                       << " uis " << getNumMisses(t_diff)
                       << " at [" << Time::toString(last_ts)
                       << "," << Time::toString(timestamp)
                       << "] sum_missed_uis " << sum_missed_uis;

            string comment = "Miss detected (DToD > "
                             +String::doubleToStringPrecision(missThreshold(), 2)
                             +"), last was "+Time::toString(last_ts);
            
            storeRefUpdates(ref_updates, last_ts, timestamp, {}, {});
            addDetail(timestamp, ref_updates, t_diff, true, true, sum_missed_uis, 0, 0, comment);
        }
        else
        {
            if (debug)
                loginf << "'" << name_ << ": utn " << target_data.utn_
                       << " update ok";

            std::string comment = "OK (DToD <= " + String::doubleToStringPrecision(missThreshold(), 2) + ")";

            storeRefUpdates(ref_updates, last_ts, timestamp, {}, {});
            addDetail(timestamp, ref_updates, t_diff, false, true, sum_missed_uis, 0, 0, comment);
        }

        period_last_tst_times[period_index] = timestamp;
    }

    // finalize unfinished periods
    if (debug)
        loginf << "'" << name_ << ": utn " << target_data.utn_ << " finalizing " << ref_periods.size()
               << " unfinished periods";

    for (unsigned int period_cnt=0; period_cnt < ref_periods.size(); ++period_cnt)
    {
        if (!finished_periods.count(period_cnt)) // previous not finalized
        {
            ptime last_period_tod = ref_periods.period(period_cnt).begin();
            bool tst_time_found = false;
            ptime last_period_end = ref_periods.period(period_cnt).end();

            if (period_last_tst_times.count(period_cnt))
            {
                last_period_tod = period_last_tst_times.at(period_cnt);
                tst_time_found = true;
            }

            traced_assert(last_period_end >= last_period_tod);

            t_diff = Time::partialSeconds(last_period_end - last_period_tod);

            pos_current = target_data.refChain().pos(last_period_end);

            boost::optional<dbContent::TargetPosition> last_pos;

            if (tst_time_found)
                last_pos = target_data.tstChain().pos(last_period_tod);
            else
                last_pos = target_data.refChain().pos(last_period_tod);

            if (isMiss(t_diff))
            {
                sum_missed_uis += getNumMisses(t_diff); // TODO substract miss_tolerance_s_?

                if (debug)
                    loginf << "'" << name_ << ": utn " << target_data.utn_
                           << " miss of " << String::timeStringFromDouble(t_diff)
                           << " uis " << getNumMisses(t_diff)
                           << " at [" << Time::toString(last_period_tod) << ","
                           << Time::toString(ref_periods.period(period_cnt).end())
                           << "] missed_uis " << sum_missed_uis;

                comment = "Miss detected in previous period "+to_string(period_cnt)
                          +" (DToD > " +String::doubleToStringPrecision(missThreshold(), 2)
                          +"), between ["+Time::toString(last_period_tod)+", "
                          +Time::toString(ref_periods.period(period_cnt).end())+"]\n";

                storeRefUpdates(ref_updates, last_period_tod, last_period_end, tst_time_found ? OptPos() : OptPos(last_pos), pos_current);
                addDetail(timestamp, ref_updates, t_diff, true, true, sum_missed_uis, 0, 0, comment);
            }
            else
            {
                comment = "Previous period "+to_string(period_cnt) +" OK (DToD <= "+String::doubleToStringPrecision(missThreshold(), 2)+")\n";

                storeRefUpdates(ref_updates, last_period_tod, last_period_end, tst_time_found ? OptPos() : OptPos(last_pos), pos_current);
                addDetail(timestamp, ref_updates, t_diff, false, true, sum_missed_uis, 0, 0, comment);
            }

            finished_periods.insert(period_cnt);
        }
    }

    if (debug)
        loginf << "'" << name_ << ": utn " << target_data.utn_
               << " sum_uis " << sum_uis;

    auto ret = make_shared<EvaluationRequirementResult::SingleDetection>(
        "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
        calculator_, details, sum_uis, sum_missed_uis, ref_periods);

    if (ignore_primary_only_ && target_data.isPrimaryOnly())
        ret->setIgnoreResult("Primary-only");

    return ret;
}

/**
*/
bool Detection::isMiss (float d_tod) const
{
    if (use_miss_tolerance_)
        d_tod -= miss_tolerance_s_;

    if (use_min_gap_length_ && d_tod < min_gap_length_s_) // supress gaps smaller as min gap length
        return false;

    if (use_max_gap_length_ && d_tod > max_gap_length_s_) // supress gaps larger as max gap length
        return false;

    return d_tod > update_interval_s_;
}

/**
*/
unsigned int Detection::getNumMisses(float d_tod) const
{
    traced_assert(isMiss(d_tod));

    if (use_miss_tolerance_)
        d_tod -= miss_tolerance_s_;

    return floor(d_tod/update_interval_s_);
}

}
