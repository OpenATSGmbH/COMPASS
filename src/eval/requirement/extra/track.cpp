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

#include "eval/requirement/extra/track.h"

#include "eval/results/extra/track.h"

#include "evaluationmanager.h"
#include "sectorlayer.h"

#include "logger.h"
#include "timeperiod.h"

#include <boost/algorithm/string/join.hpp>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/**
*/
ExtraTrack::ExtraTrack(const std::string& name, 
                       const std::string& short_name, 
                       const std::string& group_name,
                       double prob, 
                       COMPARISON_TYPE prob_check_type, 
                       EvaluationCalculator& calculator,
                       float min_duration, 
                       unsigned int min_num_updates, 
                       bool ignore_primary_only)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      min_duration_(Time::partialSeconds(min_duration)),
      min_num_updates_(min_num_updates), 
      ignore_primary_only_(ignore_primary_only)
{
}

/**
*/
float ExtraTrack::minDuration() const
{
    return Time::partialSeconds(min_duration_);
}

/**
*/
unsigned int ExtraTrack::minNumUpdates() const
{
    return min_num_updates_;
}

/**
*/
bool ExtraTrack::ignorePrimaryOnly() const
{
    return ignore_primary_only_;
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> ExtraTrack::evaluate (
        const EvaluationTargetData& target_data, std::shared_ptr<Base> instance,
        const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " min_duration " << min_duration_ << " min_num_updates " << min_num_updates_
           << " ignore_primary_only " << ignore_primary_only_;

    time_duration max_ref_time_diff = Time::partialSeconds(calculator_.settings().max_ref_time_diff_);

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    ptime timestamp;
    bool is_inside;

    bool skip_no_data_details = calculator_.settings().report_skip_no_data_details_;

    boost::optional<unsigned int> track_num;

    // collect track numbers with time periods
    vector<pair<unsigned int, TimePeriod>> finished_tracks;
    map<unsigned int, TimePeriod> active_tracks;

    for (const auto& tst_id : tst_data)
    {
        timestamp = tst_id.first;

        if (!target_data.hasMappedRefData(tst_id, max_ref_time_diff))
            continue;

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.tstPosInside(sector_layer, tst_id);

        if (!is_inside)
            continue;

        track_num = target_data.tstChain().tstTrackNum(tst_id);

        if (!track_num.has_value())
            continue;

        if (!active_tracks.count(*track_num)) // not yet existing
        {
            //active_tracks[track_num] = {tod, tod};
            active_tracks.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(*track_num),  // args for key
                                  std::forward_as_tuple(timestamp, timestamp));
            continue;
        }

        // track num exists in active tracks
        TimePeriod& period = active_tracks.at(*track_num);
        if (!period.isCloseToEnd(timestamp, seconds(300))) // gap, finish old, create new track
        {
            finished_tracks.emplace_back(*track_num, period);
            active_tracks.emplace(std::piecewise_construct,
                                  std::forward_as_tuple(*track_num),  // args for key
                                  std::forward_as_tuple(timestamp, timestamp));
            continue;
        }

        // extend active track
        period.extend(timestamp);
    }

    // finish active tracks
    while (active_tracks.size())
    {
        auto at_it = active_tracks.begin();
        finished_tracks.emplace_back(at_it->first, at_it->second);
        active_tracks.erase(at_it);
    }

    unsigned int num_pos {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_no_track_num {0};
    unsigned int num_extra {0};
    unsigned int num_ok {0};

    typedef EvaluationRequirementResult::SingleExtraTrack Result;
    typedef EvaluationDetail                              Detail;
    typedef Result::EvaluationDetails                     Details;
    Details details;

    //unsigned int extra_time_period_cnt;
    vector<string> extra_track_nums;

    //bool has_track_num = false;
    QVariant track_num_var;

    bool has_tod {false};
    ptime tod_min, tod_max;

    dbContent::TargetPosition tst_pos;

    auto addDetail = [ & ] (const ptime& ts,
                            const dbContent::TargetPosition& tst_pos,
                            const QVariant& inside,
                            const QVariant& track_num,
                            const QVariant& extra,
                            const std::string& comment)
    {
        details.push_back(Detail(ts, tst_pos).setValue(Result::DetailKey::Inside, inside)
                                             .setValue(Result::DetailKey::TrackNum, track_num)
                                             .setValue(Result::DetailKey::Extra, extra)
                                             .generalComment(comment));
    };

    for (const auto& tst_id : tst_data)
    {
        ++num_pos;

        timestamp = tst_id.first;
        tst_pos = target_data.tstChain().pos(tst_id);

//        has_track_num = target_data.tstChain().hasTstTrackNum(tst_id);

//        if (has_track_num)
        track_num = target_data.tstChain().tstTrackNum(tst_id);

        if (track_num.has_value())
            track_num_var = *track_num;
        else
            track_num_var = QVariant::Invalid;

        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.tstPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            if (!skip_no_data_details)
                addDetail(timestamp, tst_pos, false, // inside
                            track_num_var, // track_num
                            false, "Tst outside"); // extra

            ++num_pos_outside;
            continue;
        }
        ++num_pos_inside;

        if (!track_num.has_value())
        {
            addDetail(timestamp, tst_pos, true, // inside
                        track_num_var, // track_num
                        false, "No track num"); // extra
            ++num_no_track_num;
            continue;
        }

        //extra_time_period_cnt = 0;
        extra_track_nums.clear();

        for (auto& per_it : finished_tracks)
        {
            if (per_it.second.isInside(timestamp) && per_it.first != track_num)
                extra_track_nums.push_back(to_string(per_it.first));
        }

        if (extra_track_nums.size())
        {
            ++num_extra;

            if (!has_tod)
            {
                tod_min = timestamp;
                tod_max = timestamp;
                has_tod = true;
            }
            else
            {
                tod_min = min (timestamp, tod_min);
                tod_max = max (timestamp, tod_max);
            }

            addDetail(timestamp, tst_pos, true, // inside
                        track_num_var, // track_num
                        true, // extra
                        "Extra tracks: "+ boost::algorithm::join(extra_track_nums, ","));
        }
        else
        {
            ++num_ok;
            addDetail(timestamp, tst_pos, true, // inside
                        track_num_var, // track_num
                        false, "OK"); // extra
        }
    }

    bool ignore = false;
    string ignore_comment;

    if (num_extra && num_extra < min_num_updates_)
    {
        ignore = true;
        ignore_comment = "Minimum number of updates: "+to_string(num_extra)+" < "+to_string(min_num_updates_);
    }

    if (!ignore && has_tod && (tod_max-tod_min) < min_duration_)
    {
        ignore = true;
        ignore_comment = "Minimum duration: "+Time::toString(tod_max-tod_min,1)+" < "+Time::toString(min_duration_,1);
    }

    if (!ignore && ignore_primary_only_ && target_data.isPrimaryOnly())
    {
        ignore = true;
        ignore_comment = "Ignore primary-only";
    }

    traced_assert(num_pos == num_pos_inside + num_pos_outside);
    traced_assert(num_pos_inside == num_no_track_num + num_extra + num_ok);

    auto ret = make_shared<EvaluationRequirementResult::SingleExtraTrack>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, details, num_pos_inside, num_extra, num_ok);

    if (ignore)                
        ret->setIgnoreResult(ignore_comment);                

    return ret;
}

}
