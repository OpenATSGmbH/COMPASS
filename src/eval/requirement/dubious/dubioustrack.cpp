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


#include "eval/requirement/dubious/dubioustrack.h"

#include "eval/results/dubious/dubioustrack.h"

#include "evaluationmanager.h"

#include "util/stringconv.h"
#include "util/timeconv.h"
#include "util/number.h"
#include "global.h"

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace EvaluationRequirement
{

/**
*/
DubiousTrack::DubiousTrack(const std::string& name, 
                           const std::string& short_name, 
                           const std::string& group_name,
                           bool eval_only_single_ds_id, 
                           unsigned int single_ds_id,
                           float minimum_comparison_time, 
                           float maximum_comparison_time,
                           bool mark_primary_only, 
                           bool use_min_updates, 
                           unsigned int min_updates,
                           bool use_min_duration, 
                           float min_duration,
                           bool use_max_groundspeed, 
                           float max_groundspeed_kts,
                           bool use_max_acceleration, 
                           float max_acceleration,
                           bool use_max_turnrate, 
                           float max_turnrate,
                           bool use_rocd, 
                           float max_rocd, 
                           float dubious_prob,
                           double prob, 
                           COMPARISON_TYPE prob_check_type, 
                           EvaluationCalculator& calculator)
    : ProbabilityBase(name, short_name, group_name, prob, prob_check_type, false, calculator),
      eval_only_single_ds_id_(eval_only_single_ds_id), 
      single_ds_id_(single_ds_id),
      minimum_comparison_time_(minimum_comparison_time), 
      maximum_comparison_time_(maximum_comparison_time),
      mark_primary_only_(mark_primary_only), 
      use_min_updates_(use_min_updates), 
      min_updates_(min_updates),
      use_min_duration_(use_min_duration), 
      min_duration_(Time::partialSeconds(min_duration)),
      use_max_groundspeed_(use_max_groundspeed), 
      max_groundspeed_kts_(max_groundspeed_kts),
      use_max_acceleration_(use_max_acceleration), 
      max_acceleration_(max_acceleration),
      use_max_turnrate_(use_max_turnrate), 
      max_turnrate_(max_turnrate),
      use_rocd_(use_rocd), 
      max_rocd_(max_rocd), 
      dubious_prob_(dubious_prob)
{
}

/**
*/
std::shared_ptr<EvaluationRequirementResult::Single> DubiousTrack::evaluate (const EvaluationTargetData& target_data, 
                                                                             std::shared_ptr<Base> instance,
                                                                             const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " mark_primary_only " << mark_primary_only_ << " prob " << threshold()
           << " use_min_updates " << use_min_updates_ << " min_updates " << min_updates_
           << " use_min_duration " << use_min_duration_ << " min_duration " << min_duration_;

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    typedef EvaluationRequirementResult::SingleDubiousTrack Result;

    boost::optional<unsigned int> track_num;
    bool track_num_missing_reported {false};

    dbContent::TargetPosition tst_pos;

    bool is_inside;

    ptime timestamp;

    unsigned int num_updates {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_pos_inside_dubious {0};

    unsigned int num_tracks {0};
    unsigned int num_tracks_dubious {0};

    map<unsigned int, Result::DetailData> tracks;    // tn -> target
    std::vector<Result::DetailData> finished_tracks; // tn -> target

    auto genDetails = [ & ] ()
    {
        return Result::generateDetails(finished_tracks);
    };

    bool do_not_evaluate_target = false;

    // check for single source only
    if (eval_only_single_ds_id_)
    {
        traced_assert(false); // TODO

        //bool can_check = true;

        //        if (!target_data.canCheckTstMultipleSources())
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " cannot check multiple sources";
        //            can_check = false;
        //        }

        //        if (can_check && target_data.hasTstMultipleSources())
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " failed multiple sources";
        //            can_check = false;
        //        }

        //        if (can_check && !target_data.canCheckTrackLUDSID())
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " cannot check lu_ds_id";
        //            can_check = false;
        //        }

        //        if (can_check && !target_data.hasSingleLUDSID())
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " has no single lu_ds_id";
        //            can_check = false;
        //        }

        //        if (!can_check) // can not check
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " can not check";
        //            do_not_evaluate_target = true;
        //        }
        //        else if (target_data.singleTrackLUDSID() != single_ds_id_) // is not correct
        //        {
        //            //loginf << "UGA utn " << target_data.utn_ << " lu_ds_id not same";
        //            do_not_evaluate_target = true;
        //        }
    }

    for (const auto& tst_id : tst_data)
    {
        timestamp = tst_id.first;

        track_num = target_data.tstChain().tstTrackNum(tst_id);

        if (!track_num.has_value())
        {
            if (!track_num_missing_reported)
            {
                logwrn << "EvaluationRequirementDubiousTrack '" << name_ << "': evaluate: utn " << target_data.utn_
                       << " has no track number at time " << Time::toString(timestamp);
                track_num_missing_reported = true;
            }
            continue;
        }


        ++num_updates;

        // check if inside based on test position only

        tst_pos   = target_data.tstChain().pos(tst_id);
        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.tstPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            ++num_pos_outside;

            if (tracks.count(*track_num)) // exists, left sector
            {
                tracks.at(*track_num).left_sector = true;
            }

            continue;
        }

        // find corresponding track
        if (tracks.count(*track_num)) // exists
        {
            traced_assert(timestamp >= tracks.at(*track_num).tod_end);

            if (timestamp - tracks.at(*track_num).tod_end > seconds(300)) // time gap too large, new track
            {
                finished_tracks.emplace_back(tracks.at(*track_num));
                tracks.erase(*track_num);
            }
        }

        if (!tracks.count(*track_num))
        {
            tracks.emplace(std::piecewise_construct,
                           std::forward_as_tuple(*track_num),  // args for key
                           std::forward_as_tuple(*track_num, timestamp));
        }

        traced_assert(tracks.count(*track_num));

        auto& current_detail = tracks.at(*track_num);

        ++num_pos_inside;
        ++current_detail.num_pos_inside;

        current_detail.details.emplace_back(timestamp, tst_pos);

        if (current_detail.first_inside) // do detail time & pos
        {
            current_detail.tod_begin = timestamp;
            current_detail.tod_end = timestamp;

            current_detail.pos_begin = tst_pos;
            current_detail.pos_last = tst_pos;

            current_detail.first_inside = false;
        }
        else
        {
            current_detail.tod_end = timestamp;
            traced_assert(current_detail.tod_end >= current_detail.tod_begin);
            current_detail.duration = current_detail.tod_end - current_detail.tod_begin;

            current_detail.pos_last = tst_pos;
        }

        // do stats
        if (!current_detail.has_mode_ac
                && (target_data.tstChain().modeA(tst_id).has_value()
                    || target_data.tstChain().modeC(tst_id).has_value()))
            current_detail.has_mode_ac  = true;

        if (!current_detail.has_mode_s
                && (target_data.tstChain().acad(tst_id).has_value()
                    || target_data.tstChain().acid(tst_id).has_value()))
            current_detail.has_mode_s  = true;
    }

    while (tracks.size()) // move all to finished
    {
        finished_tracks.emplace_back(tracks.begin()->second);
        tracks.erase(tracks.begin());
    }

    if (!num_pos_inside)
    {
        return make_shared<EvaluationRequirementResult::SingleDubiousTrack>(
                    "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                    calculator_, genDetails(), num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious,
                    0, 0);
    }

    unsigned int dubious_groundspeed_found;
    unsigned int dubious_acceleration_found;
    unsigned int dubious_turnrate_found;
    unsigned int dubious_rocd_found;

    bool all_updates_dubious; // for short or primary track
    std::map<std::string, std::string> all_updates_dubious_reasons;

    bool has_last_tod;
    ptime last_tod;
    float time_diff;
    float acceleration;
    float turnrate;
    float rocd;

    for (auto& track_detail : finished_tracks)
    {
        all_updates_dubious = false;
        all_updates_dubious_reasons.clear();

        dubious_groundspeed_found = 0;
        dubious_acceleration_found = 0;
        dubious_turnrate_found = 0;
        dubious_rocd_found = 0;

        if (!do_not_evaluate_target && mark_primary_only_ && !track_detail.has_mode_ac && !track_detail.has_mode_s)
        {
            track_detail.dubious_reasons["Pri."] = "";

            all_updates_dubious = true;
            all_updates_dubious_reasons["Pri."] = "";
        }

        if (!do_not_evaluate_target && use_min_updates_ && !track_detail.left_sector
                && track_detail.num_pos_inside < min_updates_)
        {
            track_detail.dubious_reasons["#Up"] = to_string(track_detail.num_pos_inside);

            all_updates_dubious = true;
            all_updates_dubious_reasons["#Up"] = to_string(track_detail.num_pos_inside);
        }

        if (!do_not_evaluate_target && use_min_duration_ && !track_detail.left_sector
                && track_detail.duration < min_duration_)
        {
            track_detail.dubious_reasons["Dur."] = Time::toString(track_detail.duration, 1);

            all_updates_dubious = true;
            all_updates_dubious_reasons["Dur."] = Time::toString(track_detail.duration, 1);
        }

        has_last_tod = false;

        double max_groundspeed_ms = max_groundspeed_kts_ * KNOTS2M_S;

        for (auto& update : track_detail.details)
        {
            if (!do_not_evaluate_target && all_updates_dubious) // mark was primarty/short track if required
                Result::logComments(update, all_updates_dubious_reasons);

            auto id = target_data.tstChain().dataID(update.timestamp());

            auto tst_spd = target_data.tstChain().groundSpeed(id); // m/s

            if (!do_not_evaluate_target && use_max_groundspeed_ && tst_spd.has_value()
                    && *tst_spd > max_groundspeed_ms)
            {
                Result::logComment(update, "Spd",
                                   String::doubleToStringPrecision(*tst_spd, 1));

                ++dubious_groundspeed_found;
            }

            if (has_last_tod)
            {
                traced_assert(update.timestamp() >= last_tod);
                time_diff = Time::partialSeconds(update.timestamp() - last_tod);

                if (!do_not_evaluate_target && time_diff >= minimum_comparison_time_
                        && time_diff <= maximum_comparison_time_)
                {
                    auto id_last = target_data.tstChain().dataID(last_tod);

                    auto tst_spd_last = target_data.tstChain().groundSpeed(id_last); // m/s

                    if (use_max_acceleration_ && tst_spd.has_value() && tst_spd_last.has_value())
                    {
                        acceleration = fabs(*tst_spd - *tst_spd_last) / time_diff;

                        if (acceleration > max_acceleration_)
                        {
                            Result::logComment(update, "Acc",
                                               String::doubleToStringPrecision(acceleration, 1));

                            ++dubious_acceleration_found;
                        }
                    }

                    if (!do_not_evaluate_target && use_max_turnrate_)
                    {
                        auto tst_track_angle = target_data.tstChain().trackAngle(id);
                        auto tst_track_angle_last = target_data.tstChain().trackAngle(id_last);

                        if (tst_track_angle.has_value() && tst_track_angle_last.has_value())
                        {

                            turnrate = Number::calculateMinAngleDifference(*tst_track_angle_last, *tst_track_angle) / time_diff;

                            if (fabs(turnrate) > max_turnrate_)
                            {
                                Result::logComment(update, "TR",
                                                   String::doubleToStringPrecision(turnrate, 1));

                                ++dubious_turnrate_found;
                            }
                        }
                    }

                    if (!do_not_evaluate_target && use_rocd_)
                    {

                        auto tst_mc = target_data.tstChain().modeC(id);
                        auto tst_mc_last = target_data.tstChain().modeC(id_last);

                        if (tst_mc.has_value() && tst_mc_last.has_value())
                        {
                            rocd = fabs(*tst_mc - *tst_mc_last) / time_diff;

                            if (rocd > max_rocd_)
                            {
                                Result::logComment(update, "ROCD",
                                                   String::doubleToStringPrecision(rocd, 1));

                                ++dubious_rocd_found;
                            }
                        }
                    }
                }
            }

            // done
            last_tod = update.timestamp();
            has_last_tod = true;
        }

        if (!do_not_evaluate_target && use_max_groundspeed_ && dubious_groundspeed_found > 0)
        {
            track_detail.dubious_reasons["Spd"] = to_string(dubious_groundspeed_found);
        }

        if (!do_not_evaluate_target && use_max_acceleration_ && dubious_acceleration_found > 0)
        {
            track_detail.dubious_reasons["Acc"] = to_string(dubious_acceleration_found);
        }

        if (!do_not_evaluate_target && use_max_turnrate_ && dubious_turnrate_found > 0)
        {
            track_detail.dubious_reasons["TR"] = to_string(dubious_turnrate_found);
        }

        if (!do_not_evaluate_target && use_rocd_ && dubious_rocd_found > 0)
        {
            track_detail.dubious_reasons["ROCD"] = to_string(dubious_rocd_found);
        }

        track_detail.num_pos_inside_dubious = track_detail.numDubious(); // num of tods with issues

        if (track_detail.num_pos_inside
                && ((float) track_detail.num_pos_inside_dubious / (float)(track_detail.num_pos_inside) > dubious_prob_))
        {
            track_detail.is_dubious = true;
            ++num_tracks_dubious;
        }
        else
        {
            track_detail.is_dubious = false;
        }

        //        loginf << "EvaluationRequirementDubiousTrack '" << name_ << "': evaluate: utn " << target_data.utn_
        //               << " is_dubious_ " << track_detail.is_dubious_
        //               << " num_pos_inside_dubious_ " << track_detail.num_pos_inside_dubious_
        //               << " num_pos_inside_ " << track_detail.num_pos_inside_;

        ++num_tracks;

        num_pos_inside_dubious += track_detail.num_pos_inside_dubious;
    }

    return make_shared<EvaluationRequirementResult::SingleDubiousTrack>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, genDetails(), num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious,
                num_tracks, num_tracks_dubious);
}

/**
*/
bool DubiousTrack::markPrimaryOnly() const
{
    return mark_primary_only_;
}

/**
*/
bool DubiousTrack::useMinUpdates() const
{
    return use_min_updates_;
}

/**
*/
unsigned int DubiousTrack::minUpdates() const
{
    return min_updates_;
}

/**
*/
bool DubiousTrack::useMinDuration() const
{
    return use_min_duration_;
}

/**
*/
float DubiousTrack::minDuration() const
{
    return Time::partialSeconds(min_duration_);
}

/**
*/
bool DubiousTrack::useMaxAcceleration() const
{
    return use_max_acceleration_;
}

/**
*/
float DubiousTrack::maxAcceleration() const
{
    return max_acceleration_;
}

/**
*/
bool DubiousTrack::useMaxTurnrate() const
{
    return use_max_turnrate_;
}

/**
*/
float DubiousTrack::maxTurnrate() const
{
    return max_turnrate_;
}

/**
*/
bool DubiousTrack::useROCD() const
{
    return use_rocd_;
}

/**
*/
float DubiousTrack::maxROCD() const
{
    return max_rocd_;
}

}
