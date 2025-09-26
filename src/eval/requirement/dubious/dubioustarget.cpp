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


#include "eval/requirement/dubious/dubioustarget.h"

#include "eval/results/dubious/dubioustarget.h"

#include "evaluationmanager.h"

#include "util/stringconv.h"
#include "util/timeconv.h"
#include "util/number.h"
#include "global.h"

#include <cmath>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

#include "boost/date_time/posix_time/ptime.hpp"
#include "boost/date_time/time_duration.hpp"

namespace EvaluationRequirement
{

/**
*/
DubiousTarget::DubiousTarget(const std::string& name, 
                             const std::string& short_name, 
                             const std::string& group_name,
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
std::shared_ptr<EvaluationRequirementResult::Single> DubiousTarget::evaluate (const EvaluationTargetData& target_data, 
                                                                              std::shared_ptr<Base> instance,
                                                                              const SectorLayer& sector_layer)
{
    logdbg << "'" << name_ << "': utn " << target_data.utn_
           << " mark_primary_only " << mark_primary_only_ << " prob " << threshold()
           << " use_min_updates " << use_min_updates_ << " min_updates " << min_updates_
           << " use_min_duration " << use_min_duration_ << " min_duration " << min_duration_;

    const auto& tst_data = target_data.tstChain().timestampIndexes();

    dbContent::TargetPosition tst_pos;

    bool is_inside;

    ptime timestamp;

    unsigned int num_updates {0};
    unsigned int num_pos_outside {0};
    unsigned int num_pos_inside {0};
    unsigned int num_pos_inside_dubious {0};

    typedef EvaluationRequirementResult::SingleDubiousTarget Result;
    typedef EvaluationDetail                                 Detail;
    
    std::vector<Result::DetailData> detail_data(1);
    auto& detail = detail_data[ 0 ];

    auto genDetails = [ & ] ()
    {
        return Result::generateDetails(detail_data);
    };

    bool do_not_evaluate_target = false;

    for (const auto& tst_id : tst_data)
    {
        timestamp = tst_id.first;

        ++num_updates;

        // check if inside based on test position only

        tst_pos   = target_data.tstChain().pos(tst_id);
        is_inside = target_data.isTimeStampNotExcluded(timestamp)
                    && target_data.tstPosInside(sector_layer, tst_id);

        if (!is_inside)
        {
            ++num_pos_outside;
            detail.left_sector = true;

            continue;
        }

        detail.left_sector = false;
        ++num_pos_inside;

        if (!detail.first_inside && timestamp - detail.tod_end > seconds(300)) // not first time and time gap too large, skip
            continue;

        ++detail.num_pos_inside;

        detail.details.emplace_back(timestamp, tst_pos);

        if (detail.first_inside) // do detail time & pos
        {
            detail.tod_begin = timestamp;
            detail.tod_end   = timestamp;

            detail.pos_begin = tst_pos;
            detail.pos_last  = tst_pos;

            detail.first_inside = false;
        }
        else
        {
            detail.tod_end = timestamp;

            traced_assert(detail.tod_end >= detail.tod_begin);

            detail.duration = detail.tod_end - detail.tod_begin;
            detail.pos_last = tst_pos;
        }

        // do stats
        if (!detail.has_mode_ac
                && (target_data.tstChain().modeA(tst_id).has_value() || target_data.tstChain().modeC(tst_id).has_value()))
            detail.has_mode_ac = true;

        if (!detail.has_mode_s
                && (target_data.tstChain().acad(tst_id).has_value() || target_data.tstChain().acid(tst_id).has_value()))
            detail.has_mode_s = true;
    }

    if (!num_pos_inside)
    {
        return make_shared<EvaluationRequirementResult::SingleDubiousTarget>(
                    "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                    calculator_, genDetails(), num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious);
    }

    // have data, calculate

    unsigned int dubious_groundspeed_found;
    unsigned int dubious_acceleration_found;
    unsigned int dubious_turnrate_found;
    unsigned int dubious_rocd_found;

    bool all_updates_dubious; // for short or primary track
    std::map<std::string, std::string> all_updates_dubious_reasons;

    bool has_last_tod;
    ptime last_timestamp;
    float time_diff;
    double acceleration;
    double turnrate;
    double rocd;

    all_updates_dubious = false;
    all_updates_dubious_reasons.clear();

    dubious_groundspeed_found = 0;
    dubious_acceleration_found = 0;
    dubious_turnrate_found = 0;
    dubious_rocd_found = 0;

    if (!do_not_evaluate_target && mark_primary_only_ && !detail.has_mode_ac && !detail.has_mode_s)
    {
        detail.dubious_reasons["Pri."] = "";

        all_updates_dubious = true;
        all_updates_dubious_reasons["Pri."] = "";
    }

    if (!do_not_evaluate_target && use_min_updates_ && !detail.left_sector && detail.num_pos_inside < min_updates_)
    {
        detail.dubious_reasons["#Up"] = to_string(detail.num_pos_inside);

        all_updates_dubious = true;
        all_updates_dubious_reasons["#Up"] = to_string(detail.num_pos_inside);
    }

    if (!do_not_evaluate_target && use_min_duration_ && !detail.left_sector && detail.duration < min_duration_)
    {
        detail.dubious_reasons["Dur."] = Time::toString(detail.duration, 1);

        all_updates_dubious = true;
        all_updates_dubious_reasons["Dur."] = Time::toString(detail.duration, 1);
    }

    has_last_tod = false;

    Detail* last_update {nullptr};
    Transformation trafo_;

    bool ok;
    double x_pos, y_pos;
    double distance, t_diff, spd;

    double max_groundspeed_ms = max_groundspeed_kts_ * KNOTS2M_S;

    for (auto& update : detail.details)
    {
        if (!do_not_evaluate_target && all_updates_dubious) // mark was primarty/short track if required
            Result::logComments(update, all_updates_dubious_reasons);

        auto id = target_data.tstChain().dataID(update.timestamp());

        auto tst_spd = target_data.tstChain().groundSpeed(id); // m/s

        if (!do_not_evaluate_target && use_max_groundspeed_)
        {
            if (tst_spd.has_value() && *tst_spd > max_groundspeed_ms)
            {
                Result::logComment(update, "MSpd", String::doubleToStringPrecision(*tst_spd, 1));

                ++dubious_groundspeed_found;
            }
            else if (last_update) // check last speed to last update
            {
                tie(ok, x_pos, y_pos) = trafo_.distanceCart(
                            last_update->position(0).latitude_, last_update->position(0).longitude_,
                            update.position(0).latitude_, update.position(0).longitude_);
                distance = sqrt(pow(x_pos, 2) + pow(y_pos, 2));
                t_diff = Time::partialSeconds(update.timestamp() - last_update->timestamp());
                traced_assert(t_diff >= 0);

                if (t_diff > 0)
                {
                    spd = M_S2KNOTS * distance / t_diff;

                    if(spd > max_groundspeed_kts_)
                    {
                        Result::logComment(update, "CSpd",
                                           String::doubleToStringPrecision(spd, 1));
                        ++dubious_groundspeed_found;
                    }
                }
            }
        }

        if (has_last_tod)
        {
            traced_assert(update.timestamp() >= last_timestamp);
            time_diff = Time::partialSeconds(update.timestamp() - last_timestamp);

            if (!do_not_evaluate_target && time_diff >= minimum_comparison_time_
                    && time_diff <= maximum_comparison_time_)
            {
                auto id_last = target_data.tstChain().dataID(last_timestamp);

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
                            Result::logComment(update, "ROCD", String::doubleToStringPrecision(rocd, 1));

                            ++dubious_rocd_found;
                        }
                    }
                }
            }

            last_update = &update;
        }

        // done
        last_timestamp = update.timestamp();
        has_last_tod   = true;
    }

    if (!do_not_evaluate_target && use_max_groundspeed_ && dubious_groundspeed_found > 0)
    {
        detail.dubious_reasons["Spd"] = to_string(dubious_groundspeed_found);
    }

    if (!do_not_evaluate_target && use_max_acceleration_ && dubious_acceleration_found > 0)
    {
        detail.dubious_reasons["Acc"] = to_string(dubious_acceleration_found);
    }

    if (!do_not_evaluate_target && use_max_turnrate_ && dubious_turnrate_found > 0)
    {
        detail.dubious_reasons["TR"] = to_string(dubious_turnrate_found);
    }

    if (!do_not_evaluate_target && use_rocd_ && dubious_rocd_found > 0)
    {
        detail.dubious_reasons["ROCD"] = to_string(dubious_rocd_found);
    }

    detail.num_pos_inside_dubious = detail.numDubious(); // num of tods with issues

    if (detail.num_pos_inside && ((float) detail.num_pos_inside_dubious /(float)(detail.num_pos_inside) > dubious_prob_))
    {
        detail.is_dubious = true;
    }
    else
    {
        detail.is_dubious = false;
    }

    //        loginf << "'" << name_ << "': utn " << target_data.utn_
    //               << " is_dubious_ " << track_detail.is_dubious_
    //               << " num_pos_inside_dubious_ " << track_detail.num_pos_inside_dubious_
    //               << " num_pos_inside_ " << track_detail.num_pos_inside_;

    num_pos_inside_dubious += detail.num_pos_inside_dubious;

    return make_shared<EvaluationRequirementResult::SingleDubiousTarget>(
                "UTN:"+to_string(target_data.utn_), instance, sector_layer, target_data.utn_, &target_data,
                calculator_, genDetails(), num_updates, num_pos_outside, num_pos_inside, num_pos_inside_dubious);
}

/**
*/
bool DubiousTarget::markPrimaryOnly() const
{
    return mark_primary_only_;
}

/**
*/
bool DubiousTarget::useMinUpdates() const
{
    return use_min_updates_;
}

/**
*/
unsigned int DubiousTarget::minUpdates() const
{
    return min_updates_;
}

/**
*/
bool DubiousTarget::useMinDuration() const
{
    return use_min_duration_;
}

/**
*/
float DubiousTarget::minDuration() const
{
    return Time::partialSeconds(min_duration_);
}

/**
*/
bool DubiousTarget::useMaxAcceleration() const
{
    return use_max_acceleration_;
}

/**
*/
float DubiousTarget::maxAcceleration() const
{
    return max_acceleration_;
}

/**
*/
bool DubiousTarget::useMaxTurnrate() const
{
    return use_max_turnrate_;
}

/**
*/
float DubiousTarget::maxTurnrate() const
{
    return max_turnrate_;
}

/**
*/
bool DubiousTarget::useROCD() const
{
    return use_rocd_;
}

/**
*/
float DubiousTarget::maxROCD() const
{
    return max_rocd_;
}

}
