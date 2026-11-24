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


#include "requirement/dubious/dubioustarget.h"
#include "requirement/dubious/dubious_helpers.h"

#include "results/dubious/dubioustarget.h"

#include "evaluationmanager.h"

#include "stringconv.h"
#include "timeconv.h"
#include "number.h"
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

    DubiousHelpers::Params params;
    params.use_max_groundspeed    = use_max_groundspeed_;
    params.max_groundspeed_kts    = max_groundspeed_kts_;
    params.check_calculated_speed = true;
    params.use_max_acceleration   = use_max_acceleration_;
    params.max_acceleration       = max_acceleration_;
    params.use_max_turnrate       = use_max_turnrate_;
    params.max_turnrate           = max_turnrate_;
    params.use_max_rocd           = use_rocd_;
    params.max_rocd               = max_rocd_;
    params.min_comparison_time    = minimum_comparison_time_;
    params.max_comparison_time    = maximum_comparison_time_;

    std::vector<DubiousHelpers::Update> updates(detail.details.size());
    Transformation trafo;

    for (size_t cnt = 0; cnt < detail.details.size(); ++cnt)
    {
        const auto& update = detail.details[cnt];
        auto id = target_data.tstChain().dataID(update.timestamp());

        auto& u = updates[cnt];
        u.time_s = Time::partialSeconds(update.timestamp() - detail.details.front().timestamp());

        if (auto spd = target_data.tstChain().groundSpeed(id))
            u.speed_ms = *spd;
        if (auto trk = target_data.tstChain().trackAngle(id))
            u.track_angle_deg = *trk;
        if (auto mc = target_data.tstChain().modeC(id))
            u.mode_c_ft = *mc;

        if (cnt > 0)
        {
            const auto& last = detail.details[cnt - 1];

            bool ok;
            double x_pos, y_pos;
            tie(ok, x_pos, y_pos) = trafo.distanceCart(last.position(0).latitude_, last.position(0).longitude_,
                                                       update.position(0).latitude_, update.position(0).longitude_);
            u.distance_to_previous_m = sqrt(pow(x_pos, 2) + pow(y_pos, 2));
        }
    }

    auto flags = DubiousHelpers::checkUpdates(updates, params);

    for (size_t cnt = 0; cnt < detail.details.size(); ++cnt)
    {
        auto& update = detail.details[cnt];
        const auto& f = flags[cnt];

        if (all_updates_dubious) // mark was primary/short track if required
            Result::logComments(update, all_updates_dubious_reasons);

        if (f.reported_speed_ms)
        {
            Result::logComment(update, "MSpd", String::doubleToStringPrecision(*f.reported_speed_ms, 1));
            ++dubious_groundspeed_found;
        }
        else if (f.calculated_speed_kts)
        {
            Result::logComment(update, "CSpd", String::doubleToStringPrecision(*f.calculated_speed_kts, 1));
            ++dubious_groundspeed_found;
        }

        if (f.acceleration)
        {
            Result::logComment(update, "Acc", String::doubleToStringPrecision(*f.acceleration, 1));
            ++dubious_acceleration_found;
        }

        if (f.turnrate)
        {
            Result::logComment(update, "TR", String::doubleToStringPrecision(*f.turnrate, 1));
            ++dubious_turnrate_found;
        }

        if (f.rocd)
        {
            Result::logComment(update, "ROCD", String::doubleToStringPrecision(*f.rocd, 1));
            ++dubious_rocd_found;
        }
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

    detail.is_dubious = DubiousHelpers::isDubious(detail.num_pos_inside_dubious, detail.num_pos_inside, dubious_prob_);

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
