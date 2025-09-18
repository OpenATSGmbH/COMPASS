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

#include "reconstructortarget.h"
#include "reconstructorbase.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "targetreportaccessor.h"
#include "buffer.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "global.h"
#include "kalman_chain.h"
#include "fftmanager.h"
#include "timeddataseries.h"
#include "datasourcemanager.h"

#include <boost/optional/optional_io.hpp>

#include <GeographicLib/LocalCartesian.hpp>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;
using namespace nlohmann;

namespace dbContent 
{

const double ReconstructorTarget::on_ground_max_alt_ft_ {500};
const double ReconstructorTarget::on_ground_max_speed_ms_ {30};

ReconstructorTarget::GlobalStats ReconstructorTarget::global_stats_ = ReconstructorTarget::GlobalStats();

/********************************************************************************************
 * ContributingSourcesInfo
 ********************************************************************************************/

void ContributingSourcesInfo::add(const dbContent::targetReport::ReconstructorInfo& tr,
     bool add_to_rec_nums)
{
    logdbg << "timestamp " << Time::toString(tr.timestamp_);

    // update timestamps
    increaseTimeTo(tr.timestamp_);

    switch (tr.ds_type_)
    {
        case DataSourceType::ADSB:
            adsb_age_ = 0.0;
            break;
        case DataSourceType::MLAT:
            mlat_age_ = 0.0;
            break;
        case DataSourceType::Radar:
            radar_age_ = 0.0;
            break;
        case DataSourceType::RefTraj:
            reftraj_age_ = 0.0;
            break;
        case DataSourceType::Tracker:
            tracker_age_ = 0.0;
            break;            
        case DataSourceType::Other:
            other_age_ = 0.0;
            break;            
        default:
            logerr << "unknown ds type " << DataSourceManager::stringFromType(tr.ds_type_);
            break;
    }    

    if (tr.isPrimaryOnlyDetection())
        primary_age_ = 0.0;

    if (tr.isModeACDetection())
        mode_ac_age_ = 0.0;

    if (tr.isModeSDetection())
        modes_age_ = 0.0;

    if (add_to_rec_nums)
        rec_nums_.push_back(tr.record_num_);
}

void ContributingSourcesInfo::increaseTimeTo(boost::posix_time::ptime new_timestamp)
{
    if (!timestamp_.is_not_a_date_time()) // increase ages of valid time exists
    {
        if (timestamp_ > new_timestamp)
            logerr << " timestamp " << Time::toString(timestamp_) 
                   << " new_timestamp " << Time::toString(new_timestamp) 
                   << " diff " << (new_timestamp - timestamp_).total_milliseconds() / 1000.0;

        traced_assert(new_timestamp >= timestamp_);
        auto time_to_add = (new_timestamp - timestamp_).total_milliseconds() / 1000.0;

        if (adsb_age_)
        {
            *adsb_age_ += time_to_add;
            if (*adsb_age_ > 60.0)
                adsb_age_ = {};
        }

        if (mlat_age_)
        {
            *mlat_age_ += time_to_add;
            if (*mlat_age_ > 60.0)
                mlat_age_ = {};
        }

        if (radar_age_)
        {
            *radar_age_ += time_to_add;
            if (*radar_age_ > 60.0)
                radar_age_ = {};
        }

        if (tracker_age_)
        {
            *tracker_age_ += time_to_add;
            if (*tracker_age_ > 60.0)
                tracker_age_ = {};
        }

        if (reftraj_age_)
        {
            *reftraj_age_ += time_to_add;
            if (*reftraj_age_ > 60.0)
                reftraj_age_ = {};
        }

        if (primary_age_)
        {
            *primary_age_ += time_to_add;
            if (*primary_age_ > 60.0)
                primary_age_ = {};
        }

        if (mode_ac_age_)
        {
            *mode_ac_age_ += time_to_add;
            if (*mode_ac_age_ > 60.0)
                mode_ac_age_ = {};
        }

        if (modes_age_)
        {
            *modes_age_ += time_to_add;
            if (*modes_age_ > 60.0)
                modes_age_ = {};
        }
    }

    logdbg << "new_timestamp " << Time::toString(new_timestamp);
    timestamp_ = new_timestamp;
}

nlohmann::json ContributingSourcesInfo::contributions(ReconstructorBase& reconstructor) const
{
    json ret;

    for (auto rec_num : rec_nums_)
    {
        traced_assert (reconstructor.target_reports_.count(rec_num));
        auto& tr = reconstructor.target_reports_.at(rec_num);

        json contrib;

        contrib["rec_num"] = rec_num;
        contrib["ds_id"] = tr.ds_id_;
        contrib["ds_type"] = tr.ds_type_;
        
        ret.push_back(std::move(contrib));
    }

    return ret;
}

/********************************************************************************************
 * ReconstructorTarget::StandingADSBTarget
 ********************************************************************************************/

boost::posix_time::time_duration ReconstructorTarget::StandingADSBTarget::TimeIncrement = boost::posix_time::seconds(1);
unsigned int                     ReconstructorTarget::StandingADSBTarget::MaxUpdates    = 4;

void ReconstructorTarget::StandingADSBTarget::init(const dbContent::targetReport::ReconstructorInfo& tr)
{
    rec_num        = tr.record_num_;
    ts_init        = tr.timestamp_;
    ts_last_update = tr.timestamp_;
    ts_next_update = tr.timestamp_ + TimeIncrement;
    num_updated    = 0;
}

bool ReconstructorTarget::StandingADSBTarget::needsUpdate(const boost::posix_time::ptime& ts) const
{
    return (!isOutdated() && ts_next_update < ts);
}

bool ReconstructorTarget::StandingADSBTarget::isOutdated() const
{
    return num_updated >= MaxUpdates;
}

void ReconstructorTarget::StandingADSBTarget::addUpdate()
{
    traced_assert(!isOutdated());

    ts_last_update  = ts_next_update;
    ts_next_update += TimeIncrement;
    num_updated    += 1;
}

/********************************************************************************************
 * ReconstructorTarget
 ********************************************************************************************/

ReconstructorTarget::ReconstructorTarget(ReconstructorBase& reconstructor, 
                                         unsigned int utn,
                                         bool multithreaded_predictions,
                                         bool dynamic_insertions)
    :   reconstructor_(reconstructor)
    ,   utn_(utn)
    ,   multithreaded_predictions_(multithreaded_predictions)
    ,   dynamic_insertions_(dynamic_insertions)
{
    chain();
}

ReconstructorTarget::~ReconstructorTarget()
{
    //chain().reset(nullptr);
}

void ReconstructorTarget::addUpdateToGlobalStats(const reconstruction::UpdateStats& s)
{
    global_stats_.num_standing_adsb               += s.num_standing_adsb;
    global_stats_.num_standing_adsb_updates_min    = std::min(global_stats_.num_standing_adsb_updates_min, s.num_standing_adsb_updates_min);
    global_stats_.num_standing_adsb_updates_max    = std::max(global_stats_.num_standing_adsb_updates_max, s.num_standing_adsb_updates_max);
    global_stats_.num_standing_adsb_updates_total += s.num_standing_adsb_updates_total;

    global_stats_.num_chain_checked                 += s.num_checked;
    global_stats_.num_chain_skipped_preempt         += s.num_skipped_preemptive;
    global_stats_.num_chain_replaced                += s.num_replaced;
    global_stats_.num_chain_added                   += s.num_inserted;
    global_stats_.num_chain_fresh                   += s.num_fresh;
    global_stats_.num_chain_updates                 += s.num_updated;
    global_stats_.num_chain_updates_valid           += s.num_valid;
    global_stats_.num_chain_updates_failed          += s.num_failed;
    global_stats_.num_chain_updates_failed_numeric  += s.num_failed_numeric;
    global_stats_.num_chain_updates_failed_badstate += s.num_failed_badstate;
    global_stats_.num_chain_updates_failed_other    += s.num_failed_other;
    global_stats_.num_chain_updates_skipped         += s.num_skipped;

    global_stats_.num_chain_updates_proj_changed += s.num_proj_changed;


}

void ReconstructorTarget::addPredictionToGlobalStats(const reconstruction::PredictionStats& s)
{
    global_stats_.num_chain_predictions                 += s.num_predictions;
    global_stats_.num_chain_predictions_failed          += s.num_failed;
    global_stats_.num_chain_predictions_failed_numeric  += s.num_failed_numeric;
    global_stats_.num_chain_predictions_failed_badstate += s.num_failed_badstate;
    global_stats_.num_chain_predictions_failed_other    += s.num_failed_other;
    global_stats_.num_chain_predictions_fixed           += s.num_fixed;

    global_stats_.num_chain_predictions_proj_changed += s.num_proj_changed;
}

void ReconstructorTarget::addTargetReport (unsigned long rec_num,
                                           bool add_to_tracker)
{
    addTargetReportInternal(rec_num, add_to_tracker, true);
}

void ReconstructorTarget::addTargetReports (const ReconstructorTarget& other,
                                            bool add_to_tracker)
{
    //add single tr without reestimating
    size_t num_added = 0;
    for (auto& rn_it : other.tr_timestamps_)
        if (addTargetReportInternal(rn_it.second, add_to_tracker, false) != TargetReportAddResult::Skipped)
            ++num_added;

    //reestimate chain after adding
    if (add_to_tracker && chain())
    {
        reconstruction::UpdateStats stats;
        bool ok = chain()->reestimate(&stats);

        UNUSED_VARIABLE(ok);

        // traced_assert(stats.num_fresh == num_added); //TODO UGAGUGA
        // traced_assert(stats.num_updated >= num_added);
        // traced_assert(stats.num_failed + stats.num_skipped + stats.num_valid == stats.num_updated);

        addUpdateToGlobalStats(stats);

        // if (!ok) // collected in stats
        //     logwrn << "chain reestimation failed";
    }

    //obtain other target's standing adsb target if newer
    if (other.standing_adsb_target_ &&
        !other.standing_adsb_target_->isOutdated() &&
        (!standing_adsb_target_ || other.standing_adsb_target_->ts_init > standing_adsb_target_->ts_init))
        standing_adsb_target_ = other.standing_adsb_target_;
}

ReconstructorTarget::TargetReportAddResult ReconstructorTarget::addTargetReportInternal (unsigned long rec_num,
                                                                                         bool add_to_tracker,
                                                                                         bool reestimate)
{
    bool do_debug = false;

    traced_assert(reconstructor_.target_reports_.count(rec_num));

    const dbContent::targetReport::ReconstructorInfo& tr = reconstructor_.target_reports_.at(rec_num);

    if (tr.acad_ && acads_.size() && !acads_.count(*tr.acad_))
    {
        logerr << "ReconstructorTarget " << utn_ << ": addTargetReport: acad mismatch, target " << asStr() << " tr '" << tr.asStr() << "'";
        traced_assert(false);
    }

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
    traced_assert(rec_num == tr.record_num_);

    // //assert (tr.in_current_slice_); // can be old one
    if (std::find(target_reports_.begin(), target_reports_.end(), rec_num) != target_reports_.end())
    {
        logerr << "utn " << utn_ << " tr " << tr.asStr() << " already added";
        traced_assert(false);
    }
#endif

    //    if (!timestamp_max_.is_not_a_date_time()) // there is time
    //    {
    //        if (tr.timestamp_ < timestamp_max_)
    //            logerr << "old max " << Time::toString(timestamp_max_)
    //                   << " tr ts " << Time::toString(tr.timestamp_);

    //        traced_assert(tr.timestamp_ >= timestamp_max_);
    //    }

    if (do_debug)
        loginf << "DBG min/max";

    // update min/max ts
    if (total_timestamp_min_.is_not_a_date_time() || total_timestamp_max_.is_not_a_date_time())
    {
        total_timestamp_min_ = tr.timestamp_;
        total_timestamp_max_ = tr.timestamp_;
    }
    else
    {
        total_timestamp_min_ = min(total_timestamp_min_, tr.timestamp_);
        total_timestamp_max_ = max(total_timestamp_max_, tr.timestamp_);
    }

    if (!target_reports_.size())
    {
        timestamp_min_ = tr.timestamp_;
        timestamp_max_ = tr.timestamp_;
    }
    else
    {
        timestamp_min_ = min(timestamp_min_, tr.timestamp_);
        timestamp_max_ = max(timestamp_max_, tr.timestamp_);
    }

    if (tr.barometric_altitude_ && tr.barometric_altitude_->hasReliableValue())
    {
        if (mode_c_min_ && mode_c_max_)
        {
            mode_c_min_ = min(*mode_c_min_, tr.barometric_altitude_->altitude_);
            mode_c_max_ = max(*mode_c_max_, tr.barometric_altitude_->altitude_);
        }
        else
        {
            mode_c_min_ = tr.barometric_altitude_->altitude_;
            mode_c_max_ = tr.barometric_altitude_->altitude_;
        }
    }

    if (latitude_min_ && latitude_max_
        && longitude_min_ && longitude_max_)
    {
        latitude_min_ = min(*latitude_min_, tr.position_->latitude_);
        latitude_max_ = max(*latitude_max_, tr.position_->latitude_);

        longitude_min_ = min(*longitude_min_, tr.position_->longitude_);
        longitude_max_ = max(*longitude_max_, tr.position_->longitude_);
    }
    else
    {
        latitude_min_ = tr.position_->latitude_;
        latitude_max_ = tr.position_->latitude_;

        longitude_min_ = tr.position_->longitude_;
        longitude_max_ = tr.position_->longitude_;
    }

    if (do_debug)
        loginf << "DBG adding meta info";

    if (!ds_ids_.count(tr.ds_id_))
        ds_ids_.insert(tr.ds_id_);

    //    if (tr.track_number_ && !track_nums_.count({tr.ds_id_, *tr.track_number_}))
    //        track_nums_.insert({tr.ds_id_, *tr.track_number_});

    if (tr.mode_a_code_ && !mode_as_.count(tr.mode_a_code_->code_))
        mode_as_.insert(tr.mode_a_code_->code_);

    target_reports_.push_back(tr.record_num_);
    tr_timestamps_.insert({tr.timestamp_,tr.record_num_});
    // all sources sorted by time, ts -> record_num
    tr_ds_timestamps_[Number::recNumGetDBContId(tr.record_num_)][tr.ds_id_][tr.line_id_].insert(
        {tr.timestamp_, tr.record_num_});
    // dbcontent id -> ds_id -> ts -> record_num

    if (tr.ecat_ && *tr.ecat_ != 0) // check for FFT below
    {
        if (ecat_)
        {
            if (*tr.ecat_ != ecat_)
                logwrn << utn_ << ": ecat mismatch, target ecat "
                       << *ecat_ << " " << String::ecatToString(*ecat_)
                       << " tr " << *tr.ecat_ << " " << String::ecatToString(*tr.ecat_) << "";
        }
        else
            ecat_ = *tr.ecat_;
    }

    if (tr.acad_)
    {
        if (acads_.size() && !acads_.count(*tr.acad_))
        {
            logwrn << utn_ << ": acad mismatch, target "
                   << asStr() << " tr '" << tr.asStr() << "'";
        }

        if (!acads_.count(*tr.acad_))
        {
            acads_.insert(*tr.acad_);

            if (!ecat_ || *ecat_ == 0) // no ecat info, check if vehicle by acad
            {
                if (reconstructor_.isVehicleACAD(*tr.acad_))
                    ecat_ = (unsigned int) TargetBase::Category::Vehicle;
            }
        }
    }

    if (tr.acid_)
    {
        string acid = String::trim(*tr.acid_);

        if (!acids_.count(acid))
        {
            acids_.insert(acid);

            if (!ecat_ || *ecat_ == 0) // no ecat info, check if vehicle by acid
            {
                if (reconstructor_.isVehicleACID(acid))
                    ecat_ = (unsigned int) TargetBase::Category::Vehicle;
            }
        }
    }

    if (!ecat_ || *ecat_ == 0) // check for FFT
    {
        FFTManager& fft_man = COMPASS::instance().fftManager();

        boost::optional<float> baro_altitude_ft;

        if (tr.barometric_altitude_)
            baro_altitude_ft = tr.barometric_altitude_->altitude_;

        boost::optional<unsigned int> mode_a_code;

        if (tr.mode_a_code_)
            mode_a_code = tr.mode_a_code_->code_;
        else
            mode_a_code =  boost::none;

        bool is_from_fft;
        float fft_altitude_ft;

        std::tie(is_from_fft, fft_altitude_ft) = fft_man.isFromFFT(
            tr.position_->latitude_, tr.position_->longitude_, tr.acad_, tr.dbcont_id_ == 1,
            mode_a_code, baro_altitude_ft);

        if (is_from_fft)
            ecat_ = (unsigned int) TargetBase::Category::FFT;
    }

    if (tr.dbcont_id_ == 21)
    {
        adsb_count_++;

        if (tr.mops_)
            adsb_mops_count_[to_string(*tr.mops_)] += 1;
        else
            adsb_mops_count_["NULL"] += 1;
    }

    if (do_debug)
        loginf << "DBG add to tracker";

    TargetReportAddResult result = TargetReportAddResult::Skipped;

    if (add_to_tracker && !tr.doNotUsePosition())
    {
        if (!hasTracker())
        {
            if (do_debug)
                loginf << "DBG add to tracker: reinit";

            reinitTracker();
        }

        reconstruction::UpdateStats stats;

        if (do_debug)
            loginf << "DBG add to tracker: addToTracker";

        result = addNewTRToTracker(tr, reestimate, &stats);

        if (reestimate && result != TargetReportAddResult::Skipped)
        {
            traced_assert(stats.num_replaced <= 2);
            traced_assert(stats.num_fresh <= 2);
            traced_assert(stats.num_replaced >= 0 || stats.num_fresh == 1);
            traced_assert(stats.num_updated >= 1);
            traced_assert(stats.num_failed + stats.num_skipped + stats.num_valid == stats.num_updated);
        }

        if (do_debug)
            loginf << "DBG add to tracker: add to stats";

        addUpdateToGlobalStats(stats);
    }

    if (do_debug)
        loginf << "DBG addTargetReport done";

    return result;
}

unsigned int ReconstructorTarget::numAssociated() const
{
    return target_reports_.size();
}

unsigned long ReconstructorTarget::lastAssociated() const
{
    traced_assert(target_reports_.size());
    return target_reports_.back();
}

bool ReconstructorTarget::hasACAD () const
{
    return acads_.size();
}

bool ReconstructorTarget::hasACAD (unsigned int ta) const
{
    return acads_.count(ta);
}

bool ReconstructorTarget::hasAllOfACADs (std::set<unsigned int> tas) const
{
    traced_assert(hasACAD() && tas.size());

    for (auto other_ta : tas)
    {
        if (!acads_.count(other_ta))
            return false;
    }

    return true;
}

bool ReconstructorTarget::hasAnyOfACADs (std::set<unsigned int> tas) const
{
    traced_assert(hasACAD() && tas.size());

    for (auto other_ta : tas)
    {
        if (acads_.count(other_ta))
            return true;
    }
    return false;
}

std::string ReconstructorTarget::acadsStr() const
{
    bool first = true;

    stringstream ss;

    ss << "'";

    for (auto ta_it : acads_)
    {
        if (first)
            ss << String::hexStringFromInt(ta_it, 6, '0');
        else
            ss << ", " << String::hexStringFromInt(ta_it, 6, '0');

        first = false;
    }

    ss << "'";

    return ss.str();
}

bool ReconstructorTarget::hasACID () const
{
    return acids_.size();
}

bool ReconstructorTarget::hasACID (const std::string& acid) const
{
    return acids_.count(String::trim(acid));
}

bool ReconstructorTarget::hasModeA () const
{
    return mode_as_.size();
}

bool ReconstructorTarget::hasModeA (unsigned int code) const
{
    return mode_as_.count(code);
}

bool ReconstructorTarget::hasModeC () const
{
    return mode_c_min_ && mode_c_max_;
}

bool ReconstructorTarget::isPrimary() const
{
    return !hasACAD() && !hasACID() && !hasModeA() && !hasModeC();
}

std::string ReconstructorTarget::asStr() const
{
    stringstream ss;

    ss << "utn " << utn_; // << " tmp_utn " << tmp_utn_;  //<< " tmp " << tmp_

    if (acads_.size())
    {
        ss << " acads " << acadsStr();
    }

    if (acids_.size())
    {
        ss << " acids ";

        bool first {true};
        for (auto& it : acids_)
        {
            if (first)
                ss << "'" << it << "'";
            else
                ss << ", " << "'" << it << "'";

            first = false;
        }
    }

    if (mode_as_.size())
    {
        ss << " m3as '";

        bool first {true};
        for (auto it : mode_as_)
        {
            if (first)
                ss << String::octStringFromInt(it, 4, '0');
            else
                ss << ", " << String::octStringFromInt(it, 4, '0');

            first = false;
        }

        ss << "'";
    }


    //    if (track_nums_.size())
    //    {
    //        ss << " tns ";

    //        bool first {true};
    //        for (auto tn_it : track_nums_)
    //        {
    //            if (first)
    //                ss << "(" << tn_it.first << "," << tn_it.second << ")";
    //            else
    //                ss << ", " << "(" << tn_it.first << "," << tn_it.second << ")";

    //            first = false;
    //        }
    //    }

    return ss.str();
}

std::string ReconstructorTarget::timeStr() const
{
    if (timestamp_min_.is_not_a_date_time() || timestamp_min_.is_not_a_date_time())
        return "[]";

    return "["+Time::toString(timestamp_min_)+" - "+Time::toString(timestamp_max_)+"]";
}

bool ReconstructorTarget::hasTimestamps() const
{
    return !timestamp_min_.is_not_a_date_time() && !timestamp_min_.is_not_a_date_time();
}

bool ReconstructorTarget::isTimeInside (boost::posix_time::ptime timestamp) const
{
    if (timestamp_min_.is_not_a_date_time() || timestamp_min_.is_not_a_date_time())
        return false;

    return timestamp >= timestamp_min_ && timestamp <= timestamp_max_;
}

bool ReconstructorTarget::isTimeInside (boost::posix_time::ptime timestamp, boost::posix_time::time_duration d_max) const
{
    if (timestamp_min_.is_not_a_date_time() || timestamp_min_.is_not_a_date_time())
        return false;

    return timestamp >= (timestamp_min_ - d_max) && timestamp <= (timestamp_max_ + d_max);
}

bool ReconstructorTarget::hasDataForTime (ptime timestamp, time_duration d_max) const
{
    if (!tr_timestamps_.size() || !isTimeInside(timestamp, d_max))
        return false;

    if (tr_timestamps_.count(timestamp))
        return true; // contains exact value(s)

    //    Return iterator to lower bound
    //    Returns an iterator pointing to the first element in the container whose key is not considered to go
    //    before k (i.e., either it is equivalent or goes after).

    auto it_upper = tr_timestamps_.lower_bound(timestamp);

    // all tr_timestamps_ smaller than timestamp
    if (it_upper == tr_timestamps_.end())
    {
        traced_assert(tr_timestamps_.rbegin()->first <= timestamp);
        return (timestamp - tr_timestamps_.rbegin()->first) < d_max;
    }

    // all tr_timestamps_ bigger than timestamp
    if (it_upper == tr_timestamps_.begin())
    {
        traced_assert(tr_timestamps_.begin()->first >= timestamp);
        return (tr_timestamps_.begin()->first - timestamp) < d_max;
    }

    // have lb_it which has >= timestamp
    traced_assert(it_upper->first >= timestamp);

    if (timestamp - it_upper->first < d_max)
        return true; // got one in d_max

    it_upper--;

    traced_assert(it_upper->first < timestamp);
    return (timestamp - tr_timestamps_.begin()->first) < d_max;

    // save value
    // ptime upper = lb_it->first;

    // // reverse in time to check previous timestamps
    // while (lb_it != tr_timestamps_.end() && timestamp < lb_it->first)
    // {
    //     if (lb_it == tr_timestamps_.begin()) // exit condition on first value
    //     {
    //         if (timestamp < lb_it->first) // set as not found
    //             lb_it = tr_timestamps_.end();

    //         break;
    //     }

    //     lb_it--;
    // }

    // if (lb_it == tr_timestamps_.end())
    //     return false;

    // traced_assert(timestamp >= lb_it->first);

    // if (timestamp - lb_it->first > d_max)
    //     return false; // too much time difference

    // ptime lower = lb_it->first;

    // logdbg2 << utn_ << ": found " << Time::toString(lower)
    //        << " <= " << Time::toString(timestamp)
    //        << " <= " << Time::toString(upper);

    // return true;
}

ReconstructorTarget::TargetReportSkipResult ReconstructorTarget::skipTargetReport(const dbContent::targetReport::ReconstructorInfo& tr,
                                                                                  const InfoValidFunc& tr_valid_func) const
{
    //run base checks first
    if (reconstructor_.settings().ignore_calculated_references && tr.is_calculated_reference_)
        return TargetReportSkipResult::SkipReference;

    //then external check
    if (tr_valid_func && !tr_valid_func(tr))
        return TargetReportSkipResult::SkipFunc;

    return TargetReportSkipResult::Valid;
}

std::unique_ptr<reconstruction::KalmanChain>& ReconstructorTarget::chain() const
{
    return reconstructor_.chain(utn_);
}

namespace
{
std::string tr2String(const dbContent::targetReport::ReconstructorInfo& tr)
{
    std::stringstream ss;
    ss << "(" << tr.record_num_ << "," << tr.dbcont_id_ << "," << tr.ds_id_ << "," << Utils::Time::toString(tr.timestamp_);
    return ss.str();
}
}

ReconstructorTarget::ReconstructorInfoPair ReconstructorTarget::dataFor (ptime timestamp,
                                                                        time_duration d_max,
                                                                        const InfoValidFunc& tr_valid_func,
                                                                        const InterpOptions& interp_options) const
// lower/upper times, invalid ts if not existing
{
    bool debug = false; //interp_options.debug();

    std::multimap<boost::posix_time::ptime, unsigned long>::const_iterator it_lower, it_upper;
    bool has_lower = false;
    bool has_upper = false;

    auto it = tr_timestamps_.find(timestamp);  // more efficient than count for just checking existence

    if (it == tr_timestamps_.end()) // no match
    {
        // No match case (most common)

        // get lower bound
        it_upper = tr_timestamps_.lower_bound(timestamp);

        // all tr_timestamps_ smaller than timestamp
        if (it_upper == tr_timestamps_.end())
        {
            traced_assert(tr_timestamps_.rbegin()->first <= timestamp);
            if ((timestamp - tr_timestamps_.rbegin()->first) < d_max)
            {
                it_lower = std::prev(tr_timestamps_.end());
                it_upper = tr_timestamps_.end();
                has_lower = true;
                has_upper = false;
            }
        }
        else if (it_upper == tr_timestamps_.begin())  // all tr_timestamps_ bigger than timestamp
        {
            traced_assert(tr_timestamps_.begin()->first >= timestamp);

            if ((tr_timestamps_.begin()->first - timestamp) < d_max)
            {
                it_lower = tr_timestamps_.end();
                // it_upper  = tr_timestamps_.begin(); // is already on this value
                has_lower = false;
                has_upper = true;
            }
        }
        else
        {
            traced_assert(it_upper->first >= timestamp);

            // too much time difference?
            if (it_upper->first - timestamp <= d_max)
                has_upper = true;

            // set lower iterator to last elem
            it_lower = it_upper;
            --it_lower;

            traced_assert(it_lower->first < timestamp);

            // lower item too far away?
            if (timestamp - it_lower->first <= d_max)
                has_lower = true;
        }
        if (debug)
        {
            loginf << "initial interval:\n"
                   << "   " << (has_lower ? tr2String(dataFor(it_lower->second)) : "") << "\n"
                   << "   " << (has_upper ? tr2String(dataFor(it_upper->second)) : "");
        }
    }
    else // at least one match
    {
        auto range = tr_timestamps_.equal_range(timestamp);
        bool exists_one = range.first != range.second;
        bool exist_multiple = exists_one && std::next(range.first) != range.second;

        traced_assert(exists_one || exist_multiple);

        // look for initial upper and lower datum
        if (exists_one && !exist_multiple)  // exists one
        {
            traced_assert(range.first != tr_timestamps_.end());

            // unique timestamp in map => start from this timestamp
            it_lower = range.first;
            it_upper = range.first;
            has_lower = true;
            has_upper = true;

            if (debug)
                loginf << "found timestamp in target";
        }
        else if (exist_multiple)
        {
            traced_assert(range.first != tr_timestamps_.end());

            // multiple timestamps in map => choose one depending on init mode
            std::multimap<boost::posix_time::ptime, unsigned long>::const_iterator it_start =
                tr_timestamps_.end();

            auto init_mode = interp_options.initMode();

            if (init_mode == InterpOptions::InitMode::First)
            {
                // choose first
                it_start = range.first;
            }
            else if (init_mode == InterpOptions::InitMode::Last)
            {
                // choose last
                it_start = range.second == tr_timestamps_.end() ? range.first : --range.second;
            }
            else
            {
                // choose a valid datum or a datum with a certain record number
                bool look_for_recnum = init_mode == InterpOptions::InitMode::RecNum;
                auto rec_num = interp_options.initRecNum();

                for (auto it = range.first; it != range.second; ++it)
                {
                    const auto& d = dataFor(it->second);

                    // if recnum has been found break immediately
                    if (look_for_recnum && d.record_num_ == rec_num)
                    {
                        it_start = it;
                        break;
                    }

                    // if valid break immediately if no recnum is specified, otherwise remember as
                    // fallback and continue search
                    if (skipTargetReport(d, tr_valid_func) == TargetReportSkipResult::Valid)
                    {
                        it_start = it;
                        if (!look_for_recnum)
                            break;
                    }
                }
            }

            // fallback: use first
            if (it_start == tr_timestamps_.end())
                it_start = range.first;

            traced_assert(it_start != tr_timestamps_.end());

            it_lower = it_start;
            it_upper = it_start;
            has_lower = true;
            has_upper = true;

            if (debug)
                loginf << "found multiple timestamps in target, chose:\n"
                       << tr2String(dataFor(it_start->second));
        }
    }

    //lower and upper entries already valid?
    bool ok_lower = !has_lower || skipTargetReport(dataFor(it_lower->second), tr_valid_func) == TargetReportSkipResult::Valid;
    bool ok_upper = !has_upper || skipTargetReport(dataFor(it_upper->second), tr_valid_func) == TargetReportSkipResult::Valid;

    if (ok_lower && ok_upper)
    {
        if (debug)
            loginf << "initial interval valid, has_lower " << has_lower
                   << " has_upper " << has_upper;

        return {has_lower ? &dataFor(it_lower->second) : nullptr, has_upper ? &dataFor(it_upper->second) : nullptr};
    }

    //broaden interval until valid or out of range
    if (has_upper)
    {
        has_upper = false;
        for (auto it = it_upper; it != tr_timestamps_.end(); ++it)
        {
            if (it->first - timestamp > d_max)
                break;

            auto skip_result = skipTargetReport(dataFor(it->second), tr_valid_func);

            if (skip_result == TargetReportSkipResult::Valid)
            {
                it_upper  = it;
                has_upper = true;
                break;
            }
            else if (debug)
            {
                loginf << "skipping upper tr " << tr2String(dataFor(it->second)) << ": " << (int)skip_result;
            }
        }
    }

    if (has_lower)
    {
        has_lower = false;
        auto it = it_lower;
        while (1)
        {
            if (timestamp - it->first > d_max)
                break;

            auto skip_result = skipTargetReport(dataFor(it->second), tr_valid_func);

            if (skip_result == TargetReportSkipResult::Valid)
            {
                it_lower  = it;
                has_lower = true;
                break;
            }
            else if (debug)
            {
                loginf << "skipping lower tr " << tr2String(dataFor(it->second)) << ": " << (int)skip_result;
            }

            if (it == tr_timestamps_.begin())
                break;
            
            --it;
        }
    }

    if (debug) 
    {
        loginf << "final interval:\n" 
               << "   " << (has_lower ? tr2String(dataFor(it_lower->second)) : "") << "\n"
               << "   " << (has_upper ? tr2String(dataFor(it_upper->second)) : "");
    }

    return {has_lower ? &dataFor(it_lower->second) : nullptr,
            has_upper ? &dataFor(it_upper->second) : nullptr};
}

ReconstructorTarget::ReferencePair ReconstructorTarget::refDataFor (ptime timestamp, time_duration d_max) const
// lower/upper times, invalid ts if not existing
{
    if (references_.count(timestamp))
        return {&references_.at(timestamp), nullptr}; // contains exact value

    //    Return iterator to lower bound
    //    Returns an iterator pointing to the first element in the container whose key is not considered to go
    //    before k (i.e., either it is equivalent or goes after).

    auto lb_it = references_.lower_bound(timestamp);

    if (lb_it == references_.end())
        return {nullptr, nullptr};

    traced_assert(lb_it->first >= timestamp);

    if (lb_it->first - timestamp > d_max)
        return {nullptr, nullptr}; // too much time difference

    // save value
    ptime upper = lb_it->first;

    // TODO lb_it--; ?
    while (lb_it != references_.end() && timestamp < lb_it->first)
    {
        if (lb_it == references_.begin()) // exit condition on first value
        {
            if (timestamp < lb_it->first) // set as not found
                lb_it = references_.end();

            break;
        }

        lb_it--;
    }

    if (lb_it == references_.end())
        return {nullptr, &references_.at(upper)};

    traced_assert(timestamp >= lb_it->first);

    if (timestamp - lb_it->first > d_max)
        return {nullptr, &references_.at(upper)}; // too much time difference

    ptime lower = lb_it->first;

    return {&references_.at(lower), &references_.at(upper)};
}

//std::pair<ptime, ptime> ReconstructorTarget::timesFor (
//    ptime timestamp, time_duration  d_max) const
//// lower/upper times, invalid ts if not existing
//{
//    if (tr_timestamps_.count(timestamp))
//        return {timestamp, {}}; // contains exact value

//            //    Return iterator to lower bound
//            //    Returns an iterator pointing to the first element in the container whose key is not considered to go
//            //    before k (i.e., either it is equivalent or goes after).

//    auto lb_it = tr_timestamps_.lower_bound(timestamp);

//    if (lb_it == tr_timestamps_.end())
//        return {{}, {}};

//    traced_assert(lb_it->first >= timestamp);

//    if (lb_it->first - timestamp > d_max)
//        return {{}, {}}; // too much time difference

//            // save value
//    ptime upper = lb_it->first;

//            // TODO lb_it--; ?
//    while (lb_it != tr_timestamps_.end() && timestamp < lb_it->first)
//    {
//        if (lb_it == tr_timestamps_.begin()) // exit condition on first value
//        {
//            if (timestamp < lb_it->first) // set as not found
//                lb_it = tr_timestamps_.end();

//            break;
//        }

//        lb_it--;
//    }

//    if (lb_it == tr_timestamps_.end())
//        return {{}, upper};

//    traced_assert(timestamp >= lb_it->first);

//    if (timestamp - lb_it->first > d_max)
//        return {{}, upper}; // too much time difference

//    ptime lower = lb_it->first;

//    return {lower, upper};
//}

std::pair<dbContent::targetReport::Position, bool> ReconstructorTarget::interpolatedPosForTime (
    ptime timestamp, time_duration d_max) const
{
    dbContent::targetReport::ReconstructorInfo* lower, *upper;

    tie(lower, upper) = dataFor(
        timestamp, d_max,
        [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) { return !tr.doNotUsePosition(); });

    if (lower && !upper) // exact time
    {
        if (lower->position())
            return {*lower->position(), true};
        else
            return {{}, false};
    }

    if (!lower || !lower->position() || !upper || !upper->position())
        return {{}, false};

    dbContent::targetReport::Position& pos1 = *lower->position();
    dbContent::targetReport::Position& pos2 = *upper->position();
    float d_t = Time::partialSeconds(upper->timestamp_ - lower->timestamp_);

    logdbg2 << "d_t " << d_t;

    traced_assert(d_t >= 0);

    if (pos1.latitude_ == pos2.latitude_
        && pos1.longitude_ == pos2.longitude_) // same pos
        return {pos1, true};

    if (lower == upper) // same time
    {
        logwrn << "ref has same time twice";
        return {{}, false};
    }

    logdbg2 << "pos1 " << pos1.latitude_ << ", " << pos1.longitude_;
    logdbg2 << "pos2 " << pos2.latitude_ << ", " << pos2.longitude_;

    bool ok;
    double x_pos, y_pos;

    logdbg2 << "geo2cart";

    tie(ok, x_pos, y_pos) = trafo_.distanceCart(
        pos1.latitude_, pos1.longitude_, pos2.latitude_, pos2.longitude_);

    if (!ok)
    {
        logerr << "error with latitude " << pos2.latitude_
               << " longitude " << pos2.longitude_;
        return {{}, false};
    }

    logdbg2 << "offsets x " << fixed << x_pos
           << " y " << fixed << y_pos << " dist " << fixed << sqrt(pow(x_pos,2)+pow(y_pos,2));

    double v_x = x_pos/d_t;
    double v_y = y_pos/d_t;
    logdbg2 << "v_x " << v_x << " v_y " << v_y;

    float d_t2 = Time::partialSeconds(timestamp - lower->timestamp_);
    logdbg2 << "d_t2 " << d_t2;

    traced_assert(d_t2 >= 0);

    x_pos = v_x * d_t2;
    y_pos = v_y * d_t2;

    logdbg2 << "interpolated offsets x " << x_pos << " y " << y_pos;

    tie (ok, x_pos, y_pos) = trafo_.wgsAddCartOffset(pos1.latitude_, pos1.longitude_, x_pos, y_pos);

    //ret = ogr_cart2geo->Transform(1, &x_pos, &y_pos);

    // x_pos long, y_pos lat

    logdbg2 << "interpolated lat " << x_pos << " long " << y_pos;

    // calculate altitude

    // TODO no alt 4 u!

    //    bool has_altitude = false;
    //    float altitude = 0.0;

    //    if (pos1.has_altitude_ && !pos2.has_altitude_)
    //    {
    //        has_altitude = true;
    //        altitude = pos1.altitude_;
    //    }
    //    else if (!pos1.has_altitude_ && pos2.has_altitude_)
    //    {
    //        has_altitude = true;
    //        altitude = pos2.altitude_;
    //    }
    //    else if (pos1.has_altitude_ && pos2.has_altitude_)
    //    {
    //        float v_alt = (pos2.altitude_ - pos1.altitude_)/d_t;
    //        has_altitude = true;
    //        altitude = pos1.altitude_ + v_alt*d_t2;
    //    }

    //    logdbg2 << "pos1 has alt "
    //           << pos1.has_altitude_ << " alt " << pos1.altitude_
    //           << " pos2 has alt " << pos2.has_altitude_ << " alt " << pos2.altitude_
    //           << " interpolated has alt " << has_altitude << " alt " << altitude;

    //            //        if (in_appimage_) // inside appimage
    //            //            return {{y_pos, x_pos, has_altitude, true, altitude}, true};
    //            //        else
    //    return {{x_pos, y_pos, has_altitude, true, altitude}, true};

    return {{x_pos, y_pos}, true};
}

std::pair<dbContent::targetReport::Position, bool> ReconstructorTarget::interpolatedPosForTimeFast (
    ptime timestamp, time_duration d_max) const
{
    dbContent::targetReport::ReconstructorInfo* lower_rec_num, *upper_rec_num;

    tie(lower_rec_num, upper_rec_num) = dataFor(
        timestamp, d_max,
        [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) { return !tr.doNotUsePosition(); });

    if (lower_rec_num && !upper_rec_num) // exact time
    {
        if (lower_rec_num->position())
            return {*lower_rec_num->position(), true};
        else
            return {{}, false};
    }

    if (!lower_rec_num || !lower_rec_num->position() || !upper_rec_num || !upper_rec_num->position())
        return {{}, false};

    dbContent::targetReport::Position& pos1 = *lower_rec_num->position();
    dbContent::targetReport::Position& pos2 = *upper_rec_num->position();
    float d_t = Time::partialSeconds(upper_rec_num->timestamp_ - lower_rec_num->timestamp_);

    logdbg2 << "d_t " << d_t;

    traced_assert(d_t >= 0);

    if (pos1.latitude_ == pos2.latitude_
        && pos1.longitude_ == pos2.longitude_) // same pos
        return {pos1, true};

    if (lower_rec_num == upper_rec_num) // same time
    {
        logwrn << "ref has same time twice";
        return {{}, false};
    }

    double v_lat = (pos2.latitude_ - pos1.latitude_)/d_t;
    double v_long = (pos2.longitude_ - pos1.longitude_)/d_t;
    logdbg2 << "v_x " << v_lat << " v_y " << v_long;

    float d_t2 = Time::partialSeconds(timestamp - lower_rec_num->timestamp_);
    logdbg2 << "d_t2 " << d_t2;

    traced_assert(d_t2 >= 0);

    double int_lat = pos1.latitude_ + v_lat * d_t2;
    double int_long = pos1.longitude_ + v_long * d_t2;

    logdbg2 << "interpolated lat " << int_lat << " long " << int_long;

    // calculate altitude
    //    bool has_altitude = false;
    //    float altitude = 0.0;

    //    if (pos1.has_altitude_ && !pos2.has_altitude_)
    //    {
    //        has_altitude = true;
    //        altitude = pos1.altitude_;
    //    }
    //    else if (!pos1.has_altitude_ && pos2.has_altitude_)
    //    {
    //        has_altitude = true;
    //        altitude = pos2.altitude_;
    //    }
    //    else if (pos1.has_altitude_ && pos2.has_altitude_)
    //    {
    //        float v_alt = (pos2.altitude_ - pos1.altitude_)/d_t;
    //        has_altitude = true;
    //        altitude = pos1.altitude_ + v_alt*d_t2;
    //    }

    //    logdbg2 << "pos1 has alt "
    //           << pos1.has_altitude_ << " alt " << pos1.altitude_
    //           << " pos2 has alt " << pos2.has_altitude_ << " alt " << pos2.altitude_
    //           << " interpolated has alt " << has_altitude << " alt " << altitude;

    //    return {{int_lat, int_long, has_altitude, true, altitude}, true};

    return {{int_lat, int_long}, true};
}

std::pair<boost::optional<dbContent::targetReport::Position>,
          boost::optional<dbContent::targetReport::PositionAccuracy>> ReconstructorTarget::interpolatedRefPosForTime (
    ptime timestamp, time_duration d_max) const
{
    const reconstruction::Reference* lower_ref, *upper_ref;

    tie(lower_ref, upper_ref) = refDataFor(timestamp, d_max);

    if (lower_ref && !upper_ref) // exact time
        return {lower_ref->position(),lower_ref->positionAccuracy()};

    if (!lower_ref && !upper_ref) // none set
        return {{},{}};

    if (!lower_ref || !upper_ref) // only 1 set
    {
        auto ref_ptr = lower_ref ? lower_ref : upper_ref;

        dbContent::targetReport::Position pos = ref_ptr->position();
        dbContent::targetReport::PositionAccuracy acc = ref_ptr->positionAccuracy();

        auto pos_ts = ref_ptr->t;

        double dx {0}, dy {0};
        float d_t2 = Time::partialSeconds(timestamp - pos_ts);

        if (ref_ptr->hasVelocity())
        {
            dx = *ref_ptr->vx * d_t2;
            dy = *ref_ptr->vy * d_t2;
        }

        //Transformation trafo;
        //bool ok;
        double int_lat, int_long, h_back;

        GeographicLib::LocalCartesian proj_wo_alt (pos.latitude_, pos.longitude_, 0);
        //tie (ok, int_lat, int_long) = trafo_.wgsAddCartOffset(pos.latitude_, pos.longitude_, dx, dy);

        proj_wo_alt.Reverse(dx, dy, 0.0, int_lat, int_long, h_back);

        return {dbContent::targetReport::Position{int_lat, int_long}, acc};
    }
    else
    {
        dbContent::targetReport::Position pos1 = lower_ref->position();
        dbContent::targetReport::Position pos2 = upper_ref->position();

        float d_t = Time::partialSeconds(upper_ref->t - lower_ref->t);

        logdbg2 << "d_t " << d_t;

        traced_assert(d_t >= 0);

        if (pos1.latitude_ == pos2.latitude_
            && pos1.longitude_ == pos2.longitude_) // same pos
            return {lower_ref->position(), lower_ref->positionAccuracy()};

        if (lower_ref == upper_ref) // same time
        {
            logwrn << "ref has same time twice";
            return {dbContent::targetReport::Position
                    { (pos1.latitude_ + pos2.latitude_)/2.0, (pos1.longitude_ + pos2.longitude_)/2.0},
                    lower_ref->positionAccuracy()};
        }

        double v_lat = (pos2.latitude_ - pos1.latitude_)/d_t;
        double v_long = (pos2.longitude_ - pos1.longitude_)/d_t;
        logdbg2 << "v_x " << v_lat << " v_y " << v_long;

        float d_t2 = Time::partialSeconds(timestamp - lower_ref->t);
        logdbg2 << "d_t2 " << d_t2;

        traced_assert(d_t2 >= 0);

        double int_lat = pos1.latitude_ + v_lat * d_t2;
        double int_long = pos1.longitude_ + v_long * d_t2;

        logdbg2 << "interpolated lat " << int_lat << " long " << int_long;

        boost::optional<dbContent::targetReport::PositionAccuracy> ret_pos_acc =
            lower_ref->positionAccuracy().maxStdDev() > upper_ref->positionAccuracy().maxStdDev()
                ? lower_ref->positionAccuracy() : upper_ref->positionAccuracy();

        return {dbContent::targetReport::Position{int_lat, int_long}, ret_pos_acc};
    }
}

//bool ReconstructorTarget::hasDataForExactTime (ptime timestamp) const
//{
//    return tr_timestamps_.count(timestamp);
//}

//unsigned long  ReconstructorTarget::dataForExactTime (ptime timestamp) const
//{
//    traced_assert(hasDataForExactTime(timestamp));
//    traced_assert(target_reports_.size() > tr_timestamps_.at(timestamp));
//    return *target_reports_.at(tr_timestamps_.at(timestamp));
//}

//dbContent::targetReport::Position ReconstructorTarget::posForExactTime (ptime timestamp) const
//{
//    traced_assert(hasDataForExactTime(timestamp));

//    TargetReport& tr = dataForExactTime(timestamp);

//    dbContent::targetReport::Position pos;

//    pos.latitude_ = tr.latitude_;
//    pos.longitude_ = tr.longitude_;
//    pos.has_altitude_ = tr.has_mc_;
//    pos.altitude_ = tr.mc_;

//    return pos;
//}

bool ReconstructorTarget::hasDataFor (unsigned long rec_num) const
{
    return reconstructor_.target_reports_.count(rec_num);
}

dbContent::targetReport::ReconstructorInfo& ReconstructorTarget::dataFor (unsigned long rec_num) const
{
    //assert (reconstructor_.target_reports_.count(rec_num));
    return reconstructor_.target_reports_.at(rec_num);
}

bool ReconstructorTarget::hasPositionFor (unsigned long rec_num) const
{
    return dataFor(rec_num).position().has_value();
}

dbContent::targetReport::Position ReconstructorTarget::positionFor (unsigned long rec_num) const
{
    traced_assert(hasPositionFor(rec_num));
    return *dataFor(rec_num).position();
}


float ReconstructorTarget::duration () const
{
    if (timestamp_max_.is_not_a_date_time() || timestamp_min_.is_not_a_date_time())
        return 0;

    return Time::partialSeconds(timestamp_max_ - timestamp_min_);
}

bool ReconstructorTarget::timeOverlaps (const ReconstructorTarget& other) const
{
    if (!target_reports_.size() || !other.target_reports_.size())
        return false;

    traced_assert(!timestamp_max_.is_not_a_date_time() && !timestamp_min_.is_not_a_date_time());

    //a.start < b.end && b.start < a.end;
    return timestamp_min_ < other.timestamp_max_ && other.timestamp_min_ < timestamp_max_;
}

float ReconstructorTarget::probTimeOverlaps (const ReconstructorTarget& other) const
{
    if (timestamp_max_.is_not_a_date_time() || timestamp_min_.is_not_a_date_time())
        return 0.0;

    ptime overlap_begin = max(timestamp_min_, other.timestamp_min_);
    ptime overlap_end = min(timestamp_max_, other.timestamp_max_);

    if (overlap_begin >= overlap_end)
        return 0.0;

    float overlap_duration = Time::partialSeconds(overlap_end - overlap_begin);

    float targets_min_duration = min(duration(), other.duration());

    traced_assert(overlap_duration <= targets_min_duration);

    if (targets_min_duration == 0)
        return 0.0;

    return overlap_duration / targets_min_duration;
}

std::tuple<vector<unsigned long>, vector<unsigned long>, vector<unsigned long>>
ReconstructorTarget::compareModeACodes (
    const ReconstructorTarget& other,
    boost::posix_time::time_duration max_time_diff, bool do_debug) const
{
    vector<unsigned long> unknown;
    vector<unsigned long> same;
    vector<unsigned long> different;

    ComparisonResult cmp_res;

    if (do_debug)
        loginf << "DBG other_utn " << utn_ << " cmpMAs num target_reports " << target_reports_.size()
               << " max t_diff " << Time::toString(max_time_diff);

    for (auto tr_it : target_reports_)
    {
        dbContent::targetReport::ReconstructorInfo& tr = dataFor(tr_it);

        cmp_res = other.compareModeACode(tr, max_time_diff, do_debug);

        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMAs "
                   << " cmp_res " << (unsigned int) cmp_res
                   << " num unknown " << unknown.size() << " same " << same.size()
                   << " different " << different.size();

        if (cmp_res == ComparisonResult::UNKNOWN)
            unknown.push_back(tr.record_num_);
        else if (cmp_res == ComparisonResult::SAME)
            same.push_back(tr.record_num_);
        else if (cmp_res == ComparisonResult::DIFFERENT)
            different.push_back(tr.record_num_);
    }

    traced_assert(target_reports_.size() == unknown.size()+same.size()+different.size());

    return std::tuple<vector<unsigned long>, vector<unsigned long>, vector<unsigned long>>(unknown, same, different);
}

ComparisonResult ReconstructorTarget::compareModeACode (
    const dbContent::targetReport::ReconstructorInfo& tr, time_duration max_time_diff, bool do_debug) const
{

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA tr " << tr.asStr();

    // check if reliable value, can be different but not same
    bool tr_mode_a_unreliable = tr.mode_a_code_.has_value() && !tr.mode_a_code_->hasReliableValue();

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
               << " res unkown, has val " << tr.mode_a_code_.has_value()
               << " reliable " << tr_mode_a_unreliable;

    if (!hasDataForTime(tr.timestamp_, max_time_diff))
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " no data for time ("
                   << Time::toString(timestamp_min_) << "-" << Time::toString(timestamp_max_) << ")";

        return ComparisonResult::UNKNOWN;
    }

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    tie(lower_tr, upper_tr) = dataFor(tr.timestamp_, max_time_diff);
    // [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) { return tr.mode_a_code_.has_value(); }
    // no lambda because missing value important, TODO set data source if available

    bool lower_no_m3a = lower_tr && !lower_tr->mode_a_code_.has_value();  // TODO check
    bool upper_no_m3a = upper_tr && !upper_tr->mode_a_code_.has_value();

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
               << " lower_no_m3a " << lower_no_m3a << " upper_no_m3a " << upper_no_m3a;

    // no mode a, and one missing in one of the others
    if (!tr.mode_a_code_.has_value() && (lower_no_m3a || upper_no_m3a)) // TODO check if data sources m3a capable
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " no mode a, same";

        return ComparisonResult::SAME;
    }

    bool lower_m3a_usable = lower_tr && lower_tr->mode_a_code_.has_value() && lower_tr->mode_a_code_->hasReliableValue();
    bool upper_m3a_usable = upper_tr && upper_tr->mode_a_code_.has_value() && upper_tr->mode_a_code_->hasReliableValue();

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
               << " lower_m3a_usable " << lower_m3a_usable << " upper_m3a_usable " << upper_m3a_usable;

    // no able to compare
    if (!lower_m3a_usable && !upper_m3a_usable)
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " none usable, unknown";

        return ComparisonResult::UNKNOWN;
    }

    if ((lower_m3a_usable && !upper_m3a_usable)
        || (!lower_m3a_usable && upper_m3a_usable)) // only 1 usable
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " only 1 usable";

        dbContent::targetReport::ReconstructorInfo& ref1 = lower_m3a_usable ? *lower_tr : *upper_tr;

        if (!tr.mode_a_code_.has_value())
        {
            if (do_debug)
                loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                       << " no tr mode a, different";

            return ComparisonResult::DIFFERENT;
        }

        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " tr mode a, same " << (tr.mode_a_code_->code_ == ref1.mode_a_code_->code_);

        // mode a exists
        if (tr.mode_a_code_->code_ == ref1.mode_a_code_->code_) // is same
        {
            if (tr_mode_a_unreliable)
                return ComparisonResult::UNKNOWN;
            else
                return ComparisonResult::SAME;
        }
        else
            return ComparisonResult::DIFFERENT;
    }

    // both set & reliable

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
               << " both set & reliable";

    dbContent::targetReport::ReconstructorInfo& ref1 = *lower_tr;
    dbContent::targetReport::ReconstructorInfo& ref2 = *upper_tr;

    if (!tr.mode_a_code_.has_value())
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
                   << " no tr mode a, different";

        return ComparisonResult::DIFFERENT; // no mode a here, but in other
    }

    // everything exists

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMA"
               << " tr mode a, same " << ((tr.mode_a_code_->code_ == ref1.mode_a_code_->code_)
                                          || (tr.mode_a_code_->code_ == ref2.mode_a_code_->code_));

    if ((tr.mode_a_code_->code_ == ref1.mode_a_code_->code_)
        || (tr.mode_a_code_->code_ == ref2.mode_a_code_->code_)) // one of them is same
    {
        if (tr_mode_a_unreliable)
            return ComparisonResult::UNKNOWN;
        else
            return ComparisonResult::SAME;
    }
    else
        return ComparisonResult::DIFFERENT;
}

std::tuple<vector<unsigned long>, vector<unsigned long>, vector<unsigned long>> ReconstructorTarget::compareModeCCodes (
    const ReconstructorTarget& other, const std::vector<unsigned long>& rec_nums,
    time_duration max_time_diff, float max_alt_diff, bool do_debug) const
{
    vector<unsigned long> unknown;
    vector<unsigned long> same;
    vector<unsigned long> different;

    ComparisonResult cmp_res;

    for (auto rn_it : rec_nums)
    {
        traced_assert(hasDataFor(rn_it));
        dbContent::targetReport::ReconstructorInfo& tr = dataFor(rn_it);

        cmp_res = other.compareModeCCode(tr, max_time_diff, max_alt_diff, do_debug);

        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMCs "
                   << " cmp_res " << (unsigned int) cmp_res
                   << " num unknown " << unknown.size() << " same " << same.size()
                   << " different " << different.size();

        if (cmp_res == ComparisonResult::UNKNOWN)
            unknown.push_back(tr.record_num_);
        else if (cmp_res == ComparisonResult::SAME)
            same.push_back(tr.record_num_);
        else if (cmp_res == ComparisonResult::DIFFERENT)
            different.push_back(tr.record_num_);
    }

    traced_assert(rec_nums.size() == unknown.size()+same.size()+different.size());

    return std::tuple<vector<unsigned long>, vector<unsigned long>, vector<unsigned long>>(unknown, same, different);
}

bool ReconstructorTarget::isPrimaryAt(boost::posix_time::ptime timestamp,
                 boost::posix_time::time_duration max_time_diff, const InterpOptions& interp_options) const
{
    if (isPrimary()) // for full target
        return true;

    if (!hasDataForTime(timestamp, max_time_diff))
        return false; // unknown

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    if (interp_options.debug()) loginf << "t = " << Utils::Time::toString(timestamp);

    tie(lower_tr, upper_tr) = dataFor(timestamp, max_time_diff, {}, interp_options);

    if (lower_tr && lower_tr->isPrimaryOnlyDetection())
        return true;

    if (upper_tr && upper_tr->isPrimaryOnlyDetection())
        return true;

    return false;
}

boost::optional<float> ReconstructorTarget::modeCCodeAt (boost::posix_time::ptime timestamp,
                                                        boost::posix_time::time_duration max_time_diff,
                                                        const InterpOptions& interp_options) const
{
    if (!hasDataForTime(timestamp, max_time_diff))
        return {};

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    if (interp_options.debug()) loginf << "t = " << Utils::Time::toString(timestamp);

    tie(lower_tr, upper_tr) = dataFor(timestamp, max_time_diff, {}, interp_options);
    // [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) {return tr.barometric_altitude_.has_value() && tr.barometric_altitude_->hasReliableValue(); }
    // no lambda because missing value important, TODO set data source if available

    bool lower_mc_usable = lower_tr && lower_tr->barometric_altitude_.has_value()
                           && lower_tr->barometric_altitude_->hasReliableValue();

    bool upper_mc_usable = upper_tr && upper_tr->barometric_altitude_.has_value()
                           && upper_tr->barometric_altitude_->hasReliableValue();

    // nothing reliable
    if (!lower_mc_usable && !upper_mc_usable)
        return {};

    if ((lower_mc_usable && !upper_mc_usable)
        || (!lower_mc_usable && upper_mc_usable)) // only 1 usable
    {
        dbContent::targetReport::ReconstructorInfo& ref1 = lower_mc_usable ? *lower_tr : *upper_tr;
        traced_assert(ref1.barometric_altitude_);

        return ref1.barometric_altitude_->altitude_;
    }

    // both set & reliable
    dbContent::targetReport::ReconstructorInfo& ref1 = *lower_tr;
    dbContent::targetReport::ReconstructorInfo& ref2 = *upper_tr;

    if ((ref1.timestamp_ - timestamp).total_milliseconds() <=  (ref2.timestamp_ - timestamp).total_milliseconds())
        return ref1.barometric_altitude_->altitude_;
    else
        return ref2.barometric_altitude_->altitude_;
}

boost::optional<bool> ReconstructorTarget::groundBitAt (boost::posix_time::ptime timestamp,
                                                       boost::posix_time::time_duration max_time_diff,
                                                       const InterpOptions& interp_options) const
{
    if (!hasDataForTime(timestamp, max_time_diff))
        return {};

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    if (interp_options.debug()) loginf << "t = " << Utils::Time::toString(timestamp);

    tie(lower_tr, upper_tr) = dataFor(
        timestamp, max_time_diff,
        [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) {
            // if (tr.isPrimaryOnlyDetection()) // override for primary-only CAT010 having GBS=0
            //     return false; // bad thing to do for air PSRs
            // else
                return tr.ground_bit_.has_value(); },
        interp_options);

    bool lower_has_val = lower_tr && lower_tr->ground_bit_.has_value();
    bool upper_has_val = upper_tr && upper_tr->ground_bit_.has_value();

    // no value
    if (!lower_has_val && !upper_has_val) // TODO check if data sources m3a capable
        return {};

    if ((lower_has_val && !upper_has_val)
        || (!lower_has_val && upper_has_val)) // only 1 usable
    {
        dbContent::targetReport::ReconstructorInfo& ref1 = lower_has_val ? *lower_tr : *upper_tr;
        traced_assert(ref1.ground_bit_);

        return *ref1.ground_bit_;
    }

    // both set & reliable
    dbContent::targetReport::ReconstructorInfo& ref1 = *lower_tr;
    dbContent::targetReport::ReconstructorInfo& ref2 = *upper_tr;

    if ((ref1.timestamp_ - timestamp).total_milliseconds() <=  (ref2.timestamp_ - timestamp).total_milliseconds())
        return *ref1.ground_bit_;
    else
        return *ref2.ground_bit_;
}

boost::optional<double> ReconstructorTarget::groundSpeedAt (boost::posix_time::ptime timestamp,
                                                           boost::posix_time::time_duration max_time_diff,
                                                           const InterpOptions& interp_options) const
{
    if (!hasDataForTime(timestamp, max_time_diff))
        return {};

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    if (interp_options.debug()) loginf << "t = " << Utils::Time::toString(timestamp);

    tie(lower_tr, upper_tr) = dataFor(
        timestamp, max_time_diff,
        [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) { return tr.velocity_.has_value(); },
        interp_options);

    bool lower_has_val = lower_tr && lower_tr->velocity_.has_value();
    bool upper_has_val = upper_tr && upper_tr->velocity_.has_value();

    // no value
    if (!lower_has_val && !upper_has_val) // TODO check if data sources m3a capable
        return {};

    if ((lower_has_val && !upper_has_val)
        || (!lower_has_val && upper_has_val)) // only 1 usable
    {
        dbContent::targetReport::ReconstructorInfo& ref1 = lower_has_val ? *lower_tr : *upper_tr;
        traced_assert(ref1.velocity_);

        return ref1.velocity_->speed_;
    }

    // both set & reliable
    dbContent::targetReport::ReconstructorInfo& ref1 = *lower_tr;
    dbContent::targetReport::ReconstructorInfo& ref2 = *upper_tr;

    traced_assert(ref1.velocity_.has_value());
    traced_assert(ref2.velocity_.has_value());

    if ((ref1.timestamp_ - timestamp).total_milliseconds() <= (ref2.timestamp_ - timestamp).total_milliseconds())
        return ref1.velocity_->speed_;
    else
        return ref2.velocity_->speed_;
}

ComparisonResult ReconstructorTarget::compareModeCCode (
    const dbContent::targetReport::ReconstructorInfo& tr,
    time_duration max_time_diff, float max_alt_diff, bool do_debug) const
{
    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC tr " << tr.asStr();

    bool tr_mode_c_unreliable = tr.barometric_altitude_.has_value() && !tr.barometric_altitude_->hasReliableValue();

    if (!hasDataForTime(tr.timestamp_, max_time_diff))
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " no data for time ("
                   << Time::toString(timestamp_min_) << "-" << Time::toString(timestamp_max_) << ")";

        return ComparisonResult::UNKNOWN;
    }

    dbContent::targetReport::ReconstructorInfo* lower_tr, *upper_tr;

    tie(lower_tr, upper_tr) = dataFor(
        tr.timestamp_, max_time_diff,
        [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) {
            return tr.barometric_altitude_.has_value() && tr.barometric_altitude_->hasReliableValue(); });

    bool lower_no_mc = lower_tr && !lower_tr->barometric_altitude_.has_value();
    bool upper_no_mc = upper_tr && !upper_tr->barometric_altitude_.has_value();

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
               << " lower_no_mc " << lower_no_mc << " upper_no_mc " << upper_no_mc;

    // no mode c, and one missing in one of the others
    if (!tr.barometric_altitude_.has_value() && (lower_no_mc || upper_no_mc)) // TODO check if data sources mc capable
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " no mode c, same";

        return ComparisonResult::SAME;
    }

    bool lower_mc_usable = lower_tr && lower_tr->barometric_altitude_.has_value()
                           && lower_tr->barometric_altitude_->hasReliableValue();

    bool upper_mc_usable = upper_tr && upper_tr->barometric_altitude_.has_value()
                           && upper_tr->barometric_altitude_->hasReliableValue();

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
               << " lower_mc_usable " << lower_mc_usable << " upper_mc_usable " << upper_mc_usable;

    // no able to compare
    if (!lower_mc_usable && !upper_mc_usable)
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " none usable, unknown";

        return ComparisonResult::UNKNOWN;
    }

    if ((lower_mc_usable && !upper_mc_usable)
        || (!lower_mc_usable && upper_mc_usable)) // only 1 usable
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " only 1 usable";

        dbContent::targetReport::ReconstructorInfo& ref1 = lower_mc_usable ? *lower_tr : *upper_tr;
        traced_assert(ref1.barometric_altitude_);

        if (!tr.barometric_altitude_.has_value())
        {
            if (do_debug)
                loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                       << " no tr mode c, different";

            return ComparisonResult::DIFFERENT;
        }

        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " tr mode c, same "
                   << (fabs(tr.barometric_altitude_->altitude_ - ref1.barometric_altitude_->altitude_) < max_alt_diff);

        // barometric_altitude exists
        if (fabs(tr.barometric_altitude_->altitude_ - ref1.barometric_altitude_->altitude_) < max_alt_diff) // is same
        {
            if (tr_mode_c_unreliable)
                return ComparisonResult::UNKNOWN;
            else
                return ComparisonResult::SAME;
        }
        else
            return ComparisonResult::DIFFERENT;
    }

    // both set & reliable

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
               << " both set & reliable";

    dbContent::targetReport::ReconstructorInfo& ref1 = *lower_tr;
    dbContent::targetReport::ReconstructorInfo& ref2 = *upper_tr;

    if (!tr.barometric_altitude_.has_value())
    {
        if (do_debug)
            loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
                   << " no tr mode c, different";

        return ComparisonResult::DIFFERENT; // no mode a here, but in other
    }

    // everything exists

    if (do_debug)
        loginf << "DBG tr " << tr.record_num_ << " other_utn " << utn_ << " cmpMC"
               << " tr mode c, same " << ((fabs(tr.barometric_altitude_->altitude_ - ref1.barometric_altitude_->altitude_) < max_alt_diff)
        || (fabs(tr.barometric_altitude_->altitude_ - ref2.barometric_altitude_->altitude_)) < max_alt_diff);

    if ((fabs(tr.barometric_altitude_->altitude_ - ref1.barometric_altitude_->altitude_) < max_alt_diff)
        || (fabs(tr.barometric_altitude_->altitude_ - ref2.barometric_altitude_->altitude_)) < max_alt_diff) // one of them is same
    {
        if (tr_mode_c_unreliable)
            return ComparisonResult::UNKNOWN;
        else
            return ComparisonResult::SAME;
    }
    else
        return ComparisonResult::DIFFERENT;
}

//fl_unknown, fl_on_ground, alt_baro_ft
std::tuple<bool, bool, float> ReconstructorTarget::getAltitudeState (
    const boost::posix_time::ptime& ts,
    const boost::posix_time::time_duration& max_time_diff,
    const ReconstructorTarget::InterpOptions& interp_options) const
{
    boost::optional<float> mode_c_code = modeCCodeAt (ts, max_time_diff, interp_options);
    boost::optional<bool> gbs          = groundBitAt (ts, max_time_diff, interp_options);
    boost::optional<double> speed_ms   = groundSpeedAt (ts, max_time_diff, interp_options);

    bool fl_unknown {true};
    bool fl_on_ground {false};
    float alt_baro_ft {0};

    if ((gbs && *gbs)
        || (mode_c_code && *mode_c_code <= on_ground_max_alt_ft_)
        || (speed_ms && *speed_ms <= on_ground_max_speed_ms_)) // gbs, very close to ground, or slower than 30 m/s (~55kts)
    {
        fl_on_ground = true;
        fl_unknown = false;
    }
    else if (mode_c_code) // not on ground
    {
        alt_baro_ft = *mode_c_code;

        fl_unknown = false;
        fl_on_ground = false;
    }

    return std::tuple<bool, bool, float>(fl_unknown, fl_on_ground, alt_baro_ft);
}

AltitudeState ReconstructorTarget::getAltitudeStateStruct(const boost::posix_time::ptime& ts, 
                                                          const boost::posix_time::time_duration& max_time_diff,
                                                          const InterpOptions& interp_options) const
{
    AltitudeState as;
    std::tie(as.fl_unknown, as.fl_on_ground, as.alt_baro_ft) = getAltitudeState(ts, max_time_diff, interp_options);

    return as;
}

TimedDataSeries<unsigned int> ReconstructorTarget::getMode3ASeries() const
{
    auto ts_begin = tr_timestamps_.begin()->first;
    auto ts_end = tr_timestamps_.rbegin()->first;

    float value_usage_seconds = 10; //+/-

    std::function<int(unsigned long)> confidence_func =
        [this](unsigned long rec_num) -> int
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.mode_a_code_.has_value())
        {
            const targetReport::ModeACode& value = *info.mode_a_code_;

            if (value.valid_ && (!*value.valid_))
                return -1;

            if (value.garbled_ && *value.garbled_)
                return -1;

            if (info.dbcont_id_ == 21
                || (info.isModeSDetection() && value.valid_ && *value.valid_ && !value.garbled_ && !value.smoothed_)
                || (value.valid_ && *value.valid_ && value.garbled_ && !(*value.garbled_) && !value.smoothed_)
                || (value.valid_ && *value.valid_ && value.garbled_ && !(*value.garbled_) && value.smoothed_ && !(*value.smoothed_)))
                return 5 + (info.dbcont_id_ != 62);
            if (value.valid_ && *value.valid_ && !value.garbled_) // might be smoothed
                return 3 + (info.dbcont_id_ != 62);
            if (info.isModeSDetection())
                return 1 + (info.dbcont_id_ != 62);

            return 0;
        }

        return -1;  // Always return confidence = -1 if no value
    };

    std::function<boost::optional<unsigned int>(unsigned long)> value_func =
        [this](unsigned long rec_num) -> boost::optional<unsigned int>
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.mode_a_code_.has_value())
            return info.mode_a_code_->code_;

        return boost::none;
    };

    TimedDataSeries<unsigned int> ts(
        ts_begin, ts_end, value_usage_seconds, confidence_func, value_func);

    for (auto tr_it : tr_timestamps_)
        ts.insert(tr_it.first, tr_it.second); // , utn_ == 8

    return ts;
}

TimedDataSeries<float> ReconstructorTarget::getAltitudeSeries() const
{
    auto ts_begin = tr_timestamps_.begin()->first;
    auto ts_end = tr_timestamps_.rbegin()->first;

    float value_usage_seconds = 5; //+/-

    std::function<int(unsigned long)> confidence_func =
        [this](unsigned long rec_num) -> int
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.barometric_altitude_.has_value())
        {
            const targetReport::BarometricAltitude& value = *info.barometric_altitude_;

            if (value.valid_ && (!*value.valid_))
                return -1;

            if (value.garbled_ && *value.garbled_)
                return -1;

            if (info.dbcont_id_ == 21
                || (info.isModeSDetection() && value.valid_ && *value.valid_ && !value.garbled_)
                || (value.valid_ && *value.valid_ && value.garbled_ && !(*value.garbled_)))
                return 5 + (info.dbcont_id_ != 62);
            if (value.valid_ && *value.valid_ && !value.garbled_)
                return 3 + (info.dbcont_id_ != 62);
            if (info.isModeSDetection())
                return 1 + (info.dbcont_id_ != 62);

            return 0;
        }

        return -1;  // Always return confidence = -1 if no value
    };

    std::function<boost::optional<float>(unsigned long)> value_func =
        [this](unsigned long rec_num) -> boost::optional<float>
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.barometric_altitude_.has_value())
            return info.barometric_altitude_->altitude_;

        return boost::none;
    };

    TimedDataSeries<float> ts(
        ts_begin, ts_end, value_usage_seconds, confidence_func, value_func);

    for (auto tr_it : tr_timestamps_)
        ts.insert(tr_it.first, tr_it.second); // , utn_ == 8

    return ts;
}

TimedDataSeries<bool> ReconstructorTarget::getGroundBitSeries() const
{
    auto ts_begin = tr_timestamps_.begin()->first;
    auto ts_end = tr_timestamps_.rbegin()->first;

    float altitude_usage_seconds = 5; //+/-

    std::function<int(unsigned long)> confidence_func =
        [this](unsigned long rec_num) -> int
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.ground_bit_.has_value())
        {
            if (info.data_source_is_ground_only)
                return 2;

            if (info.dbcont_id_ == 21 || info.isModeSDetection())
                return 1;

            return 0;
        }

        return -1;  // Always return confidence = -1 if no value
    };

    std::function<boost::optional<bool>(unsigned long)> value_func =
        [this](unsigned long rec_num) -> boost::optional<bool>
    {
        const dbContent::targetReport::ReconstructorInfo& info =
            reconstructor_.target_reports_.at(rec_num);

        if (info.data_source_is_ground_only)
            return true;

        if (info.ground_bit_.has_value())
            return *info.ground_bit_;

        return boost::none;
    };

    TimedDataSeries<bool> ts(
        ts_begin, ts_end, altitude_usage_seconds, confidence_func, value_func);

    for (auto tr_it : tr_timestamps_)
        ts.insert(tr_it.first, tr_it.second); // , utn_ == 8

    return ts;
}

void ReconstructorTarget::updateCounts()
{
    for (auto& rn_it : target_reports_)
    {
        traced_assert(reconstructor_.target_reports_.count(rn_it));
        dbContent::targetReport::ReconstructorInfo& tr = reconstructor_.target_reports_.at(rn_it);

        if (tr.timestamp_ >= reconstructor_.currentSlice().write_before_time_) // tr.in_current_slice_
            continue;

        counts_[Number::recNumGetDBContId(rn_it)] += 1;
    }
}

std::map <std::string, unsigned int> ReconstructorTarget::getDBContentCounts() const
{
    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    std::map <std::string, unsigned int> counts;

    for (auto& cnt_it : counts_)
    {
        counts[dbcont_man.dbContentWithId(cnt_it.first)] = cnt_it.second;
    }

    return counts;
}

std::shared_ptr<Buffer> ReconstructorTarget::getReferenceBuffer()
{
    logdbg2 << "utn " << utn_ << " ref size " << references_.size();

    string dbcontent_name = "RefTraj";
    unsigned int dbcontent_id = 255;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    PropertyList buffer_list;

    // basics
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sac_id_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sic_id_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_));

    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_));

    // pos
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_latitude_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_longitude_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_));

    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_bit_));

    // track num begin, end
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_begin_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_end_));

    // spd
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vx_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vy_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_angle_));

    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rocd_));

    // accs
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ax_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ay_));

    // stddevs
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_x_stddev_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_y_stddev_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_xy_cov_));

    // secondary
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_));

    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_utn_));

    // yo mom so acc
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_long_acc_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_trans_acc_));
    buffer_list.addProperty(dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_vert_rate_));

    // contrib
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_adsb_age_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_mlat_age_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_radar_age_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_tracker_age_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_reftraj_age_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_other_age_));

    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_sources_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_sources_num_));

    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_primary_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_modeac_));
    buffer_list.addProperty(dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_modes_));

    std::shared_ptr<Buffer> buffer = std::make_shared<Buffer>(buffer_list, dbcontent_name);

    // exit if no data
    if (tr_timestamps_.size() <= 1 || references_.size() <= 1)
        return buffer;

    NullableVector<unsigned int>& ds_id_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_).name());
    NullableVector<unsigned char>& sac_vec = buffer->get<unsigned char> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sac_id_).name());
    NullableVector<unsigned char>& sic_vec = buffer->get<unsigned char> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sic_id_).name());
    NullableVector<unsigned int>& line_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_).name());

    NullableVector<float>& tod_vec = buffer->get<float> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_).name());
    NullableVector<ptime>& ts_vec = buffer->get<ptime> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_).name());

    NullableVector<double>& lat_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_latitude_).name());
    NullableVector<double>& lon_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_longitude_).name());
    NullableVector<float>& mc_vec = buffer->get<float> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_).name());

    // track num begin, end
    NullableVector<bool>& track_begin_vec = buffer->get<bool> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_begin_).name());
    NullableVector<bool>& track_end_vec = buffer->get<bool> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_end_).name());
    NullableVector<unsigned int>& track_num_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_).name());

    // speed, track angle
    NullableVector<double>& vx_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vx_).name());
    NullableVector<double>& vy_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vy_).name());

    NullableVector<double>& speed_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_).name());
    NullableVector<double>& track_angle_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_angle_).name());

    NullableVector<float>& rocd_vec = buffer->get<float> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_rocd_).name());

    // accs
    NullableVector<double>& ax_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ax_).name());
    NullableVector<double>& ay_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ay_).name());

    // stddevs
    NullableVector<double>& x_stddev_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_x_stddev_).name());
    NullableVector<double>& y_stddev_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_y_stddev_).name());
    NullableVector<double>& xy_cov_vec = buffer->get<double> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_xy_cov_).name());

    // ground bit
    NullableVector<bool>& gb_vec = buffer->get<bool> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_bit_).name());

    NullableVector<unsigned int>& m3a_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_).name());
    NullableVector<unsigned int>& acad_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_).name());
    NullableVector<string>& acid_vec = buffer->get<string> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_).name());

    // mom
    NullableVector<unsigned char>& mom_long_acc_vec = buffer->get<unsigned char> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_long_acc_).name());
    NullableVector<unsigned char>& mom_trans_acc_vec = buffer->get<unsigned char> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_trans_acc_).name());
    NullableVector<unsigned char>& mom_vert_rate_vec = buffer->get<unsigned char> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mom_vert_rate_).name());

    // contribution
    NullableVector<float>& cont_adsb_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_adsb_age_).name());
    NullableVector<float>& cont_mlat_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_mlat_age_).name());
    NullableVector<float>& cont_radar_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_radar_age_).name());
    NullableVector<float>& cont_tracker_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_tracker_age_).name());
    NullableVector<float>& cont_reftraj_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_reftraj_age_).name());
    NullableVector<float>& cont_other_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_other_age_).name());

    NullableVector<json>& cont_sources_vec = buffer->get<json>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_sources_).name());
    NullableVector<unsigned int>& cont_source_num_vec = buffer->get<unsigned int>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_contrib_sources_num_).name());        

    NullableVector<float>& cont_primary_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_primary_).name());
    NullableVector<float>& cont_modeac_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_modeac_).name());
    NullableVector<float>& cont_modes_age_vec = buffer->get<float>(
        dbcontent_man.getVariable(dbcontent_name, DBContent::var_reftraj_update_age_modes_).name());

    NullableVector<unsigned int>& utn_vec = buffer->get<unsigned int> (
        dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_utn_).name());

    unsigned int ref_sac = reconstructor_.settings().ds_sac;
    unsigned int ref_sic = reconstructor_.settings().ds_sic;
    unsigned int ref_ds_id = Number::dsIdFrom(ref_sac, ref_sic);
    traced_assert(reconstructor_.settings().ds_line >= 0 && reconstructor_.settings().ds_line <= 3);

    double speed_ms, bearing_rad, xy_cov;
    double ax, ay, bearing_new_rad, turnrate_rad, a_ms2;
    double rocd_ft_s;
    double dt;

    unsigned int buffer_cnt = 0;

    const auto& ref_calc_settings = reconstructor_.referenceCalculatorSettings();

    boost::posix_time::time_duration d_max = Time::partialSeconds(10);
    boost::posix_time::time_duration track_end_time = Time::partialSeconds(30);
    boost::posix_time::time_duration reftraj_ui = Time::partialSeconds(ref_calc_settings.chainEstimatorSettings().resample_dt);

    auto m3a_series = getMode3ASeries();
    auto altitude_series = getAltitudeSeries();
    auto gbs_series = getGroundBitSeries();

    ContributingSourcesInfo contrib_info;

    // collect correct target reports for each reference update
    // double iteration over reference_tr_usages_ and references_
    auto ref_it = references_.begin();
    traced_assert(ref_it != references_.end());
    references_tr_contributions_[ref_it->first] = contrib_info;

    logdbg << "utn " << utn_;

    for (auto tr_it = reference_tr_usages_.begin(); tr_it != reference_tr_usages_.end(); ++tr_it)
    {
        // advance ref_it to the next element where ref_it.first >= tr_it.first
        while (ref_it != references_.end() && ref_it->first < tr_it->first)
        {
            logdbg << "utn " << utn_ << " stepping to new ref up, current " << Time::toString(ref_it->first);
            // step info
            contrib_info.increaseTimeTo(ref_it->first);
            references_tr_contributions_[ref_it->first] = contrib_info;

            // step to next reference update
            ++ref_it;

            contrib_info.rec_nums_.clear();
        }

        if (ref_it == references_.end())
            break;

        traced_assert (tr_it->first <= ref_it->first);

        // ref_it is ok and points to next ref update in time
        if (!reconstructor_.target_reports_.count(tr_it->second))
        {
            logerr << "unknown record number " << tr_it->second;
            continue;
        }

        logdbg << "utn " << utn_ << " adding tr " << Time::toString(tr_it->first);
        contrib_info.add(reconstructor_.target_reports_.at(tr_it->second),
                            (ref_it->first - tr_it->first) < reftraj_ui);
        
    }

    // finish up for ref updates without target reports
    while (ref_it != references_.end())
    {
        if (!references_tr_contributions_.count(ref_it->first))
        {
            contrib_info.rec_nums_.clear();
            contrib_info.increaseTimeTo(ref_it->first);
            references_tr_contributions_[ref_it->first] = contrib_info;
        }

        // step to next reference update
        ++ref_it;
    }

    // generate buffer
    for (auto& ref_it : references_)
    {
        // loginf << "utn " << utn_
        //        << " ref ts " << Time::toString(ref_it.second.t)
        //        << " wbt " << Time::toString(reconstructor_.currentSlice().write_before_time_) <<
        //     " skip " << (ref_it.second.t >= reconstructor_.currentSlice().write_before_time_);

        if (ref_it.second.t >= reconstructor_.currentSlice().write_before_time_) // skip, too early
            continue;

        if (!ref_ts_prev_.is_not_a_date_time())
            traced_assert(ref_it.second.t > ref_ts_prev_);

        // final filtering using max stddev
        if (ref_calc_settings.filter_references_max_stddev_ &&
            ref_it.second.x_stddev.has_value() && 
            ref_it.second.y_stddev.has_value())
        {
            double stddev_max = std::max(ref_it.second.x_stddev.value(), ref_it.second.y_stddev.value());
            if (stddev_max > ref_calc_settings.filter_references_max_stddev_m_)
                continue;
        }

        traced_assert(references_tr_contributions_.count(ref_it.second.t));

        ds_id_vec.set(buffer_cnt, ref_ds_id);
        sac_vec.set(buffer_cnt, ref_sac);
        sic_vec.set(buffer_cnt, ref_sic);
        line_vec.set(buffer_cnt, reconstructor_.settings().ds_line);

        ts_vec.set(buffer_cnt, ref_it.second.t);
        tod_vec.set(buffer_cnt, ref_it.second.t.time_of_day().total_milliseconds() / 1000.0);

        lat_vec.set(buffer_cnt, ref_it.second.lat);
        lon_vec.set(buffer_cnt, ref_it.second.lon);

        // add to min/max pos

        if (latitude_min_ && latitude_max_
            && longitude_min_ && longitude_max_)
        {
            latitude_min_ = min(*latitude_min_, ref_it.second.lat);
            latitude_max_ = max(*latitude_max_, ref_it.second.lat);

            longitude_min_ = min(*longitude_min_, ref_it.second.lon);
            longitude_max_ = max(*longitude_max_, ref_it.second.lon);
        }
        else
        {
            latitude_min_ = ref_it.second.lat;
            latitude_max_ = ref_it.second.lat;

            longitude_min_ = ref_it.second.lon;
            longitude_max_ = ref_it.second.lon;
        }

        utn_vec.set(buffer_cnt, utn_);

        track_num_vec.set(buffer_cnt, utn_);

        // track end
        if (!ref_ts_prev_.is_not_a_date_time()
            && ref_it.second.t - ref_ts_prev_ > track_end_time) // have time before and dt > track end time
        {
            if (buffer_cnt != 0)
                track_end_vec.set(buffer_cnt-1, true); // set at previous update if possible

            track_begin_ = true;

            has_prev_v_ = false; // info too old
            has_prev_baro_alt_ = false;
        }

        track_end_vec.set(buffer_cnt, false);
        track_begin_vec.set(buffer_cnt, track_begin_);

        track_begin_ = false;

        // set speed

        if (ref_it.second.vx.has_value() && ref_it.second.vy.has_value())
        {
            vx_vec.set(buffer_cnt, *ref_it.second.vx);
            vy_vec.set(buffer_cnt, *ref_it.second.vy);

            speed_ms = sqrt(pow(*ref_it.second.vx, 2)+pow(*ref_it.second.vy, 2)); // for 1s
            bearing_rad = atan2(*ref_it.second.vx, *ref_it.second.vy);

            speed_vec.set(buffer_cnt, speed_ms * M_S2KNOTS);
            track_angle_vec.set(buffer_cnt, bearing_rad * RAD2DEG);

            // set ax, ay
            if (has_prev_v_)
            {
                ax = *ref_it.second.vx - v_x_prev_;
                ay = *ref_it.second.vy - v_y_prev_;

                ax_vec.set(buffer_cnt, ax);
                ay_vec.set(buffer_cnt, ay);

                a_ms2 = sqrt(pow(ax, 2)+pow(ay, 2)); // for 1s2

                // LONG ACC
                if (fabs(a_ms2) < 1) // like 0
                    mom_long_acc_vec.set(buffer_cnt, (unsigned char) MOM_LONG_ACC::ConstantGroundspeed);
                else if (a_ms2 > 0)
                    mom_long_acc_vec.set(buffer_cnt, (unsigned char) MOM_LONG_ACC::IncreasingGroundspeed);
                else if (a_ms2 < 0)
                    mom_long_acc_vec.set(buffer_cnt, (unsigned char) MOM_LONG_ACC::DecreasingGroundspeed);

                bearing_new_rad = atan2(*ref_it.second.vx + ax, *ref_it.second.vy + ay); // for 1s

                turnrate_rad = bearing_new_rad - bearing_rad;

                //loginf << " turnrate_rad " << turnrate_rad;

                while (turnrate_rad > M_PI)
                    turnrate_rad -= 2*M_PI;
                while (turnrate_rad < -M_PI)
                    turnrate_rad += 2*M_PI;

                traced_assert(fabs(turnrate_rad) <= M_PI);

                // TRANS ACC
                if (fabs(turnrate_rad) < M_PI/20) // like 0
                    mom_trans_acc_vec.set(buffer_cnt, (unsigned char) MOM_TRANS_ACC::ConstantCourse);
                else if (turnrate_rad > 0)
                    mom_trans_acc_vec.set(buffer_cnt, (unsigned char) MOM_TRANS_ACC::RightTurn);
                else if (turnrate_rad < 0)
                    mom_trans_acc_vec.set(buffer_cnt, (unsigned char) MOM_TRANS_ACC::LeftTurn);
            }

            v_x_prev_ = *ref_it.second.vx;
            v_y_prev_ = *ref_it.second.vy;
            has_prev_v_ = true;
        }
        else
            has_prev_v_ = false;

        if (mom_long_acc_vec.isNull(buffer_cnt))
            mom_long_acc_vec.set(buffer_cnt, (unsigned char) MOM_LONG_ACC::Undetermined);

        if (mom_trans_acc_vec.isNull(buffer_cnt))
            mom_trans_acc_vec.set(buffer_cnt, (unsigned char) MOM_TRANS_ACC::Undetermined);

        // set stddevs

        if (ref_it.second.x_stddev.has_value() && ref_it.second.y_stddev.has_value() && ref_it.second.xy_cov.has_value())
        {
            x_stddev_vec.set(buffer_cnt, *ref_it.second.x_stddev);
            y_stddev_vec.set(buffer_cnt, *ref_it.second.y_stddev);

            xy_cov = *ref_it.second.xy_cov;

            // to inverse of this asterix rep
            // if (xy_cov < 0)
            //     xy_cov = - pow(xy_cov, 2);
            // else
            //     xy_cov = pow(xy_cov, 2);

            // if (xy_cov < 0)
            //     xy_cov_vec.set(buffer_cnt, -sqrt(-xy_cov));
            // else
            //     xy_cov_vec.set(buffer_cnt, sqrt(xy_cov));

            xy_cov_vec.set(buffer_cnt, xy_cov);
        }

        // set other data
        {
            if (altitude_series.hasValueAt(ref_it.second.t))
                mc_vec.set(buffer_cnt, altitude_series.getValueAt(ref_it.second.t));

            if (!mc_vec.isNull(buffer_cnt)) // has current alt
            {
                if (has_prev_baro_alt_)
                {
                    dt = (ts_vec.get(buffer_cnt) - ref_ts_prev_).total_milliseconds() / 1000.0;

                    rocd_ft_s = (mc_vec.get(buffer_cnt) - baro_alt_prev_) / dt;
                    rocd_vec.set(buffer_cnt, rocd_ft_s * 60); // ft per minute

                    // MOM Vertical Rate
                    if (rocd_ft_s == 0)
                        mom_vert_rate_vec.set(buffer_cnt, (unsigned char) MOM_VERT_RATE::Level);
                    else if (rocd_ft_s > 0)
                        mom_vert_rate_vec.set(buffer_cnt, (unsigned char) MOM_VERT_RATE::Climb);
                    else if (rocd_ft_s < 0)
                        mom_vert_rate_vec.set(buffer_cnt, (unsigned char) MOM_VERT_RATE::Descent);
                }

                has_prev_baro_alt_ = true;
                baro_alt_prev_ = mc_vec.get(buffer_cnt);
            }
            else
                has_prev_baro_alt_ = false;

        }

        if (mom_vert_rate_vec.isNull(buffer_cnt))
            mom_vert_rate_vec.set(buffer_cnt, (unsigned char) MOM_VERT_RATE::Undetermined);

        { // mode a

            if (m3a_series.hasValueAt(ref_it.second.t))
                m3a_vec.set(buffer_cnt, m3a_series.getValueAt(ref_it.second.t));
        }

        { // acad
            ReconstructorInfoPair info = dataFor(
                ref_it.second.t, d_max,
                [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) {
                    return tr.acad_.has_value(); });

            if (info.first && info.first->acad_ && acad_vec.isNull(buffer_cnt))
                acad_vec.set(buffer_cnt, *info.first->acad_);

            if (info.second && info.second->acad_ && acad_vec.isNull(buffer_cnt))
                acad_vec.set(buffer_cnt, *info.second->acad_);

        }

        { // acid
            ReconstructorInfoPair info = dataFor(
                ref_it.second.t, d_max,
                [ & ] (const dbContent::targetReport::ReconstructorInfo& tr) {
                    return tr.acid_.has_value(); });

            if (info.first && info.first->acid_ && acid_vec.isNull(buffer_cnt))
                acid_vec.set(buffer_cnt, *info.first->acid_);

            if (info.second && info.second->acid_ && acid_vec.isNull(buffer_cnt))
                acid_vec.set(buffer_cnt, *info.second->acid_);

        }

        { // gbs

            if (gbs_series.hasValueAt(ref_it.second.t))
                gb_vec.set(buffer_cnt, gbs_series.getValueAt(ref_it.second.t));
        }

        if (references_tr_contributions_.count(ref_it.second.t))
        {
            auto& contrib = references_tr_contributions_.at(ref_it.second.t);

            if (contrib.adsb_age_)
                cont_adsb_age_vec.set(buffer_cnt, *contrib.adsb_age_);

            if (contrib.mlat_age_)
                cont_mlat_age_vec.set(buffer_cnt, *contrib.mlat_age_);

            if (contrib.radar_age_)
                cont_radar_age_vec.set(buffer_cnt, *contrib.radar_age_);

            if (contrib.reftraj_age_)
                cont_reftraj_age_vec.set(buffer_cnt, *contrib.reftraj_age_);

            if (contrib.tracker_age_)
                cont_tracker_age_vec.set(buffer_cnt, *contrib.tracker_age_);

            if (contrib.other_age_)
                cont_other_age_vec.set(buffer_cnt, *contrib.other_age_);

            if (contrib.rec_nums_.size())
                cont_sources_vec.set(buffer_cnt, contrib.contributions(reconstructor_));
            cont_source_num_vec.set(buffer_cnt, contrib.rec_nums_.size());

            if (contrib.primary_age_)
                cont_primary_age_vec.set(buffer_cnt, *contrib.primary_age_);
            if (contrib.mode_ac_age_)
                cont_modeac_age_vec.set(buffer_cnt, *contrib.mode_ac_age_);
            if (contrib.modes_age_)
                cont_modes_age_vec.set(buffer_cnt, *contrib.modes_age_);                                                
        }

        ++buffer_cnt;

        ref_ts_prev_ = ref_it.second.t;
    }

    // check last update for track end
    if (buffer->size())
    {
        unsigned int last_index = buffer_cnt - 1;

        if (reconstructor_.currentSlice().next_slice_begin_ - ts_vec.get(last_index) > track_end_time)
            track_end_vec.set(last_index, true);
    }

    counts_[dbcontent_id] += buffer->size();

    logdbg2 << "utn " << utn_ << " buffer size " << buffer->size();
    //assert (buffer->size());

    return buffer;
}

void ReconstructorTarget::removeOutdatedTargetReports()
{
    auto tmp_tr_timestamps = std::move(tr_timestamps_);

    target_reports_.clear();
    tr_timestamps_.clear();
    tr_ds_timestamps_.clear();

    //@TODO: check record number?
    standing_adsb_target_.reset();

    // if (chain())
    //     chain()->removeUpdatesBefore(reconstructor_.currentSlice().remove_before_time_);

    for (auto& ts_it : tmp_tr_timestamps)
    {
        if (reconstructor_.target_reports_.count(ts_it.second))
        {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
            traced_assert(!reconstructor_.target_reports_.at(ts_it.second).in_current_slice_);
#endif

            addTargetReportInternal(ts_it.second, false, false);
        }
    } 

    references_.clear();
}

void ReconstructorTarget::removeTargetReportsLaterOrEqualThan(boost::posix_time::ptime ts)
{
    auto tmp_tr_timestamps = std::move(tr_timestamps_);

    target_reports_.clear();
    tr_timestamps_.clear();
    tr_ds_timestamps_.clear();

    //@TODO: check timestamp?
    standing_adsb_target_.reset();

    // if (chain()) // moved to repeat run in probimmreconstructor
    //     chain()->removeUpdatesLaterThan(ts);

    for (auto& ts_it : tmp_tr_timestamps)
    {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
        traced_assert(reconstructor_.target_reports_.count(ts_it.second));

        dbContent::targetReport::ReconstructorInfo& tr = reconstructor_.target_reports_.at(ts_it.second);
        traced_assert(tr.timestamp_ == ts_it.first);
#endif

        if (ts_it.first < ts) // add if older than ts
            addTargetReportInternal(ts_it.second, false, false);
        else
            break;
    }

    references_.clear();
}

void ReconstructorTarget::targetCategory(TargetBase::Category category)
{
    ecat_ = static_cast<unsigned int>(category);
}

TargetBase::Category ReconstructorTarget::targetCategory() const
{
    if (!ecat_) {
        return Category::Unknown;
    }
    return TargetBase::fromECAT(ecat_);
}


bool ReconstructorTarget::hasTracker() const
{
    return (chain() != nullptr);
}

size_t ReconstructorTarget::trackerCount() const
{
    return chain()->size();
}

boost::posix_time::ptime ReconstructorTarget::trackerTime(size_t idx) const
{
    return chain()->getUpdate(idx).t;
}

void ReconstructorTarget::reinitTracker()
{
    chain().reset(new reconstruction::KalmanChain);

    //override some estimator settings for the chain
    chain()->settings().mode            = dynamic_insertions_ ? reconstruction::KalmanChain::Settings::Mode::DynamicInserts :
                                  reconstruction::KalmanChain::Settings::Mode::StaticAdd;
    chain()->settings().prediction_mode = reconstruction::KalmanChain::Settings::PredictionMode::Interpolate;
    chain()->settings().verbosity       = 0;
    chain()->settings().debug           = false; //utn_ == 537;

    chain()->configureEstimator(reconstructor_.referenceCalculatorSettings().chainEstimatorSettings());
    chain()->init(reconstructor_.referenceCalculatorSettings().kalman_type_assoc);

    ReconstructorBase* rec_ptr = &reconstructor_;

    chain()->setMeasurementAssignFunc(
        [ rec_ptr ] (reconstruction::Measurement& mm, unsigned long rec_num)
        {
            rec_ptr->createMeasurement(mm, rec_num);
        });

    chain()->setMeasurementCheckFunc(
        [ rec_ptr ] (unsigned long rec_num)
        {
            return rec_ptr->target_reports_.find(rec_num) != rec_ptr->target_reports_.end();
        });
}

bool ReconstructorTarget::compareChainUpdates(const dbContent::targetReport::ReconstructorInfo& tr,
                                              const dbContent::targetReport::ReconstructorInfo& tr_other) const
{
    if (!tr.position_accuracy_.has_value())
        return false;
    if (!tr_other.position_accuracy_.has_value())
        return true;

    return tr.position_accuracy_.value().maxStdDev() < tr_other.position_accuracy_.value().maxStdDev();
}

bool ReconstructorTarget::checkChainBeforeAdd(const dbContent::targetReport::ReconstructorInfo& tr,
                                              int idx) const
{
    unsigned long rec_num  = chain()->getUpdate(idx).mm_id; // UGA not unsigned int
    traced_assert(reconstructor_.target_reports_.count(rec_num));
    const auto&  tr_chain = reconstructor_.target_reports_.at(rec_num);

    return compareChainUpdates(tr, tr_chain);
}

bool ReconstructorTarget::checkChainBeforeAdd(const dbContent::targetReport::ReconstructorInfo& tr,
                                              std::pair<int, int>& idxs_remove) const
{
    // too near to first chain measurement + chain measurement better => fail
    if (idxs_remove.first >= 0 && !checkChainBeforeAdd(tr, idxs_remove.first))
        return false;

    // too near to second chain measurement + chain measurement better => fail
    if (idxs_remove.second >= 0 && !checkChainBeforeAdd(tr, idxs_remove.second))
        return false;

    // ok => replace any too near chain measurements afterwards
    return true;
}



ReconstructorTarget::TargetReportAddResult ReconstructorTarget::addNewTRToTracker(const dbContent::targetReport::ReconstructorInfo& tr, 
                                                                                  bool reestimate,
                                                                                  reconstruction::UpdateStats* stats)
{
    //handle standing adsb
    if (reconstructor_.settings().use_stopped_adsb_tracking_)
    {
        if (tr.dbcont_id_ == 21)
        {
            //invalidate standing adsb target in any case if new target report stems from adsb
            standing_adsb_target_.reset();
        }
        else if (standing_adsb_target_ && 
                standing_adsb_target_->needsUpdate(tr.timestamp_))
        {
            //check if standing adsb target needs new extended updates
            const auto& tr_adsb_standing = reconstructor_.target_reports_.at(standing_adsb_target_->rec_num);

            size_t num_updates = 0;

            //add needed updates
            while (standing_adsb_target_->needsUpdate(tr.timestamp_))
            {
                //@TODO: save some reestimation runs
                //@TODO: log fails/skips
                addToTracker(tr_adsb_standing, standing_adsb_target_->ts_next_update, reestimate);
                standing_adsb_target_->addUpdate();

                ++num_updates;
            }

            if (stats)
            {
                stats->num_standing_adsb_updates_total += num_updates;
                stats->num_standing_adsb_updates_min = std::min(stats->num_standing_adsb_updates_min, (size_t)standing_adsb_target_->num_updated);
                stats->num_standing_adsb_updates_max = std::max(stats->num_standing_adsb_updates_max, (size_t)standing_adsb_target_->num_updated);
            }
        }

        //reset outdated adsb target
        if (standing_adsb_target_ && standing_adsb_target_->isOutdated())
            standing_adsb_target_.reset();
    }

    //add new tr to tracker
    auto ret = addToTracker(tr, tr.timestamp_, reestimate, stats);

    if (reconstructor_.settings().use_stopped_adsb_tracking_)
    {
        //if successful => check if new target report is new standing adsb target
        if (ret == TargetReportAddResult::Added && 
            tr.dbcont_id_ == 21 &&
            !tr.isMoving())
        {
            standing_adsb_target_ = StandingADSBTarget(tr);

            if (stats)
                stats->num_standing_adsb += 1;
        }
    }

    return ret;
}

ReconstructorTarget::TargetReportAddResult ReconstructorTarget::addToTracker(const dbContent::targetReport::ReconstructorInfo& tr, 
                                                                             const boost::posix_time::ptime& ts,
                                                                             bool reestimate, 
                                                                             reconstruction::UpdateStats* stats)
{
    bool do_debug = false;

    traced_assert(chain());

    if (stats)
        ++stats->num_checked;

    if (do_debug)
        loginf << "DBG indexes near";

    auto idxs_remove = chain()->indicesNear(ts, reconstructor_.referenceCalculatorSettings().min_dt);

    if (do_debug)
        loginf << "DBG checkChainBeforeAdd " << idxs_remove.first << ", " << idxs_remove.second;

    //preemptive check failed => just skip
    if (!checkChainBeforeAdd(tr, idxs_remove))
    {
        if (stats)
            ++stats->num_skipped_preemptive;

        if (do_debug)
            loginf << "DBG skipped";

        return TargetReportAddResult::Skipped;
    }

    if (do_debug)
        loginf << "DBG remove";

    //measurement to be inserted => remove any replaced chain updates?
    if (idxs_remove.second >= 0)
    {
        chain()->remove(idxs_remove.second, false);
        //loginf << "removing0";
    }
    if (idxs_remove.first >= 0)
    {
        chain()->remove(idxs_remove.first, false);
        //loginf << "removing1";
    }

    if (stats)
    {
        stats->num_inserted += 1;
        stats->num_replaced += (idxs_remove.first >= 0 ? 1 : 0) + (idxs_remove.second >= 0 ? 1 : 0);
    }
    
    if (do_debug)
        loginf << "DBG insert";

    //insert measurement
    boost::optional<boost::posix_time::ptime> ts_mm;
    if (tr.timestamp_ != ts)
        ts_mm = tr.timestamp_;
    
    bool reestim_ok = chain()->insert(tr.record_num_, ts, ts_mm, reestimate, stats);

    if (!reestimate)
        return TargetReportAddResult::Added;

    if (do_debug)
        loginf << "DBG done";

    return reestim_ok ? TargetReportAddResult::Reestimated : TargetReportAddResult::ReestimationFailed;
}

bool ReconstructorTarget::canPredict(boost::posix_time::ptime ts) const
{
    if (!chain())
        return false;

    return chain()->canPredict(ts);
}

bool ReconstructorTarget::hasChainState(boost::posix_time::ptime ts) const
{
    if (!chain())
        return false;

    return chain()->hasUpdateFor(ts);
}

bool ReconstructorTarget::predictPositionClose(boost::posix_time::ptime ts, double lat, double lon) const
{
    if (!chain())
        return false;

    return chain()->predictPositionClose(ts, lat, lon);
}

bool ReconstructorTarget::predict(reconstruction::Measurement* mm,
                                  kalman::GeoProbState* gp_state,
                                  kalman::GeoProbState* gp_state_mm,
                                  const boost::posix_time::ptime& ts,
                                  reconstruction::PredictionStats* stats) const
{
    traced_assert(chain());

    bool ok = false;

    if (stats)
    {
        ok = chain()->predict(mm, gp_state, gp_state_mm, ts, stats);
    }
    else
    {
        reconstruction::PredictionStats pstats;
        ok = chain()->predict(mm, gp_state, gp_state_mm, ts, &pstats);

        //log immediately (!take care when using this method in a multithreaded context!)
        ReconstructorTarget::addPredictionToGlobalStats(pstats);
    }
    
    traced_assert(ok);

    return ok;
}

bool ReconstructorTarget::predictMT(reconstruction::Measurement* mm,
                                    kalman::GeoProbState* gp_state,
                                    kalman::GeoProbState* gp_state_mm,
                                    const boost::posix_time::ptime& ts,
                                    unsigned int thread_id,
                                    reconstruction::PredictionStats* stats) const
{
    traced_assert(chain());

    bool ok = chain()->predictMT(mm, gp_state, gp_state_mm, ts, reconstructor_.chainPredictors(), thread_id, stats);
    traced_assert(ok);

    return ok;
}

bool ReconstructorTarget::getChainState(reconstruction::Measurement& mm, 
                                        const boost::posix_time::ptime& ts,
                                        reconstruction::PredictionStats* stats) const
{
    traced_assert(chain());

    bool ok = false;

    if (stats)
    {
        ok = chain()->getChainState(mm, ts, stats);
    }
    else
    {
        reconstruction::PredictionStats pstats;
        ok = chain()->getChainState(mm, ts, &pstats);

        //log immediately (!take care when using this method in a multithreaded context!)
        ReconstructorTarget::addPredictionToGlobalStats(pstats);
    }
    
    traced_assert(ok);

    return ok;
}

bool ReconstructorTarget::hasChain() const
{
    return chain().get() != nullptr;
}

const reconstruction::KalmanChain& ReconstructorTarget::getChain() const
{
    traced_assert(chain());

    return *chain();
}

//bool ReconstructorTarget::hasADSBMOPSVersion()
//{
//    return mops_versions_.size();
//}

//std::set<unsigned int> ReconstructorTarget::getADSBMOPSVersions()
//{
//    traced_assert(hasADSBMOPSVersion());

//    return mops_versions_;
//}

} // namespace dbContent
