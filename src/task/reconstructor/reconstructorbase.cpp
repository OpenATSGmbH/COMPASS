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

#include "reconstructorbase.h"
#include "reconstructortask.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "logger.h"
#include "timeconv.h"
#include "datasourcemanager.h"
#include "evaluationmanager.h"
#include "reconstructorassociatorbase.h"

#include "taskmanager.h"
#include "report/report.h"
#include "report/section.h"
#include "report/sectioncontenttext.h"
#include "report/sectioncontenttable.h"

#include "kalman_chain.h"
#include "kalman_online_tracker.h"
#include "tbbhack.h"

#include "dbcontent/variable/metavariable.h"
#include "targetreportaccessor.h"
#include "number.h"
#include "dbcontentstatusinfo.h"

#include "viewpoint.h"
#include "viewpointgenerator.h"
#include "grid2d.h"
#include "grid2dlayer.h"
#include "grid2dlayerrenderer.h"
#include "grid2drendersettings.h"

#include "sector/sectorlayer.h"

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace Utils;

std::set<std::string> ReconstructorBase::TargetsContainer::unspecific_acids_;

void ReconstructorBaseSettings::setVehicleACADs(const std::string& value)
{
    vehicle_acads_ = value;

    vehicle_acads_set_.clear();

    for (const auto& acad_str : String::split(vehicle_acads_, ','))
    {
        try {
            unsigned int acad = String::intFromHexString(acad_str);
            vehicle_acads_set_.insert(acad);
        } catch (...) {

            logwrn << "impossible hex value '" << acad_str << "'";
        }
    }

    loginf << "value '" << value
           << "' vector " << String::compress(vehicle_acads_set_, ',');
}

void ReconstructorBaseSettings::setVehicleACIDs(const std::string& value)
{
    vehicle_acids_ = value;

    vehicle_acids_set_.clear();

    for (std::string acid_str : String::split(vehicle_acids_, ','))
    {
        acid_str = String::trim(acid_str);
        boost::to_upper(acid_str);
        vehicle_acids_set_.insert(acid_str);
    }

    loginf << "value '" << value
           << "' vector " << String::compress(vehicle_acids_set_, ',');
}

unsigned int ReconstructorBase::TargetsContainer::createNewTarget(const dbContent::targetReport::ReconstructorInfo& tr)
{
    unsigned int utn;

    if (targets_.size())
        utn = targets_.rbegin()->first + 1; // new utn
    else
        utn = 0;

    if (tr.acad_)
    {
        if (acad_2_utn_.count(*tr.acad_))
            logerr << "tr " << tr.asStr() << " acad already present in "
                   << targets_.at(acad_2_utn_.at(*tr.acad_)).asStr();

        traced_assert(!acad_2_utn_.count(*tr.acad_));
    }

    if (tr.acid_ && !unspecific_acids_.count(*tr.acid_))
    {
        if (acid_2_utn_.count(*tr.acid_))
        {
            traced_assert(targets_.count(acid_2_utn_.at(*tr.acid_)));

            auto existing_target = targets_.at(acid_2_utn_.at(*tr.acid_));

            logwrn << "tr " << tr.asStr() << " acid already present in "
                   << existing_target.asStr();

            if (tr.acad_ && existing_target.acads_.size()
                && existing_target.hasACAD(*tr.acad_))
            {
                logerr << "acad matches, this seems to be an association error";
                traced_assert(false);
            }

            logwrn << "no acad match, assuming duplicate acid '"
                << *tr.acid_ << "', removing from association criteria";

            acid_2_utn_.erase(*tr.acid_);
            unspecific_acids_.insert(*tr.acid_);
        }

        traced_assert(!acid_2_utn_.count(*tr.acid_));
    }

    if (tr.track_number_)
    {
        if (tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_))
            logerr << "tr " << tr.asStr() << " track num already present in "
                   << targets_.at(tn2utn_[tr.ds_id_][tr.line_id_].at(*tr.track_number_).first).asStr();

        traced_assert(!tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_));
    }

    traced_assert(reconstructor_);

    traced_assert(!targets_.count(utn));

    targets_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(utn),   // args for key
        std::forward_as_tuple(*reconstructor_, utn, true, true));  // args for mapped value tmp_utn false

    // add to lookup

    targets_.at(utn).created_in_current_slice_ = true;

    utn_vec_.push_back(utn);

    return utn;
}


void ReconstructorBase::TargetsContainer::removeUTN(unsigned int other_utn)
{
    traced_assert(targets_.count(other_utn));

    targets_.erase(other_utn);

    // remove from utn list
    auto other_it = std::find(utn_vec_.begin(), utn_vec_.end(), other_utn);
    traced_assert(other_it != utn_vec_.end());
    utn_vec_.erase(other_it);

    // add to removed utns for re-use
    //removed_utns_.push_back(other_utn);
}

void ReconstructorBase::TargetsContainer::replaceInLookup(unsigned int other_utn, unsigned int utn)
{
    traced_assert(targets_.count(utn));

    // remove from acad lookup
    for (auto& acad_it : acad_2_utn_)
    {
        if (acad_it.second == other_utn)
            acad_2_utn_[acad_it.first] = utn;
    }

    // remove from acid lookup
    for (auto& acid_it : acid_2_utn_)
    {
        if (acid_it.second == other_utn)
            acid_2_utn_[acid_it.first] = utn;
    }

    // remove in tn2utn_ lookup
    // ds_id -> line id -> track num -> utn, last tod
    for (auto& ds_it : tn2utn_)
    {
        for (auto& line_it : ds_it.second)
        {
            for (auto& tn_it : line_it.second)
            {
                if (tn_it.second.first == other_utn)
                {
                    // replace utn
                    tn2utn_[ds_it.first][line_it.first][tn_it.first] =
                        std::pair<unsigned int, boost::posix_time::ptime> (
                            (unsigned int) utn, tn_it.second.second);
                }
            }
        }
    }
}

void ReconstructorBase::TargetsContainer::addToLookup(unsigned int utn, dbContent::targetReport::ReconstructorInfo& tr)
{
    traced_assert(targets_.count(utn));

    unsigned int dbcont_id  = Number::recNumGetDBContId(tr.record_num_);

    if (tr.track_number_ && (dbcont_id == 62 || dbcont_id == 255))
        tn2utn_[tr.ds_id_][tr.line_id_][*tr.track_number_] =
            std::pair<unsigned int, boost::posix_time::ptime>(utn, tr.timestamp_);

    if (tr.acad_)
        acad_2_utn_[*tr.acad_] = utn;

    if (tr.acid_ && !unspecific_acids_.count(*tr.acid_))
        acid_2_utn_[*tr.acid_] = utn;
}

void ReconstructorBase::TargetsContainer::checkACADLookup()
{
    unsigned int acad;

    unsigned int count = 0;

    for (auto& target_it : targets_)
    {
        if (!target_it.second.hasACAD())
            continue;

        if (target_it.second.acads_.size() != 1)
            logerr << "double acad in target "
                   << target_it.second.asStr();

        traced_assert(target_it.second.acads_.size() == 1);

        acad = *target_it.second.acads_.begin();

        if (!acad_2_utn_.count(acad))
        {
            logerr << "acad "
                   << String::hexStringFromInt(acad, 6, '0')
                   << " not in lookup";
        }

        ++count;
    }

    for (auto& acad_it : acad_2_utn_)
    {
        traced_assert(targets_.count(acad_it.second));
        traced_assert(targets_.at(acad_it.second).hasACAD(acad_it.first));
    }

    traced_assert(acad_2_utn_.size() == count);
}

bool ReconstructorBase::TargetsContainer::canAssocByACAD(
    dbContent::targetReport::ReconstructorInfo& tr, bool do_debug)
{
    if (!tr.acad_)
        return false;

    if (do_debug)
        loginf << "DBG can use stored utn in acad_2_utn_ " << acad_2_utn_.count(*tr.acad_);

    return acad_2_utn_.count(*tr.acad_);
}

// -1 if failed, else utn
int ReconstructorBase::TargetsContainer::assocByACAD(
    dbContent::targetReport::ReconstructorInfo& tr, bool do_debug)
{
    traced_assert(tr.acad_);
    traced_assert(acad_2_utn_.count(*tr.acad_));

    if (do_debug)
        loginf << "DBG use stored utn in acad_2_utn_: " << acad_2_utn_.at(*tr.acad_);

    int utn = acad_2_utn_.at(*tr.acad_);

    traced_assert(targets_.count(utn));
    traced_assert(targets_.at(utn).hasACAD(*tr.acad_));

    return utn;
}

boost::optional<unsigned int> ReconstructorBase::TargetsContainer::utnForACAD(unsigned int acad)
{
    auto it = std::find_if(targets_.begin(), targets_.end(),
                           [acad](const std::pair<const unsigned int, dbContent::ReconstructorTarget> & t) -> bool {
                               return t.second.hasACAD(acad);
                           });

    if (it != targets_.end())
        return it->first;
    else
        return {};
}

bool ReconstructorBase::TargetsContainer::canAssocByACID(
    dbContent::targetReport::ReconstructorInfo& tr, bool do_debug)
{
    if (!tr.acid_)
        return false;

    if (unspecific_acids_.count(*tr.acid_))
    {
        if (do_debug)
            loginf << "DBG can not use stored utn in acid_2_utn_, unspecifiec acid '" << *tr.acid_ << "'";

        return false;
    }

    if (acid_2_utn_.count(*tr.acid_)) // already exists, but check if mode s address changed
    {
        traced_assert(targets_.count(acid_2_utn_.at(*tr.acid_)));

        // tr has acad, target has an acad but not the target reports
        // happens if same acid is used by 2 different transponders
        if (tr.acad_ && targets_.at(acid_2_utn_.at(*tr.acid_)).hasACAD()
            && !targets_.at(acid_2_utn_.at(*tr.acid_)).hasACAD(!tr.acad_))
        {
            if (do_debug)
                loginf << "DBG same acid used by different transponders '" << *tr.acid_ << "', utn "
                       << targets_.at(acid_2_utn_.at(*tr.acid_)).asStr()
                       << " tr " << tr.asStr();

            return false;
        }
    }

    if (do_debug)
        loginf << "DBG can use stored utn in acid_2_utn_ " << acid_2_utn_.count(*tr.acid_);

    return acid_2_utn_.count(*tr.acid_);
}

 // -1 if failed, else utn
int ReconstructorBase::TargetsContainer::assocByACID(dbContent::targetReport::ReconstructorInfo& tr, bool do_debug)
{
    traced_assert(tr.acid_);
    traced_assert(acid_2_utn_.count(*tr.acid_));
    traced_assert(!unspecific_acids_.count(*tr.acid_));

    if (do_debug)
        loginf << "DBG use stored utn in acid_2_utn_: " << acid_2_utn_.at(*tr.acid_);

    int utn = acid_2_utn_.at(*tr.acid_);

    traced_assert(targets_.count(utn));
    traced_assert(targets_.at(utn).hasACID(*tr.acid_));

    return utn;
}

bool ReconstructorBase::TargetsContainer::canAssocByTrackNumber(
    dbContent::targetReport::ReconstructorInfo& tr, bool do_debug)
{
    if (!tr.track_number_) // only if track number is set
        return false;

    unsigned int dbcont_id = Number::recNumGetDBContId(tr.record_num_);

    if (dbcont_id != 62 && dbcont_id != 255) // only if trustworty track numbers in 62 and reftraj
        return false;

    if (do_debug)
        loginf << "DBG canAssocByTrackNumber can use stored utn in tn2utn_ "
               << tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_);

    if (!tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_))
        return false;

    int utn {-1};

    boost::posix_time::ptime timestamp_prev;

    std::tie(utn, timestamp_prev) = tn2utn_.at(tr.ds_id_).at(tr.line_id_).at(*tr.track_number_);

    if (do_debug || reconstructor_->task().debugSettings().debugUTN(utn))
        loginf << "DBG can assoc by tn " << *tr.track_number_ << " to utn " << utn;

    // tr has acad, target has an acad but not the target reports
    // happens if same acid is used by 2 different transponders
    if (tr.acad_ && targets_.at(utn).hasACAD()
        && !targets_.at(utn).hasACAD(!tr.acad_))
    {
        logwrn << " same track num reused by different ACAD transponders, tr " << *tr.track_number_ << ", utn "
               << targets_.at(utn).asStr() << " tr " << tr.asStr() << ", unassociating";

        eraseTrackNumberLookup(tr);

        if (tr.acid_ && acid_2_utn_.count(*tr.acid_))
        {
            if (do_debug || reconstructor_->task().debugSettings().debugUTN(utn))
                loginf << "DBG removing from acid lookup " << *tr.acid_ << " to utn " << acid_2_utn_.at(*tr.acid_);

            acid_2_utn_.erase(*tr.acid_);
            unspecific_acids_.insert(*tr.acid_);
        }

        return false;
    }

    return true;
}

int ReconstructorBase::TargetsContainer::assocByTrackNumber(
    dbContent::targetReport::ReconstructorInfo& tr,
    const boost::posix_time::time_duration& track_max_time_diff, bool do_debug)
{
    int utn {-1};

    boost::posix_time::ptime timestamp_prev;

    traced_assert(tr.track_number_);

    traced_assert(tn2utn_.at(tr.ds_id_).at(tr.line_id_).count(*tr.track_number_));
    std::tie(utn, timestamp_prev) = tn2utn_.at(tr.ds_id_).at(tr.line_id_).at(*tr.track_number_);
    traced_assert(targets_.count(utn));

    // check for larger time offset
    if (tr.timestamp_ - timestamp_prev > track_max_time_diff) // too old
    {
        if (do_debug)
            loginf << "DBG stored utn " << utn << " in tn2utn_ too large time offset (tdiff "
                   << Time::toString(tr.timestamp_ - timestamp_prev) << " > "
                   << Time::toString(track_max_time_diff)
                      << "), remove & create new target";

        // remove previous track number assoc
        traced_assert(tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_));
        tn2utn_[tr.ds_id_][tr.line_id_].erase(*tr.track_number_);

        if (reconstructor_->task().debugSettings().debugUTN(utn))
            loginf << "DBG disassociated utn " << utn << " from max tdiff track using tr " << tr.asStr();

        return -1; // disassoc case
    }

     if (do_debug || reconstructor_->task().debugSettings().debugUTN(utn))
        loginf << "DBG assoc by tn " << *tr.track_number_ << " to utn " << utn;

    return utn;
}

void ReconstructorBase::TargetsContainer::eraseTrackNumberLookup(dbContent::targetReport::ReconstructorInfo& tr)
{
    traced_assert(tr.track_number_);
    traced_assert(tn2utn_[tr.ds_id_][tr.line_id_].count(*tr.track_number_));
    tn2utn_[tr.ds_id_][tr.line_id_].erase(*tr.track_number_);
}

void ReconstructorBase::TargetsContainer::clear()
{
    utn_vec_.clear();

    acad_2_utn_.clear(); // acad dec -> utn
    acid_2_utn_.clear(); // acid trim -> utn

    // ds_id -> line id -> track num -> utn, last tod
    tn2utn_.clear();

    targets_.clear(); // utn -> tgt

    //removed_utns_.clear();
}

ReconstructorBase::ReconstructorBase(const std::string& class_id, 
                                     const std::string& instance_id,
                                     ReconstructorTask& task, 
                                     std::unique_ptr<AccuracyEstimatorBase>&& acc_estimator)
    :   Configurable (class_id, instance_id, &task)
    ,   targets_container_(this)
    ,   acc_estimator_(std::move(acc_estimator))
    ,   task_(task)
    ,   chain_predictors_(new reconstruction::KalmanChainPredictors)
{
    accessor_ = make_shared<dbContent::DBContentAccessor>();

    // reference computation
    {
        registerParameter("ref_Q_std", &ref_calc_settings_.Q_std.Q_std_static,
                          ReferenceCalculatorSettings().Q_std.Q_std_static);
        registerParameter("ref_Q_std_ground", &ref_calc_settings_.Q_std.Q_std_ground,
                          ReferenceCalculatorSettings().Q_std.Q_std_ground);
        registerParameter("ref_Q_std_air", &ref_calc_settings_.Q_std.Q_std_air,
                          ReferenceCalculatorSettings().Q_std.Q_std_air);
        registerParameter("ref_Q_std_unknown", &ref_calc_settings_.Q_std.Q_std_unknown,
                          ReferenceCalculatorSettings().Q_std.Q_std_unknown);

        registerParameter("dynamic_process_noise", &ref_calc_settings_.dynamic_process_noise,
                          ReferenceCalculatorSettings().dynamic_process_noise);

        //registerParameter("ref_min_chain_size", &ref_calc_settings_.min_chain_size   , ReferenceCalculatorSettings().min_chain_size);
        registerParameter("ref_min_dt", &ref_calc_settings_.min_dt, ReferenceCalculatorSettings().min_dt);
        registerParameter("ref_max_dt", &ref_calc_settings_.max_dt, ReferenceCalculatorSettings().max_dt);
        registerParameter("ref_max_distance", &ref_calc_settings_.max_distance,
                          ReferenceCalculatorSettings().max_distance);

        registerParameter("ref_smooth_rts", &ref_calc_settings_.smooth_rts, ReferenceCalculatorSettings().smooth_rts);

        registerParameter("ref_resample_result", &ref_calc_settings_.resample_result,
                          ReferenceCalculatorSettings().resample_result);
        registerParameter("ref_resample_Q_std", &ref_calc_settings_.resample_Q_std.Q_std_static,
                          ReferenceCalculatorSettings().resample_Q_std.Q_std_static);
        registerParameter("ref_resample_Q_std_ground", &ref_calc_settings_.resample_Q_std.Q_std_ground,
                          ReferenceCalculatorSettings().resample_Q_std.Q_std_ground);
        registerParameter("ref_resample_Q_std_air", &ref_calc_settings_.resample_Q_std.Q_std_air,
                          ReferenceCalculatorSettings().resample_Q_std.Q_std_air);
        registerParameter("ref_resample_Q_std_unknown", &ref_calc_settings_.resample_Q_std.Q_std_unknown,
                          ReferenceCalculatorSettings().resample_Q_std.Q_std_unknown);
        registerParameter("ref_resample_dt", &ref_calc_settings_.resample_dt,
                          ReferenceCalculatorSettings().resample_dt);

        registerParameter("ref_max_proj_distance_cart", &ref_calc_settings_.max_proj_distance_cart,
                          ReferenceCalculatorSettings().max_proj_distance_cart);

        registerParameter("ref_resample_systracks", &ref_calc_settings_.resample_systracks,
                          ReferenceCalculatorSettings().resample_systracks);
        registerParameter("ref_resample_systracks_dt", &ref_calc_settings_.resample_systracks_dt,
                          ReferenceCalculatorSettings().resample_systracks_dt);
        registerParameter("ref_resample_systracks_max_dt", &ref_calc_settings_.resample_systracks_max_dt,
                          ReferenceCalculatorSettings().resample_systracks_max_dt);

        registerParameter("filter_references_max_stddev"  , &ref_calc_settings_.filter_references_max_stddev_,
                          ReferenceCalculatorSettings().filter_references_max_stddev_);
        registerParameter("filter_references_max_stddev_m", &ref_calc_settings_.filter_references_max_stddev_m_,
                          ReferenceCalculatorSettings().filter_references_max_stddev_m_);
    }

    traced_assert(acc_estimator_);
}

void ReconstructorBase::registerBaseSettings(ReconstructorBaseSettings& settings)
{
    // base settings
    {
        registerParameter("ds_line", &settings.ds_line, settings.ds_line);

        registerParameter("slice_duration_in_minutes", &settings.slice_duration_in_minutes,
                          settings.slice_duration_in_minutes);
        registerParameter("outdated_duration_in_minutes", &settings.outdated_duration_in_minutes,
                          settings.outdated_duration_in_minutes);

        registerParameter("delete_all_calc_reftraj", &settings.delete_all_calc_reftraj,
                          settings.delete_all_calc_reftraj);
    }

    // association stuff
    registerParameter("max_time_diff", &settings.max_time_diff_, settings.max_time_diff_);
    registerParameter("max_altitude_diff", &settings.max_altitude_diff_, settings.max_altitude_diff_);
    registerParameter("track_max_time_diff", &settings.track_max_time_diff_, settings.track_max_time_diff_);
    registerParameter("do_track_number_disassociate_using_distance",
                      &settings.do_track_number_disassociate_using_distance_,
                      settings.do_track_number_disassociate_using_distance_);
    registerParameter("tn_disassoc_distance_factor", &settings.tn_disassoc_distance_factor_,
                      settings.tn_disassoc_distance_factor_);

    registerParameter("target_prob_min_time_overlap", &settings.target_prob_min_time_overlap_,
                      settings.target_prob_min_time_overlap_);
    registerParameter("target_min_updates", &settings.target_min_updates_, settings.target_min_updates_);
    registerParameter("target_max_positions_dubious_verified_rate",
                      &settings.target_max_positions_dubious_verified_rate_,
                      settings.target_max_positions_dubious_verified_rate_);
    registerParameter("target_max_positions_dubious_unknown_rate",
                      &settings.target_max_positions_dubious_unknown_rate_,
                      settings.target_max_positions_dubious_unknown_rate_);

    registerParameter("target_max_positions_not_ok_verified_rate",
                      &settings.target_max_positions_not_ok_verified_rate_,
                      settings.target_max_positions_not_ok_verified_rate_);
    registerParameter("target_max_positions_not_ok_unknown_rate",
                      &settings.target_max_positions_not_ok_unknown_rate_,
                      settings.target_max_positions_not_ok_unknown_rate_);

    registerParameter("targets_min_assoc_score",
                      &settings.targets_min_assoc_score_,
                      settings.targets_min_assoc_score_);                      

    registerParameter("min_aircraft_modec", &settings.min_aircraft_modec_, settings.min_aircraft_modec_);

    registerParameter("vehicle_acids", &settings.vehicle_acids_, {});
    settings.setVehicleACIDs(settings.vehicle_acids_);
    registerParameter("vehicle_acads", &settings.vehicle_acads_, {});
    settings.setVehicleACADs(settings.vehicle_acads_);    

    registerParameter("use_stopped_adsb_tracking_",
                      &settings.use_stopped_adsb_tracking_,
                      settings.use_stopped_adsb_tracking_);

    ref_calc_settings_.track_stopped_adsb = settings.use_stopped_adsb_tracking_;
}

/**
 */
ReconstructorBase::~ReconstructorBase()
{
    acc_estimator_ = nullptr;
}

void ReconstructorBase::init()
{
    traced_assert(!init_);

    //call before init_impl()
    resetTimeframe();
    applyTimeframeLimits();

    //not needed at the moment
    //initChainPredictors();

    //invoke derived behaviour
    init_impl();

    current_slice_begin_ = timestamp_min_;
    next_slice_begin_    = timestamp_min_; // first slice

    loginf << "start" 
           << " data time min " << Time::toString(timestamp_min_)
           << " data time max " << Time::toString(timestamp_max_);

    init_ = true;
}

void ReconstructorBase::resetTimeframeSettings()
{
    auto timeframe = timeFrame();

    settings().data_timestamp_min = timeframe.first;
    settings().data_timestamp_max = timeframe.second;

    emit configChanged();
}

bool ReconstructorBase::isVehicleACID(const std::string& acid)
{
    return settings().vehicle_acids_set_.count(acid);
}

bool ReconstructorBase::isVehicleACAD(unsigned int value)
{
    return settings().vehicle_acads_set_.count(value);
}

std::pair<boost::posix_time::ptime, boost::posix_time::ptime> ReconstructorBase::timeFrame() const
{
    //get full data time range (should)
    if (!COMPASS::instance().dbContentManager().hasMinMaxTimestamp())
        return std::pair<boost::posix_time::ptime, boost::posix_time::ptime>();

    boost::posix_time::ptime data_t0, data_t1;
    std::tie(data_t0, data_t1) = COMPASS::instance().dbContentManager().minMaxTimestamp();

    if (data_t0 >= data_t1)
        return std::pair<boost::posix_time::ptime, boost::posix_time::ptime>();

    return std::pair<boost::posix_time::ptime, boost::posix_time::ptime>(data_t0, data_t1);
}

void ReconstructorBase::resetTimeframe()
{
    auto timeframe = timeFrame();

    if (timeframe.first.is_not_a_date_time() || timeframe.second.is_not_a_date_time())
        logerr << "invalid data timeframe";
    
    traced_assert(!timeframe.first.is_not_a_date_time() && !timeframe.second.is_not_a_date_time());

    timestamp_min_ = timeframe.first;
    timestamp_max_ = timeframe.second;
}

void ReconstructorBase::applyTimeframeLimits()
{
    //get time range from settings
    boost::posix_time::ptime settings_t0 = settings().data_timestamp_min;
    boost::posix_time::ptime settings_t1 = settings().data_timestamp_max;

    //both set?
    if (settings_t0.is_not_a_date_time() || settings_t1.is_not_a_date_time())
        return;
    
    if (settings_t0 >= settings_t1)
    {
        logwrn << "chosen timeframe is invalid, returning...";
        return;
    }

    //limit reconstructor time range
    auto tmin = std::max(settings_t0, timestamp_min_);
    auto tmax = std::min(settings_t1, timestamp_max_);

    if (tmin >= tmax)
    {
        logwrn << "combined timeframe is invalid, returning...";
        return;
    }

    timestamp_min_ = tmin;
    timestamp_max_ = tmax;
}

void ReconstructorBase::initIfNeeded()
{
    if (!init_)
        init();

    traced_assert(init_);
}

bool ReconstructorBase::hasNextTimeSlice()
{
    initIfNeeded();

    traced_assert(!current_slice_begin_.is_not_a_date_time());
    traced_assert(!timestamp_max_.is_not_a_date_time());

    first_slice_ = current_slice_begin_ == timestamp_min_;

    loginf << "first_slice " << first_slice_;

    return next_slice_begin_ < timestamp_max_;
}

int ReconstructorBase::numSlices()
{
    if (timestamp_min_.is_not_a_date_time() || timestamp_max_.is_not_a_date_time())
        return -1;

    return (int)std::ceil(Utils::Time::partialSeconds(timestamp_max_ - timestamp_min_)
                           / Utils::Time::partialSeconds(settings().sliceDuration()));
}

std::unique_ptr<ReconstructorBase::DataSlice> ReconstructorBase::getNextTimeSlice()
{
    initIfNeeded();

    traced_assert(isInit());
    traced_assert(hasNextTimeSlice());

    current_slice_begin_ = next_slice_begin_;

    traced_assert(!current_slice_begin_.is_not_a_date_time());
    traced_assert(!timestamp_max_.is_not_a_date_time());

    traced_assert(current_slice_begin_ <= timestamp_max_);

    boost::posix_time::ptime current_slice_end = current_slice_begin_ + settings().sliceDuration();

    //TimeWindow window {current_slice_begin_, current_slice_end};

    logdbg << "current_slice_begin " << Time::toString(current_slice_begin_)
           << " current_slice_end " << Time::toString(current_slice_end);

    first_slice_ = current_slice_begin_ == timestamp_min_;

    remove_before_time_ = current_slice_begin_ - settings().outdatedDuration();
    write_before_time_ = current_slice_end - settings().outdatedDuration();

    next_slice_begin_ = current_slice_end; // for next iteration

    bool is_last_slice = !hasNextTimeSlice();

    if (is_last_slice)
        write_before_time_ = current_slice_end + boost::posix_time::seconds(1);

    //assert (current_slice_begin_ <= timestamp_max_); can be bigger

    std::unique_ptr<DataSlice> slice (new DataSlice());

    slice->slice_count_ = slice_cnt_;
    slice->slice_begin_ = current_slice_begin_;
    slice->next_slice_begin_ = next_slice_begin_;
    slice->timestamp_min_ = timestamp_min_;
    slice->timestamp_max_ = timestamp_max_;;
    slice->first_slice_ = first_slice_;
    slice->is_last_slice_ = is_last_slice;

    slice->remove_before_time_ = remove_before_time_;
    slice->write_before_time_ = write_before_time_;

    slice->loading_done_ = false;
    slice->processing_done_ = false;
    slice->write_done_ = false;

    ++slice_cnt_;

    loginf << "slice_cnt " << slice_cnt_
           << " slice_begin " << Time::toString(slice->slice_begin_)
           << " next_slice_begin " << Time::toString(slice->next_slice_begin_)
           << " remove_before_time " << Time::toString(slice->remove_before_time_)
           << " write_before_time "  << Time::toString(slice->write_before_time_)
           << " timestamp_min " << Time::toString(slice->timestamp_min_)
           << " timestamp_max " << Time::toString(slice->timestamp_max_)
           << " first_slice " << slice->first_slice_ << " is_last_slice " << slice->is_last_slice_;

    return slice;
}

void ReconstructorBase::reset()
{
    loginf;

    dbContent::ReconstructorTarget::globalStats().reset();

    accessor_->clear();
    accessors_.clear();

    slice_cnt_ = 0;
    current_slice_begin_ = {};
    next_slice_begin_ = {};
    timestamp_min_ = {};
    timestamp_max_ = {};
    first_slice_ = false;

    remove_before_time_ = {};
    write_before_time_ = {};

    target_reports_.clear();
    tr_timestamps_.clear();
    tr_ds_.clear();
    tr_batches_.clear();

    targets_container_.clear();

    chains_.clear();

    traced_assert(acc_estimator_);
    acc_estimator_->init(this);

    cancelled_ = false;
    init_      = false;
}

/**
 */
void ReconstructorBase::initChainPredictors()
{
    if (chain_predictors_->isInit())
        return;

//int num_threads = std::max(1, tbb::task_scheduler_init::default_num_threads());

#if TBB_VERSION_MAJOR <= 2020
    int num_threads = tbb::task_scheduler_init::default_num_threads(); // TODO PHIL
#else
    int num_threads = tbb::info::default_concurrency();
#endif

    traced_assert(num_threads > 0);

    traced_assert(chain_predictors_);

    chain_predictors_->init(referenceCalculatorSettings().kalman_type_assoc,
                            referenceCalculatorSettings().chainEstimatorSettings(),
                            num_threads);
}

/**
 */
void ReconstructorBase::processSlice()
{
    traced_assert(!currentSlice().remove_before_time_.is_not_a_date_time());

    loginf << "start " << Time::toString(currentSlice().timestamp_min_)
           << " first_slice " << currentSlice().first_slice_;

    processing_ = true;

    if (!currentSlice().first_slice_)
    {
        logdbg << "removing data before "
               << Time::toString(currentSlice().remove_before_time_);

        accessor_->removeContentBeforeTimestamp(currentSlice().remove_before_time_);
    }

    loginf << "adding, size " << currentSlice().data_.size();

    accessor_->add(currentSlice().data_);

    loginf << "processing slice";

    processSlice_impl();

    processing_ = false;

    currentSlice().processing_done_ = true;

    if (task().debugSettings().analyze_)
    {
        if (task().debugSettings().analyze_association_)
            doUnassociatedAnalysis();

        if (task().debugSettings().analyze_outlier_detection_)
            doOutlierAnalysis();
    }

    if (task().debugSettings().stats_ && currentSlice().is_last_slice_)
    {
        doReconstructionStatistics();
    }

    logdbg << "done";
}

void ReconstructorBase::clearOldTargetReports()
{
    loginf << "remove_before_time "
           << Time::toString(currentSlice().remove_before_time_)
           << " size " << target_reports_.size();

    tr_timestamps_.clear();
    tr_ds_.clear();
    tr_batches_.clear();

    for (auto tr_it = target_reports_.begin(); tr_it != target_reports_.end() /* not hoisted */; /* no increment */)
    {
        if (tr_it->second.timestamp_ < currentSlice().remove_before_time_)
        {
            //loginf << "removing " << Time::toString(ts_it->second.timestamp_);
            tr_it = target_reports_.erase(tr_it);
        }
        else
        {
            //loginf << "keeping " << Time::toString(ts_it->second.timestamp_);

            tr_it->second.in_current_slice_ = false;
            tr_it->second.buffer_index_ = std::numeric_limits<unsigned int>::max(); // set to impossible value

            // add to lookup structures
            tr_timestamps_.insert({tr_it->second.timestamp_, tr_it->second.record_num_});

            tr_ds_[Number::recNumGetDBContId(tr_it->second.record_num_)]
                  [tr_it->second.ds_id_][tr_it->second.line_id_].push_back(
                          tr_it->second.record_num_);

            ++tr_it;
        }
    }

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING

    for (auto& tr_it : target_reports_)
    {
        traced_assert(tr_it.second.timestamp_ >= currentSlice().remove_before_time_);
        traced_assert(!tr_it.second.in_current_slice_);
    }

    for (auto& ts_it : tr_timestamps_)
        traced_assert(ts_it.first >= currentSlice().remove_before_time_);

#endif

    loginf << "size after " << target_reports_.size();

    // clear old data from targets
    for (auto& tgt_it : targets_container_.targets_)
        tgt_it.second.removeOutdatedTargetReports();

    // clear old data from chains
    for (auto& chain_it : chains_)
    {
        if (chain_it.second)
        {
            chain_it.second->removeUpdatesBefore(currentSlice().remove_before_time_);

            if (!chain_it.second->checkMeasurementAvailability())
            {
                logerr << "not all measurements available for chain with UTN " << chain_it.first;
                traced_assert(false);
            }

            if (!chain_it.second->hasData())
                chain_it.second = nullptr;
        }
    }
}

void ReconstructorBase::createTargetReports()
{
    loginf << "current_slice_begin "
           << Time::toString(currentSlice().slice_begin_);

    boost::posix_time::ptime ts;
    unsigned long record_num;

    dbContent::targetReport::ReconstructorInfo info;

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    accessors_.clear();

    num_new_target_reports_in_slice_ = 0;

    //unsigned int calc_ref_ds_id = Number::dsIdFrom(ds_sac_, ds_sic_);

    std::set<unsigned int> unused_ds_ids = task_.unusedDSIDs();
    std::map<unsigned int, std::set<unsigned int>> unused_lines = task_.unusedDSIDLines();

    auto& ds_man = COMPASS::instance().dataSourceManager();

    std::set<unsigned int> ground_only_ds_ids = ds_man.groundOnlyDBDataSources();
    std::map<unsigned int, dbContent::DataSourceType> ds_types = ds_man.dsTypes();

    std::vector<std::shared_ptr<SectorLayer>> used_sector_layers;
    //bool inside_any;

    if (task_.useSectorsExtend())
    {
        auto& eval_man = COMPASS::instance().evaluationManager();

        for (auto& sect_it : task_.usedSectors())
        {
            traced_assert(eval_man.hasSectorLayer(sect_it.first));

            if (!sect_it.second)
                continue;

            for (const auto& s : eval_man.sectorLayer(sect_it.first)->sectors())
                s->createFastInsideTest();

            used_sector_layers.push_back(eval_man.sectorLayer(sect_it.first));
        }
    }

    for (auto& buf_it : *accessor_)
    {
        loginf << "processing " << buf_it.first;
        traced_assert(dbcont_man.existsDBContent(buf_it.first));

        if (settings().do_not_associate_systracks_ && buf_it.first == "CAT062")
            continue;

        if (!dbcont_man.dbContent(buf_it.first).containsTargetReports())
            continue;

        unsigned int dbcont_id = dbcont_man.dbContent(buf_it.first).id();

        accessors_.emplace(dbcont_id, accessor_->targetReportAccessor(buf_it.first));

        dbContent::TargetReportAccessor& tgt_acc = accessors_.at(dbcont_id);
        unsigned int buffer_size = tgt_acc.size();

        std::vector<bool> position_usable;
        position_usable.resize(buffer_size);

        tbb::parallel_for(uint(0), buffer_size, [&](unsigned int cnt)
            {
                if (!tgt_acc.position(cnt))
                {
                    position_usable[cnt] = false;
                    return;
                }

                if (task_.useSectorsExtend())
                {
                    bool inside_any = false;

                    for (const auto& sect_lay_it : used_sector_layers)
                    {
                        if (sect_lay_it->isInside(tgt_acc.position(cnt)->latitude_,
                                                  tgt_acc.position(cnt)->longitude_,
                                                  task_.sectorDeltaDeg()))
                        {
                            inside_any = true;
                            break;
                        }
                    }

                    position_usable[cnt] = inside_any;
                }
                else
                    position_usable[cnt] = true;
            });

        for (unsigned int cnt=0; cnt < buffer_size; cnt++)
        {
            record_num = tgt_acc.recordNumber(cnt);

            ts = tgt_acc.timestamp(cnt);

            //loginf << "ts " << Time::toString(ts);

            if (!position_usable.at(cnt))
                continue;

            if (target_reports_.count(record_num)) // already exist, update buffer_index_
            {
#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
                //assert (target_reports_.count(record_num));
                traced_assert(!target_reports_.at(record_num).in_current_slice_);

                if (ts < currentSlice().remove_before_time_)
                {
                    logerr << "old data not removed ts "
                           << Time::toString(ts)
                           << " dbcont " << buf_it.first
                           << " buffer_size " << buffer_size
                           << " remove before " << Time::toString(currentSlice().remove_before_time_);
                }

                traced_assert(ts >= currentSlice().remove_before_time_);

                traced_assert(target_reports_.at(record_num).record_num_ == record_num); // be sure
                traced_assert(target_reports_.at(record_num).timestamp_ == ts); // be very sure
#endif

                target_reports_.at(record_num).buffer_index_ = cnt;
            }
            else // not yet, insert
            {
                // base info
                info.buffer_index_ = cnt;
                info.record_num_ = record_num;
                info.dbcont_id_ = dbcont_id;
                info.ds_id_ = tgt_acc.dsID(cnt);

                traced_assert(ds_types.count(info.ds_id_));
                info.ds_type_ = ds_types.at(info.ds_id_);

                info.line_id_ = tgt_acc.lineID(cnt);
                info.timestamp_ = ts;

                // reconstructor info
                info.in_current_slice_ = true;

                info.is_calculated_reference_ =
                    ds_man.hasDBDataSource(info.ds_id_)
                    && ds_man.dbDataSource(info.ds_id_).sac() == ReconstructorBaseSettings::REC_DS_SAC
                    && ds_man.dbDataSource(info.ds_id_).sic() == ReconstructorBaseSettings::REC_DS_SIC;

                info.acad_ = tgt_acc.acad(cnt);
                info.acid_ = tgt_acc.acid(cnt);

                info.mode_a_code_ = tgt_acc.modeACode(cnt);

                info.track_number_ = tgt_acc.trackNumber(cnt);
                info.track_begin_ = tgt_acc.trackBegin(cnt);
                info.track_end_ = tgt_acc.trackEnd(cnt);

                info.position_ = tgt_acc.position(cnt);
                info.position_accuracy_ = tgt_acc.positionAccuracy(cnt);

                info.unsused_ds_pos_ =
                    !info.position().has_value()
                        || (unused_ds_ids.count(info.ds_id_)
                        || (unused_lines.count(info.ds_id_) && unused_lines.at(info.ds_id_).count(info.line_id_)));

                info.barometric_altitude_ = tgt_acc.barometricAltitude(cnt);

                info.velocity_ = tgt_acc.velocity(cnt);
                info.velocity_accuracy_ = tgt_acc.velocityAccuracy(cnt);

                info.track_angle_ = tgt_acc.trackAngle(cnt);
                info.ground_bit_ = tgt_acc.groundBit(cnt);
                info.data_source_is_ground_only_ = ground_only_ds_ids.count(info.ds_id_);

                info.mops_ = tgt_acc.mopsVersion(cnt);
                info.ecat_ = tgt_acc.ecat(cnt);

                // insert info
                target_reports_[record_num] = info;

                // insert into lookups
                tr_timestamps_.insert({ts, record_num});
                // dbcontent id -> ds_id -> ts ->  record_num

                tr_ds_[dbcont_id][info.ds_id_][info.line_id_].push_back(record_num);

                ++num_new_target_reports_in_slice_;
            }
        }
    }

#if DO_RECONSTRUCTOR_PEDANTIC_CHECKING
    for (auto& tr_it : target_reports_)
    {
        if (tr_it.second.buffer_index_ >= accessor(tr_it.second).size())
            logerr << "tr " << tr_it.second.asStr()
                   << " buffer index " << tr_it.second.buffer_index_
                   << " accessor size " << accessor(tr_it.second).size() << " is maxint "
                   << (tr_it.second.buffer_index_ == std::numeric_limits<unsigned int>::max());

        traced_assert(tr_it.second.buffer_index_ < accessor(tr_it.second).size()); // fails
    }
#endif

    loginf << "done with " << num_new_target_reports_in_slice_
           << " new target reports";
}

void ReconstructorBase::createTargetReportBatches()
{
    loginf << "begin";

    dbContent::DBContentStatusInfo status_info;

    status_info.process(accessor_->buffers());

    //std::map<unsigned int, std::map<unsigned int, std::map<unsigned int, std::vector<unsigned long>>>> tr_ds_;
    // dbcontent id -> ds_id -> line id -> record_num, sorted by ts

    std::vector<std::string> data_source_type_order {"RefTraj", "ADSB", "Tracker", "MLAT", "Radar", "Other"};

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    size_t num_truncated_noinfo = 0;
    size_t num_truncated_oor    = 0;
    size_t num_status_info      = 0;

    for (auto& ds_type : data_source_type_order)
    {
        logdbg << "ds_type " << ds_type;

        for (auto& dbcont_it : tr_ds_)
        {
            logdbg << "dbcont " << dbcont_it.first;

            for (auto& ds_it : dbcont_it.second)
            {
                if (ds_man.dbDataSource(ds_it.first).dsType() != ds_type)
                    continue;

                logdbg << "ds_type " << ds_type << " dbcont " << dbcont_it.first << " ds " << ds_man.dbDataSource(ds_it.first).name();

                for (auto& line_it : ds_it.second)
                {
                    logdbg << "line " << line_it.first;

                    std::vector<boost::posix_time::ptime> ds_ui_times;

                    if (status_info.hasInfo(ds_it.first, line_it.first))
                        ds_ui_times = status_info.getInfo(ds_it.first, line_it.first);

                    std::map<boost::posix_time::ptime, std::vector<unsigned long>> batches;

                    for (auto& rn_it : line_it.second)
                    {
                        traced_assert(target_reports_.count(rn_it));

                        auto timestamp = target_reports_.at(rn_it).timestamp_;

                        boost::posix_time::ptime batch_time;
                        
                        if (ds_ui_times.size() >= 2)
                        {
                            boost::posix_time::ptime t_interval;
#if 0
                            // Find the appropriate batch time from ds_ui_times
                            //PWa: lower_bound = first element in range which is not ordered before value => it >= value!
                            auto it = std::lower_bound(ds_ui_times.begin(), ds_ui_times.end(), timestamp); 
                            if (it != ds_ui_times.end() && 
                                (it == ds_ui_times.begin() || timestamp >= *it))
                                t_interval = *it;
#else
                            //assume intervals as [t0,t1)
                            //first elem in ds_ui_times > timestamp
                            auto it = std::upper_bound(ds_ui_times.begin(), ds_ui_times.end(), timestamp); 

                            if (it == ds_ui_times.end())
                            {
                                // last value <= timestamp => check if timestamp is exactly the last value
                                if (ds_ui_times.back() == timestamp)
                                {
                                    // it is => add to last interval
                                    // alternatively: skip as out of range
                                    t_interval = ds_ui_times[ ds_ui_times.size() - 2 ];
                                }
                            }
                            else if (it == ds_ui_times.begin())
                            {
                                // first value > timestamp => out of range
                            }
                            else
                            {
                                // in interval => use interval begin
                                size_t idx = std::distance(ds_ui_times.begin(), it) - 1;
                                t_interval = ds_ui_times[ idx ];
                            }
#endif

                            if (!t_interval.is_not_a_date_time())
                            {
                                ++num_status_info;

                                // Use the batch time from ds_ui_times
                                batch_time = t_interval;
                            }
                            else
                            {
                                ++num_truncated_oor;

                                // Outside of given times, use truncated UTC 1 second timestamps
                                batch_time = Time::truncateToFullSeconds(timestamp);
                            }
                        }
                        else
                        {
                            ++num_truncated_noinfo;
                            
                            // No ds_ui_times given, use truncated UTC 1 second timestamps
                            batch_time = Time::truncateToFullSeconds(timestamp);
                        }
                        
                        batches[batch_time].push_back(rn_it);
                    }

                    // Insert batches into final_batches_ using TargetReportBatch struct
                    for (auto& batch_it : batches)
                    {
                        tr_batches_.emplace(
                            batch_it.first,
                            TargetReportBatch(ds_it.first, line_it.first, batch_it.first, std::move(batch_it.second))
                        );
                    }
                }                
            }
        }
    }

    //loginf << "BATCHES truncated noinfo " << num_truncated_noinfo << " truncated oor " << num_truncated_oor << " status info used " << num_status_info;
    //assert(false);

    logdbg << "done";
}

void ReconstructorBase::removeTargetReportsLaterOrEqualThan(const boost::posix_time::ptime& ts)
{
    // remove target reports from targets & clean
    for (auto& tgt_it : targets_container_.targets_)
        tgt_it.second.removeTargetReportsLaterOrEqualThan(ts);
}

std::map<unsigned int, std::map<unsigned long, unsigned int>> ReconstructorBase::createAssociations()
{
    loginf;

    std::map<unsigned int, std::map<unsigned long, unsigned int>> associations;
    unsigned int num_assoc {0};

    for (auto& tgt_it : targets_container_.targets_)
    {
        for (auto rn_it : tgt_it.second.target_reports_)
        {
            traced_assert(target_reports_.count(rn_it));

            dbContent::targetReport::ReconstructorInfo& tr = target_reports_.at(rn_it);

            if (tr.timestamp_ < currentSlice().write_before_time_) // tr.in_current_slice_
            {
                associations[Number::recNumGetDBContId(rn_it)][rn_it] = tgt_it.first;
                ++num_assoc;
            }
        }
        tgt_it.second.associations_written_ = true;

        tgt_it.second.updateCounts();
    }

    loginf << "done with " << num_assoc << " associated";

    return associations;
}

std::map<std::string, std::shared_ptr<Buffer>> ReconstructorBase::createAssociationBuffers(
    std::map<unsigned int, std::map<unsigned long,unsigned int>> associations)
{
    logdbg;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    // write association info to buffers

    std::map<std::string, std::shared_ptr<Buffer>> assoc_data;

    for (auto& cont_assoc_it : associations) // dbcontent -> rec_nums
    {
        unsigned int num_associated {0};
        unsigned int num_not_associated {0};

        unsigned int dbcontent_id = cont_assoc_it.first;
        string dbcontent_name = dbcontent_man.dbContentWithId(cont_assoc_it.first);
        //DBContent& dbcontent = dbcontent_man.dbContent(dbcontent_name);

        std::map<unsigned long, unsigned int>& tr_associations = cont_assoc_it.second;

        logdbg << "db content " << dbcontent_name;

        string rec_num_name =
            dbcontent_man.metaVariable(DBContent::meta_var_rec_num_.name()).getFor(dbcontent_name).name();

        string utn_name =
            dbcontent_man.metaVariable(DBContent::meta_var_utn_.name()).getFor(dbcontent_name).name();

        PropertyList properties;
        properties.addProperty(utn_name,  DBContent::meta_var_utn_.dataType());
        properties.addProperty(rec_num_name,  DBContent::meta_var_rec_num_.dataType());

        assoc_data [dbcontent_name].reset(new Buffer(properties));

        shared_ptr<Buffer> buffer  = assoc_data.at(dbcontent_name);

        NullableVector<unsigned int>& utn_col_vec = buffer->get<unsigned int>(utn_name);
        NullableVector<unsigned long>& rec_num_col_vec = buffer->get<unsigned long>(rec_num_name);

        traced_assert(tr_ds_.count(dbcontent_id));

        unsigned int buf_cnt = 0;
        for (auto& ds_it : tr_ds_.at(dbcontent_id))  // iterate over all rec nums
        {
            for (auto& line_it : ds_it.second)
            {
                for (auto& rn_it : line_it.second)
                {
                    traced_assert(target_reports_.count(rn_it));

                    if (target_reports_.at(rn_it).timestamp_ >= currentSlice().write_before_time_)
                        continue;

                    rec_num_col_vec.set(buf_cnt, rn_it);

                    if (tr_associations.count(rn_it))
                    {
                        utn_col_vec.set(buf_cnt, tr_associations.at(rn_it));
                        ++num_associated;
                    }
                    else
                        ++num_not_associated;
                    // else null

                    ++buf_cnt;
                }
            }
        }

        logdbg << "dcontent " << dbcontent_name
               <<  " assoc " << num_associated << " not assoc " << num_not_associated
               << " buffer size " << buffer->size();

        logdbg << "dcontent " << dbcontent_name << " done";
    }

    logdbg << "done";

    return assoc_data;
}

std::map<std::string, std::shared_ptr<Buffer>> ReconstructorBase::createReferenceBuffers()
{
    logdbg << "num " << targets_container_.targets_.size();

    std::shared_ptr<Buffer> buffer;

    for (auto& tgt_it : targets_container_.targets_)
    {
        if (!buffer)
            buffer = tgt_it.second.getReferenceBuffer(); // also updates count
        else
        {
            auto tmp = tgt_it.second.getReferenceBuffer();
            buffer->seizeBuffer(*tmp);
        }
    }

    if (buffer && buffer->size())
    {
        NullableVector<boost::posix_time::ptime>& ts_vec = buffer->get<boost::posix_time::ptime>(
            DBContent::meta_var_timestamp_.name());

        logdbg << "buffer size " << buffer->size()
               << " ts min " << Time::toString(ts_vec.get(0))
               << " max " << Time::toString(ts_vec.get(ts_vec.contentSize()-1));

        DataSourceManager& src_man = COMPASS::instance().dataSourceManager();

        unsigned int ds_id = Number::dsIdFrom(settings().ds_sac, settings().ds_sic);

        if (!src_man.hasConfigDataSource(ds_id))
        {
            logdbg << "creating data source";

            src_man.createConfigDataSource(ds_id);
            traced_assert(src_man.hasConfigDataSource(ds_id));
        }

        dbContent::ConfigurationDataSource& src = src_man.configDataSource(ds_id);

        src.name(settings().ds_name);
        src.dsType("RefTraj"); // same as dstype

        return std::map<std::string, std::shared_ptr<Buffer>> {{buffer->dbContentName(), buffer}};
    }
    else
    {
        logdbg << "empty buffer";

        return std::map<std::string, std::shared_ptr<Buffer>> {};
    }
}

void ReconstructorBase::doReconstructionStatistics()
{
    auto& stats = dbContent::ReconstructorTarget::globalStats();

    const int Decimals = 3;

    auto perc = [ & ] (size_t num, size_t num_total)
    {
        if (num_total == 0)
            return std::string("0%");

        return QString::number((double)num / (double)num_total * 100.0, 'f', Decimals).toStdString() + "%";
    };

    auto getTable = [ & ] ()
    {
        auto& section = COMPASS::instance().taskManager().currentReport()->getSection("Reconstruction Statistics");
        if (!section.hasTable("Reconstruction Statistics"))
            section.addTable("Reconstruction Statistics", 4, {"", "", "Value", "Value [%]"}, false);

        return &section.getTable("Reconstruction Statistics");
    };

    if (task().debugSettings().stats_kalman_chains_)
    {
        auto table = getTable();

        std::string num_chain_skipped_preempt_p         = perc(stats.num_chain_skipped_preempt        , stats.num_chain_checked       );
        std::string num_chain_replaced_p                = perc(stats.num_chain_replaced               , stats.num_chain_checked       );
        std::string num_chain_added_p                   = perc(stats.num_chain_added                  , stats.num_chain_checked       );
        std::string num_chain_updates_valid_p           = perc(stats.num_chain_updates_valid          , stats.num_chain_updates       );
        std::string num_chain_updates_failed_p          = perc(stats.num_chain_updates_failed         , stats.num_chain_updates       );
        std::string num_chain_updates_failed_numeric_p  = perc(stats.num_chain_updates_failed_numeric , stats.num_chain_updates_failed);
        std::string num_chain_updates_failed_badstate_p = perc(stats.num_chain_updates_failed_badstate, stats.num_chain_updates_failed);
        std::string num_chain_updates_failed_other_p    = perc(stats.num_chain_updates_failed_other   , stats.num_chain_updates_failed);
        std::string num_chain_updates_skipped_p         = perc(stats.num_chain_updates_skipped        , stats.num_chain_updates       );
        std::string num_chain_updates_proj_changed_p    = perc(stats.num_chain_updates_proj_changed   , stats.num_chain_updates       );

        std::string num_chain_predictions_failed_p          = perc(stats.num_chain_predictions_failed         , stats.num_chain_predictions       );
        std::string num_chain_predictions_failed_numeric_p  = perc(stats.num_chain_predictions_failed_numeric , stats.num_chain_predictions_failed);
        std::string num_chain_predictions_failed_badstate_p = perc(stats.num_chain_predictions_failed_badstate, stats.num_chain_predictions_failed);
        std::string num_chain_predictions_failed_other_p    = perc(stats.num_chain_predictions_failed_other   , stats.num_chain_predictions_failed);
        std::string num_chain_predictions_fixed_p           = perc(stats.num_chain_predictions_fixed          , stats.num_chain_predictions       );
        std::string num_chain_predictions_proj_changed_p    = perc(stats.num_chain_predictions_proj_changed   , stats.num_chain_predictions       );

        table->addRow({"Chain Updates", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"mm checked", "", stats.num_chain_checked, ""});
        table->addRow({"skipped pre", "", stats.num_chain_skipped_preempt, num_chain_skipped_preempt_p});
        table->addRow({"replaced", "", stats.num_chain_replaced, num_chain_replaced_p});
        table->addRow({"added", "", stats.num_chain_added, num_chain_added_p});
        table->addRow({"mm fresh", "", stats.num_chain_fresh, ""});
        table->addRow({"valid", "", stats.num_chain_updates_valid, num_chain_updates_valid_p});
        table->addRow({"failed", "", stats.num_chain_updates_failed, num_chain_updates_failed_p});
        table->addRow({"", "numeric", stats.num_chain_updates_failed_numeric, num_chain_updates_failed_numeric_p});
        table->addRow({"", "bad state", stats.num_chain_updates_failed_badstate, num_chain_updates_failed_badstate_p});
        table->addRow({"", "other", stats.num_chain_updates_failed_other, num_chain_updates_failed_other_p});
        table->addRow({"skipped", "", stats.num_chain_updates_skipped, num_chain_updates_skipped_p});
        table->addRow({"total", "", stats.num_chain_updates, ""});
        table->addRow({"proj changed", "", stats.num_chain_updates_proj_changed, num_chain_updates_proj_changed_p});
        table->addRow({"", "", "", ""});

        table->addRow({"Chain Predictions", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"failed", "", stats.num_chain_predictions_failed , num_chain_predictions_failed_p});
        table->addRow({"", "numeric", stats.num_chain_predictions_failed_numeric, num_chain_predictions_failed_numeric_p});
        table->addRow({"", "bad state", stats.num_chain_predictions_failed_badstate, num_chain_predictions_failed_badstate_p});
        table->addRow({"", "other", stats.num_chain_predictions_failed_other, num_chain_predictions_failed_other_p});
        table->addRow({"fixed", "", stats.num_chain_predictions_fixed, num_chain_predictions_fixed_p});
        table->addRow({"total", "", stats.num_chain_predictions, ""});
        table->addRow({"proj changed", "", stats.num_chain_predictions_proj_changed, num_chain_predictions_proj_changed_p});
        table->addRow({"", "", "", ""});
    }

    if (task().debugSettings().stats_reference_calculation_)
    {
        auto table = getTable();

        std::string num_rec_updates_ccoeff_corr_p     = perc(stats.num_rec_updates_ccoeff_corr    , stats.num_rec_updates       );
        std::string num_rec_updates_valid_p           = perc(stats.num_rec_updates_valid          , stats.num_rec_updates       );
        std::string num_rec_updates_failed_p          = perc(stats.num_rec_updates_failed         , stats.num_rec_updates       );
        std::string num_rec_updates_failed_numeric_p  = perc(stats.num_rec_updates_failed_numeric , stats.num_rec_updates_failed);
        std::string num_rec_updates_failed_badstate_p = perc(stats.num_rec_updates_failed_badstate, stats.num_rec_updates_failed);
        std::string num_rec_updates_failed_other_p    = perc(stats.num_rec_updates_failed_other   , stats.num_rec_updates_failed);
        std::string num_rec_updates_raf_p             = perc(stats.num_rec_updates_raf            , stats.num_rec_updates       );
        std::string num_rec_updates_raf_numeric_p     = perc(stats.num_rec_updates_raf_numeric    , stats.num_rec_updates_raf   );
        std::string num_rec_updates_raf_badstate_p    = perc(stats.num_rec_updates_raf_badstate   , stats.num_rec_updates_raf   );
        std::string num_rec_updates_raf_other_p       = perc(stats.num_rec_updates_raf_other      , stats.num_rec_updates_raf   );
        std::string num_rec_updates_skipped_p         = perc(stats.num_rec_updates_skipped        , stats.num_rec_updates       );
        std::string num_rec_smooth_steps_failed_p     = perc(stats.num_rec_smooth_steps_failed    , stats.num_rec_updates       );

        table->addRow({"Rec Updates", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"ccoeff corr", "", stats.num_rec_updates_ccoeff_corr, num_rec_updates_ccoeff_corr_p});
        table->addRow({"valid", "", stats.num_rec_updates_valid, num_rec_updates_valid_p});
        table->addRow({"failed", "", stats.num_rec_updates_failed, num_rec_updates_failed_p});
        table->addRow({"", "numeric", stats.num_rec_updates_failed_numeric, num_rec_updates_failed_numeric_p});
        table->addRow({"", "bad state", stats.num_rec_updates_failed_badstate, num_rec_updates_failed_badstate_p});
        table->addRow({"", "other", stats.num_rec_updates_failed_other, num_rec_updates_failed_other_p});
        table->addRow({"reinit after fail", "", stats.num_rec_updates_raf, num_rec_updates_raf_p});
        table->addRow({"", "numeric", stats.num_rec_updates_raf_numeric, num_rec_updates_raf_numeric_p});
        table->addRow({"", "bad state", stats.num_rec_updates_raf_badstate, num_rec_updates_raf_badstate_p});
        table->addRow({"", "other", stats.num_rec_updates_raf_other, num_rec_updates_raf_other_p});
        table->addRow({"skipped", "", stats.num_rec_updates_skipped, num_rec_updates_skipped_p});
        table->addRow({"total", "", stats.num_rec_updates, ""});
        table->addRow({"", "", "", ""});

        table->addRow({"Rec Smooth Steps", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"failed steps", "", stats.num_rec_smooth_steps_failed, num_rec_smooth_steps_failed_p});
        table->addRow({"failed targets", "", stats.num_rec_smooth_target_failed, ""});
        table->addRow({"", "", "", ""});

        table->addRow({"Rec Interp Steps", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"", "failed", stats.num_rec_interp_failed, ""});
        table->addRow({"", "", "", ""});
    }

    if (task().debugSettings().stats_jpda_)
    {
        auto table = getTable();

        table->addRow({"JPDA", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"jpda runs", "", stats.num_jpda_runs, ""});
        table->addRow({"jpda success", "", stats.num_jpda_success, ""});
        table->addRow({"jpda failed", "", stats.num_jpda_failed, ""});
        table->addRow({"jpda runs greedy", "", stats.num_jpda_runs_greedy, ""});
        table->addRow({"jpda hypotheses max", "", stats.num_jpda_hyp_max, ""});
        table->addRow({"jpda measurements max", "", stats.num_jpda_mms_max, ""});
        table->addRow({"jpda assignments total", "", stats.num_jpda_assignments, ""});
        table->addRow({"jpda clutter total", "", stats.num_jpda_clutters, ""});
        table->addRow({"jpda tentatives total", "", stats.num_jpda_tentatives, ""});
        table->addRow({"jpda assignment ratio", "", perc(stats.num_jpda_assignments, stats.num_po_unassoc), ""});
        table->addRow({"jpda assignment ratio per-batch", "", perc(stats.jpda_assignment_ratio_sum, stats.num_jpda_success), ""});
        table->addRow({"", "", "", ""});
    }

    if (task().debugSettings().stats_tentative_targets_)
    {
        auto table = getTable();

        table->addRow({"Tentative Targets", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"tentative created", "", stats.num_tentative_created, ""});
        table->addRow({"tentative confirmed", "", stats.num_tentative_confirmed, ""});
        table->addRow({"tentative terminated", "", stats.num_tentative_terminated, ""});
        table->addRow({"tentative terminated coasting", "", stats.num_tentative_terminated_coasting, ""});
        table->addRow({"tentative terminated low prob", "", stats.num_tentative_terminated_low_prob, ""});
        table->addRow({"tentative remaining", "", stats.num_tentative_remaining, ""});
        table->addRow({"tentative tr reassoc", "", stats.num_tentative_tr_reassoc, ""});
        table->addRow({"tentative tr lost", "", stats.num_tentative_tr_lost, ""});
        table->addRow({"", "", "", ""});
    }

    if (task().debugSettings().stats_association_)
    {
        auto table = getTable();

        table->addRow({"Primary Onlys", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        table->addRow({"secondarys unassoc", "", stats.num_sec_unassoc, ""});
        table->addRow({"secondarys reassociated", "", stats.num_sec_reassociated, ""});
        table->addRow({"secondary reassoc ratio", "", perc(stats.num_sec_reassociated, stats.num_sec_unassoc), ""});
        table->addRow({"primary onlys unassoc", "", stats.num_po_unassoc, ""});
        table->addRow({"primary onlys reassociated", "", stats.num_po_reassociated, ""});
        table->addRow({"primary only reassoc ratio", "", perc(stats.num_po_reassociated, stats.num_po_unassoc), ""});
        table->addRow({"reassociated", "", stats.num_sec_reassociated + stats.num_po_reassociated, ""});
        table->addRow({"reassoc ratio", "", perc(stats.num_sec_reassociated + stats.num_po_reassociated, stats.num_po_unassoc + stats.num_sec_unassoc), ""});
        table->addRow({"", "", "", ""});
    }

    if (task().debugSettings().stats_adsb_)
    {
        auto table = getTable();

        table->addRow({"Standing ADSB", "", "", ""}, {}, "", "", {}, ResultReport::CellStyleTextBold);

        bool min_max_set = stats.num_standing_adsb_updates_min == std::numeric_limits<size_t>::max();

        table->addRow({"rec nums added", "", stats.num_standing_adsb, ""});
        table->addRow({"updates added min", "", min_max_set ? stats.num_standing_adsb_updates_min : -1, ""});
        table->addRow({"updates added max", "", min_max_set ? stats.num_standing_adsb_updates_max : -1, ""});
        table->addRow({"updates added total", "", stats.num_standing_adsb_updates_total, ""});
        table->addRow({"", "", "", ""});
    }
}

void ReconstructorBase::doUnassociatedAnalysis()
{
    auto& dbcont_man = COMPASS::instance().dbContentManager();

    traced_assert(dbcont_man.hasMinMaxPosition());

    unsigned int slice_cnt = currentSlice().slice_count_;
    unsigned int run_cnt = currentSlice().run_count_;

    string name = "Unassocated "+to_string(slice_cnt)+" Run"+to_string(currentSliceRepeatRun());

    // unassociated grid
    double lat_min, lat_max, lon_min, lon_max;

    tie(lat_min, lat_max) = dbcont_man.minMaxLatitude();
    tie(lon_min, lon_max) = dbcont_man.minMaxLongitude();

    QRectF roi(lon_min, lat_min, lon_max - lon_min, lat_max - lat_min);
    traced_assert(!roi.isEmpty());

    auto vp = task().getDebugViewpoint(
        name+" Unassociated Grid", "Grid");

    auto anno = vp->annotations().getOrCreateAnnotation("Unassociated Grid");

    unsigned num_cells_x, num_cells_y;

    std::tie(num_cells_x, num_cells_y) = Number::computeGeoWindowResolution(
        lat_min, lat_max, lon_min, lon_max,
        task().debugSettings().grid_max_resolution_, task().debugSettings().max_num_grid_cells_);

    Grid2D grid;
    grid.create(roi, grid2d::GridResolution().setCellCount(num_cells_x, num_cells_y));

    auto& section = COMPASS::instance().taskManager().currentReport()->getSection(
        "Association:Unassociated");

    for (auto rec_num : associator().unassociatedRecNums())
    {
        traced_assert(target_reports_.count(rec_num));

        auto& tr = target_reports_.at(rec_num);

        traced_assert(tr.position_);

        grid.addValue(tr.position_->longitude_, tr.position_->latitude_, 1.0);
    }

    //vp->appendToDescription("max value: "+String::doubleToStringPrecision(val_max, 2));

    Grid2DLayers layers;
    grid.addToLayers(layers, "factor", grid2d::ValueType::ValueTypeMax);

    Grid2DRenderSettings rsettings;
    rsettings.min_value       = 0.0;
    rsettings.max_value       = 1.0;

    rsettings.color_map.create(ColorMap::ColorScale::Green2Red, 2);

    auto result = Grid2DLayerRenderer::render(layers.layer(0), rsettings);

    auto f = new ViewPointGenFeatureGeoImage(result.first, result.second);
    anno->addFeature(f);

    if (!section.hasTable("Unassociated Target Reports"))
        section.addTable("Unassociated Target Reports", 8,
                         {"Slice", "Run", "#Unassoc.", "#All", "Unassoc. [%]",
                                               "#Unassoc.Total", "#Total", "Unassoc.Total [%]"}, true);

    unsigned int num_unassociated_target_reports = associator().unassociatedRecNums().size();

    num_new_target_reports_total_ += num_new_target_reports_in_slice_;
    num_unassociated_target_reports_total_ += num_unassociated_target_reports;

    auto& table = section.getTable("Unassociated Target Reports");

    nlohmann::json::array_t row{slice_cnt, run_cnt};

    if (num_new_target_reports_in_slice_)
    {
        row.insert(row.end(), {num_unassociated_target_reports, num_new_target_reports_in_slice_,
                               String::percentToString(
                                   100.0*num_unassociated_target_reports/ (float)num_new_target_reports_in_slice_)});
    }
    else
        row.insert(row.end(), {{}, {}, {}});


    if (num_new_target_reports_total_)
    {
        row.insert(row.end(), {num_unassociated_target_reports_total_, num_new_target_reports_total_,
                               String::percentToString(
                                   100.0*num_unassociated_target_reports_total_/ (float)num_new_target_reports_total_)});
    }
    else
        row.insert(row.end(), {{}, {}, {}});

    nlohmann::json vp_json;
    vp->toJSON(vp_json);
    //section.addFigure("Avg. Unused Scatterplot", {vp_json});

    // slice was already switched
    vp_json[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Maximum"] = Time::toString(
        next_slice_begin_ - settings().sliceDuration() - boost::posix_time::milliseconds(1));
    vp_json[ViewPoint::VP_FILTERS_KEY]["Timestamp"]["Timestamp Minimum"] =
        Time::toString(current_slice_begin_ - settings().sliceDuration());

    // vp_json[ViewPoint::VP_FILTERS_KEY]["Record Number"]["Record NumberCondition0"] =
    //     String::compress(associator().unassociatedRecNums(), ',');

    vp_json[ViewPoint::VP_SELECTED_RECNUMS_KEY] = associator().unassociatedRecNums();

    table.addRow(row, {vp_json});

    //loginf << "UGA json '" << vp_json.dump() << "'";
}

void ReconstructorBase::doOutlierAnalysis()
{
    // for (auto tr_it = target_reports_.begin(); tr_it != target_reports_.end() /* not hoisted */; /* no increment */)
    // {

    // }
}

bool ReconstructorBase::processing() const
{
    return processing_;
}

void ReconstructorBase::cancel()
{
    cancelled_ = true;
}

void ReconstructorBase::saveTargets()
{
    loginf << "num " << targets_container_.targets_.size();

    processing_ = true;

    DBContentManager& cont_man = COMPASS::instance().dbContentManager();

    cont_man.createNewTargets(targets_container_.targets_);

    cont_man.saveTargets();

    processing_ = false;

    logdbg << "done";
}

const dbContent::TargetReportAccessor& ReconstructorBase::accessor(
    const dbContent::targetReport::ReconstructorInfo& tr) const
{
    traced_assert(accessors_.count(tr.dbcont_id_));
    return accessors_.at(tr.dbcont_id_);
}

ReconstructorTask& ReconstructorBase::task() const
{
    return task_;
}

ReconstructorBase::DataSlice& ReconstructorBase::currentSlice()
{
    return task_.processingSlice();
}

const ReconstructorBase::DataSlice& ReconstructorBase::currentSlice() const
{
    return task_.processingSlice();
}

double ReconstructorBase::determineProcessNoiseVariance(const dbContent::targetReport::ReconstructorInfo& ri,
                                                        const dbContent::ReconstructorTarget& target,
                                                        const ReferenceCalculatorSettings::ProcessNoise& Q)
{
    auto Q_std = determineProcessNoise(ri, target, Q);
    return Q_std * Q_std;
}

double ReconstructorBase::determineProcessNoise(const dbContent::targetReport::ReconstructorInfo& ri,
                                                const dbContent::ReconstructorTarget& target,
                                                const ReferenceCalculatorSettings::ProcessNoise& Q) 
{
    //no dynamic process noise => return static noise
    if (!ref_calc_settings_.dynamic_process_noise)
        return Q.Q_std_static;

    double f_ground {1.0};

    if (target.targetCategory() != TargetBase::Category::Unknown)
        f_ground = dbContent::Target::processNoiseFactorGround(target.targetCategory());

    if (target.targetCategory() != TargetBase::Category::Unknown
        && dbContent::Target::isGroundOnly(target.targetCategory()))
        return Q.Q_std_ground * f_ground;

    double f_air {1.0};

    if (target.targetCategory() != TargetBase::Category::Unknown)
        f_air = dbContent::Target::processNoiseFactorAir(target.targetCategory());

    auto alt_state = target.getAltitudeStateStruct(ri.timestamp_, Time::partialSeconds(settings().max_time_diff_));

    if (alt_state.fl_unknown)
        return Q.Q_std_unknown; // use unknown value with factor 1

    if (alt_state.fl_on_ground)
        return Q.Q_std_ground * f_ground; // on ground

    double Q_std;

#if 0
    //interp between min/max altitude
    traced_assert(ref_calc_settings_.Q_altitude_min_ft < ref_calc_settings_.Q_altitude_max_ft);

    double alt_ft = std::max(ref_calc_settings_.Q_altitude_min_ft,
                            std::min(ref_calc_settings_.Q_altitude_max_ft, (double)alt_state.alt_baro_ft));
    double t = (alt_ft - ref_calc_settings_.Q_altitude_min_ft)
                / (ref_calc_settings_.Q_altitude_max_ft - ref_calc_settings_.Q_altitude_min_ft);
    Q_std = (1.0 - t) * Q.Q_std_ground * f_ground + t * Q.Q_std_air * f_air;
#else
    //
    Q_std = Q.Q_std_air * f_air; // in air
#endif

    return Q_std;
}

void ReconstructorBase::createMeasurement(reconstruction::Measurement& mm, 
                                          const dbContent::targetReport::ReconstructorInfo& ri,
                                          const dbContent::ReconstructorTarget* target)
{
    mm = {};

    mm.source_id = ri.record_num_;
    mm.t         = ri.timestamp_;
    
    auto pos = ri.position();
    traced_assert(pos.has_value());

    auto vel = ri.velocity_;

    auto pos_acc = acc_estimator_->positionAccuracy(ri);

    if (pos_acc.x_stddev_ == 0 || pos_acc.y_stddev_ == 0)
    {
        logerr << "stddevs 0,  x " << pos_acc.x_stddev_
               << " y " << pos_acc.y_stddev_ << " ds_id " << ri.ds_id_ << " dbcont_id " << ri.dbcont_id_;
        traced_assert(false);
    }

    auto vel_acc = acc_estimator_->velocityAccuracy(ri);
    auto acc_acc = acc_estimator_->accelerationAccuracy(ri);

    //position
    mm.lat = pos.value().latitude_;
    mm.lon = pos.value().longitude_;

    //if target is available determine process noise on per target report basis
    if (target)
    {
        mm.Q_var        = (float)determineProcessNoiseVariance(ri, *target, ref_calc_settings_.Q_std         );
        mm.Q_var_interp = (float)determineProcessNoiseVariance(ri, *target, ref_calc_settings_.resample_Q_std);
    }

    //velocity
    if (vel.has_value())
    {
        auto speed_vec = Utils::Number::speedAngle2SpeedVec(vel->speed_, vel->track_angle_);

        mm.vx = speed_vec.first;
        mm.vy = speed_vec.second;

        //@TODO: vz?
    }

    //@TODO: acceleration?

    //accuracies
    mm.x_stddev = pos_acc.x_stddev_;
    mm.y_stddev = pos_acc.y_stddev_;
    mm.xy_cov   = pos_acc.xy_cov_;

    mm.vx_stddev = vel_acc.vx_stddev_;
    mm.vy_stddev = vel_acc.vy_stddev_;

    mm.ax_stddev = acc_acc.ax_stddev_;
    mm.ay_stddev = acc_acc.ay_stddev_;

    //fix invalid correlation coefficient
    const double ccoeff     = pos_acc.xy_cov_ / (pos_acc.x_stddev_ * pos_acc.y_stddev_);
    const double eps        = 1e-09;
    const double ccoeff_min = -1.0 + eps;
    const double ccoeff_max =  1.0 - eps;

    if (!std::isfinite(ccoeff) || ccoeff < ccoeff_min || ccoeff > ccoeff_max)
    {
        const double stddev_mean = pos_acc.avgStdDev();

        mm.x_stddev = stddev_mean;
        mm.y_stddev = stddev_mean;
        mm.xy_cov   = 0.0;

        mm.pos_acc_corrected = true;
    }

    //other flags
    mm.stopped = !ri.isMoving();
}

void ReconstructorBase::createMeasurement(reconstruction::Measurement& mm,
                                          unsigned long rec_num,
                                          const dbContent::ReconstructorTarget* target)
{
    auto it = target_reports_.find(rec_num);
    traced_assert(it != target_reports_.end());

    createMeasurement(mm, it->second, target);
}

const dbContent::targetReport::ReconstructorInfo* ReconstructorBase::getInfo(unsigned long rec_num) const
{
    auto it = target_reports_.find(rec_num);
    if (it == target_reports_.end())
        return nullptr;

    return &it->second;
}

reconstruction::KalmanChainPredictors& ReconstructorBase::chainPredictors()
{
    traced_assert(chain_predictors_);

    return *chain_predictors_;
}

boost::optional<unsigned int> ReconstructorBase::utnForACAD(unsigned int acad)
{
    return targets_container_.utnForACAD(acad);
}

std::unique_ptr<reconstruction::KalmanChain>& ReconstructorBase::chain(unsigned int utn)
{
    return chains_[utn];
}

void ReconstructorBase::informConfigChanged()
{
    emit configChanged();
}

std::unique_ptr<reconstruction::KalmanChain> ReconstructorBase::createConfiguredChain(bool dynamic_insertions) const
{
    std::unique_ptr<reconstruction::KalmanChain> chain;
    chain.reset(new reconstruction::KalmanChain);

    //override some estimator settings for the chain
    chain->settings().mode            = dynamic_insertions ? reconstruction::KalmanChain::Settings::Mode::DynamicInserts :
                                                             reconstruction::KalmanChain::Settings::Mode::StaticAdd;
    chain->settings().prediction_mode = reconstruction::KalmanChain::Settings::PredictionMode::Interpolate;
    chain->settings().verbosity       = 0;
    chain->settings().debug           = false;

    chain->settings().prediction_max_tdiff_sec = settings().max_time_diff_;

    chain->configureEstimator(referenceCalculatorSettings().chainEstimatorSettings());
    chain->init(referenceCalculatorSettings().kalman_type_assoc);

    auto rec_ptr = const_cast<ReconstructorBase*>(this);

    chain->setMeasurementAssignFunc(
        [ rec_ptr ] (reconstruction::Measurement& mm, unsigned long rec_num)
        {
            rec_ptr->createMeasurement(mm, rec_num);
        });

    chain->setMeasurementCheckFunc(
        [ rec_ptr ] (unsigned long rec_num)
        {
            return rec_ptr->target_reports_.find(rec_num) != rec_ptr->target_reports_.end();
        });

    return chain;
}

std::unique_ptr<reconstruction::KalmanEstimator> ReconstructorBase::createConfiguredEstimator(bool extract_wgs84_pos) const
{
    std::unique_ptr<reconstruction::KalmanEstimator> kalman;
    kalman.reset(new reconstruction::KalmanEstimator);

    kalman->settings() = referenceCalculatorSettings().chainEstimatorSettings();
    kalman->settings().extract_wgs84_pos = extract_wgs84_pos;
    kalman->settings().verbosity         = 0;
    kalman->settings().debug             = false;

    kalman->init(referenceCalculatorSettings().kalman_type_assoc);

    return kalman;
}

std::unique_ptr<reconstruction::KalmanOnlineTracker> ReconstructorBase::createConfiguredOnlineEstimator(bool extract_wgs84_pos) const
{
    std::unique_ptr<reconstruction::KalmanOnlineTracker> kalman;
    kalman.reset(new reconstruction::KalmanOnlineTracker);

    kalman->settings().prediction_max_tdiff_sec = settings().max_time_diff_;

    kalman->estimatorSettings() = referenceCalculatorSettings().chainEstimatorSettings();
    kalman->estimatorSettings().extract_wgs84_pos = extract_wgs84_pos;
    kalman->estimatorSettings().verbosity         = 0;
    kalman->estimatorSettings().debug             = false;

    kalman->init(referenceCalculatorSettings().kalman_type_assoc);

    return kalman;
}
