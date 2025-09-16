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

#include "evaluationtargetdata.h"
#include "evaluationtarget.h"
#include "evaluationdefs.h"

#include "task/result/report/reportdefs.h"

#include "logger.h"
#include "stringconv.h"
#include "dbcontent/target/target.h"
#include "compass.h"
#include "dbcontent/dbcontentmanager.h"
#include "evaluationmanager.h"
#include "evaluationcalculator.h"
#include "util/timeconv.h"
#include "sector/airspace.h"
#include "sectorlayer.h"
#include "evalsectionid.h"
#include "reportdefs.h"

#include <boost/algorithm/string.hpp>

#include "traced_assert.h"
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include <QString>
#include <QAction>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;
using namespace dbContent::TargetReport;

//const unsigned int debug_utn = 3275;

double EvaluationTargetData::interest_thres_req_high_ = 0.05;
double EvaluationTargetData::interest_thres_req_mid_  = 0.01;

double EvaluationTargetData::interest_thres_sum_high_ = 0.1;
double EvaluationTargetData::interest_thres_sum_mid_  = 0.05;

/**
 */
EvaluationTargetData::EvaluationTargetData(unsigned int utn, 
                                           EvaluationData& eval_data,
                                           std::shared_ptr<dbContent::DBContentAccessor> accessor,
                                           EvaluationCalculator& calculator,
                                           EvaluationManager& eval_man,
                                           DBContentManager& dbcont_man)
    :   utn_       (utn)
    ,   eval_data_ (eval_data)
    ,   accessor_  (accessor)
    ,   calculator_(calculator)
    ,   eval_man_(eval_man)
    ,   dbcont_man_(dbcont_man)
    ,   ref_chain_ (accessor_, calculator_.dbContentNameRef())
    ,   tst_chain_ (accessor_, calculator_.dbContentNameTst())
{
}

/**
 */
EvaluationTargetData::~EvaluationTargetData() = default;

/**
 */
void EvaluationTargetData::addRefIndex (boost::posix_time::ptime timestamp, unsigned int index)
{
    ref_chain_.addIndex(timestamp, index);
}

/**
 */
void EvaluationTargetData::addTstIndex (boost::posix_time::ptime timestamp, unsigned int index)
{
    tst_chain_.addIndex(timestamp, index);
}

/**
 */
bool EvaluationTargetData::hasData() const
{
    return (hasRefData() || hasTstData());
}

/**
 */
bool EvaluationTargetData::hasRefData () const
{
    return ref_chain_.hasData();
}

/**
 */
bool EvaluationTargetData::hasTstData () const
{
    //return !tst_chain_.empty();
    return tst_chain_.hasData();
}

/**
 */
void EvaluationTargetData::finalize () const
{
    //    loginf << "utn " << utn_
    //           << " ref " << hasRefData() << " up " << ref_rec_nums_.size()
    //           << " tst " << hasTstData() << " up " << tst_rec_nums_.size();

    ref_chain_.finalize();
    tst_chain_.finalize();

    updateACIDs();
    updateACADs();
    updateModeACodes();
    updateModeCMinMax();
    updatePositionMinMax();

    updateUseInfo();

    DBContentManager& dbcont_man = COMPASS::instance().dbContentManager();

    if (dbcont_man.hasTargetsInfo()   &&
        dbcont_man.existsTarget(utn_) &&
        dbcont_man.target(utn_).hasADSBMOPS())
    {
        has_adsb_info_ = true;
        has_mops_versions_ = true;
        mops_versions_ = dbcont_man.target(utn_).adsbMopsList();
    }

    //    std::set<unsigned int> mops_version;
    //    std::tuple<bool, unsigned int, unsigned int> nucp_info;
    //    std::tuple<bool, unsigned int, unsigned int> nacp_info;

    //    if (eval_man_->hasADSBInfo() && target_addresses_.size())
    //    {
    //        for (auto ta_it : target_addresses_)
    //        {
    //            if (eval_man_->hasADSBInfo(ta_it))
    //            {
    //                tie(mops_version, nucp_info, nacp_info) = eval_man_->adsbInfo(ta_it);
    //                mops_versions_.insert(mops_version.begin(), mops_version.end());

    //                if (get<0>(nucp_info))
    //                {
    //                    if (has_nucp_nic_)
    //                    {
    //                        min_nucp_nic_ = min (min_nucp_nic_, get<1>(nucp_info));
    //                        max_nucp_nic_ = max (max_nucp_nic_, get<2>(nucp_info));
    //                    }
    //                    else
    //                    {
    //                        min_nucp_nic_ = get<1>(nucp_info);
    //                        max_nucp_nic_ = get<2>(nucp_info);
    //                        has_nucp_nic_ = true;
    //                    }
    //                }

    //                if (get<0>(nacp_info))
    //                {
    //                    if (has_nacp)
    //                    {
    //                        min_nacp_ = min (min_nacp_, get<1>(nacp_info));
    //                        max_nacp_ = max (max_nacp_, get<2>(nacp_info));
    //                    }
    //                    else
    //                    {
    //                        min_nacp_ = get<1>(nacp_info);
    //                        max_nacp_ = get<2>(nacp_info);
    //                        has_nacp = true;
    //                    }
    //                }
    //            }
    //        }

    //        has_adsb_info_ = mops_versions_.size();
    //        //        loginf << "UGA utn " << utn_ << " mops " << mopsVersionsStr()
    //        //               << " nucp_nic " <<  (has_nucp_nic_ ? nucpNicStr() : " none ")
    //        //               << " nacp "<<  (has_nacp ? nacpStr() : " none ");
    //    }

    calculateTestDataMappings();
    computeSectorInsideInfo();
}

/**
 */
void EvaluationTargetData::updateToChanges() const
{
    clearInterestFactors();
    updateUseInfo();
}

/**
 */
unsigned int EvaluationTargetData::numUpdates () const
{
    return ref_chain_.size() + tst_chain_.size();
}

/**
 */
unsigned int EvaluationTargetData::numRefUpdates () const
{
    return ref_chain_.size() ;
}

/**
 */
unsigned int EvaluationTargetData::numTstUpdates () const
{
    return tst_chain_.size() ;
}

/**
 */
ptime EvaluationTargetData::timeBegin() const
{
    if (ref_chain_.size() && tst_chain_.size())
        return min(ref_chain_.timeBegin(), tst_chain_.timeBegin());
    else if (ref_chain_.size())
        return ref_chain_.timeBegin();
    else if (tst_chain_.size())
        return tst_chain_.timeBegin();
    else
        throw std::runtime_error("EvaluationTargetData: timeBegin: no data");
}

/**
 */
std::string EvaluationTargetData::timeBeginStr() const
{
    if (hasData())
        return Time::toString(timeBegin());
    else
        return "";
}

/**
 */
ptime EvaluationTargetData::timeEnd() const
{
    if (ref_chain_.size() && tst_chain_.size())
        return max(ref_chain_.timeEnd(), tst_chain_.timeEnd());
    else if (ref_chain_.size())
        return ref_chain_.timeEnd();
    else if (tst_chain_.size())
        return tst_chain_.timeEnd();
    else
        throw std::runtime_error("EvaluationTargetData: timeEnd: no data");
}

/**
 */
std::string EvaluationTargetData::timeEndStr() const
{
    if (hasData())
        return Time::toString(timeEnd());
    else
        return "";
}

/**
 */
time_duration EvaluationTargetData::timeDuration() const
{
    if (hasData())
        return timeEnd() - timeBegin();
    else
        return {};
}

/**
 */
std::string EvaluationTargetData::timeDurationStr() const
{
    if (hasData())
        return Time::toString(timeDuration());
    else
        return "";
}

/**
 */
std::set<unsigned int> EvaluationTargetData::modeACodes() const
{
    logdbg << "utn " << utn_ << " num codes " << mode_a_codes_.size();
    return mode_a_codes_;
}

/**
 */
std::string EvaluationTargetData::modeACodesStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (auto& ma_it : mode_a_codes_)
    {
        if (cnt != 0)
            out << ", ";

        out << String::octStringFromInt(ma_it, 4, '0');

        ++cnt;
    }

    return out.str();
}

/**
 */
bool EvaluationTargetData::hasModeC() const
{
    return has_mode_c_;
}

/**
 */
float EvaluationTargetData::modeCMin() const
{
    traced_assert(has_mode_c_);
    return mode_c_min_;
}

/**
 */
std::string EvaluationTargetData::modeCMinStr() const
{
    if (has_mode_c_)
        return to_string((int)mode_c_min_);
    else
        return "";
}

/**
 */
float EvaluationTargetData::modeCMax() const
{
    traced_assert(has_mode_c_);
    return mode_c_max_;
}

/**
 */
std::string EvaluationTargetData::modeCMaxStr() const
{
    if (has_mode_c_)
        return to_string((int)mode_c_max_);
    else
        return "";
}

/**
 */
bool EvaluationTargetData::isPrimaryOnly () const
{
    return !acids_.size() && !acads_.size() && !mode_a_codes_.size() && !has_mode_c_;
}

/**
 */
bool EvaluationTargetData::isModeS () const
{
    return acids_.size() || acads_.size();
}

/**
 */
bool EvaluationTargetData::isModeACOnly () const
{
    return (mode_a_codes_.size() || has_mode_c_) && !isModeS();
}

/**
 */
void EvaluationTargetData::updateUseInfo() const
{
    const auto& target = dbcont_man_.target(utn_);

    use_in_eval_           = target.useInEval();
    excluded_time_windows_ = target.evalExcludedTimeWindows();
    excluded_requirements_ = target.evalExcludedRequirements();
}

/**
 */
bool EvaluationTargetData::use() const
{
    return use_in_eval_;
}

/**
 */
const dbContent::TargetReport::Chain& EvaluationTargetData::refChain() const
{
    return ref_chain_;
}

/**
 */
const dbContent::TargetReport::Chain& EvaluationTargetData::tstChain() const
{
    return tst_chain_;
}

/**
 */
bool EvaluationTargetData::hasMappedRefData(const DataID& tst_id,
                                            time_duration d_max) const
{
    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
        return false;

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_pos_)
        return true;

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
            return false;

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
            return false;

        return true;
    }

    return false;
}

/**
 */
std::pair<ptime, ptime> EvaluationTargetData::mappedRefTimes(const DataID& tst_id,
                                                             time_duration d_max)  const
{
    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
        return {{}, {}};

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_pos_)
        return {mapping.timestamp_ref1_, {}};

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
            return {{}, {}};

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
            return {{}, {}};

        return {mapping.timestamp_ref1_, mapping.timestamp_ref2_};
    }

    return {{}, {}};
}

/**
 */
boost::optional<dbContent::TargetPosition> EvaluationTargetData::mappedRefPos(const DataID& tst_id) const
{
    auto index = tst_chain_.indexFromDataID(tst_id);

    const auto& tdm = tst_data_mappings_.at(index.idx_internal);
    if (!tdm.has_ref_pos_)
        return {};

    return tdm.pos_ref_;
}

/**
 */
boost::optional<dbContent::TargetPosition> EvaluationTargetData::mappedRefPos(
    const DataID& tst_id, time_duration d_max, bool debug) const
{
    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    if (debug)
        loginf << "utn " << utn_ << " timestamp "
               << Time::toString(timestamp) << " d_max " << Time::toString(d_max);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (debug)
        loginf << "utn " << utn_ << " has_ref1 "
               << mapping.has_ref1_ << " has_ref2 " << mapping.has_ref2_;

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
    {
        if (debug)
            loginf << "utn " << utn_ << " no ref data";

        return {};
    }

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_pos_)
    {
        if (debug)
            loginf << "utn " << utn_ << " exact match";

        return mapping.pos_ref_;
    }

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        if (debug)
            loginf << "utn " << utn_ << " both ref data";

        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "lower too far";

            if (debug)
                loginf << "utn " << utn_ << " lower too far "
                    << Time::toString(mapping.timestamp_ref1_ - timestamp);

            return {};
        }

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "upper too far";

            if (debug)
                loginf << "utn " << utn_ << " higher too far "
                       << Time::toString(mapping.timestamp_ref2_ - timestamp);

            return {};
        }

        if (!mapping.has_ref_pos_)
        {
            //            if (utn_ == debug_utn)
            //                loginf << "no ref pos";

            if (debug)
                loginf << "utn " << utn_ << " no ref_pos in mapping";

            return {};
        }

        //        if (utn_ == debug_utn)
        //            loginf << "2pos tod " << String::timeStringFromDouble(tod)
        //                   << " has_alt " << mapping.pos_ref_.has_altitude_
        //                   << " alt_calc " << mapping.pos_ref_.altitude_calculated_
        //                   << " alt " << mapping.pos_ref_.altitude_;

        return mapping.pos_ref_;
    }

    if (debug)
        loginf << "utn " << utn_ << " only 1";

    return {};
}

boost::optional<double> EvaluationTargetData::mappedRefMinAcc(
    const dbContent::TargetReport::Chain::DataID& tst_id, boost::posix_time::time_duration d_max,
    bool debug) const
{
    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    if (debug)
        loginf << "utn " << utn_ << " timestamp "
               << Time::toString(timestamp) << " d_max " << Time::toString(d_max);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (debug)
        loginf << "utn " << utn_ << " has_ref1 "
               << mapping.has_ref1_ << " has_ref2 " << mapping.has_ref2_;

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
    {
        if (debug)
            loginf << "utn " << utn_ << " no ref data";

        return {};
    }

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_pos_)
    {
        if (debug)
            loginf << "utn " << utn_ << " exact match";

        auto acc1 = ref_chain_.posAccuracy(mapping.dataid_ref1_);

        if (acc1)
            return acc1->max();
        else
            return {};
    }

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        if (debug)
            loginf << "utn " << utn_ << " both ref data";

        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "lower too far";

            if (debug)
                loginf << "utn " << utn_ << " lower too far "
                    << Time::toString(mapping.timestamp_ref1_ - timestamp);

            return {};
        }

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "upper too far";

            if (debug)
                loginf << "utn " << utn_ << " higher too far "
                       << Time::toString(mapping.timestamp_ref2_ - timestamp);

            return {};
        }

        auto acc1 = ref_chain_.posAccuracy(mapping.dataid_ref1_);
        auto acc2 = ref_chain_.posAccuracy(mapping.dataid_ref2_);

        if (acc1 && acc2)
            return std::max(acc1->max(), acc2->max());
        else if (acc1)
            return acc1->max();
        else if (acc2)
            return acc2->max();            
        else
            return {};
    }

    if (debug)
        loginf << "utn " << utn_ << " only 1";

    return {};    
}

/**
 */
boost::optional<dbContent::TargetVelocity> EvaluationTargetData::mappedRefSpeed(const DataID& tst_id,
                                                                                time_duration d_max) const
{
    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
        return {};

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_spd_)
        return mapping.spd_ref_;

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "lower too far";

            return {};
        }

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
        {
            //            if (utn_ == debug_utn)
            //                loginf << "upper too far";

            return {};
        }

        if (!mapping.has_ref_spd_)
        {
            //            if (utn_ == debug_utn)
            //                loginf << "no ref pos";

            return {};
        }

        //        if (utn_ == debug_utn)
        //            loginf << "2pos tod " << String::timeStringFromDouble(tod)
        //                   << " has_alt " << mapping.pos_ref_.has_altitude_
        //                   << " alt_calc " << mapping.pos_ref_.altitude_calculated_
        //                   << " alt " << mapping.pos_ref_.altitude_;

        return mapping.spd_ref_;
    }

    return {};
}

/**
 */
boost::optional<bool> EvaluationTargetData::mappedRefGroundBit(const DataID& tst_id,
                                                               time_duration d_max) const
// has gbs, gbs true
{
    //    bool has_gbs = false;
    //    bool gbs = false;

    auto timestamp = tst_id.timestamp();
    auto index     = tst_chain_.indexFromDataID(tst_id);

    const DataMapping& mapping = tst_data_mappings_.at(index.idx_internal);

    if (!mapping.has_ref1_ && !mapping.has_ref2_) // no ref data
        return {};

    if (mapping.has_ref1_ && mapping.timestamp_ref1_ == timestamp && mapping.has_ref_spd_)
    {
        auto gbs = ref_chain_.groundBit(mapping.dataid_ref1_);

        if (gbs.has_value() && *gbs)
            return gbs;
    }

    if (mapping.has_ref1_ && mapping.has_ref2_) // interpolated
    {
        traced_assert(mapping.timestamp_ref1_ <= timestamp);
        traced_assert(mapping.timestamp_ref2_ >= timestamp);

        if (timestamp - mapping.timestamp_ref1_ > d_max) // lower to far
            return {};

        if (mapping.timestamp_ref2_ - timestamp > d_max) // upper to far
            return {};

        auto gbs = ref_chain_.groundBit(mapping.dataid_ref1_);

        if (gbs.has_value() && *gbs)
            return gbs;

        return ref_chain_.groundBit(mapping.dataid_ref2_);
    }

    return {};
}

/**
 */
unsigned int EvaluationTargetData::tstDSID(const dbContent::TargetReport::Chain::DataID& tst_id) const
{
    //auto index     = tst_chain_.indexFromDataID(tst_id);
    return tst_chain_.dsID(tst_id);
}

/**
 */
boost::optional<bool> EvaluationTargetData::tstGroundBitInterpolated(
    const DataID& ref_id, const boost::posix_time::time_duration& d_max) const // true is on ground
{
    auto ref_timestamp = ref_chain_.timestampFromDataID(ref_id);

    DataMappingTimes times = tst_chain_.findDataMappingTimes(ref_timestamp);

    if (times.has_other1_ && (ref_timestamp - times.timestamp_other1_).abs() < d_max)
    {
        auto gbs = tst_chain_.groundBit(times.dataid_other1_);

        if (gbs.has_value() && *gbs)
            return gbs;
    }

    if (times.has_other2_ && (ref_timestamp - times.timestamp_other2_).abs() < d_max)
    {
        return tst_chain_.groundBit(times.dataid_other2_);
    }

    return {};
}


//bool EvaluationTargetData::canCheckTstMultipleSources() const
//{
//    if (!eval_data_.tst_multiple_srcs_name_.size())
//        return false;

//    if (!eval_data_.tst_buffer_->has<bool>(eval_data_.tst_multiple_srcs_name_))
//        return false;

//    NullableVector<bool>& tst_multiple_srcs_vec =
//            eval_data_.tst_buffer_->get<bool>(eval_data_.tst_multiple_srcs_name_);

//    for (auto tst_index : tst_indices_)
//    {
//        if (!tst_multiple_srcs_vec.isNull(tst_index))
//            return true;
//    }

//    return false;
//}

//bool EvaluationTargetData::hasTstMultipleSources() const
//{
//    traced_assert(canCheckTstMultipleSources());

//    NullableVector<bool>& tst_multiple_srcs_vec =
//            eval_data_.tst_buffer_->get<bool>(eval_data_.tst_multiple_srcs_name_);

//    for (auto tst_index : tst_indices_) // one must be not null according to canCheckTstMultipleSources
//    {
//        if (!tst_multiple_srcs_vec.isNull(tst_index) && tst_multiple_srcs_vec.get(tst_index))
//            return true;
//    }

//    return false;
//}

//bool EvaluationTargetData::canCheckTrackLUDSID() const
//{
//    if (!eval_data_.tst_track_lu_ds_id_name_.size())
//        return false;

//    if (!eval_data_.tst_buffer_->has<unsigned int>(eval_data_.tst_track_lu_ds_id_name_))
//        return false;

//    NullableVector<unsigned int> tst_ls_ds_id_vec =
//            eval_data_.tst_buffer_->get<unsigned int>(eval_data_.tst_track_lu_ds_id_name_);

//    for (auto tst_index : tst_indices_)
//    {
//        if (!tst_ls_ds_id_vec.isNull(tst_index))
//            return true;
//    }

//    return false;
//}

//bool EvaluationTargetData::hasSingleLUDSID() const
//{
//    traced_assert(canCheckTrackLUDSID());

//    // check if only single source updates
//    traced_assert(canCheckTstMultipleSources());
//    traced_assert(!hasTstMultipleSources());

//    bool lu_ds_id_found = false;
//    unsigned int lu_ds_id;

//    NullableVector<unsigned int> tst_ls_ds_id_vec =
//            eval_data_.tst_buffer_->get<unsigned int>(eval_data_.tst_track_lu_ds_id_name_);

//    for (auto tst_index : tst_indices_)
//    {
//        if (!tst_ls_ds_id_vec.isNull(tst_index))
//        {
//            if (!lu_ds_id_found)
//            {
//                lu_ds_id_found = true;
//                lu_ds_id = tst_ls_ds_id_vec.get(tst_index);

//                continue;
//            }

//            if (lu_ds_id_found && tst_ls_ds_id_vec.get(tst_index) != lu_ds_id)
//                return false;
//        }
//    }

//    return true;
//}

//unsigned int EvaluationTargetData::singleTrackLUDSID() const
//{
//    traced_assert(hasSingleLUDSID());

//    NullableVector<unsigned int> tst_ls_ds_id_vec =
//            eval_data_.tst_buffer_->get<unsigned int>(eval_data_.tst_track_lu_ds_id_name_);

//    for (auto tst_index : tst_indices_)
//    {
//        if (!tst_ls_ds_id_vec.isNull(tst_index))
//            return tst_ls_ds_id_vec.get(tst_index);
//    }

//    traced_assert(false); // can not be reached
//}

/**
 */
double EvaluationTargetData::latitudeMin() const
{
    traced_assert(has_pos_);
    return latitude_min_;
}

/**
 */
double EvaluationTargetData::latitudeMax() const
{
    traced_assert(has_pos_);
    return latitude_max_;
}

/**
 */
double EvaluationTargetData::longitudeMin() const
{
    traced_assert(has_pos_);
    return longitude_min_;
}

/**
 */
double EvaluationTargetData::longitudeMax() const
{
    traced_assert(has_pos_);
    return longitude_max_;
}

/**
 */
bool EvaluationTargetData::hasPos() const
{
    return has_pos_;
}

/**
 */
bool EvaluationTargetData::hasADSBInfo() const
{
    return has_adsb_info_;
}

/**
 */
bool EvaluationTargetData::hasMOPSVersion() const
{
    return has_mops_versions_;
}

/**
 */
std::set<unsigned int> EvaluationTargetData::mopsVersions() const
{
    traced_assert(has_mops_versions_);
    return mops_versions_;
}

/**
 */
std::string EvaluationTargetData::mopsVersionStr() const
{
    if (hasMOPSVersion())
    {
        std::ostringstream out;

        unsigned int cnt=0;
        for (const auto it : mops_versions_)
        {
            if (cnt != 0)
                out << ", ";

            out << it;
            ++cnt;
        }

        return out.str().c_str();
    }
    else
        return "?";
}

//std::string EvaluationTargetData::nucpNicStr() const
//{
//    if (hasNucpNic())
//    {
//        if (min_nucp_nic_ == max_nucp_nic_)
//            return to_string(min_nucp_nic_);
//        else
//            return to_string(min_nucp_nic_)+"-"+to_string(max_nucp_nic_);
//    }
//    else
//        return "?";
//}

//std::string EvaluationTargetData::nacpStr() const
//{
//    if (hasNacp())
//    {
//        if (min_nacp_ == max_nacp_)
//            return to_string(min_nacp_);
//        else
//            return to_string(min_nacp_)+"-"+to_string(max_nacp_);
//    }
//    else
//        return "?";
//}

//bool EvaluationTargetData::hasNucpNic() const
//{
//    return has_nucp_nic_;
//}

//bool EvaluationTargetData::hasNacp() const
//{
//    return has_nacp;
//}

/**
 */
std::set<string> EvaluationTargetData::acids() const
{
    return acids_;
}

/**
 */
string EvaluationTargetData::acidsStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (auto& cs_it : acids_)
    {
        if (cnt != 0)
            out << ", ";

        out << cs_it;
        ++cnt;
    }

    return out.str().c_str();
}

/**
 */
std::set<unsigned int> EvaluationTargetData::acads() const
{
    return acads_;
}

/**
 */
std::string EvaluationTargetData::acadsStr() const
{
    std::ostringstream out;

    unsigned int cnt=0;
    for (auto& ta_it : acads_)
    {
        if (cnt != 0)
            out << ", ";

        out << String::hexStringFromInt(ta_it, 6, '0');
        ++cnt;
    }

    return out.str().c_str();
}

/**
 */
void EvaluationTargetData::clearInterestFactors() const
{
    interest_factors_.clear();
}

/**
 */
void EvaluationTargetData::addInterestFactor (const Evaluation::RequirementSumResultID& id, 
                                              double factor, 
                                              bool reset) const
{
    logdbg << "utn " << utn_
           << " req_section_id " << EvalSectionID::requirementResultSumID(id)
           << " factor " << factor
           << " reset " << reset;

    if (reset)
        interest_factors_[ id ] = 0;

    interest_factors_[ id ] += factor;
}

/**
 */
const EvaluationTargetData::InterestMap& EvaluationTargetData::interestFactors() const
{
    return interest_factors_;
}

/**
 */
std::string EvaluationTargetData::stringForInterestFactor(const Evaluation::RequirementSumResultID& id, 
                                                          double factor)
{
    return EvalSectionID::requirementResultSumID(id) + 
           " (" + String::doubleToStringPrecision(factor, InterestFactorPrecision) + ")";
}

/**
 */
std::string EvaluationTargetData::enabledInterestFactorsString(const InterestMap& interest_factors,
                                                               const InterestEnabledFunc& interest_enabled_func)
{
    traced_assert(interest_enabled_func);

    std::string ret;
    if (interest_factors.empty())
        return ret;

    auto coloredText = [ & ] (const std::string& txt, const QColor& color)
    {
        return "<font color=\"" + color.name().toStdString() + "\">" + txt + "</font>";
    };

    auto generateRow = [ & ] (const Evaluation::RequirementSumResultID& id, double factor, int prec, int spacing)
    {
        auto factor_color = EvaluationTargetData::fgColorForInterestFactorRequirement(factor);
        auto prec_str     = String::doubleToStringPrecision(factor, prec);

        std::string ret;
        ret += "<tr>";
        ret += "<td align=\"left\">"  + coloredText(EvalSectionID::requirementResultSumID(id), factor_color) + "</td>";
        ret += "<td>" + QString().fill(' ', spacing).toStdString() + "</td>";
        ret += "<td align=\"right\">" + coloredText(prec_str, factor_color) + "</td>";
        ret += "</tr>";

        return ret;
    };

    //<font color=\"#ff0000\">bar</font>

    const int Spacing = 4;

    ret = "<html><body><table>";

    size_t added = 0;
    for (auto& fac_it : interest_factors)
    {
        if (!interest_enabled_func(fac_it.first))
            continue;

        ret += generateRow(fac_it.first, fac_it.second, InterestFactorPrecision, Spacing);

        ++added;
    }

    if (added < 1)
        return "";

    ret += "</table></body></html>";

    return ret;
}

/**
 */
QColor EvaluationTargetData::bgColorForInterestFactorRequirement(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_req_mid_)
        return ResultReport::Colors::BGGreen;
    else if (factor < EvaluationTargetData::interest_thres_req_high_)
        return ResultReport::Colors::BGOrange;
    
    return ResultReport::Colors::BGRed;
}

/**
 */
QColor EvaluationTargetData::fgColorForInterestFactorRequirement(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_req_mid_)
        return ResultReport::Colors::TextGreen;
    else if (factor < EvaluationTargetData::interest_thres_req_high_)
        return ResultReport::Colors::TextOrange;
    
    return ResultReport::Colors::TextRed;
}

/**
 */
QColor EvaluationTargetData::bgColorForInterestFactorSum(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_sum_mid_)
        return ResultReport::Colors::BGGreen;
    else if (factor < EvaluationTargetData::interest_thres_sum_high_)
        return ResultReport::Colors::BGOrange;
    
    return ResultReport::Colors::BGRed;
}

/**
 */
QColor EvaluationTargetData::fgColorForInterestFactorSum(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_sum_mid_)
        return ResultReport::Colors::TextGreen;
    else if (factor < EvaluationTargetData::interest_thres_sum_high_)
        return ResultReport::Colors::TextOrange;
    
    return ResultReport::Colors::TextRed;
}

/**
 */
unsigned int EvaluationTargetData::bgStyleForInterestFactorSum(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_sum_mid_)
        return ResultReport::CellStyleBGColorGreen;
    else if (factor < EvaluationTargetData::interest_thres_sum_high_)
        return ResultReport::CellStyleBGColorOrange;
    
    return ResultReport::CellStyleBGColorRed;
}

/**
 */
unsigned int EvaluationTargetData::fgStyleForInterestFactorSum(double factor)
{
    if (factor < EvaluationTargetData::interest_thres_sum_mid_)
        return ResultReport::CellStyleTextColorGreen;
    else if (factor < EvaluationTargetData::interest_thres_sum_high_)
        return ResultReport::CellStyleTextColorOrange;
    
    return ResultReport::CellStyleTextColorRed;
}

/**
 */
QAction* EvaluationTargetData::interestFactorAction(const Evaluation::RequirementSumResultID& id, 
                                                    double interest_factor)
{
    QColor      color = EvaluationTargetData::bgColorForInterestFactorRequirement(interest_factor);
    std::string name  = EvaluationTargetData::stringForInterestFactor(id, interest_factor);

    QAction* action = new QAction(QString::fromStdString(name));

    QImage img(16, 16, QImage::Format_ARGB32);
    img.fill(color);
    QIcon icon = QIcon(QPixmap::fromImage(img));

    action->setIcon(icon);

    return action;
}

/**
 */
void EvaluationTargetData::updateACIDs() const
{
    acids_.clear();

    if (ref_chain_.size())
    {
        auto acids = ref_chain_.acids();
        acids_.insert(acids.begin(), acids.end());
    }

    if (tst_chain_.size())
    {
        auto acids = tst_chain_.acids();
        acids_.insert(acids.begin(), acids.end());
    }
}

/**
 */
void EvaluationTargetData::updateACADs() const
{
    acads_.clear();

    if (ref_chain_.size())
    {
        auto acads = ref_chain_.acads();
        acads_.insert(acads.begin(), acads.end());
    }

    if (tst_chain_.size())
    {
        auto acads = ref_chain_.acads();
        acads_.insert(acads.begin(), acads.end());
    }
}

/**
 */
void EvaluationTargetData::updateModeACodes() const
{
    logdbg << "utn " << utn_;

    mode_a_codes_.clear();

    if (ref_chain_.size())
    {
        auto values = ref_chain_.modeACodes();
        mode_a_codes_.insert(values.begin(), values.end());
    }

    if (tst_chain_.size())
    {
        auto values = tst_chain_.modeACodes();
        mode_a_codes_.insert(values.begin(), values.end());
    }

    logdbg << "utn " << utn_ << " num codes " << mode_a_codes_.size();
}

/**
 */
void EvaluationTargetData::updateModeCMinMax() const
{
    logdbg << "utn " << utn_;

    // garbled, valid flags?

    has_mode_c_ = false;
    //float mode_c_value;

    if (ref_chain_.size() && ref_chain_.hasModeC())
    {
        has_mode_c_ = true;
        mode_c_min_ = ref_chain_.modeCMin();
        mode_c_max_ = ref_chain_.modeCMax();
    }

    if (tst_chain_.size() && tst_chain_.hasModeC())
    {
        if (!has_mode_c_)
        {
            has_mode_c_ = true;
            mode_c_min_ = tst_chain_.modeCMin();
            mode_c_max_ = tst_chain_.modeCMax();
        }
        else
        {
            mode_c_min_ = min(mode_c_min_, tst_chain_.modeCMin());
            mode_c_max_ = max(mode_c_max_, tst_chain_.modeCMax());
        }
    }
}

/**
 */
void EvaluationTargetData::updatePositionMinMax() const
{
    has_pos_ = false;

    if (ref_chain_.size() && ref_chain_.hasPos())
    {
        has_pos_ = true;
        latitude_min_ = ref_chain_.latitudeMin();
        latitude_max_ = ref_chain_.latitudeMax();
        longitude_min_ = ref_chain_.longitudeMin();
        longitude_max_ = ref_chain_.longitudeMax();
    }

    if (tst_chain_.size() && tst_chain_.hasPos())
    {
        if (!has_pos_)
        {
            has_pos_ = true;
            latitude_min_ = tst_chain_.latitudeMin();
            latitude_max_ = tst_chain_.latitudeMax();
            longitude_min_ = tst_chain_.longitudeMin();
            longitude_max_ = tst_chain_.longitudeMax();
        }
        else
        {
            latitude_min_ = min(latitude_min_, tst_chain_.latitudeMin());
            latitude_max_ = max(latitude_max_, tst_chain_.latitudeMax());
            longitude_min_ = min(longitude_min_, tst_chain_.longitudeMin());
            longitude_max_ = max(longitude_max_, tst_chain_.longitudeMax());
        }
    }
}

//void EvaluationTargetData::updateADSBInfo() const
//{
//    has_adsb_info_ = false;

//    string mops_name {"mops_version"};
//    string nacp_name {"nac_p"};
//    string nucp_nic_name {"nucp_nic"};
//    string sil_name {"sil"};

//    if (ref_chain_.size() && ref_buffer_->dbContentName() == "ADSB")
//    {
//        traced_assert(ref_buffer_->has<int>(mops_name));
//        traced_assert(ref_buffer_->has<char>(nacp_name));
//        traced_assert(ref_buffer_->has<char>(nucp_nic_name));
//        traced_assert(ref_buffer_->has<char>(sil_name));

//        NullableVector<int>& mops = ref_buffer_->get<int>(mops_name);
//        NullableVector<char>& nacps = ref_buffer_->get<char>(nacp_name);
//        NullableVector<char>& nucp_nics = ref_buffer_->get<char>(nucp_nic_name);
//        NullableVector<char>& sils = ref_buffer_->get<char>(sil_name);

//        for (auto ind_it : ref_indexes_)
//        {
//            if (!mops.isNull(ind_it) && mops_versions_.count(mops.get(ind_it)))
//            {
//                mops_versions_.insert(mops.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!nacps.isNull(ind_it) && nacps_.count(nacps.get(ind_it)))
//            {
//                mops_versions_.insert(nacps.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!nucp_nics.isNull(ind_it) && nucp_nics_.count(nucp_nics.get(ind_it)))
//            {
//                nucp_nics_.insert(nucp_nics.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!sils.isNull(ind_it) && sils_.count(sils.get(ind_it)))
//            {
//                sils_.insert(sils.get(ind_it));
//                has_adsb_info_ = true;
//            }
//        }
//    }

//    if (tst_chain_.size() && tst_buffer_->dbContentName() == "ADSB")
//    {
//        traced_assert(tst_buffer_->has<int>(mops_name));
//        traced_assert(tst_buffer_->has<char>(nacp_name));
//        traced_assert(tst_buffer_->has<char>(nucp_nic_name));
//        traced_assert(tst_buffer_->has<char>(sil_name));

//        NullableVector<int>& mops = tst_buffer_->get<int>(mops_name);
//        NullableVector<char>& nacps = tst_buffer_->get<char>(nacp_name);
//        NullableVector<char>& nucp_nics = tst_buffer_->get<char>(nucp_nic_name);
//        NullableVector<char>& sils = tst_buffer_->get<char>(sil_name);

//        for (auto ind_it : tst_indexes_)
//        {
//            if (!mops.isNull(ind_it) && mops_versions_.count(mops.get(ind_it)))
//            {
//                mops_versions_.insert(mops.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!nacps.isNull(ind_it) && nacps_.count(nacps.get(ind_it)))
//            {
//                mops_versions_.insert(nacps.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!nucp_nics.isNull(ind_it) && nucp_nics_.count(nucp_nics.get(ind_it)))
//            {
//                nucp_nics_.insert(nucp_nics.get(ind_it));
//                has_adsb_info_ = true;
//            }

//            if (!sils.isNull(ind_it) && sils_.count(sils.get(ind_it)))
//            {
//                sils_.insert(sils.get(ind_it));
//                has_adsb_info_ = true;
//            }
//        }
//    }
//}

/**
 */
void EvaluationTargetData::calculateTestDataMappings() const
{
    logdbg << "utn " << utn_;

    traced_assert(!tst_data_mappings_.size());

    tst_data_mappings_.resize(tst_chain_.size());

    for (auto& tst_it : tst_chain_.timestampIndexes())
    {
        tst_data_mappings_[tst_it.second.idx_internal] = ref_chain_.calculateDataMapping(tst_it.first);
    }

    unsigned int cnt = 0;

    for (const auto& tst_map_it : tst_data_mappings_)
    {
        if (tst_map_it.has_ref_pos_)
            ++cnt;
    }

    logdbg << "utn " << utn_ << " done, num map "
           << tst_data_mappings_.size() << " ref pos " << cnt;
}

/**
*/
void EvaluationTargetData::computeSectorInsideInfo() const
{
    inside_ref_           = {};
    inside_tst_           = {};
    inside_map_           = {};
    inside_sector_layers_ = {};

    traced_assert(calculator_.sectorsLoaded());

    auto sector_layers = calculator_.sectorLayers();

    //store sector layers
    {
        size_t cnt = 0;
        for (const auto& sl : sector_layers)
            inside_sector_layers_[ sl.get() ] = cnt++;
    }

    const SectorLayer* min_height_layer_ptr = nullptr;

    if (calculator_.filterMinimumHeight())
        min_height_layer_ptr = calculator_.minHeightFilterLayer().get();

    size_t num_sector_layers = sector_layers.size();
    size_t num_ref           = ref_chain_.size();
    size_t num_tst           = tst_chain_.size();
    size_t num_map           = tst_data_mappings_.size();

    size_t num_extra         = 2; //ground bit available / above min height filter

    size_t num_cols          = num_sector_layers + num_extra;

    traced_assert(num_map == num_tst);

    auto gb_max_sec = boost::posix_time::seconds(InterpGroundBitMaxSeconds);

    if (num_ref)
    {
        //check ref data for inside
        inside_ref_.resize(num_ref, num_cols);
        inside_ref_.setZero();

        for (const auto& elem : ref_chain_.timestampIndexes())
        {
            DataID id(elem.first, elem.second);

            auto pos = ref_chain_.pos(id);
            auto gb  = availableRefGroundBit(id, gb_max_sec);

            computeSectorInsideInfo(inside_ref_, pos, elem.second.idx_internal, gb, min_height_layer_ptr);
        }
    }
    if (num_tst)
    {
        //check test data for inside
        inside_tst_.resize(num_tst, num_cols);
        inside_tst_.setZero();

        for (const auto& elem : tst_chain_.timestampIndexes())
        {
            DataID id(elem.first, elem.second);

            auto pos = tst_chain_.pos(id);
            auto gb  = availableTstGroundBit(id, gb_max_sec);

            computeSectorInsideInfo(inside_tst_, pos, elem.second.idx_internal, gb, min_height_layer_ptr);
        }
    }
    if (num_map)
    {
        //check mapped positions for inside
        inside_map_.resize(num_tst, num_cols);
        inside_map_.setZero();

        for (const auto& elem : tst_chain_.timestampIndexes())
        {
            DataID id(elem.first, elem.second);

            auto pos = mappedRefPos(id);
            if (pos.has_value())
            {
                auto gb = availableTstGroundBit(id, gb_max_sec);
                computeSectorInsideInfo(inside_map_, pos.value(), elem.second.idx_internal, gb, min_height_layer_ptr);
            }
        }
    }
}

/**
 */
boost::optional<bool> EvaluationTargetData::availableRefGroundBit(
        const Chain::DataID& id, const boost::posix_time::time_duration& d_max) const
{
    auto index = ref_chain_.indexFromDataID(id);
    auto ts    = ref_chain_.timestampFromDataID(id);

    DataID indexed_id = DataID(ts, index);

    auto gbs = ref_chain_.groundBit(indexed_id);

    if (gbs.has_value() && *gbs)
        return gbs;

    return tstGroundBitInterpolated(ts, d_max);
}

/**
 */
boost::optional<bool> EvaluationTargetData::availableTstGroundBit(
        const Chain::DataID& id, const boost::posix_time::time_duration& d_max) const
{
    auto index = tst_chain_.indexFromDataID(id);
    auto ts    = tst_chain_.timestampFromDataID(id);

    DataID indexed_id = DataID(ts, index);

    auto gbs = ref_chain_.groundBit(indexed_id);

    if (gbs.has_value() && *gbs)
        return gbs;

    return mappedRefGroundBit(indexed_id, d_max);
}

/**
*/
void EvaluationTargetData::computeSectorInsideInfo(InsideCheckMatrix& mat, 
                                                   const dbContent::TargetPosition& pos,
                                                   unsigned int idx_internal,
                                                   const boost::optional<bool>& ground_bit,
                                                   const SectorLayer* min_height_filter) const
{
    traced_assert(idx_internal < mat.rows());

    size_t num_sector_layers = inside_sector_layers_.size();
    size_t extra_offset      = num_sector_layers;

    bool has_gb = ground_bit.has_value();
    bool gb_set = ground_bit.has_value() ? ground_bit.value() : false;

    //check airspace if enabled
    bool above_ok = true;
    if (min_height_filter)
    {
        auto res = AirSpace::isAbove(min_height_filter,
                                     pos,
                                     has_gb,
                                     gb_set);

        if (res == AirSpace::AboveCheckResult::Below)
            above_ok = false;
    }

    // calc if insice test sensor coverage, true if not circles
    bool inside_cov = calculator_.tstSrcsCoverage().isInside(pos.latitude_, pos.longitude_);

    // check sector layers
    for (const auto& sl : inside_sector_layers_)
    {
        auto layer = sl.first;
        traced_assert(layer);

        auto lidx = sl.second;

        if (!inside_cov) // outside test sensor coverage
            mat(idx_internal, lidx) = false;
        else // check with sectors
        {
            bool pos_inside = above_ok && layer->isInside(pos, has_gb, gb_set);

            //check pos against layer and write to mat
            mat(idx_internal, lidx) = pos_inside;
        }
    }

    mat(idx_internal, extra_offset     ) = has_gb; // TODO unsused
    mat(idx_internal, extra_offset + 1 ) = above_ok; // TODO unsused
}

/**
*/
// bool EvaluationTargetData::refPosAbove(const DataID& id) const
// {
//     auto index = ref_chain_.indexFromDataID(id);

//     return checkAbove(inside_ref_, index);
// }

// /**
// */
// bool EvaluationTargetData::refPosGroundBitAvailable(const DataID& id) const
// {
//     auto index = ref_chain_.indexFromDataID(id);

//     return checkGroundBit(inside_ref_, index);
// }

/**
*/
bool EvaluationTargetData::refPosInside(const SectorLayer& layer, 
                                        const DataID& id) const
{
    auto index = ref_chain_.indexFromDataID(id);

    return checkInside(layer, inside_ref_, index);
}

/**
*/
// bool EvaluationTargetData::tstPosAbove(const DataID& id) const
// {
//     auto index = tst_chain_.indexFromDataID(id);

//     return checkAbove(inside_tst_, index);
// }

// /**
// */
// bool EvaluationTargetData::tstPosGroundBitAvailable(const DataID& id) const
// {
//     auto index = tst_chain_.indexFromDataID(id);

//     return checkGroundBit(inside_tst_, index);
//}

/**
*/
bool EvaluationTargetData::tstPosInside(const SectorLayer& layer, 
                                        const DataID& id) const
{
    auto index = tst_chain_.indexFromDataID(id);

    return checkInside(layer, inside_tst_, index);
}

/**
*/
// bool EvaluationTargetData::mappedRefPosAbove(const DataID& id) const
// {
//     auto index = tst_chain_.indexFromDataID(id); // really tst

//     return checkAbove(inside_map_, index);
// }

// /**
// */
// bool EvaluationTargetData::mappedRefPosGroundBitAvailable(const DataID& id) const
// {
//     auto index = tst_chain_.indexFromDataID(id);

//     return checkGroundBit(inside_map_, index);
// }

/**
*/
bool EvaluationTargetData::mappedRefPosInside(const SectorLayer& layer, 
                                              const DataID& id) const
{
    auto index = tst_chain_.indexFromDataID(id);

    return checkInside(layer, inside_map_, index);
}

/**
*/
// bool EvaluationTargetData::checkGroundBit(const InsideCheckMatrix& mat,
//                                           const Index& index) const
// {
//     //check cached inside info
//     auto idx_internal = index.idx_internal;
//     traced_assert(idx_internal < mat.rows());

//     auto extra_offset = (int)inside_sector_layers_.size();
//     traced_assert(extra_offset < mat.cols());

//     return mat(idx_internal, extra_offset);
// }

// /**
// */
// bool EvaluationTargetData::checkAbove(const InsideCheckMatrix& mat,
//                                       const Index& index) const
// {
//     //check cached inside info
//     auto idx_internal = index.idx_internal;
//     traced_assert(idx_internal < mat.rows());

//     auto extra_offset = (int)inside_sector_layers_.size();
//     traced_assert(extra_offset + 1 < mat.cols());

//     return mat(idx_internal, extra_offset + 1);
// }

/**
*/
bool EvaluationTargetData::checkInside(const SectorLayer& layer,
                                       const InsideCheckMatrix& mat,
                                       const Index& index) const
{
    auto lit = inside_sector_layers_.find(&layer);
    traced_assert(lit != inside_sector_layers_.end());
    
    //check cached inside info
    auto idx_internal = index.idx_internal;
    traced_assert(idx_internal < mat.rows());

    auto lidx = (int)lit->second;
    traced_assert(lidx < mat.cols());

    return mat(idx_internal, lidx);
}

const std::set<std::string>& EvaluationTargetData::excludedRequirements() const
{
    return excluded_requirements_;
}

const Utils::TimeWindowCollection& EvaluationTargetData::excludedTimeWindows() const
{
    return excluded_time_windows_;
}

bool EvaluationTargetData::isTimeStampNotExcluded(const boost::posix_time::ptime& ts) const
{
    // check global, begin/end are excluded by load filter
    if (eval_man_.useTimestampFilter() && eval_man_.excludedTimeWindows().contains(ts))
        return false;

    return !excluded_time_windows_.contains(ts);
}

/**
 */
EvaluationTarget EvaluationTargetData::toTarget() const
{
    EvaluationTarget target(utn_, nlohmann::json());

    const auto& dbcontent_ref = refChain().dbContent();
    const auto& dbcontent_tst = tstChain().dbContent();

    target.comment(dbcont_man_.utnComment(utn_));

    target.useInEval(use());
    target.evalExcludedRequirements(excludedRequirements());
    target.evalExcludedTimeWindows(excludedTimeWindows());

    target.timeBegin(timeBegin());
    target.timeEnd(timeEnd());
    target.aircraftAddresses(acads());
    target.aircraftIdentifications(acids());
    target.modeACodes(modeACodes());
    if (has_mode_c_) target.modeCMinMax(modeCMin(), modeCMax());
    target.dbContentCount(dbcontent_ref, numRefUpdates());
    target.dbContentCount(dbcontent_tst, numTstUpdates());
    //if (hasMOPSVersion()) target.adsbMOPSVersions(mopsVersions());
    if (hasPos()) target.setPositionBounds(latitudeMin(), latitudeMax(), longitudeMin(), longitudeMax());
    target.targetCategory(dbcont_man_.emitterCategory(utn_));
    target.interestFactors(interestFactors());
    target.dbContentRef(dbcontent_ref);
    target.dbContentTest(dbcontent_tst);

    return target;
}

/**
 */
void EvaluationTargetData::updateTarget(DBContentManager& dbcontent_manager,
                                        EvaluationTarget& target)
{
    target.comment(dbcontent_manager.utnComment(target.utn_));
}
