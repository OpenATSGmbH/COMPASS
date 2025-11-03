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

#include "targetreportchain.h"
#include "targetposition.h"
#include "targetvelocity.h"
//#include "dbcontent/variable/variable.h"
//#include "dbcontent/variable/metavariable.h"
//#include "dbcontent/dbcontentmanager.h"
#include "dbcontent.h"
#include "dbcontentaccessor.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "global.h"

#include <algorithm>

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace Utils;
using namespace boost::posix_time;

namespace dbContent {

namespace TargetReport {

Chain::Chain(std::shared_ptr<dbContent::DBContentAccessor> accessor, const std::string& dbcontent_name)
    : accessor_(accessor), dbcontent_name_(dbcontent_name)
{

}

Chain::~Chain()
{

}

void Chain::addIndex (boost::posix_time::ptime timestamp, unsigned int index)
{
    unsigned int idx_int = (unsigned int)indexes_.size();

    timestamp_index_lookup_.insert({timestamp, Index(index, idx_int)});
    indexes_.push_back(index);
}

bool Chain::hasData() const
{
    return !timestamp_index_lookup_.empty();
}

void Chain::finalize () const
{
    updateACIDs();
    updateACADs();
    updateModeACodes();
    updateModeCMinMax();
    updatePositionMinMax();
}

unsigned int Chain::size () const
{
    return timestamp_index_lookup_.size();
}

unsigned int Chain::ignoredSize() const
{
    if (ignored_positions_.has_value())
    {
        unsigned int num_ignored = 0;

        for (bool ignored : ignored_positions_.value())
            if (ignored)
                ++num_ignored;

        return num_ignored;
    }
    return 0;
}

ptime Chain::timeBegin() const
{
    if (timestamp_index_lookup_.size())
        return timestamp_index_lookup_.begin()->first;
    else
        throw std::runtime_error("Chain: timeBegin: no data");
}

std::string Chain::timeBeginStr() const
{
    if (hasData())
        return Time::toString(timeBegin());
    else
        return "";
}

ptime Chain::timeEnd() const
{
    if (timestamp_index_lookup_.size())
        return timestamp_index_lookup_.rbegin()->first;
    else
        throw std::runtime_error("Chain: timeEnd: no data");
}

std::string Chain::timeEndStr() const
{
    if (hasData())
        return Time::toString(timeEnd());
    else
        return "";
}

time_duration Chain::timeDuration() const
{
    if (hasData())
        return timeEnd() - timeBegin();
    else
        return {};
}

std::set<unsigned int> Chain::modeACodes() const
{
    return mode_a_codes_;
}

std::string Chain::modeACodesStr() const
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

bool Chain::hasModeC() const
{
    return has_mode_c_;
}

float Chain::modeCMin() const
{
    traced_assert(has_mode_c_);
    return mode_c_min_;
}

std::string Chain::modeCMinStr() const
{
    if (has_mode_c_)
        return to_string((int)mode_c_min_);
    else
        return "";
}

float Chain::modeCMax() const
{
    traced_assert(has_mode_c_);
    return mode_c_max_;
}

std::string Chain::modeCMaxStr() const
{
    if (has_mode_c_)
        return to_string((int)mode_c_max_);
    else
        return "";
}

bool Chain::isPrimaryOnly () const
{
    return !acids_.size() && !acads_.size() && !mode_a_codes_.size() && !has_mode_c_;
}

const Chain::IndexMap& Chain::timestampIndexes() const
{
    return timestamp_index_lookup_;
}

double Chain::latitudeMin() const
{
    traced_assert(has_pos_);
    return latitude_min_;
}

double Chain::latitudeMax() const
{
    traced_assert(has_pos_);
    return latitude_max_;
}

double Chain::longitudeMin() const
{
    traced_assert(has_pos_);
    return longitude_min_;
}

double Chain::longitudeMax() const
{
    traced_assert(has_pos_);
    return longitude_max_;
}

bool Chain::hasPos() const
{
    return has_pos_;
}

Chain::DataID Chain::dataID(const boost::posix_time::ptime& timestamp) const
{
    const IndexMap& index_map = timestamp_index_lookup_;

    auto range = index_map.equal_range(timestamp);

    traced_assert(range.first != index_map.end());

    traced_assert(range.first->second.idx_internal < index_map.size());

    return DataID(timestamp).addIndex(range.first->second);
}

std::vector<DataID> Chain::dataIDsBetween(const boost::posix_time::ptime& timestamp0,
                                          const boost::posix_time::ptime& timestamp1,
                                          bool include_t0,
                                          bool include_t1) const
{
    traced_assert(timestamp0 <= timestamp1);
    auto it_start = timestamp_index_lookup_.lower_bound(timestamp0);

    if (it_start == timestamp_index_lookup_.end())
        return {};

    std::vector<DataID> ids;

    for (auto it = it_start; it != timestamp_index_lookup_.end(); ++it)
    {
        bool ok0 = (include_t0 && it->first >= timestamp0) || (!include_t0 && it->first > timestamp0);
        bool ok1 = (include_t1 && it->first <= timestamp1) || (!include_t1 && it->first < timestamp1);
        
        if(ok0 && ok1)
            ids.push_back(DataID(it->first).addIndex(it->second));
    }

    return ids;
}

unsigned int Chain::dsID(const DataID& id) const
{
    auto index     = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<unsigned int>& dsid_vec =
            accessor_->getMetaVar<unsigned int>(dbcontent_name_, DBContent::meta_var_ds_id_);

    traced_assert(!dsid_vec.isNull(index_ext));

    return dsid_vec.get(index_ext);
}

dbContent::TargetPosition Chain::pos(const DataID& id) const
{
    auto timestamp = timestampFromDataID(id);

    traced_assert(timestamp_index_lookup_.count(timestamp));

    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    dbContent::TargetPosition pos;

    NullableVector<double>& latitude_vec  = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_latitude_);
    NullableVector<double>& longitude_vec = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_longitude_);
    NullableVector<float>& altitude_vec   = accessor_->getMetaVar<float>(dbcontent_name_, DBContent::meta_var_mc_);

    NullableVector<float>* altitude_trusted_vec {nullptr};
    NullableVector<float>* altitude_secondary_vec {nullptr};

    if (dbcontent_name_ == "CAT062")
    {
        altitude_trusted_vec = &accessor_->getVar<float>(dbcontent_name_, DBContent::var_cat062_fl_measured_);
        altitude_secondary_vec = &accessor_->getVar<float>(dbcontent_name_, DBContent::var_cat062_baro_alt_);
    }

    traced_assert(!latitude_vec.isNull(index_ext));
    traced_assert(!longitude_vec.isNull(index_ext));

    pos.latitude_  = latitude_vec.get(index_ext);
    pos.longitude_ = longitude_vec.get(index_ext);

    if (altitude_trusted_vec && !altitude_trusted_vec->isNull(index_ext))
    {
        pos.has_altitude_        = true;
        pos.altitude_calculated_ = false;
        pos.altitude_            = altitude_trusted_vec->get(index_ext);
    }
    else if (!altitude_vec.isNull(index_ext))
    {
        pos.has_altitude_        = true;
        pos.altitude_calculated_ = false;
        pos.altitude_            = altitude_vec.get(index_ext);
    }
    else if (altitude_secondary_vec && !altitude_secondary_vec->isNull(index_ext))
    {
        pos.has_altitude_        = true;
        pos.altitude_calculated_ = true;
        pos.altitude_            = altitude_secondary_vec->get(index_ext);
    }
    else // calculate
    {
        bool found;
        float alt_calc;

        tie(found,alt_calc) = estimateAltitude(timestamp, index.idx_internal);

        if (found)
        {
            pos.has_altitude_        = true;
            pos.altitude_calculated_ = true;
            pos.altitude_            = alt_calc;
        }
    }

    return pos;
}

std::vector<TargetPosition> Chain::positionsBetween(const boost::posix_time::ptime& timestamp0,
                                                    const boost::posix_time::ptime& timestamp1,
                                                    bool include_t0,
                                                    bool include_t1) const
{
    auto ids = dataIDsBetween(timestamp0, timestamp1, include_t0, include_t1);

    std::vector<TargetPosition> positions(ids.size());

    for (size_t i = 0; i < ids.size(); ++i)
        positions[ i ] = pos(ids[ i ]);

    return positions;
}

boost::optional<TargetPosition> Chain::posOpt(const DataID& id) const
{
    auto timestamp = timestampFromDataID(id);

    if (!timestamp_index_lookup_.count(timestamp))
        return {};
    else
        return pos(id);
}

boost::optional<TargetPositionAccuracy> Chain::posAccuracy(const DataID& id) const
{
    //auto timestamp = timestampFromDataID(id);
    auto index     = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    return getPositionAccuracy(accessor_, dbcontent_name_, index_ext);
}

boost::optional<dbContent::TargetVelocity> Chain::speed(const DataID& id) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<double>& speed_vec = accessor_->getMetaVar<double>(
                dbcontent_name_, DBContent::meta_var_ground_speed_);
    NullableVector<double>& track_angle_vec = accessor_->getMetaVar<double>(
                dbcontent_name_, DBContent::meta_var_track_angle_);

    if (speed_vec.isNull(index_ext) || track_angle_vec.isNull(index_ext))
        return {};

    traced_assert(!speed_vec.isNull(index_ext));
    traced_assert(!track_angle_vec.isNull(index_ext));

    dbContent::TargetVelocity spd;

    spd.speed_ = speed_vec.get(index_ext) * KNOTS2M_S; // true north to mathematical
    spd.track_angle_ = track_angle_vec.get(index_ext);

    return spd;
}

boost::optional<TargetVelocityAccuracy> Chain::speedAccuracy(const DataID& id) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    return getVelocityAccuracy(accessor_, dbcontent_name_, index_ext);
}

boost::optional<std::string> Chain::acid(const DataID& id) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<string>& callsign_vec = accessor_->getMetaVar<string>(dbcontent_name_, DBContent::meta_var_acid_);

    if (callsign_vec.isNull(index_ext))
        return {};

    traced_assert(!callsign_vec.isNull(index_ext));

    return boost::trim_copy(callsign_vec.get(index_ext)); // remove spaces
}

boost::optional<unsigned int> Chain::modeA(const DataID& id, bool ignore_invalid, bool ignore_garbled) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<unsigned int>& modea_vec = accessor_->getMetaVar<unsigned int>(
                dbcontent_name_, DBContent::meta_var_m3a_);

    if (modea_vec.isNull(index_ext))
        return {};

    if (ignore_invalid && accessor_->hasMetaVar<bool>(dbcontent_name_, DBContent::meta_var_m3a_v_))
    {
        NullableVector<bool>& modea_v_vec = accessor_->getMetaVar<bool>(dbcontent_name_, DBContent::meta_var_m3a_v_);

        if (!modea_v_vec.isNull(index_ext) && !modea_v_vec.get(index_ext)) // not valid
            return {};
    }

    if (ignore_garbled && accessor_->hasMetaVar<bool>(dbcontent_name_, DBContent::meta_var_m3a_g_))
    {
        NullableVector<bool>& modea_g_vec = accessor_->getMetaVar<bool>(dbcontent_name_, DBContent::meta_var_m3a_g_);

        if (!modea_g_vec.isNull(index_ext) && modea_g_vec.get(index_ext)) // garbled
            return {};
    }

    traced_assert(!modea_vec.isNull(index_ext));

    return modea_vec.get(index_ext);
}

boost::optional<float> Chain::modeC(const DataID& id, bool ignore_invalid, bool ignore_garbled) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    if (dbcontent_name_ == "CAT062")
    {
        NullableVector<float>& altitude_trusted_vec = accessor_->getVar<float>(
                    dbcontent_name_, DBContent::var_cat062_fl_measured_);

        if (!altitude_trusted_vec.isNull(index_ext))
            return altitude_trusted_vec.get(index_ext);
    }

    NullableVector<float>& modec_vec = accessor_->getMetaVar<float>(dbcontent_name_, DBContent::meta_var_mc_);

    if (modec_vec.isNull(index_ext))
        return {};

    if (ignore_invalid && accessor_->hasMetaVar<bool>(dbcontent_name_, DBContent::meta_var_mc_v_))
    {
        NullableVector<bool>& mc_v_vec = accessor_->getMetaVar<bool>(dbcontent_name_, DBContent::meta_var_mc_v_);

        if (!mc_v_vec.isNull(index_ext) && !mc_v_vec.get(index_ext)) // not valid
            return {};
    }

    if (ignore_garbled && accessor_->hasMetaVar<bool>(dbcontent_name_, DBContent::meta_var_mc_g_))
    {
        NullableVector<bool>& modec_g_vec = accessor_->getMetaVar<bool>(dbcontent_name_, DBContent::meta_var_mc_g_);

        if (!modec_g_vec.isNull(index_ext) && modec_g_vec.get(index_ext)) // garbled
            return {};
    }

    return modec_vec.get(index_ext);
}

boost::optional<unsigned int> Chain::acad(const DataID& id) const
{
    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<unsigned int>& ta_vec = accessor_->getMetaVar<unsigned int>(
                dbcontent_name_, DBContent::meta_var_acad_);

    if (ta_vec.isNull(index_ext))
        return {};

    return ta_vec.get(index_ext);
}

boost::optional<bool> Chain::groundBit(const DataID& id) const
{
    if (!accessor_->hasMetaVar<bool>(dbcontent_name_, DBContent::meta_var_ground_bit_))
        return {};

    auto index = indexFromDataID(id);

    unsigned int index_ext = index.idx_external;

    NullableVector<bool>& db_vec = accessor_->getMetaVar<bool>(
                dbcontent_name_, DBContent::meta_var_ground_bit_);

    if (db_vec.isNull(index_ext))
        return {};

    return db_vec.get(index_ext);
}

boost::optional<unsigned int> Chain::tstTrackNum(const DataID& id) const
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<unsigned int>(dbcontent_name_, DBContent::meta_var_track_num_))
        return {};

    NullableVector<unsigned int>& tn_vec = accessor_->getMetaVar<unsigned int>(
                dbcontent_name_, DBContent::meta_var_track_num_);

    if (tn_vec.isNull(index_ext))
        return {};

    return tn_vec.get(index_ext);
}

boost::optional<float> Chain::groundSpeed(const DataID& id) const // m/s
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<double>(dbcontent_name_, DBContent::meta_var_ground_speed_))
        return {};

    NullableVector<double>& gs_vec = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_ground_speed_);

    if (gs_vec.isNull(index_ext))
        return {};

    return gs_vec.get(index_ext) * KNOTS2M_S; // kts 2 m/s
}

boost::optional<float> Chain::trackAngle(const DataID& id) const // deg
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<double>(dbcontent_name_, DBContent::meta_var_track_angle_))
        return {};

    NullableVector<double>& vec = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_track_angle_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

boost::optional<double> Chain::acceleration(const DataID& id) const // m/s2
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<double>(dbcontent_name_, DBContent::meta_var_ax_)
        || !accessor_->hasMetaVar<double>(dbcontent_name_, DBContent::meta_var_ay_))
        return {};

    NullableVector<double>& ax_vec = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_ax_);
    NullableVector<double>& ay_vec = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_ay_);

    if (ax_vec.isNull(index_ext) || ay_vec.isNull(index_ext))
        return {};

    return sqrt(pow(ax_vec.get(index_ext), 2) + pow(ay_vec.get(index_ext), 2));
}

boost::optional<float> Chain::rocd(const DataID& id) const // ft/min
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<float>(dbcontent_name_, DBContent::meta_var_rocd_))
        return {};

    NullableVector<float>& vec = accessor_->getMetaVar<float>(dbcontent_name_, DBContent::meta_var_rocd_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

boost::optional<unsigned char> Chain::momLongAcc(const DataID& id) const
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<unsigned char>(dbcontent_name_, DBContent::meta_var_mom_long_acc_))
        return {};

    NullableVector<unsigned char>& vec = accessor_->getMetaVar<unsigned char>(
        dbcontent_name_, DBContent::meta_var_mom_long_acc_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

boost::optional<unsigned char> Chain::momTransAcc(const DataID& id) const
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<unsigned char>(dbcontent_name_, DBContent::meta_var_mom_trans_acc_))
        return {};

    NullableVector<unsigned char>& vec = accessor_->getMetaVar<unsigned char>(
        dbcontent_name_, DBContent::meta_var_mom_trans_acc_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

boost::optional<unsigned char> Chain::momVertRate(const DataID& id) const
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<unsigned char>(dbcontent_name_, DBContent::meta_var_mom_vert_rate_))
        return {};

    NullableVector<unsigned char>& vec = accessor_->getMetaVar<unsigned char>(
        dbcontent_name_, DBContent::meta_var_mom_vert_rate_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

boost::optional<unsigned char> Chain::trackCoasting(const DataID& id) const
{
    auto index = indexFromDataID(id);

    auto index_ext = index.idx_external;

    if (!accessor_->hasMetaVar<unsigned char>(dbcontent_name_, DBContent::meta_var_track_coasting_))
        return {};

    NullableVector<unsigned char>& vec = accessor_->getMetaVar<unsigned char>(
        dbcontent_name_, DBContent::meta_var_track_coasting_);

    if (vec.isNull(index_ext))
        return {};

    return vec.get(index_ext);
}

std::pair<bool, float> Chain::estimateAltitude (const boost::posix_time::ptime& timestamp,
                                                unsigned int index_internal) const
{
    traced_assert(index_internal < indexes_.size());

    NullableVector<float>& altitude_vec = accessor_->getMetaVar<float>(dbcontent_name_, DBContent::meta_var_mc_);
    NullableVector<ptime>& ts_vec = accessor_->getMetaVar<ptime>(dbcontent_name_, DBContent::meta_var_timestamp_);

    NullableVector<float>* altitude_trusted_vec {nullptr};

    if (dbcontent_name_ == "CAT062")
        altitude_trusted_vec = &accessor_->getVar<float>(dbcontent_name_, DBContent::var_cat062_fl_measured_);

    bool found_prev {false};
    float altitude_prev {0.0};
    bool found_after {false};
    float altitude_after {0.0};

    // search for prev index
    ptime timestamp_prev;
    auto prev_it  = indexes_.begin() + index_internal;
    auto after_it = prev_it;

    const time_duration max_tdiff = seconds(120);

    while (prev_it != indexes_.end() && timestamp - ts_vec.get(*prev_it) < max_tdiff)
    {
        if (altitude_trusted_vec && !altitude_trusted_vec->isNull(*prev_it))
        {
            found_prev = true;
            altitude_prev = altitude_trusted_vec->get(*prev_it);
            timestamp_prev = ts_vec.get(*prev_it);

            break;
        }
        else if (!altitude_vec.isNull(*prev_it))
        {
            found_prev = true;
            altitude_prev = altitude_vec.get(*prev_it);
            timestamp_prev = ts_vec.get(*prev_it);

            break;
        }

        if (prev_it == indexes_.begin()) // undefined decrement
            break;

        --prev_it;
    }

    // search after index
    ptime timestamp_after;

    while (after_it != indexes_.end() && ts_vec.get(*after_it) - timestamp < max_tdiff)
    {
        if (altitude_trusted_vec && !altitude_trusted_vec->isNull(*after_it))
        {
            found_after = true;
            altitude_after = altitude_trusted_vec->get(*after_it);
            timestamp_after = ts_vec.get(*after_it);

            break;
        }
        else if (!altitude_vec.isNull(*after_it))
        {
            found_after = true;
            altitude_after = altitude_vec.get(*after_it);
            timestamp_after = ts_vec.get(*after_it);

            break;
        }
        ++after_it;
    }

    if (found_prev && found_after)
    {
        if (timestamp_after == timestamp_prev || timestamp_prev == timestamp)
        { // all same time
            return {true, altitude_prev};
        }

        if (timestamp_after <= timestamp_prev || timestamp_prev >= timestamp)
        {
            logerr << "ts_prev " << Time::toString(timestamp_prev)
                   << " ts " << Time::toString(timestamp) << " ts_after " << Time::toString(timestamp_after);

            return {false, 0}; // should never happen
        }

        float d_alt_ft = altitude_after - altitude_prev;
        float d_t = Time::partialSeconds(timestamp_after - timestamp_prev);

        float alt_spd_ft_s = d_alt_ft/d_t;

        float d_t2 = Time::partialSeconds(timestamp - timestamp_prev);

        float alt_calc = altitude_prev + alt_spd_ft_s*d_t2;

        return {true, alt_calc};
    }
    else if (found_prev && timestamp - timestamp_prev < max_tdiff)
        return {true, altitude_prev};
    else if (found_after && timestamp_after - timestamp < max_tdiff)
        return {true, altitude_after};
    else
    {
        return {false, 0}; // none found
    }
}

Index Chain::indexFromDataID(const DataID& id) const
{
    traced_assert(id.valid());

    if (!id.hasIndex())
    {
        auto id_ret = dataID(id.timestamp());
        traced_assert(id_ret.valid());

        return id_ret.index();
    }

    return id.index();
}

boost::posix_time::ptime Chain::timestampFromDataID(const DataID& id) const
{
    traced_assert(id.valid());
    return id.timestamp();
}

DataMapping Chain::calculateDataMapping(ptime timestamp) const
{
    DataMapping ret;

    ret.timestamp_ = timestamp;

    if (timestamp_index_lookup_.count(timestamp)) // found exact time
    {
        ret.has_ref1_ = true;
        ret.timestamp_ref1_ = timestamp;
        ret.dataid_ref1_ = dataID(timestamp_index_lookup_.find(timestamp)->first);

        ret.has_ref2_ = false;

        addPositionsSpeedsToMapping(ret);

        return ret;
    }

    //    Return iterator to lower bound
    //    Returns an iterator pointing to the first element in the container whose key is not considered to go
    //    before k (i.e., either it is equivalent or goes after).

    auto lb_it = timestamp_index_lookup_.lower_bound(timestamp);

    //auto ub_it = ref_chain_->upper_bound(tod);

    if (lb_it != timestamp_index_lookup_.end()) // upper tod found
    {
        traced_assert(lb_it->first >= timestamp);

        // save upper value
        ret.has_ref2_ = true;
        ret.timestamp_ref2_ = lb_it->first;
        ret.dataid_ref2_ = dataID(lb_it->first);

        // search lower values by decrementing iterator
        while (lb_it != timestamp_index_lookup_.end()
               && (timestamp < lb_it->first || lb_it->first == ret.timestamp_ref2_))
        {
            if (lb_it == timestamp_index_lookup_.begin()) // exit condition on first value
            {
                if (timestamp < lb_it->first) // set as not found
                    lb_it = timestamp_index_lookup_.end();

                break;
            }

            lb_it--;
        }

        if (lb_it != timestamp_index_lookup_.end() && lb_it->first != ret.timestamp_ref2_) // lower tod found
        {
            traced_assert(timestamp >= lb_it->first);

            // add lower value
            ret.has_ref1_ = true;
            ret.timestamp_ref1_ = lb_it->first;
            ret.dataid_ref1_ = dataID(lb_it->first);
        }
        else // not found, clear previous
        {
            ret.has_ref2_ = false;
            ret.timestamp_ref2_ = {};
            ret.dataid_ref1_ = {};
        }
    }

    addPositionsSpeedsToMapping(ret);

    return ret;
}

void Chain::addPositionsSpeedsToMapping (DataMapping& mapping) const
{
    if (!mapping.has_ref1_ && !mapping.has_ref2_)
        return;

    if (mapping.has_ref1_ && !mapping.has_ref2_)
    {
        boost::optional<dbContent::TargetPosition> pos1 = posOpt(mapping.dataid_ref1_);

        if (!pos1.has_value())
            return;

        mapping.has_ref_pos_ = true;
        mapping.pos_ref_ = *pos1;

        boost::optional<dbContent::TargetVelocity> spd1 = speed(mapping.dataid_ref1_);

        if (spd1.has_value())
        {
            mapping.has_ref_spd_ = true;
            mapping.spd_ref_ = *spd1;
        }

        return;
    }

    boost::optional<dbContent::TargetPosition> pos1 = posOpt(mapping.dataid_ref1_);
    boost::optional<dbContent::TargetPosition> pos2 = posOpt(mapping.dataid_ref2_);

    if (!pos1.has_value() || !pos2.has_value())
        return;

    ptime lower_ts = mapping.timestamp_ref1_;
    ptime upper_ts = mapping.timestamp_ref2_;

    float d_t = Time::partialSeconds(upper_ts - lower_ts);

    boost::optional<dbContent::TargetVelocity> spd1;
    boost::optional<dbContent::TargetVelocity> spd2;

    double acceleration_ms2;
    double speed_ms, angle_deg;

    boost::optional<double> acc1, acc2;
    double delta_acc;
    boost::optional<double> rocd1, rocd2;
    double delta_rocd;

    logdbg << "d_t " << d_t;

    traced_assert(d_t > 0);

    if (pos1->latitude_ == pos2->latitude_ && pos1->longitude_ == pos2->longitude_) // same pos
    {
        mapping.has_ref_pos_ = true;
        mapping.pos_ref_ = *pos1;

        mapping.spd_ref_.track_angle_ = NAN;
        mapping.spd_ref_.speed_       = NAN;
    }
    else
    {
        if (lower_ts == upper_ts) // same time
        {
            logwrn << "ref has same time twice";
        }
        else
        {
            logdbg << "pos1 "
                   << pos1->latitude_ << ", " << pos1->longitude_;
            logdbg << "pos2 "
                   << pos2->latitude_ << ", " << pos2->longitude_;

            bool ok;
            double x_pos, y_pos;

            tie(ok, x_pos, y_pos) = trafo_.distanceCart(
                        pos1->latitude_, pos1->longitude_, pos2->latitude_, pos2->longitude_);

            //                logdbg << "geo2cart";
            //                bool ret = ogr_geo2cart->Transform(1, &x_pos, &y_pos); // wgs84 to cartesian offsets
            if (!ok)
            {
                logerr << "error with latitude " << pos2->latitude_
                       << " longitude " << pos2->longitude_;
            }
            else // calculate interpolated position
            {
                logdbg << "offsets x " << fixed << x_pos
                       << " y " << fixed << y_pos << " dist " << fixed << sqrt(pow(x_pos,2)+pow(y_pos,2));

                // double x_pos_orig = x_pos;
                // double y_pos_orig = y_pos;

                double v_x = x_pos/d_t;
                double v_y = y_pos/d_t;
                logdbg << "v_x " << v_x << " v_y " << v_y;

                float d_t2 = Time::partialSeconds(mapping.timestamp_ - lower_ts);
                logdbg << "d_t2 " << d_t2;

                traced_assert(d_t2 >= 0);

                x_pos = v_x * d_t2;
                y_pos = v_y * d_t2;

                logdbg << "interpolated offsets x "
                       << x_pos << " y " << y_pos;

                tie (ok, x_pos, y_pos) = trafo_.wgsAddCartOffset(pos1->latitude_, pos1->longitude_, x_pos, y_pos);

                // x_pos long, y_pos lat

                logdbg << "interpolated lat "
                       << x_pos << " long " << y_pos;

                // calculate altitude
                bool has_altitude = false;
                float altitude = 0.0;

                if (pos1->has_altitude_ && !pos2->has_altitude_)
                {
                    has_altitude = true;
                    altitude = pos1->altitude_;
                }
                else if (!pos1->has_altitude_ && pos2->has_altitude_)
                {
                    has_altitude = true;
                    altitude = pos2->altitude_;
                }
                else if (pos1->has_altitude_ && pos2->has_altitude_)
                {
                    float v_alt = (pos2->altitude_ - pos1->altitude_)/d_t;
                    has_altitude = true;
                    altitude = pos1->altitude_ + v_alt*d_t2;
                }

                logdbg << "pos1 has alt "
                       << pos1->has_altitude_ << " alt " << pos1->altitude_
                       << " pos2 has alt " << pos2->has_altitude_ << " alt " << pos2->altitude_
                       << " interpolated has alt " << has_altitude << " alt " << altitude;

                mapping.has_ref_pos_ = true;

                mapping.pos_ref_ = dbContent::TargetPosition(x_pos, y_pos, has_altitude, true, altitude);

                // calulcate interpolated speed / track angle

                mapping.has_ref_spd_ = false;

                spd1 = speed(mapping.dataid_ref1_);
                spd2 = speed(mapping.dataid_ref2_);

                if (spd1.has_value() && spd2.has_value())
                {
                    acceleration_ms2 = (spd2->speed_ - spd1->speed_)/d_t;
                    speed_ms = spd1->speed_ + acceleration_ms2 * d_t2;

                    //loginf << "UGA spd1 " << spd1.speed_ << " 2 " << spd2.speed_ << " ipld " << speed;

#if 1
                    double angle_diff = Number::calculateMinAngleDifference(spd2->track_angle_, spd1->track_angle_);
                    double turnrate   = angle_diff / d_t;

                    angle_deg = spd1->track_angle_ + turnrate * d_t2;
#else
                    angle_deg = Number::interpolateBearing(
                                0, 0, x_pos_orig, y_pos_orig, spd1->track_angle_, spd2->track_angle_, d_t2 / d_t);
#endif

                    //                        loginf << "UGA ang1 " << spd1.track_angle_ << " 2 " << spd2.track_angle_
                    //                               << " angle_diff " << angle_diff << " turnrate " << turnrate << " ipld " << angle;

                    mapping.has_ref_spd_          = true;
                    mapping.spd_ref_.speed_       = speed_ms;
                    mapping.spd_ref_.track_angle_ = angle_deg;
                }

                // acceleration
                acc1 = acceleration(mapping.dataid_ref1_);
                acc2 = acceleration(mapping.dataid_ref2_);

                if (acc1.has_value() && acc2.has_value())
                {
                    delta_acc = (*acc2 - *acc1)/d_t;

                    mapping.has_ref_acc_ = true;
                    mapping.ref_acc_ = *acc1 + delta_acc * d_t2;
                }

                // rocd
                rocd1 = acceleration(mapping.dataid_ref1_);
                rocd2 = acceleration(mapping.dataid_ref2_);

                if (rocd1.has_value() && rocd2.has_value())
                {
                    delta_rocd = (*rocd2 - *rocd1)/d_t;

                    mapping.has_ref_rocd_ = true;
                    mapping.ref_rocd_ = *rocd1 + delta_rocd * d_t2;
                }
            }
        }
    }
    // else do nothing
}

DataMappingTimes Chain::findDataMappingTimes(ptime timestamp_ref) const // ref tod
{
    DataMappingTimes ret;

    ret.timestamp_ = timestamp_ref;

    //    Return iterator to lower bound
    //    Returns an iterator pointing to the first element in the container whose key is not considered to go
    //    before k (i.e., either it is equivalent or goes after).

    auto lb_it = timestamp_index_lookup_.lower_bound(timestamp_ref);

    if (lb_it != timestamp_index_lookup_.end()) // upper tod found
    {
        traced_assert(lb_it->first >= timestamp_ref);

        // save upper value
        ret.has_other2_ = true;
        ret.timestamp_other2_ = lb_it->first;
        ret.dataid_other2_ = dataID(lb_it->first);

        // search lower values by decrementing iterator
        while (lb_it != timestamp_index_lookup_.end() && (timestamp_ref < lb_it->first || lb_it->first == ret.timestamp_other2_))
        {
            if (lb_it == timestamp_index_lookup_.begin()) // exit condition on first value
            {
                if (timestamp_ref < lb_it->first) // set as not found
                    lb_it = timestamp_index_lookup_.end();

                break;
            }

            lb_it--;
        }

        if (lb_it != timestamp_index_lookup_.end() && lb_it->first != ret.timestamp_other2_) // lower tod found
        {
            traced_assert(timestamp_ref >= lb_it->first);

            // add lower value
            ret.has_other1_ = true;
            ret.timestamp_other1_ = lb_it->first;
            ret.dataid_other1_ = dataID(lb_it->first);
        }
        else // not found, clear previous
        {
            ret.has_other2_ = false;
            ret.timestamp_other2_ = {};
            ret.dataid_other2_ = {};
        }
    }

    return ret;
}

void Chain::setIgnoredPositions(std::vector<bool> ignored_positions)
{
    traced_assert(indexes_.size() == ignored_positions.size());

    ignored_positions_ = ignored_positions;
}

bool Chain::ignorePosition(const DataID& id) const
{
    if (!ignored_positions_.has_value())
        return false;

    auto index  = indexFromDataID(id);

    traced_assert(index.idx_internal < ignored_positions_->size());

    return ignored_positions_->at(index.idx_internal);
}

std::set<string> Chain::acids() const
{
    return acids_;
}

string Chain::acidsStr() const
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

std::set<unsigned int> Chain::acads() const
{
    return acads_;
}

std::string Chain::acadsStr() const
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

void Chain::updateACIDs() const
{
    acids_.clear();

    if (timestamp_index_lookup_.size())
    {
        NullableVector<string>& value_vec = accessor_->getMetaVar<string>(dbcontent_name_, DBContent::meta_var_acid_);
        map<boost::optional<string>, vector<unsigned int>> distinct_values = value_vec.distinctValuesWithIndexes(indexes_);

        for (auto& val_it : distinct_values)
        {
            if (val_it.first && !acids_.count(String::trim(*val_it.first)))
                acids_.insert(String::trim(*val_it.first));
        }
    }
}

void Chain::updateACADs() const
{
    acads_.clear();

    if (timestamp_index_lookup_.size())
    {
        NullableVector<unsigned int>& value_vec = accessor_->getMetaVar<unsigned int>(
                    dbcontent_name_, DBContent::meta_var_acad_);
        map<boost::optional<unsigned int>, vector<unsigned int>> distinct_values = value_vec.distinctValuesWithIndexes(indexes_);

        for (auto& val_it : distinct_values)
        {
            if (val_it.first && !acads_.count(*val_it.first))
                acads_.insert(*val_it.first);
        }
    }

}

void Chain::updateModeACodes() const
{
    logdbg;

    mode_a_codes_.clear();

    if (timestamp_index_lookup_.size())
    {
        NullableVector<unsigned int>& mode_a_codes = accessor_->getMetaVar<unsigned int>(
                    dbcontent_name_, DBContent::meta_var_m3a_);
        map<boost::optional<unsigned int>, vector<unsigned int>> distinct_codes = mode_a_codes.distinctValuesWithIndexes(indexes_);
        //unsigned int null_cnt = mode_a_codes.nullValueIndexes(ref_rec_nums_).size();

        for (auto& ma_it : distinct_codes)
        {
            if (ma_it.first && !mode_a_codes_.count(*ma_it.first))
            {
                logdbg << "new ref m3a "
                       << String::octStringFromInt(*ma_it.first, 4, '0');
                mode_a_codes_.insert(*ma_it.first);
            }
        }
    }


    logdbg << "num codes " << mode_a_codes_.size();
}

void Chain::updateModeCMinMax() const
{
    logdbg;

    // garbled, valid flags?

    has_mode_c_ = false;
    float mode_c_value;

    if (timestamp_index_lookup_.size())
    {
        NullableVector<float>& modec_codes_ft = accessor_->getMetaVar<float>(dbcontent_name_, DBContent::meta_var_mc_);

        NullableVector<float>* altitude_trusted_vec {nullptr};

        if (dbcontent_name_ == "CAT062")
            altitude_trusted_vec = &accessor_->getVar<float>(dbcontent_name_, DBContent::var_cat062_fl_measured_);

        for (auto ind_it : indexes_)
        {
            if (altitude_trusted_vec && !altitude_trusted_vec->isNull(ind_it))
                mode_c_value = altitude_trusted_vec->get(ind_it);
            else if (!modec_codes_ft.isNull(ind_it))
                mode_c_value = modec_codes_ft.get(ind_it);
            else
                continue;

            // mode_c_value is set

            if (!has_mode_c_)
            {
                has_mode_c_ = true;
                mode_c_min_ = mode_c_value;
                mode_c_max_ = mode_c_value;
            }
            else
            {
                mode_c_min_ = min(mode_c_min_, mode_c_value);
                mode_c_max_ = max(mode_c_max_, mode_c_value);
            }
        }
    }


}

void Chain::updatePositionMinMax() const
{
    has_pos_ = false;

    if (timestamp_index_lookup_.size())
    {
        NullableVector<double>& lats = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_latitude_);
        NullableVector<double>& longs = accessor_->getMetaVar<double>(dbcontent_name_, DBContent::meta_var_longitude_);

        for (auto ind_it : indexes_)
        {
            traced_assert(!lats.isNull(ind_it));
            traced_assert(!longs.isNull(ind_it));

            if (!has_pos_)
            {
                has_pos_ = true;
                latitude_min_ = lats.get(ind_it);
                latitude_max_ = lats.get(ind_it);
                longitude_min_ = longs.get(ind_it);
                longitude_max_ = longs.get(ind_it);
            }
            else
            {
                latitude_min_ = min(latitude_min_, lats.get(ind_it));
                latitude_max_ = max(latitude_max_, lats.get(ind_it));
                longitude_min_ = min(longitude_min_, longs.get(ind_it));
                longitude_max_ = max(longitude_max_, longs.get(ind_it));
            }
        }
    }

}


} // namespace TargetReport

} // namespace dbContent
