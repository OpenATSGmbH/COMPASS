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

#pragma once

#include "bufferaccessor.h"

#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class DBContentManager;

namespace dbContent 
{

namespace targetReport
{
    class Position;
    class PositionAccuracy;
    class BarometricAltitude;
    class Velocity;
    class VelocityAccuracy;
    class ModeACode;
}

/**
*/
class TargetReportAccessor : public BufferAccessor
{
public:
    TargetReportAccessor(const std::string& dbcontent_name, 
                         const std::shared_ptr<Buffer>& buffer,
                         const DBContentManager& dbcont_man);
    TargetReportAccessor(const std::shared_ptr<DBContentVariableLookup>& lookup);
    virtual ~TargetReportAccessor() = default;

    // have to be always present, not optional
    boost::posix_time::ptime timestamp(unsigned int index) const;
    unsigned long recordNumber(unsigned int index) const;
    unsigned int dsID(unsigned int index) const;
    unsigned int lineID(unsigned int index) const;

    boost::optional<unsigned char> mopsVersion(unsigned int index) const;
    boost::optional<unsigned char> nucp(unsigned int index) const;
    boost::optional<unsigned char> nacp(unsigned int index) const;
    boost::optional<unsigned int> ecat(unsigned int index) const;
    boost::optional<unsigned char> getGeoAltAcc(unsigned int index) const;

    boost::optional<unsigned int> acad(unsigned int index) const;
    boost::optional<std::string> acid(unsigned int index) const;

    boost::optional<targetReport::Position> position(unsigned int index) const;
    boost::optional<targetReport::PositionAccuracy> positionAccuracy(unsigned int index) const;
    boost::optional<targetReport::BarometricAltitude> barometricAltitude(unsigned int index) const;
    boost::optional<float> geometricAltitude(unsigned int index) const;
    boost::optional<double> radarRange(unsigned int index) const;
    boost::optional<double> radarAzimuth(unsigned int index) const;
    boost::optional<targetReport::Velocity> velocity(unsigned int index) const;
    boost::optional<targetReport::VelocityAccuracy> velocityAccuracy(unsigned int index) const;
    boost::optional<double> trackAngle(unsigned int index) const;
    boost::optional<bool> groundBit(unsigned int index) const;
    
    boost::optional<targetReport::ModeACode> modeACode(unsigned int index) const;
    boost::optional<unsigned int> modeA(unsigned int index) const;
    boost::optional<bool> modeAValid(unsigned int index) const;
    boost::optional<bool> modeAGarbled(unsigned int index) const;
    boost::optional<bool> modeASmoothed(unsigned int index) const;

    boost::optional<float> modeC(unsigned int index) const;
    boost::optional<bool> modeCValid(unsigned int index) const;
    boost::optional<bool> modeCGarbled(unsigned int index) const;

    boost::optional<unsigned int> trackNumber(unsigned int index) const;
    boost::optional<bool> trackBegin(unsigned int index) const;
    boost::optional<bool> trackEnd(unsigned int index) const;

    static dbContent::VariableSet getReadSetFor(const std::string& dbcontent_name);

private:
    void cacheVectors();

    template <typename T>
    const NullableVector<T>* varVector(const Property& var_property)
    {
        return hasVar<T>(var_property) ? &getVar<T>(var_property) : nullptr;
    }
    template <typename T>
    const NullableVector<T>* metaVarVector(const Property& metavar_property)
    {
        return hasMetaVar<T>(metavar_property) ? &getMetaVar<T>(metavar_property) : nullptr;
    }
    template <typename T>
    boost::optional<T> getOptional(const NullableVector<T>* vec, unsigned int index) const
    {
        if (!vec || vec->isNull(index))
            return boost::optional<T>();
        else
            return vec->get(index);
    }

    template <typename T>
    T getNotOptional(const NullableVector<T>* vec, unsigned int index) const
    {
        traced_assert(vec);
        traced_assert(!vec->isNull(index));
        return vec->get(index);
    }

    bool is_radar_    = false;
    bool is_adsb_     = false;
    bool is_tracker_  = false;
    bool is_ref_traj_ = false;

    //general
    const NullableVector<boost::posix_time::ptime>* meta_timestamp_vec_ = nullptr;
    const NullableVector<unsigned long>*            meta_rec_num_vec_   = nullptr;
    const NullableVector<unsigned int>*             meta_ds_id_vec_     = nullptr;
    const NullableVector<unsigned int>*             meta_line_id_vec_   = nullptr;
    const NullableVector<unsigned int>*             meta_acad_vec_      = nullptr;
    const NullableVector<std::string>*              meta_acid_vec_      = nullptr;

    //position
    const NullableVector<double>* meta_latitude_vec_      = nullptr;
    const NullableVector<double>* meta_longitude_vec_     = nullptr;
    const NullableVector<float>*  cat062_alt_trusted_vec_ = nullptr;
    const NullableVector<float>*  cat062_alt_sec_vec_     = nullptr;
    const NullableVector<float>*  cat021_alt_geo_vec_     = nullptr;
    const NullableVector<bool>*   meta_ground_bit_vec_    = nullptr;

    const NullableVector<double>* meta_radar_range_vec_    = nullptr;
    const NullableVector<double>* meta_radar_azimuth_vec_  = nullptr;
 
    //position accuracy
    const NullableVector<unsigned char>* cat021_mops_version_vec_            = nullptr;
    const NullableVector<unsigned char>* cat021_nac_p_vec_                   = nullptr;
    const NullableVector<unsigned char>* cat021_nucp_nic_vec_                = nullptr;
    const NullableVector<double>*        meta_pos_std_dev_x_m_vec_           = nullptr;
    const NullableVector<double>*        meta_pos_std_dev_y_m_vec_           = nullptr;
    const NullableVector<double>*        meta_pos_std_dev_xy_corr_coeff_vec_ = nullptr;

    //velocity / angle
    const NullableVector<double>* meta_speed_vec_       = nullptr;
    const NullableVector<double>* meta_track_angle_vec_ = nullptr;

    //velocity accuracy
    const NullableVector<unsigned char>* cat021_nucv_nacv_vec_ = nullptr;
    const NullableVector<double>*        cat062_vx_stddev_vec_ = nullptr;
    const NullableVector<double>*        cat062_vy_stddev_vec_ = nullptr;

    //mode a
    const NullableVector<unsigned int>* meta_mode_a_vec_          = nullptr;
    const NullableVector<bool>*         meta_mode_a_garbled_vec_  = nullptr;
    const NullableVector<bool>*         meta_mode_a_valid_vec_    = nullptr;
    const NullableVector<bool>*         meta_mode_a_smoothed_vec_ = nullptr;

    //mode c
    const NullableVector<float>* meta_mode_c_vec_         = nullptr;
    const NullableVector<bool>*  meta_mode_c_garbled_vec_ = nullptr;
    const NullableVector<bool>*  meta_mode_c_valid_vec_   = nullptr;

    //track
    const NullableVector<unsigned int>* meta_track_num_vec_   = nullptr;
    const NullableVector<bool>*         meta_track_begin_vec_ = nullptr;
    const NullableVector<bool>*         meta_track_end_vec_   = nullptr;

    const NullableVector<unsigned int>* cat021_ecat_vec_ = nullptr;
    const NullableVector<unsigned char>* cat021_geo_alt_acc_vec_ = nullptr;
};

} // namespace dbContent
