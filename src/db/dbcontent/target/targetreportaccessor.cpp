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

#include "targetreportaccessor.h"
#include "dbcontent.h"
#include "dbcontentmanager.h"
#include "global.h"
#include "accuracy.h"
#include "logger.h"

#include "targetreportdefs.h"

#include <mutex>
#include <set>

namespace dbContent 
{

/**
*/
TargetReportAccessor::TargetReportAccessor(const std::string& dbcontent_name, 
                                           const std::shared_ptr<Buffer>& buffer,
                                           const DBContentManager& dbcont_man)
    :   BufferAccessor(dbcontent_name, buffer, dbcont_man)
{
    cacheVectors();
}

/**
*/
TargetReportAccessor::TargetReportAccessor(const std::shared_ptr<DBContentVariableLookup>& lookup)
    :   BufferAccessor(lookup)
{
    cacheVectors();
}

/**
*/
dbContent::VariableSet TargetReportAccessor::getReadSetFor(const std::string& dbcontent_name, DBContentManager& dbcont_man)
{
    dbContent::VariableSet read_set;

    auto add = [ & ] (const Property& p, bool is_meta_var)
    {
        if (is_meta_var)
        {
            if (dbcont_man.metaCanGetVariable(dbcontent_name, p))
                read_set.add(dbcont_man.metaGetVariable(dbcontent_name, p));
        }
        else
        {
            if (dbcont_man.canGetVariable(dbcontent_name, p))
                read_set.add(dbcont_man.getVariable(dbcontent_name, p));
        }
    };

    add(dbcontent_vars::meta_var_timestamp_, true);
    add(dbcontent_vars::meta_var_rec_num_, true);
    add(dbcontent_vars::meta_var_ds_id_, true);
    add(dbcontent_vars::meta_var_line_id_, true);
    add(dbcontent_vars::meta_var_utn_, true);

    add(dbcontent_vars::meta_var_acad_, true);
    add(dbcontent_vars::meta_var_acid_, true);

    add(dbcontent_vars::meta_var_latitude_, true);
    add(dbcontent_vars::meta_var_longitude_, true);
    add(dbcontent_vars::var_cat062_fl_measured_, false);
    add(dbcontent_vars::var_cat062_baro_alt_, false);
    add(dbcontent_vars::meta_var_ground_bit_, true);
    add(dbcontent_vars::meta_var_detection_type_, true);

    add(dbcontent_vars::var_cat021_mops_version_, false);
    add(dbcontent_vars::var_cat021_nacp_, false);
    add(dbcontent_vars::var_cat021_nucp_nic_, false);
    add(dbcontent_vars::var_cat021_sil_, false);
    add(dbcontent_vars::var_cat021_pos_check_failed_, false);
    add(dbcontent_vars::var_cat021_range_check_failed_, false);
    add(dbcontent_vars::var_cat021_cpr_valid_, false);
    add(dbcontent_vars::var_cat021_ldpj_, false);

    add(dbcontent_vars::meta_var_x_stddev_, true);
    add(dbcontent_vars::meta_var_y_stddev_, true);
    add(dbcontent_vars::meta_var_xy_cov_, true);

    add(dbcontent_vars::meta_var_ground_speed_, true);
    add(dbcontent_vars::meta_var_track_angle_, true);
    add(dbcontent_vars::var_cat021_sgv_stp_, false);
    add(dbcontent_vars::var_cat021_toa_pos_, false);

    add(dbcontent_vars::var_cat021_nucv_nacv_, false);
    add(dbcontent_vars::var_cat062_vx_stddev_, false);
    add(dbcontent_vars::var_cat062_vy_stddev_, false);

    add(dbcontent_vars::var_cat062_ax_, false);
    add(dbcontent_vars::var_cat062_ay_, false);

    add(dbcontent_vars::meta_var_m3a_, true);
    add(dbcontent_vars::meta_var_m3a_g_, true);
    add(dbcontent_vars::meta_var_m3a_v_, true);
    add(dbcontent_vars::meta_var_m3a_smoothed_, true);
    add(dbcontent_vars::var_cat062_m3a_age_, false);

    add(dbcontent_vars::meta_var_mc_, true);
    add(dbcontent_vars::meta_var_mc_g_, true);
    add(dbcontent_vars::meta_var_mc_v_, true);

    add(dbcontent_vars::meta_var_track_num_, true);
    add(dbcontent_vars::meta_var_track_begin_, true);
    add(dbcontent_vars::meta_var_track_end_, true);

    return read_set;
}

/**
*/
void TargetReportAccessor::cacheVectors()
{
    const auto& dbcontent_name = lookup_->dbContentName();

    is_radar_    = (dbcontent_name == "CAT001" || dbcontent_name == "CAT048");
    is_adsb_     = (dbcontent_name == "CAT021");
    is_tracker_  = (dbcontent_name == "CAT062");
    is_ref_traj_ = (dbcontent_name == "RefTraj");

    //general
    meta_timestamp_vec_ = metaVarVector<boost::posix_time::ptime>(dbcontent_vars::meta_var_timestamp_);
    traced_assert(meta_timestamp_vec_);
    meta_rec_num_vec_   = metaVarVector<unsigned long>(dbcontent_vars::meta_var_rec_num_);
    traced_assert(meta_rec_num_vec_);
    meta_ds_id_vec_     = metaVarVector<unsigned int>(dbcontent_vars::meta_var_ds_id_);
    traced_assert(meta_ds_id_vec_);
    meta_line_id_vec_   = metaVarVector<unsigned int>(dbcontent_vars::meta_var_line_id_);
    traced_assert(meta_line_id_vec_);

    meta_utn_vec_       = metaVarVector<unsigned int>(dbcontent_vars::meta_var_utn_);

    meta_acad_vec_      = metaVarVector<unsigned int>(dbcontent_vars::meta_var_acad_);
    meta_acid_vec_      = metaVarVector<std::string>(dbcontent_vars::meta_var_acid_);

    //position
    meta_latitude_vec_      = metaVarVector<double>(dbcontent_vars::meta_var_latitude_);
    meta_longitude_vec_     = metaVarVector<double>(dbcontent_vars::meta_var_longitude_);
    cat062_alt_trusted_vec_ = varVector<float>(dbcontent_vars::var_cat062_fl_measured_);
    cat062_alt_sec_vec_     = varVector<float>(dbcontent_vars::var_cat062_baro_alt_);
    cat021_alt_geo_vec_     = varVector<float>(dbcontent_vars::var_cat021_geo_alt_);
    meta_ground_bit_vec_    = metaVarVector<bool>(dbcontent_vars::meta_var_ground_bit_);
    meta_detection_type_vec_ = metaVarVector<unsigned char>(dbcontent_vars::meta_var_detection_type_);

    //position accuracy
    cat021_mops_version_vec_            = varVector<unsigned char>(dbcontent_vars::var_cat021_mops_version_);
    cat021_nac_p_vec_                   = varVector<unsigned char>(dbcontent_vars::var_cat021_nacp_);
    cat021_nucp_nic_vec_                = varVector<unsigned char>(dbcontent_vars::var_cat021_nucp_nic_);
    cat021_sil_vec_                     = varVector<unsigned char>(dbcontent_vars::var_cat021_sil_);
    cat021_pos_check_failed_vec_        = varVector<bool>(dbcontent_vars::var_cat021_pos_check_failed_);
    cat021_range_check_failed_vec_      = varVector<bool>(dbcontent_vars::var_cat021_range_check_failed_);
    cat021_cpr_valid_vec_               = varVector<bool>(dbcontent_vars::var_cat021_cpr_valid_);
    cat021_ldpj_vec_                    = varVector<bool>(dbcontent_vars::var_cat021_ldpj_);

    meta_pos_std_dev_x_m_vec_           = metaVarVector<double>(dbcontent_vars::meta_var_x_stddev_);
    meta_pos_std_dev_y_m_vec_           = metaVarVector<double>(dbcontent_vars::meta_var_y_stddev_);
    meta_pos_std_dev_xy_corr_coeff_vec_ = metaVarVector<double>(dbcontent_vars::meta_var_xy_cov_);

    meta_radar_range_vec_    = metaVarVector<double>(dbcontent_vars::var_radar_range_);
    meta_radar_azimuth_vec_  = metaVarVector<double>(dbcontent_vars::var_radar_azimuth_);

    //velocity / angle
    meta_speed_vec_       = metaVarVector<double>(dbcontent_vars::meta_var_ground_speed_);
    meta_track_angle_vec_ = metaVarVector<double>(dbcontent_vars::meta_var_track_angle_);
    cat021_sgv_stp_vec_   = varVector<bool>(dbcontent_vars::var_cat021_sgv_stp_);
    cat021_toa_pos_vec_   = varVector<float>(dbcontent_vars::var_cat021_toa_pos_);

    //velocity accuracy
    cat021_nucv_nacv_vec_ = varVector<unsigned char>(dbcontent_vars::var_cat021_nucv_nacv_);
    cat062_vx_stddev_vec_ = varVector<double>(dbcontent_vars::var_cat062_vx_stddev_);
    cat062_vy_stddev_vec_ = varVector<double>(dbcontent_vars::var_cat062_vy_stddev_);

    //acceleration
    cat062_ax_vec_ = varVector<double>(dbcontent_vars::var_cat062_ax_);
    cat062_ay_vec_ = varVector<double>(dbcontent_vars::var_cat062_ay_);

    //mode a
    meta_mode_a_vec_          = metaVarVector<unsigned int>(dbcontent_vars::meta_var_m3a_);
    meta_mode_a_garbled_vec_  = metaVarVector<bool>(dbcontent_vars::meta_var_m3a_g_);
    meta_mode_a_valid_vec_    = metaVarVector<bool>(dbcontent_vars::meta_var_m3a_v_);
    meta_mode_a_smoothed_vec_ = metaVarVector<bool>(dbcontent_vars::meta_var_m3a_smoothed_);
    cat062_m3a_age_vec_       = varVector<float>(dbcontent_vars::var_cat062_m3a_age_);

    //mode c
    meta_mode_c_vec_         = metaVarVector<float>(dbcontent_vars::meta_var_mc_);
    meta_mode_c_garbled_vec_ = metaVarVector<bool>(dbcontent_vars::meta_var_mc_g_);
    meta_mode_c_valid_vec_   = metaVarVector<bool>(dbcontent_vars::meta_var_mc_v_);

    //track
    meta_track_num_vec_   = metaVarVector<unsigned int>(dbcontent_vars::meta_var_track_num_);
    meta_track_begin_vec_ = metaVarVector<bool>(dbcontent_vars::meta_var_track_begin_);
    meta_track_end_vec_   = metaVarVector<bool>(dbcontent_vars::meta_var_track_end_);

    cat021_ecat_vec_ = varVector<unsigned int>(dbcontent_vars::var_cat021_ecat_);
    cat021_geo_alt_acc_vec_ = varVector<unsigned char>(dbcontent_vars::var_cat021_geo_alt_accuracy_);
}

/**
*/
bool TargetReportAccessor::hasTimestamp(unsigned int index) const
{
    return meta_timestamp_vec_ && !meta_timestamp_vec_->isNull(index);
}

/**
*/
boost::posix_time::ptime TargetReportAccessor::timestamp(unsigned int index) const
{
    return getNotOptional<boost::posix_time::ptime>(meta_timestamp_vec_, index);
}

/**
*/
unsigned long TargetReportAccessor::recordNumber(unsigned int index) const
{
    return getNotOptional<unsigned long>(meta_rec_num_vec_, index);
}

/**
*/
unsigned int TargetReportAccessor::dsID(unsigned int index) const
{
    return getNotOptional<unsigned int>(meta_ds_id_vec_, index);
}

/**
 */
unsigned int TargetReportAccessor::lineID(unsigned int index) const
{
    return getNotOptional<unsigned int>(meta_line_id_vec_, index);
}

/**
*/
boost::optional<unsigned int> TargetReportAccessor::utn(unsigned int index) const
{
    return getOptional<unsigned int>(meta_utn_vec_, index);
}

/**
*/
boost::optional<unsigned char> TargetReportAccessor::mopsVersion(unsigned int index) const
{
    return getOptional<unsigned char>(cat021_mops_version_vec_, index);
}

boost::optional<unsigned char> TargetReportAccessor::nucp(unsigned int index) const
{
    return getOptional<unsigned char>(cat021_nucp_nic_vec_, index);
}

boost::optional<unsigned char> TargetReportAccessor::nacp(unsigned int index) const
{
    return getOptional<unsigned char>(cat021_nac_p_vec_, index);
}

boost::optional<unsigned char> TargetReportAccessor::sil(unsigned int index) const
{
    return getOptional<unsigned char>(cat021_sil_vec_, index);
}

boost::optional<bool> TargetReportAccessor::posCheckFailed(unsigned int index) const
{
    return getOptional<bool>(cat021_pos_check_failed_vec_, index);
}

boost::optional<bool> TargetReportAccessor::rangeCheckFailed(unsigned int index) const
{
    return getOptional<bool>(cat021_range_check_failed_vec_, index);
}

boost::optional<bool> TargetReportAccessor::cprValid(unsigned int index) const
{
    return getOptional<bool>(cat021_cpr_valid_vec_, index);
}

boost::optional<bool> TargetReportAccessor::localDecodingPositionJump(unsigned int index) const
{
    return getOptional<bool>(cat021_ldpj_vec_, index);
}

boost::optional<bool> TargetReportAccessor::sgvStopped(unsigned int index) const
{
    return getOptional<bool>(cat021_sgv_stp_vec_, index);
}

boost::optional<unsigned int> TargetReportAccessor::ecat(unsigned int index) const
{
    return getOptional<unsigned int>(cat021_ecat_vec_, index);
}

boost::optional<unsigned char> TargetReportAccessor::getGeoAltAcc(unsigned int index) const
{
    return getOptional<unsigned char>(cat021_geo_alt_acc_vec_, index);
}

/**
*/
boost::optional<unsigned int> TargetReportAccessor::acad(unsigned int index) const
{
    return getOptional<unsigned int>(meta_acad_vec_, index);
}

/**
*/
boost::optional<std::string> TargetReportAccessor::acid(unsigned int index) const
{
    return getOptional<std::string>(meta_acid_vec_, index);
}

/**
*/
boost::optional<targetReport::Position> TargetReportAccessor::position(unsigned int index) const
{
    if (!meta_latitude_vec_ || 
        !meta_longitude_vec_ ||
        meta_latitude_vec_->isNull(index) ||
        meta_longitude_vec_->isNull(index))
        return {};

    return targetReport::Position(meta_latitude_vec_->get(index),
                                  meta_longitude_vec_->get(index));
}

/**
*/
boost::optional<targetReport::PositionAccuracy> TargetReportAccessor::positionAccuracy(unsigned int index) const
{
    if (is_adsb_)
    {
        if (!cat021_mops_version_vec_ || cat021_mops_version_vec_->isNull(index))
            return {};

        auto mops_version = cat021_mops_version_vec_->get(index);

        double qi_epu{0};
        double x_stddev {0}, y_stddev {0}, xy_cov{0};

        if (mops_version == 0)
        {
            if (!cat021_nucp_nic_vec_ || cat021_nucp_nic_vec_->isNull(index))
                return {};

            auto nuc_p = cat021_nucp_nic_vec_->get(index);

            if (!targetReport::AccuracyTables::adsb_v0_accuracies.count(nuc_p)) // value unknown, also for 0 (undefined)
                return {};

            qi_epu = targetReport::AccuracyTables::adsb_v0_accuracies.at(nuc_p);
        }
        else if (mops_version == 1 || mops_version == 2)
        {
            if (cat021_nac_p_vec_ && !cat021_nac_p_vec_->isNull(index))
            {
                auto nacp = cat021_nac_p_vec_->get(index);

                if (!targetReport::AccuracyTables::adsb_v12_accuracies.count(nacp))
                    return {}; // value unknown

                qi_epu = targetReport::AccuracyTables::adsb_v12_accuracies.at(nacp);
            }
            else if (cat021_nucp_nic_vec_ && !cat021_nucp_nic_vec_->isNull(index))
            {
                // NACp unavailable, fall back to NIC Rc.
                // NIC Rc is a containment radius, not a 95% accuracy bound.
                // Approximate EPU ~ Rc / 2.0 (conservative conversion).
                auto nic = cat021_nucp_nic_vec_->get(index);

                if (!targetReport::AccuracyTables::adsb_v12_nic_accuracies.count(nic))
                    return {};

                qi_epu = targetReport::AccuracyTables::adsb_v12_nic_accuracies.at(nic) / 2.0;
            }
            else
                return {};
        }
        else
        {
            return {}; // unknown mops version
        }

        // EPU is the 95% horizontal error radius (R95). For a 2-D isotropic error (Rayleigh), R95≈2.45σ, so σ≈EPU/2.45
        float sigma = qi_epu / 2.45;

        x_stddev = sigma;
        y_stddev = sigma;

        // if (!meta_speed_vec_->isNull(index) && !meta_track_angle_vec_->isNull(index))
        // {
        // speed based adaptation possible later
        // }

        return targetReport::PositionAccuracy(x_stddev, y_stddev, xy_cov);
    }
    else if (is_radar_)
    {
        //tbi
    }
    else // cat010, cat020, cat062, reftraj
    {
        if (!meta_pos_std_dev_x_m_vec_ || 
            !meta_pos_std_dev_y_m_vec_ ||
            meta_pos_std_dev_x_m_vec_->isNull(index) ||
            meta_pos_std_dev_y_m_vec_->isNull(index))
        {
            return {};
        }

        double xy_cov = 0.0;
        if (meta_pos_std_dev_xy_corr_coeff_vec_ && !meta_pos_std_dev_xy_corr_coeff_vec_->isNull(index))
        {
            xy_cov = meta_pos_std_dev_xy_corr_coeff_vec_->get(index);

            // if (!is_ref_traj_) // already adjusted during ASTERIX import
            // {
            //     if (xy_cov < 0)
            //         xy_cov = -std::pow(xy_cov, 2);
            //     else
            //         xy_cov =  std::pow(xy_cov, 2);
            // }
        }
        double x_stddev = meta_pos_std_dev_x_m_vec_->get(index);
        double y_stddev = meta_pos_std_dev_y_m_vec_->get(index);

        Utils::Accuracy::checkMaxCovariance(x_stddev, y_stddev, xy_cov);

        return targetReport::PositionAccuracy(x_stddev, y_stddev, xy_cov);
    }

    //not implemented for dbcontent
    return {};
}

/**
*/
boost::optional<targetReport::BarometricAltitude> TargetReportAccessor::barometricAltitude(unsigned int index) const
{
    if (is_tracker_ && cat062_alt_trusted_vec_ && !cat062_alt_trusted_vec_->isNull(index))
    {
        return targetReport::BarometricAltitude(targetReport::BarometricAltitude::Source::Barometric_CAT062_Trusted,
                                                cat062_alt_trusted_vec_->get(index),
                                                meta_mode_c_valid_vec_ && !meta_mode_c_valid_vec_->isNull(index) ? meta_mode_c_valid_vec_->get(index) : boost::optional<bool>(),
                                                meta_mode_c_garbled_vec_ && !meta_mode_c_garbled_vec_->isNull(index) ? meta_mode_c_garbled_vec_->get(index) : boost::optional<bool>());
    }
    else if (meta_mode_c_vec_ && !meta_mode_c_vec_->isNull(index))
    {
        return targetReport::BarometricAltitude(targetReport::BarometricAltitude::Source::Barometric_ModeC,
                                                meta_mode_c_vec_->get(index),
                                                meta_mode_c_valid_vec_ && !meta_mode_c_valid_vec_->isNull(index) ? meta_mode_c_valid_vec_->get(index) : boost::optional<bool>(),
                                                meta_mode_c_garbled_vec_ && !meta_mode_c_garbled_vec_->isNull(index) ? meta_mode_c_garbled_vec_->get(index) : boost::optional<bool>());
    }
    else if (is_tracker_ && cat062_alt_sec_vec_ && !cat062_alt_sec_vec_->isNull(index))
    {
        return targetReport::BarometricAltitude(targetReport::BarometricAltitude::Source::Barometric_CAT062_Secondary,
                                                cat062_alt_sec_vec_->get(index),
                                                meta_mode_c_valid_vec_ && !meta_mode_c_valid_vec_->isNull(index) ? meta_mode_c_valid_vec_->get(index) : boost::optional<bool>(),
                                                meta_mode_c_garbled_vec_ && !meta_mode_c_garbled_vec_->isNull(index) ? meta_mode_c_garbled_vec_->get(index) : boost::optional<bool>());
    }

    //not implemented for dbcontent
    return {};
}

boost::optional<float> TargetReportAccessor::trackedBarometricAltitude(unsigned int index) const
{
    // CAT062 Barometric Altitude Calculated (I062/135) - the tracker's own
    // altitude output, in contrast to the per-sensor measured FL that
    // barometricAltitude() prefers for trackers
    if (!is_tracker_)
        return boost::none;

    return getOptional<float>(cat062_alt_sec_vec_, index);
}

boost::optional<float> TargetReportAccessor::geometricAltitude(unsigned int index) const
{
    return getOptional<float>(cat021_alt_geo_vec_, index);
}

boost::optional<double> TargetReportAccessor::radarRange(unsigned int index) const
{
    return getOptional<double>(meta_radar_range_vec_, index);
}
boost::optional<double> TargetReportAccessor::radarAzimuth(unsigned int index) const
{
    return getOptional<double>(meta_radar_azimuth_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::adsbToATimeSource(unsigned int index) const
{
    if (!is_adsb_)
        return {};

    return cat021_toa_pos_vec_ && !cat021_toa_pos_vec_->isNull(index);
}

boost::optional<targetReport::Velocity> TargetReportAccessor::velocity(unsigned int index) const
{
    if (meta_speed_vec_
            && !meta_speed_vec_->isNull(index) && meta_speed_vec_->get(index) <= ADSB_MAX_STOPPED_SPEED) // kts
        return targetReport::Velocity(0.0, 0.0);

    if (!meta_speed_vec_ || 
        !meta_track_angle_vec_ ||
        meta_speed_vec_->isNull(index) ||
        meta_track_angle_vec_->isNull(index))
        return {};

    return targetReport::Velocity(meta_track_angle_vec_->get(index), meta_speed_vec_->get(index)  * KNOTS2M_S);
}

/**
*/
boost::optional<targetReport::VelocityAccuracy> TargetReportAccessor::velocityAccuracy(unsigned int index) const
{
    if (is_adsb_)
    {
        if (!cat021_nucv_nacv_vec_ || cat021_nucv_nacv_vec_->isNull(index))
            return {};

        auto nuc_r = cat021_nucv_nacv_vec_->get(index);

        if (!targetReport::AccuracyTables::adsb_nucr_nacv_accuracies.count(nuc_r))
            return {}; // no info

        double vx_stddev = targetReport::AccuracyTables::adsb_nucr_nacv_accuracies.at(nuc_r);
        double vy_stddev = vx_stddev;

        return targetReport::VelocityAccuracy(vx_stddev, vy_stddev);
    }
    else if (is_tracker_)
    {
        if (!cat062_vx_stddev_vec_ ||
            !cat062_vy_stddev_vec_ ||
            cat062_vx_stddev_vec_->isNull(index) ||
            cat062_vy_stddev_vec_->isNull(index))
            return {};

        double vx_stddev = cat062_vx_stddev_vec_->get(index);
        double vy_stddev = cat062_vy_stddev_vec_->get(index);

        // stored stddevs must not be zero when set, indicates bad source data
        if (vx_stddev == 0 || vy_stddev == 0)
        {
            static std::set<unsigned int> warned_ds_ids;
            static std::mutex warned_mutex;

            unsigned int ds_id_val = dsID(index);

            std::lock_guard<std::mutex> lock(warned_mutex);

            if (!warned_ds_ids.count(ds_id_val))
            {
                warned_ds_ids.insert(ds_id_val);

                logwrn << "zero stored velocity stddev vx " << vx_stddev
                       << " vy " << vy_stddev << " ds_id " << ds_id_val
                       << ", suppressing further warnings for this data source";
            }
        }

        return targetReport::VelocityAccuracy(vx_stddev, vy_stddev);
    }

    //not implemented for dbcontent
    return {};
}

/**
*/
boost::optional<targetReport::Acceleration> TargetReportAccessor::acceleration(unsigned int index) const
{
    if (!cat062_ax_vec_ ||
        !cat062_ay_vec_ ||
        cat062_ax_vec_->isNull(index) ||
        cat062_ay_vec_->isNull(index))
        return {};

    return targetReport::Acceleration(cat062_ax_vec_->get(index), cat062_ay_vec_->get(index));
}

/**
*/
boost::optional<double> TargetReportAccessor::trackAngle(unsigned int index) const
{
    return getOptional<double>(meta_track_angle_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::groundBit(unsigned int index) const
{
    return getOptional<bool>(meta_ground_bit_vec_, index);
}

boost::optional<unsigned char> TargetReportAccessor::detectionType(unsigned int index) const
{
    return getOptional<unsigned char>(meta_detection_type_vec_, index);
}

boost::optional<targetReport::ModeACode> TargetReportAccessor::modeACode(unsigned int index) const
{
    boost::optional<unsigned int> code = modeA(index);

    if (!code)
        return {};
    else
        return targetReport::ModeACode(code.value(),
                                       modeAValid(index),
                                       modeAGarbled(index),
                                       modeASmoothed(index));
}

/**
*/
boost::optional<unsigned int> TargetReportAccessor::modeA(unsigned int index) const
{
    return getOptional<unsigned int>(meta_mode_a_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::modeAValid(unsigned int index) const
{
    return getOptional<bool>(meta_mode_a_valid_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::modeAGarbled(unsigned int index) const
{
    return getOptional<bool>(meta_mode_a_garbled_vec_, index);
}

/**
*/
boost::optional<float> TargetReportAccessor::modeAAge(unsigned int index) const
{
    return getOptional<float>(cat062_m3a_age_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::modeASmoothed(unsigned int index) const
{
    return getOptional<bool>(meta_mode_a_smoothed_vec_, index);
}

/**
*/
boost::optional<float> TargetReportAccessor::modeC(unsigned int index) const
{
    return getOptional<float>(meta_mode_c_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::modeCValid(unsigned int index) const
{
    return getOptional<bool>(meta_mode_c_valid_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::modeCGarbled(unsigned int index) const
{
    return getOptional<bool>(meta_mode_c_garbled_vec_, index);
}

/**
*/
boost::optional<unsigned int> TargetReportAccessor::trackNumber(unsigned int index) const
{
    return getOptional<unsigned int>(meta_track_num_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::trackBegin(unsigned int index) const
{
    return getOptional<bool>(meta_track_begin_vec_, index);
}

/**
*/
boost::optional<bool> TargetReportAccessor::trackEnd(unsigned int index) const
{
    return getOptional<bool>(meta_track_end_vec_, index);
}

} // namespace dbContent
