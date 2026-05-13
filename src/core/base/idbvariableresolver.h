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

#include <string>
#include <vector>

#include "property.h"

namespace dbContent {
class VariableSet;
}

class IDBVariableResolver
{
public:
    virtual ~IDBVariableResolver() = default;

    // --- Property-based resolution (for specific named filters) ---

    // meta variable resolution (e.g. meta_var_m3a_, meta_var_timestamp_)
    virtual bool metaCanGetVariable(const std::string& dbcontent_name,
                                    const Property& meta_var) const = 0;
    virtual std::string metaGetVariableDBColumn(const std::string& dbcontent_name,
                                                const Property& meta_var) const = 0;
    virtual std::string metaGetVariableName(const std::string& dbcontent_name,
                                            const Property& meta_var) const = 0;

    // direct variable resolution (e.g. var_cat062_baro_alt_)
    virtual bool canGetVariable(const std::string& dbcontent_name,
                                const Property& var) const = 0;
    virtual std::string getVariableDBColumn(const std::string& dbcontent_name,
                                            const Property& var) const = 0;
    virtual std::string getVariableName(const std::string& dbcontent_name,
                                        const Property& var) const = 0;
    virtual bool variableHasDBContent(const std::string& dbcontent_name,
                                      const Property& var) const = 0;

    // --- Name-based resolution (for DBFilterCondition / generic filters) ---

    // existence checks (mirrors DBContentManager)
    virtual bool existsMetaVariable(const std::string& var_name) const = 0;
    virtual bool metaVariableExistsIn(const std::string& var_name,
                                      const std::string& dbcontent_name) const = 0;
    virtual bool existsDBContent(const std::string& dbcontent_name) const = 0;
    virtual bool dbContentHasVariable(const std::string& dbcontent_name,
                                      const std::string& var_name) const = 0;

    // returns dbcontent names that a meta variable has concrete variables for
    virtual std::vector<std::string> metaVariableDBContentNames(
        const std::string& var_name) const = 0;

    // variable attribute queries (mirrors Variable methods)
    // var_dbcontent_name == META_OBJECT_NAME => resolve via meta variable
    // var_dbcontent_name == concrete name    => resolve via direct variable
    virtual std::string variableDBColumnName(const std::string& dbcontent_name,
                                             const std::string& var_name,
                                             const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableDBTableName(const std::string& dbcontent_name,
                                            const std::string& var_name,
                                            const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableDBExpression(const std::string& dbcontent_name,
                                             const std::string& var_name,
                                             const std::string& var_dbcontent_name) const = 0;
    virtual PropertyDataType variableDataType(const std::string& dbcontent_name,
                                              const std::string& var_name,
                                              const std::string& var_dbcontent_name) const = 0;
    virtual bool variableHasNonStandardRepresentation(const std::string& dbcontent_name,
                                                      const std::string& var_name,
                                                      const std::string& var_dbcontent_name) const = 0;
    virtual std::string variableValueFromRepresentation(const std::string& dbcontent_name,
                                                        const std::string& var_name,
                                                        const std::string& var_dbcontent_name,
                                                        const std::string& value_str) const = 0;

    // VariableSet interaction (wraps Variable& lookup + VariableSet operations)
    virtual bool readSetHasVariable(const std::string& dbcontent_name,
                                    const std::string& var_name,
                                    const std::string& var_dbcontent_name,
                                    const dbContent::VariableSet& read_set) const = 0;
    virtual void addVariableToReadSet(const std::string& dbcontent_name,
                                      const std::string& var_name,
                                      const std::string& var_dbcontent_name,
                                      dbContent::VariableSet& read_set) const = 0;
};

// Well-known variable properties used across the application.
// Canonical definitions - other layers may alias these (e.g. DBContent::meta_var_ds_id_).
namespace dbcontent_vars
{
    // meta variables
    inline const Property meta_var_rec_num_       {"Record Number", PropertyDataType::ULONGINT};
    inline const Property meta_var_ds_id_         {"DS ID", PropertyDataType::UINT};
    inline const Property meta_var_sac_id_        {"SAC", PropertyDataType::UCHAR};
    inline const Property meta_var_sic_id_        {"SIC", PropertyDataType::UCHAR};
    inline const Property meta_var_line_id_       {"Line ID", PropertyDataType::UINT};
    inline const Property meta_var_time_of_day_   {"Time of Day", PropertyDataType::FLOAT};
    inline const Property meta_var_timestamp_     {"Timestamp", PropertyDataType::TIMESTAMP};
    inline const Property meta_var_m3a_           {"Mode 3/A Code", PropertyDataType::UINT};
    inline const Property meta_var_m3a_g_        {"Mode 3/A Garbled", PropertyDataType::BOOL};
    inline const Property meta_var_m3a_v_        {"Mode 3/A Valid", PropertyDataType::BOOL};
    inline const Property meta_var_m3a_smoothed_ {"Mode 3/A Smoothed", PropertyDataType::BOOL};
    inline const Property meta_var_acad_          {"Aircraft Address", PropertyDataType::UINT};
    inline const Property meta_var_acid_          {"Aircraft Identification", PropertyDataType::STRING};
    inline const Property meta_var_mc_            {"Mode C Code", PropertyDataType::FLOAT};
    inline const Property meta_var_mc_g_          {"Mode C Garbled", PropertyDataType::BOOL};
    inline const Property meta_var_mc_v_          {"Mode C Valid", PropertyDataType::BOOL};
    inline const Property meta_var_ground_bit_    {"Ground Bit", PropertyDataType::BOOL};

    inline const Property meta_var_track_num_       {"Track Number", PropertyDataType::UINT};
    inline const Property meta_var_track_begin_     {"Track Begin", PropertyDataType::BOOL};
    inline const Property meta_var_track_confirmed_ {"Track Confirmed", PropertyDataType::BOOL};
    inline const Property meta_var_track_coasting_  {"Track Coasting", PropertyDataType::UCHAR};
    inline const Property meta_var_track_end_       {"Track End", PropertyDataType::BOOL};

    inline const Property meta_var_latitude_        {"Latitude", PropertyDataType::DOUBLE};
    inline const Property meta_var_longitude_       {"Longitude", PropertyDataType::DOUBLE};
    inline const Property meta_var_detection_type_  {"Type", PropertyDataType::UCHAR};
    inline const Property meta_var_x_               {"X", PropertyDataType::DOUBLE};
    inline const Property meta_var_y_               {"Y", PropertyDataType::DOUBLE};

    inline const Property meta_var_artas_hash_      {"ARTAS Hash", PropertyDataType::STRING};
    inline const Property meta_var_utn_             {"UTN", PropertyDataType::UINT};

    inline const Property meta_var_vx_              {"Vx", PropertyDataType::DOUBLE};
    inline const Property meta_var_vy_              {"Vy", PropertyDataType::DOUBLE};
    inline const Property meta_var_ground_speed_    {"Track Groundspeed", PropertyDataType::DOUBLE};
    inline const Property meta_var_track_angle_     {"Track Angle", PropertyDataType::DOUBLE};
    inline const Property meta_var_horizontal_man_  {"Track Horizontal Manoeuvre", PropertyDataType::BOOL};

    inline const Property meta_var_ax_              {"Ax", PropertyDataType::DOUBLE};
    inline const Property meta_var_ay_              {"Ay", PropertyDataType::DOUBLE};

    inline const Property meta_var_mom_long_acc_    {"MOM Longitudinal Acc", PropertyDataType::UCHAR};
    inline const Property meta_var_mom_trans_acc_   {"MOM Transversal Acc", PropertyDataType::UCHAR};
    inline const Property meta_var_mom_vert_rate_   {"MOM Vertical Rate", PropertyDataType::UCHAR};

    inline const Property meta_var_x_stddev_        {"X StdDev", PropertyDataType::DOUBLE};
    inline const Property meta_var_y_stddev_        {"Y StdDev", PropertyDataType::DOUBLE};
    inline const Property meta_var_xy_cov_          {"X/Y Covariance", PropertyDataType::DOUBLE};

    inline const Property meta_var_max_stddev_xy_  {"X/Y Covariance", PropertyDataType::DOUBLE};

    inline const Property meta_var_latitude_stddev_   {"Latitude StdDev", PropertyDataType::DOUBLE};
    inline const Property meta_var_longitude_stddev_  {"Longitude StdDev", PropertyDataType::DOUBLE};
    inline const Property meta_var_latlon_cov_        {"Lat/Lon Cov", PropertyDataType::DOUBLE};

    inline const Property meta_var_climb_descent_   {"Track Climbing/Descending", PropertyDataType::UCHAR};
    inline const Property meta_var_rocd_            {"Rate Of Climb/Descent", PropertyDataType::FLOAT};
    inline const Property meta_var_spi_             {"SPI", PropertyDataType::BOOL};

    inline const Property meta_var_message_type_    {"Message Type", PropertyDataType::UCHAR};

    // direct (non-meta) variables
    inline const Property var_radar_range_          {"Range", PropertyDataType::DOUBLE};
    inline const Property var_radar_azimuth_        {"Azimuth", PropertyDataType::DOUBLE};
    inline const Property var_radar_altitude_       {"Mode C Code", PropertyDataType::FLOAT};

    inline const Property var_cat020_contrib_recv_  {"Contributing Receivers", PropertyDataType::JSON};

    inline const Property var_cat021_toa_position_  {"ToA Position", PropertyDataType::FLOAT};
    inline const Property var_cat021_tomr_position_ {"ToMR Position", PropertyDataType::FLOAT};
    inline const Property var_cat021_tort_          {"ToRT", PropertyDataType::FLOAT};
    inline const Property var_cat021_tod_dep_       {"Time of Day Deprecated", PropertyDataType::FLOAT};

    inline const Property var_cat021_mops_version_  {"MOPS Version", PropertyDataType::UCHAR};
    inline const Property var_cat021_nacp_          {"NACp", PropertyDataType::UCHAR};
    inline const Property var_cat021_nucp_nic_      {"NUCp or NIC", PropertyDataType::UCHAR};
    inline const Property var_cat021_nucv_nacv_     {"NUCr or NACv", PropertyDataType::UCHAR};
    inline const Property var_cat021_sil_           {"SIL", PropertyDataType::UCHAR};
    inline const Property var_cat021_pos_check_failed_ {"Position Check Failed", PropertyDataType::BOOL};
    inline const Property var_cat021_geo_alt_       {"Geometric Height", PropertyDataType::FLOAT};
    inline const Property var_cat021_geo_alt_accuracy_ {"Geometric Altitude Accuracy", PropertyDataType::UCHAR};
    inline const Property var_cat021_ecat_          {"Emitter Category", PropertyDataType::UINT};

    inline const Property var_cat021_latitude_hr_   {"Latitude HR", PropertyDataType::DOUBLE};
    inline const Property var_cat021_longitude_hr_  {"Longitude HR", PropertyDataType::DOUBLE};

    inline const Property var_cat021_sgv_gss_       {"SGV GSS", PropertyDataType::FLOAT};
    inline const Property var_cat021_sgv_hgt_       {"SGV HGT", PropertyDataType::DOUBLE};
    inline const Property var_cat021_sgv_htt_       {"SGV HTT", PropertyDataType::BOOL};
    inline const Property var_cat021_sgv_hrd_       {"SGV HRD", PropertyDataType::BOOL};
    inline const Property var_cat021_sgv_stp_       {"SGV STP", PropertyDataType::BOOL};

    inline const Property var_cat062_tris_          {"Target Report Identifiers", PropertyDataType::STRING};
    inline const Property var_cat062_tri_recnums_   {"TRI Record Numbers", PropertyDataType::JSON};
    inline const Property var_cat062_track_begin_   {"Track Begin", PropertyDataType::BOOL};
    inline const Property var_cat062_coasting_      {"Coasting", PropertyDataType::UCHAR};
    inline const Property var_cat062_track_end_     {"Track End", PropertyDataType::BOOL};
    inline const Property var_cat062_mono_sensor_   {"Monosensor", PropertyDataType::BOOL};
    inline const Property var_cat062_type_lm_       {"Type LM", PropertyDataType::UCHAR};
    inline const Property var_cat062_baro_alt_      {"Barometric Altitude Calculated", PropertyDataType::FLOAT};
    inline const Property var_cat062_fl_measured_    {"Flight Level Measured", PropertyDataType::FLOAT};

    inline const Property var_cat062_num_contrib_sensors_     {"Num Contributing Sensors", PropertyDataType::UCHAR};
    inline const Property var_cat062_num_contrib_sensors_tn_  {"Num Contributing Sensors Track Number", PropertyDataType::UCHAR};
    inline const Property var_cat062_sum_num_contrib_sensors_ {"Sum Number Contributing Sensors", PropertyDataType::UCHAR};

    inline const Property var_cat062_wtc_           {"Wake Turbulence Category FPL", PropertyDataType::STRING};
    inline const Property var_cat062_callsign_fpl_  {"Callsign FPL", PropertyDataType::STRING};

    inline const Property var_cat062_vx_stddev_     {"Vx StdDev", PropertyDataType::DOUBLE};
    inline const Property var_cat062_vy_stddev_     {"Vy StdDev", PropertyDataType::DOUBLE};

    inline const Property var_cat063_sensor_sac_    {"Sensor SAC", PropertyDataType::UCHAR};
    inline const Property var_cat063_sensor_sic_    {"Sensor SIC", PropertyDataType::UCHAR};
    inline const Property var_cat063_con_           {"CON", PropertyDataType::UCHAR};

    inline const Property var_cat065_batch_number_  {"Batch Number", PropertyDataType::UCHAR};

    // reference trajectory
    inline const Property var_reftraj_contrib_adsb_age_    {"Contributing ADS-B Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_contrib_mlat_age_    {"Contributing MLAT Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_contrib_radar_age_   {"Contributing Radar Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_contrib_tracker_age_ {"Contributing Tracker Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_contrib_reftraj_age_ {"Contributing RefTraj Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_contrib_other_age_   {"Contributing Other Age", PropertyDataType::FLOAT};

    inline const Property var_reftraj_contrib_sources_     {"Contributing Sources", PropertyDataType::JSON};
    inline const Property var_reftraj_contrib_sources_num_ {"Contributing Sources Number", PropertyDataType::UINT};

    inline const Property var_reftraj_update_age_primary_  {"Primary Update Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_update_age_modeac_   {"Mode A/C Update Age", PropertyDataType::FLOAT};
    inline const Property var_reftraj_update_age_modes_    {"Mode S Update Age", PropertyDataType::FLOAT};

    inline const Property selected_var_             {"selected", PropertyDataType::BOOL};
} // namespace dbcontent_vars
