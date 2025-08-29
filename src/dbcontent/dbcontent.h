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

#include "configurable.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/variable/variableset.h"
#include "traced_assert.h"

#include <QObject>

#include <memory>
#include <string>
#include <vector>

class COMPASS;
class PropertyList;

class DBContentWidget;
class Buffer;
class Job;
class DBContentReadDBJob;
class UpdateBufferDBJob;
class DBContentManager;
class DBContentDeleteDBJob;

namespace dbContent
{
class VariableSet;

//bits 8/7
//    (TRANS)
//    Transversal Acceleration
//    = 00 Constant Course
//    = 01 Right Turn
//    = 10 Left Turn
//    = 11 Undetermined
//      bits 6/5

enum class MOM_TRANS_ACC
{
    ConstantCourse=0,
    RightTurn, // 1
    LeftTurn, // 2
    Undetermined // 3
};

//      (LONG)
//      Longitudinal Acceleration
//    = 00 Constant Groundspeed
//    = 01 Increasing Groundspeed
//    = 10 Decreasing Groundspeed
//    = 11 Undetermined

enum class MOM_LONG_ACC
{
    ConstantGroundspeed=0,
    IncreasingGroundspeed, // 1
    DecreasingGroundspeed, // 2
    Undetermined // 3
};

//      bits 4/3
//      (VERT)
//      Vertical Rate
//    = 00 Level
//    = 01 Climb
//    = 10 Descent
//    = 11 Undetermined

enum class MOM_VERT_RATE
{
    Level=0,
    Climb, // 1
    Descent, // 2
    Undetermined // 3
};

}

/**
 */
class DBContent : public QObject, public Configurable
{
    Q_OBJECT

signals:
    void updateProgressSignal(float percent);
    void updateDoneSignal(DBContent& dbcontent);

public slots:
    void databaseOpenedSlot();
    void databaseClosedSlot();

    void readJobIntermediateSlot(std::shared_ptr<Buffer> buffer);
    void readJobObsoleteSlot();
    void readJobDoneSlot();

    void updateProgressSlot(float percent);
    void updateDoneSlot();

    void deleteJobDoneSlot();

public:
    static const Property meta_var_rec_num_;
    static const Property meta_var_ds_id_;
    static const Property meta_var_sac_id_;
    static const Property meta_var_sic_id_;
    static const Property meta_var_line_id_;
    static const Property meta_var_time_of_day_;
    static const Property meta_var_timestamp_; // boost::posix_time::ptime
    static const Property meta_var_m3a_;
    static const Property meta_var_m3a_g_;
    static const Property meta_var_m3a_v_;
    static const Property meta_var_m3a_smoothed_;
    static const Property meta_var_acad_;
    static const Property meta_var_acid_;
    static const Property meta_var_mc_;
    static const Property meta_var_mc_g_;
    static const Property meta_var_mc_v_;
    static const Property meta_var_ground_bit_;

    static const Property meta_var_track_num_;
    static const Property meta_var_track_begin_;
    static const Property meta_var_track_confirmed_;
    static const Property meta_var_track_coasting_;
    static const Property meta_var_track_end_;

    static const Property meta_var_latitude_;
    static const Property meta_var_longitude_;
    static const Property meta_var_detection_type_;
    static const Property meta_var_x_;
    static const Property meta_var_y_;

    static const Property meta_var_artas_hash_;
    static const Property meta_var_utn_;

    static const Property meta_var_vx_;
    static const Property meta_var_vy_;
    static const Property meta_var_ground_speed_; // kts
    static const Property meta_var_track_angle_; // deg
    static const Property meta_var_horizontal_man_;

    static const Property meta_var_ax_;
    static const Property meta_var_ay_;

    static const Property meta_var_mom_long_acc_;
    static const Property meta_var_mom_trans_acc_;
    static const Property meta_var_mom_vert_rate_;

    static const Property meta_var_x_stddev_;
    static const Property meta_var_y_stddev_;
    static const Property meta_var_xy_cov_;

    static const Property meta_var_latitude_stddev_;
    static const Property meta_var_longitude_stddev_;
    static const Property meta_var_latlon_cov_;

    static const Property meta_var_climb_descent_;
    static const Property meta_var_rocd_;
    static const Property meta_var_spi_;

    static const Property meta_var_message_type_;

    static const Property var_radar_range_;
    static const Property var_radar_azimuth_;
    static const Property var_radar_altitude_;

    static const Property var_cat020_crontrib_recv_;

    static const Property var_cat021_toa_position_;     // "ToA Position" 071.Time of Applicability for Position
    static const Property var_cat021_tomr_position_; // "ToMR Position" 0.73 Time of Message Reception for Position
    static const Property var_cat021_tort_; // "ToRT" 077.Time of Report Transmission
    static const Property var_cat021_tod_dep_; // "Time of Day Deprecated" 030.Time of Day

    static const Property var_cat021_mops_version_;
    static const Property var_cat021_nacp_;
    static const Property var_cat021_nucp_nic_;
    static const Property var_cat021_nucv_nacv_;
    static const Property var_cat021_sil_;
    static const Property var_cat021_geo_alt_;
    static const Property var_cat021_geo_alt_accuracy_;
    static const Property var_cat021_ecat_;

    static const Property var_cat021_latitude_hr_;
    static const Property var_cat021_longitude_hr_;

    static const Property var_cat021_sgv_gss_; // ground speed, kts
    static const Property var_cat021_sgv_hgt_; // heading / ground track, deg, based on htt
    static const Property var_cat021_sgv_htt_; // heading 0 / ground track 1
    static const Property var_cat021_sgv_hrd_; // true north 0 / magnetic north 1

    static const Property var_cat062_tris_;
    static const Property var_cat062_tri_recnums_;
    static const Property var_cat062_track_begin_;
    static const Property var_cat062_coasting_;
    static const Property var_cat062_track_end_;
    static const Property var_cat062_mono_sensor_;
    static const Property var_cat062_type_lm_;
    static const Property var_cat062_baro_alt_;
    static const Property var_cat062_fl_measured_; // trusted, not valid

    //Rate of Climb/Descent float feet / min
    //Ax Ay float m/s2
    // trans long vert "MOM Longitudinal Acc" "MOM Transversal Acc" "MOM Vertical Rate" uchar

    static const Property var_cat062_wtc_;
    static const Property var_cat062_callsign_fpl_;

    static const Property var_cat062_vx_stddev_;
    static const Property var_cat062_vy_stddev_;

    static const Property var_cat063_sensor_sac_;
    static const Property var_cat063_sensor_sic_;

    static const Property var_cat065_batch_number_;

    static const Property selected_var;

public:
    DBContent(COMPASS& compass, 
              const std::string& class_id, 
              const std::string& instance_id,
              DBContentManager* manager);
    virtual ~DBContent();

    bool hasVariable(const std::string& name) const;
    dbContent::Variable& variable(const std::string& name) const;
    void renameVariable(const std::string& name, 
                        const std::string& new_name);
    void deleteVariable(const std::string& name);

    const std::map<std::string, std::unique_ptr<dbContent::Variable>>& variables() const { return variables_; }

    bool hasVariableDBColumnName(const std::string& col_name) const;

    size_t numVariables() const { return variables_.size(); }

    const std::string& name() const { return name_; }
    void name(const std::string& name)
    {
        traced_assert(name.size() > 0);
        name_ = name;
    }

    unsigned int id();

    const std::string& info() const { return info_; }
    void info(const std::string& info) { info_ = info; }

    bool loadable() const { return is_loadable_; }

    void load(dbContent::VariableSet& read_set, 
              bool use_datasrc_filters, 
              bool use_filters,
              const std::string& custom_filter_clause=""); // main load function
    void loadFiltered(dbContent::VariableSet& read_set, 
                      std::string custom_filter_clause);
    
    // load function for custom filtering
    void quitLoading();

    bool prepareInsert(std::shared_ptr<Buffer>& buffer);
    void updateDataSourcesBeforeInsert(std::shared_ptr<Buffer>& buffer);
    void finalizeInsert(std::shared_ptr<Buffer>& buffer);

    void updateData(dbContent::Variable& key_var, 
                    std::shared_ptr<Buffer> buffer);

    // counts and targets have to be adjusted outside
    void deleteDBContentData(bool cleanup_db = false);
    void deleteDBContentData(unsigned int sac, 
                             unsigned int sic,
                             bool cleanup_db = false);
    void deleteDBContentData(unsigned int sac, 
                             unsigned int sic, 
                             unsigned int line_id,
                             bool cleanup_db = false);

    bool isLoading();
    bool isDeleting();

    bool hasData();
    size_t count();
    size_t loadedCount();

    virtual void generateSubConfigurable(const std::string& class_id, const std::string& instance_id);

    bool hasKeyVariable();
    dbContent::Variable& getKeyVariable();

    std::string status();

    DBContentWidget* widget();
    void closeWidget();

    bool existsInDB() const;

    std::string dbTableName() const;

    bool containsTargetReports() const;
    bool containsStatusContent() const;
    bool isReferenceContent() const;

protected:
    virtual void checkSubConfigurables();

    void checkStaticVariable(const Property& property);

    COMPASS&          compass_;
    DBContentManager& dbcont_manager_;

    std::string  name_;
    unsigned int id_ {0};
    std::string  info_;
    std::string  db_table_name_;
    std::string  ds_type_;

    bool contains_target_reports_ {false};
    bool contains_status_content_ {false};
    bool is_reftraj_content_ {false};

    bool is_loadable_{false};  // loadable on its own
    size_t count_{0};

    bool insert_active_ = false;

    std::shared_ptr<DBContentReadDBJob> read_job_{nullptr};
    std::shared_ptr<UpdateBufferDBJob> update_job_{nullptr};
    std::shared_ptr<DBContentDeleteDBJob> delete_job_{nullptr};

    /// Container with all variables (variable identifier -> variable pointer)
    std::map<std::string, std::unique_ptr<dbContent::Variable>> variables_;

    std::unique_ptr<DBContentWidget> widget_;
};
