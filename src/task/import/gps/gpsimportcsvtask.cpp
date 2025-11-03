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

#include "gpsimportcsvtask.h"
#include "gpsimportcsvtaskdialog.h"
#include "compass.h"
#include "dbinterface.h"
//#include "savedfile.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "files.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "datasourcemanager.h"
#include "buffer.h"
//#include "dbcontent/variable/variableset.h"
//#include "dbcontent/variable/metavariable.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "util/stringconv.h"
#include "global.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <nmeaparse/nmea.h>

//#include <unsupported/Eigen/Splines>

#include <QApplication>
#include <QMessageBox>

const float tod_24h = 24 * 60 * 60;

using namespace Utils;
using namespace std;
using namespace nmea;

GPSImportCSVTask::GPSImportCSVTask(const std::string& class_id, const std::string& instance_id,
                                   TaskManager& task_manager)
    : Task(task_manager),
      Configurable(class_id, instance_id, &task_manager, "task_import_gps_csv.json")
{
    tooltip_ = "Allows importing of GPS trails as CSV into the opened database.";

    registerParameter("current_filename", &current_filename_, std::string());

    //02/16/23 09:24:24.000
    registerParameter("timestamp_format", &timestamp_format_, std::string("%m/%d/%y %H:%M:%S%F"));

    registerParameter("ds_name", &ds_name_, std::string("GPS Trail"));
    registerParameter("ds_sac", &ds_sac_, 0u);
    registerParameter("ds_sic", &ds_sic_, 0u);

    registerParameter("tod_offset", &tod_offset_, 0.0f);

    registerParameter("set_mode_3a_code", &set_mode_3a_code_, false);
    registerParameter("mode_3a_code", &mode_3a_code_, 0u);

    registerParameter("set_target_address", &set_target_address_, false);
    registerParameter("target_address", &target_address_, 0u);

    registerParameter("set_callsign", &set_callsign_, false);
    registerParameter("callsign", &callsign_, std::string());

    //registerParameter("line_id", &line_id_, 0); always defaults to 0

    createSubConfigurables();

    if (current_filename_.size())
        parseCurrentFile();
}

GPSImportCSVTask::~GPSImportCSVTask()
{
}

void GPSImportCSVTask::generateSubConfigurable(const std::string& class_id,
                                               const std::string& instance_id)
{
    throw std::runtime_error("GPSImportCSVTask: generateSubConfigurable: unknown class_id " +
                             class_id);
}

GPSImportCSVTaskDialog* GPSImportCSVTask::dialog()
{
    if (!dialog_)
    {
        dialog_.reset(new GPSImportCSVTaskDialog(*this));

        connect(dialog_.get(), &GPSImportCSVTaskDialog::importSignal,
                this, &GPSImportCSVTask::dialogImportSlot);

        connect(dialog_.get(), &GPSImportCSVTaskDialog::cancelSignal,
                this, &GPSImportCSVTask::dialogCancelSlot);
    }

    traced_assert(dialog_);
    return dialog_.get();
}

void GPSImportCSVTask::importFilename(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    current_filename_ = filename;

    parseCurrentFile();
}

bool GPSImportCSVTask::checkPrerequisites()
{
    if (!COMPASS::instance().dbInterface().ready())  // must be connected
        return false;

    if (!COMPASS::instance().dbContentManager().existsDBContent("RefTraj"))
        return false;

    return true;
}

bool GPSImportCSVTask::isRecommended()
{
    return false;
}

bool GPSImportCSVTask::isRequired() { return false; }

std::string GPSImportCSVTask::currentError() const
{
    return current_error_;
}

std::string GPSImportCSVTask::dsName() const
{
    return ds_name_;
}

void GPSImportCSVTask::dsName(const std::string& ds_name)
{
    loginf << "value '" << ds_name << "'";

    ds_name_ = ds_name;
}

unsigned int GPSImportCSVTask::dsSAC() const
{
    return ds_sac_;
}

void GPSImportCSVTask::dsSAC(unsigned int ds_sac)
{
    loginf << "value " << ds_sac;

    ds_sac_ = ds_sac;
}

unsigned int GPSImportCSVTask::dsSIC() const
{
    return ds_sic_;
}

void GPSImportCSVTask::dsSIC(unsigned int ds_sic)
{
    loginf << "value " << ds_sic;

    ds_sic_ = ds_sic;
}

float GPSImportCSVTask::todOffset() const
{
    return tod_offset_;
}

void GPSImportCSVTask::todOffset(float tod_offset)
{
    loginf << "value " << tod_offset;

    tod_offset_ = tod_offset;
}

bool GPSImportCSVTask::setMode3aCode() const
{
    return set_mode_3a_code_;
}

void GPSImportCSVTask::setMode3aCode(bool value)
{
    loginf << "value " << value;

    set_mode_3a_code_ = value;
}

unsigned int GPSImportCSVTask::mode3aCode() const
{
    return mode_3a_code_;
}

void GPSImportCSVTask::mode3aCode(unsigned int value)
{
    loginf << "value " << value;

    mode_3a_code_ = value;
}

bool GPSImportCSVTask::setTargetAddress() const
{
    return set_target_address_;
}

void GPSImportCSVTask::setTargetAddress(bool value)
{
    loginf << "value " << value;

    set_target_address_ = value;
}

unsigned int GPSImportCSVTask::targetAddress() const
{
    return target_address_;
}

void GPSImportCSVTask::targetAddress(unsigned int target_address)
{
    loginf << "value " << target_address;

    target_address_ = target_address;
}

bool GPSImportCSVTask::setCallsign() const
{
    return set_callsign_;
}

void GPSImportCSVTask::setCallsign(bool value)
{
    loginf << "value " << value;

    set_callsign_ = value;
}

std::string GPSImportCSVTask::callsign() const
{
    return callsign_;
}

void GPSImportCSVTask::callsign(const std::string& callsign)
{
    loginf << "value '" << callsign << "'";

    callsign_ = callsign;
}

unsigned int GPSImportCSVTask::lineID() const
{
    return line_id_;
}

void GPSImportCSVTask::lineID(unsigned int line_id)
{
    loginf << "value " << line_id;

    line_id_ = line_id;
}

std::string GPSImportCSVTask::currentText() const
{
    return current_text_;
}


bool GPSImportCSVTask::canImportFile()
{
    if (!current_filename_.size())
        return false;

    if (!Files::fileExists(current_filename_))
    {
        loginf << "not possible since file '" << current_filename_
               << "does not exist";
        return false;
    }

    return gps_positions_.size(); // only if fixes exist
}

void GPSImportCSVTask::parseCurrentFile ()
{
    loginf << "file '" << current_filename_ << "'";

    current_error_ = "";
    current_text_ = "";

    gps_positions_.clear();

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    // From a file
    string line;
    ifstream file(current_filename_);
    unsigned int line_cnt = 0;

    std::vector<std::string> parts;

    GPSPosition fix;
    bool ok;
    double x_pos, y_pos, track_angle, speed;
    float d_t;

    while (getline(file, line))
    {
        logdbg << "line '" << line << "'";

        //        "GPSTime","Latitude","Longitude","MSL"
        if (line_cnt == 0)
            traced_assert(line.find("\"GPSTime\",\"Latitude\",\"Longitude\",\"MSL\"") != std::string::npos);
        else
        {
            //        02/16/23 09:24:24.000,41.242547397,-8.674719837,67.207

            parts = String::split(line, ',');

            if (parts.size() != 4)
            {
                logerr << "wrong number of parts " << parts.size();
                continue;
            }

            fix.timestamp_ = Time::fromString(parts.at(0), timestamp_format_);

            if (fix.timestamp_.is_not_a_date_time())
            {
                logerr << "wrong timestamp '" << parts.at(0) << "'";
                continue;
            }

            fix.latitude_ = std::numeric_limits<double>::quiet_NaN();
            fix.latitude_ = std::stod(parts.at(1));

            if (std::isnan(fix.latitude_))
            {
                logerr << "wrong latitude '" << parts.at(1) << "'";
                continue;
            }

            fix.longitude_ = std::numeric_limits<double>::quiet_NaN();
            fix.longitude_ = std::stod(parts.at(2));

            if (std::isnan(fix.longitude_))
            {
                logerr << "wrong longitude '" << parts.at(2) << "'";
                continue;
            }

            fix.altitude_ = std::numeric_limits<double>::quiet_NaN();
            fix.altitude_ = std::stod(parts.at(3));

            if (std::isnan(fix.altitude_))
            {
                logerr << "wrong altitude '" << parts.at(3) << "'";
                continue;
            }

            fix.has_speed_ = false;

            if (gps_positions_.size()) // has previous pos
            {
                GPSPosition& prev_fix = gps_positions_.back();

                tie(ok, x_pos, y_pos) = trafo_.distanceCart(
                            prev_fix.latitude_, prev_fix.longitude_, fix.latitude_, fix.longitude_);

                if (!ok)
                {
                    logerr << "error with latitude " << fix.latitude_
                           << " longitude " << fix.longitude_;
                }
                else
                {

                    logdbg << "offsets x " << fixed << x_pos
                           << " y " << fixed << y_pos << " dist " << fixed << sqrt(pow(x_pos,2)+pow(y_pos,2));

                    d_t = Time::partialSeconds(fix.timestamp_ - prev_fix.timestamp_);

                    double v_x = x_pos/d_t;
                    double v_y = y_pos/d_t;
                    logdbg << "v_x " << v_x << " v_y " << v_y;


                    logdbg << "d_t " << d_t;

                    //assert (ok); TODO?

                    // x_pos long, y_pos lat

                    logdbg << "interpolated lat "
                           << x_pos << " long " << y_pos;

                    track_angle = atan2(v_x,v_y); // bearing rad
                    speed = sqrt(pow(v_x, 2) + pow(v_y, 2));

                    if (!std::isnan(track_angle) && !std::isinf(track_angle) && !std::isnan(speed) && !std::isinf(speed))
                    {
                        fix.has_speed_ = true;
                        fix.vx_ = v_x;
                        fix.vy_ = v_y;
                        fix.track_angle_ = RAD2DEG * track_angle;
                        fix.speed_ = M_S2KNOTS * speed;
                    }
                }
            }

            gps_positions_.push_back(fix);

        }
        ++line_cnt;
    }

    // Show the final fix information
    //cout << gps.fix.toString() << endl;

    stringstream ss;

    ss << "Parsed " << line_cnt << " lines.\n";

    current_text_ = ss.str();

    loginf << "parsed " << gps_positions_.size() << " fixes in "
           << line_cnt << " lines";

    QApplication::restoreOverrideCursor();
}

bool GPSImportCSVTask::canRun() { return canImportFile(); }

void GPSImportCSVTask::run()
{
    loginf << "filename '" << current_filename_ << " fixes " << gps_positions_.size();

    traced_assert(gps_positions_.size());
    traced_assert(!buffer_);

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    string dbcontent_name = "RefTraj";
    traced_assert(dbcontent_man.existsDBContent(dbcontent_name));

    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_sac_id_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_sic_id_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ds_id_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_line_id_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_timestamp_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_latitude_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_longitude_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_m3a_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_mc_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_num_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_vx_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_vy_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_angle_));

    loginf << "getting variables";

    using namespace dbContent;

    Variable& sac_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sac_id_);
    Variable& sic_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_sic_id_);
    Variable& ds_id_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ds_id_);
    Variable& line_id_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_line_id_);
    Variable& tod_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_time_of_day_);
    Variable& ts_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_timestamp_);
    Variable& lat_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_latitude_);
    Variable& long_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_longitude_);
    Variable& m3a_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_m3a_);
    Variable& mc_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_mc_);
    Variable& ta_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_);
    Variable& ti_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_);
    Variable& tn_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_);
    Variable& vx_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vx_);
    Variable& vy_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vy_);
    Variable& speed_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_);
    Variable& track_angle_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_angle_);

    PropertyList properties;
    properties.addProperty(sac_var.name(), PropertyDataType::UCHAR);
    properties.addProperty(sic_var.name(), PropertyDataType::UCHAR);
    properties.addProperty(ds_id_var.name(), PropertyDataType::UINT);
    properties.addProperty(line_id_var.name(), PropertyDataType::UINT);
    properties.addProperty(tod_var.name(), PropertyDataType::FLOAT);
    properties.addProperty(ts_var.name(), PropertyDataType::TIMESTAMP);
    properties.addProperty(lat_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(long_var.name(), PropertyDataType::DOUBLE);

    if (set_mode_3a_code_)
        properties.addProperty(m3a_var.name(), PropertyDataType::UINT);

    properties.addProperty(mc_var.name(), PropertyDataType::FLOAT);

    if (set_target_address_)
        properties.addProperty(ta_var.name(), PropertyDataType::UINT);

    if (set_callsign_)
        properties.addProperty(ti_var.name(), PropertyDataType::STRING);

    properties.addProperty(tn_var.name(), PropertyDataType::UINT);

    properties.addProperty(vx_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(vy_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(speed_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(track_angle_var.name(), PropertyDataType::DOUBLE);

    loginf << "creating buffer";

    buffer_ = make_shared<Buffer>(properties, dbcontent_name);

    NullableVector<unsigned char>& sac_vec = buffer_->get<unsigned char>(sac_var.name());
    NullableVector<unsigned char>& sic_vec = buffer_->get<unsigned char>(sic_var.name());
    NullableVector<unsigned int>& ds_id_vec = buffer_->get<unsigned int>(ds_id_var.name());
    NullableVector<unsigned int>& line_id_vec = buffer_->get<unsigned int>(line_id_var.name());
    NullableVector<float>& tod_vec = buffer_->get<float>(tod_var.name());
    NullableVector<boost::posix_time::ptime>& ts_vec = buffer_->get<boost::posix_time::ptime>(ts_var.name());
    NullableVector<double>& lat_vec = buffer_->get<double>(lat_var.name());
    NullableVector<double>& long_vec = buffer_->get<double>(long_var.name());
    NullableVector<float>& mc_vec = buffer_->get<float>(mc_var.name());

    NullableVector<unsigned int>& tn_vec = buffer_->get<unsigned int>(tn_var.name());

    NullableVector<double>& vx_vec = buffer_->get<double>(vx_var.name());
    NullableVector<double>& vy_vec = buffer_->get<double>(vy_var.name());
    NullableVector<double>& speed_vec = buffer_->get<double>(speed_var.name());
    NullableVector<double>& track_angle_vec = buffer_->get<double>(track_angle_var.name());

    unsigned int cnt = 0;
    unsigned int ds_id = Number::dsIdFrom(ds_sac_, ds_sic_);

    traced_assert(dbcontent_man.hasMaxRefTrajTrackNum());
    unsigned int track_num = dbcontent_man.maxRefTrajTrackNum();

    loginf << "max reftraj track num " << track_num;

    // config data source
    {
        DataSourceManager& src_man = COMPASS::instance().dataSourceManager();

        if (!src_man.hasDBDataSource(ds_id))
            src_man.addNewDataSource(ds_id);

        traced_assert(src_man.hasDBDataSource(ds_id));

        dbContent::DBDataSource& src = src_man.dbDataSource(ds_id);

        src.name(ds_name_);
        src.dsType(dbcontent_name); // same as dstype

    }

    float tod;
    //boost::posix_time::ptime timestamp;
    //double speed_ms, track_angle_rad, vx, vy;

    loginf << "filling buffer";

    for (auto& fix_it : gps_positions_)
    {
        // timestamp
        fix_it.timestamp_ += Time::partialSeconds(tod_offset_); // add time offset

        // tod
        tod = fix_it.timestamp_.time_of_day().total_milliseconds() / 1000.0;

        // check for out-of-bounds because of midnight-jump
        while (tod < 0.0f)
            tod += tod_24h;
        while (tod > tod_24h)
            tod -= tod_24h;

        traced_assert(tod >= 0.0f);
        traced_assert(tod <= tod_24h);

        sac_vec.set(cnt, ds_sac_);
        sic_vec.set(cnt, ds_sic_);
        ds_id_vec.set(cnt, ds_id);
        line_id_vec.set(cnt, line_id_);

        tod_vec.set(cnt, tod);
        ts_vec.set(cnt, fix_it.timestamp_);
        lat_vec.set(cnt, fix_it.latitude_);
        long_vec.set(cnt, fix_it.longitude_);

        mc_vec.set(cnt, fix_it.altitude_ * 1/FT2M); // m 2 ft

        if (set_mode_3a_code_)
            buffer_->get<unsigned int>(m3a_var.name()).set(cnt, mode_3a_code_);

        if (set_target_address_)
            buffer_->get<unsigned int>(ta_var.name()).set(cnt, target_address_);

        if (set_callsign_)
            buffer_->get<string>(ti_var.name()).set(cnt, callsign_);

        tn_vec.set(cnt, track_num);

        if (fix_it.has_speed_)
        {
            track_angle_vec.set(cnt, fix_it.track_angle_);
            speed_vec.set(cnt, fix_it.speed_ * M_S2KNOTS);

            vx_vec.set(cnt, fix_it.vx_);
            vy_vec.set(cnt, fix_it.vy_);
        }

        ++cnt;
    }

    dbcontent_man.maxRefTrajTrackNum(track_num+1); // increment for next

    loginf << "inserting data";

    connect(&dbcontent_man, &DBContentManager::insertDoneSignal, this, &GPSImportCSVTask::insertDoneSlot,
            Qt::UniqueConnection);

    dbcontent_man.insertData({{dbcontent_name, buffer_}});
}

void GPSImportCSVTask::insertDoneSlot()
{
    loginf;

    buffer_ = nullptr;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    disconnect(&dbcontent_man, &DBContentManager::insertDoneSignal,
               this, &GPSImportCSVTask::insertDoneSlot);

    COMPASS::instance().dataSourceManager().saveDBDataSources();
    emit COMPASS::instance().dataSourceManager().dataSourcesChangedSignal();

    done_ = true;

    //    COMPASS::instance().interface().databaseContentChanged();
    //    object.updateToDatabaseContent();

    QMessageBox msg_box;

    msg_box.setWindowTitle("Import GPS CSV");
    msg_box.setText("Import of "+QString::number(gps_positions_.size())+" GPS fixes done.");
    msg_box.setStandardButtons(QMessageBox::Ok);

    if (allow_user_interactions_)
        msg_box.exec();

    emit doneSignal();
}

void GPSImportCSVTask::dialogImportSlot()
{
    traced_assert(canRun());

    traced_assert(dialog_);
    dialog_->hide();

    run();
}

void GPSImportCSVTask::dialogCancelSlot()
{
    traced_assert(dialog_);
    dialog_->hide();
}

//void GPSImportCSVTask::checkParsedData ()
//{
//    loginf;
//}
