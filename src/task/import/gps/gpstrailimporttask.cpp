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

#include "gpstrailimporttask.h"
#include "gpstrailimporttaskdialog.h"
#include "compass.h"
#include "dbinterface.h"
#include "stringconv.h"
#include "taskmanager.h"
#include "files.h"
#include "dbcontent/dbcontentmanager.h"
#include "dbcontent/dbcontent.h"
#include "dbcontent/variable/variable.h"
#include "datasourcemanager.h"
#include "buffer.h"
#include "util/number.h"
#include "util/timeconv.h"
#include "projection/transformation.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <nmeaparse/nmea.h>

#include <QApplication>
#include <QMessageBox>

const float tod_24h = 24 * 60 * 60;

using namespace Utils;
using namespace std;
using namespace nmea;

/**
*/
boost::posix_time::ptime getTimeFrom (const nmea::GPSTimestamp& ts)
{
    return boost::posix_time::ptime(boost::gregorian::date(ts.year,
                                                           ts.month,
                                                           ts.day),
                                    boost::posix_time::time_duration(ts.hour,
                                                                     ts.min,
                                                                     ts.sec)
                                    + Time::partialSeconds(ts.sec, true)); // add partial w/o s
}

/**
*/
GPSTrailImportTask::Settings::Settings()
:   ds_name           ("GPS Trail")
,   ds_sac            (255)
,   ds_sic            (0)
,   use_tod_offset    (false)
,   tod_offset        (0.0f)
,   use_override_date (false)
,   override_date_str ("1970-01-01")
,   set_mode_3a_code  (false)
,   mode_3a_code      (0)
,   set_target_address(false)
,   target_address    (0)
,   set_callsign      (false)
,   callsign          ("")
{
}

/**
*/
GPSTrailImportTask::GPSTrailImportTask(const std::string& class_id, 
                                       const std::string& instance_id,
                                       TaskManager& task_manager)
:   Task        (task_manager)
,   Configurable(class_id, instance_id, &task_manager, "task_import_gps.json")
{
    tooltip_ = "Allows importing of GPS trails as NMEA into the opened database.";

    registerParameter("ds_name", &settings_.ds_name, Settings().ds_name);
    registerParameter("ds_sac", &settings_.ds_sac, Settings().ds_sac);
    registerParameter("ds_sic", &settings_.ds_sic, Settings().ds_sic);

    registerParameter("use_tod_offset", &settings_.use_tod_offset, Settings().use_tod_offset);
    registerParameter("tod_offset", &settings_.tod_offset, Settings().tod_offset);

    registerParameter("use_override_date", &settings_.use_override_date, Settings().use_override_date);
    registerParameter("override_date_str", &settings_.override_date_str, Settings().override_date_str);
    override_date_ = boost::gregorian::from_string(settings_.override_date_str);

    registerParameter("set_mode_3a_code", &settings_.set_mode_3a_code, Settings().set_mode_3a_code);
    registerParameter("mode_3a_code", &settings_.mode_3a_code, Settings().mode_3a_code);

    registerParameter("set_target_address", &settings_.set_target_address, Settings().set_target_address);
    registerParameter("target_address", &settings_.target_address, Settings().target_address);

    registerParameter("set_callsign", &settings_.set_callsign, Settings().set_callsign);
    registerParameter("callsign", &settings_.callsign, Settings().callsign);

    createSubConfigurables();
}

/**
*/
GPSTrailImportTask::~GPSTrailImportTask() = default;

/**
*/
void GPSTrailImportTask::generateSubConfigurable(const std::string& class_id,
                                                 const std::string& instance_id)
{
    throw std::runtime_error("GPSTrailImportTask: generateSubConfigurable: unknown class_id " + class_id);
}

/**
*/
GPSTrailImportTaskDialog* GPSTrailImportTask::dialog()
{
    if (!dialog_)
    {
        dialog_.reset(new GPSTrailImportTaskDialog(*this));

        connect(dialog_.get(), &GPSTrailImportTaskDialog::importSignal,
                this, &GPSTrailImportTask::dialogImportSlot);

        connect(dialog_.get(), &GPSTrailImportTaskDialog::cancelSignal,
                this, &GPSTrailImportTask::dialogCancelSlot);
    }

    traced_assert(dialog_);
    return dialog_.get();
}

/**
*/
void GPSTrailImportTask::importFilename(const std::string& filename)
{
    loginf << "filename '" << filename << "'";

    current_filename_ = filename;

    parseCurrentFile();

    emit fileChanged();
}

/**
*/
bool GPSTrailImportTask::checkPrerequisites()
{
    if (!COMPASS::instance().dbInterface().ready())  // must be connected
        return false;

    if (!COMPASS::instance().dbContentManager().existsDBContent("RefTraj"))
        return false;

    return true;
}

/**
*/
bool GPSTrailImportTask::isRecommended()
{
    return false;
}

/**
*/
bool GPSTrailImportTask::isRequired() { return false; }

/**
*/
std::string GPSTrailImportTask::currentError() const
{
    return current_error_;
}

/**
*/
std::string GPSTrailImportTask::dsName() const
{
    return settings_.ds_name;
}

/**
*/
void GPSTrailImportTask::dsName(const std::string& ds_name)
{
    loginf << "value '" << ds_name << "'";

    settings_.ds_name = ds_name;
}

/**
*/
unsigned int GPSTrailImportTask::dsSAC() const
{
    return settings_.ds_sac;
}

/**
*/
void GPSTrailImportTask::dsSAC(unsigned int ds_sac)
{
    loginf << "value " << ds_sac;

    settings_.ds_sac = ds_sac;
}

/**
*/
unsigned int GPSTrailImportTask::dsSIC() const
{
    return settings_.ds_sic;
}

/**
*/
void GPSTrailImportTask::dsSIC(unsigned int ds_sic)
{
    loginf << "value " << ds_sic;

    settings_.ds_sic = ds_sic;
}

/**
*/
bool GPSTrailImportTask::useTodOffset() const
{
    return settings_.use_tod_offset;
}

/**
*/
void GPSTrailImportTask::useTodOffset(bool value)
{
    loginf << "value " << value;

    settings_.use_tod_offset = value;
}

/**
*/
double GPSTrailImportTask::todOffset() const
{
    return settings_.tod_offset;
}

/**
*/
void GPSTrailImportTask::todOffset(double tod_offset)
{
    loginf << "value " << String::doubleToStringPrecision(tod_offset, 3);

    settings_.tod_offset = tod_offset;
}

/**
*/
const boost::gregorian::date& GPSTrailImportTask::overrideDate() const
{
    return override_date_;
}

/**
*/
void GPSTrailImportTask::overrideDate(const boost::gregorian::date& date)
{
    override_date_ = date;

    settings_.override_date_str = boost::gregorian::to_iso_extended_string(override_date_);

    loginf << "value '" << settings_.override_date_str << "'";
}

/**
*/
bool GPSTrailImportTask::setMode3aCode() const
{
    return settings_.set_mode_3a_code;
}

/**
*/
void GPSTrailImportTask::setMode3aCode(bool value)
{
    loginf << "value " << value;

    settings_.set_mode_3a_code = value;
}

/**
*/
unsigned int GPSTrailImportTask::mode3aCode() const
{
    return settings_.mode_3a_code;
}

/**
*/
void GPSTrailImportTask::mode3aCode(unsigned int value)
{
    loginf << "value " << value;

    settings_.mode_3a_code = value;
}

/**
*/
bool GPSTrailImportTask::setTargetAddress() const
{
    return settings_.set_target_address;
}

/**
*/
void GPSTrailImportTask::setTargetAddress(bool value)
{
    loginf << "value " << value;

    settings_.set_target_address = value;
}

/**
*/
unsigned int GPSTrailImportTask::targetAddress() const
{
    return settings_.target_address;
}

/**
*/
void GPSTrailImportTask::targetAddress(unsigned int target_address)
{
    loginf << "value " << target_address;

    settings_.target_address = target_address;
}

/**
*/
bool GPSTrailImportTask::setCallsign() const
{
    return settings_.set_callsign;
}

/**
*/
void GPSTrailImportTask::setCallsign(bool value)
{
    loginf << "value " << value;

    settings_.set_callsign = value;
}

/**
*/
std::string GPSTrailImportTask::callsign() const
{
    return settings_.callsign;
}

/**
*/
void GPSTrailImportTask::callsign(const std::string& callsign)
{
    loginf << "value '" << callsign << "'";

    settings_.callsign = callsign;
}

/**
*/
unsigned int GPSTrailImportTask::lineID() const
{
    return line_id_;
}

/**
*/
void GPSTrailImportTask::lineID(unsigned int line_id)
{
    loginf << "value " << line_id;

    line_id_ = line_id;
}

/**
*/
bool GPSTrailImportTask::useOverrideDate() const
{
    return settings_.use_override_date;
}

/**
*/
void GPSTrailImportTask::useOverrideDate(bool value)
{
    settings_.use_override_date = value;
}

/**
*/
std::string GPSTrailImportTask::currentText() const
{
    return current_text_;
}

/**
*/
bool GPSTrailImportTask::canImportFile()
{
    loginf << "current_filename '" << current_filename_ << "'";

    if (!current_filename_.size())
        return false;

    if (!Files::fileExists(current_filename_))
    {
        loginf << "not possible since file '" << current_filename_
               << "does not exist";
        return false;
    }

    loginf << "gps_fixes " << gps_fixes_.size();

    return gps_fixes_.size(); // only if fixes exist
}

/**
*/
void GPSTrailImportTask::parseCurrentFile ()
{
    loginf << "file '" << current_filename_ << "'";

    current_error_ = "";
    current_text_ = "";

    gps_fixes_.clear();
    quality_counts_.clear();
    gps_fixes_cnt_ = 0;
    gps_fixes_skipped_quality_cnt_ = 0;
    gps_fixes_skipped_time_cnt_ = 0;
    gps_fixes_zero_datetime_ = 0;

    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));

    NMEAParser parser;
    GPSService gps(parser);
    //parser.log = true;

    boost::posix_time::ptime last_ts;

    //cout << "Fix  Sats  Sig\t\tSpeed    Dir  Lat         , Lon           Accuracy" << endl;
    // Handle any changes to the GPS Fix... This is called whenever it's updated.
    gps.onUpdate += [&gps, this, &last_ts](){

        //        loginf << "start"
        //               << " time " << gps.fix.timestamp.toString()
        //               << " rawTime " << gps.fix.timestamp.rawTime
        //               << " rawDate " << gps.fix.timestamp.rawDate
        //               << " " << (gps.fix.locked() ? "[*] " : "[ ] ") << setw(2) << setfill(' ')
        //               << gps.fix.trackingSatellites << "/" << setw(2) << setfill(' ') << gps.fix.visibleSatellites << " "
        //               << fixed << setprecision(2) << setw(5) << setfill(' ') << gps.fix.almanac.averageSNR() << " dB   "
        //               << fixed << setprecision(2) << setw(6) << setfill(' ') << gps.fix.speed << " km/h ["
        //               << GPSFix::travelAngleToCompassDirection(gps.fix.travelAngle, true) << "]  "
        //               << fixed << setprecision(6) << gps.fix.latitude << "\xF8 " "N, " << gps.fix.longitude << "\xF8 " "E"
        //               << "  +/- " << setprecision(1) << gps.fix.horizontalAccuracy() << "m  ";

        ++gps_fixes_cnt_;

        //        if (!gps.fix.locked())
        //        {
        //            ++gps_fixes_skipped_lost_lock_;
        //            return;
        //        }

        if (gps.fix.timestamp.rawDate == 0 && gps.fix.timestamp.rawTime == 0)
        {
           ++gps_fixes_zero_datetime_;
           return;
        }

        if (gps.fix.quality == 0)
        {
            ++gps_fixes_skipped_quality_cnt_;
            return;
        }

        if (gps.fix.travelAngle == 0.0 && gps.fix.speed == 0.0)
        {
            ++gps_fixes_without_speedvec_;
        }

        if (gps_fixes_.size() && last_ts == getTimeFrom(gps.fix.timestamp))
        {
            if (gps_fixes_.back().latitude == gps.fix.latitude
                    && gps_fixes_.back().longitude == gps.fix.longitude)
                ++gps_fixes_skipped_time_cnt_;
            else // different position
            {
                gps_fixes_.back() = gps.fix;
            }
            return;
        }
        else // new
        {
            quality_counts_[gps.fix.quality] += 1;

            gps_fixes_.push_back(gps.fix);

            last_ts = getTimeFrom(gps.fix.timestamp);
        }
    };

    // From a file
    string line;
    ifstream file(current_filename_);
    unsigned int line_cnt = 0;

    while (getline(file, line))
    {
        try
        {
            if (line.back() == '\r')
                line.pop_back();

            parser.readLine(line);
        }
        catch (NMEAParseError& e)
        {
            logerr << "line " << line_cnt << ": error '" << e.message << "'";
            if (!current_error_.size())
                current_error_ = "Line "+to_string(line_cnt)+": "+e.message;
            else
                current_error_ += "\nLine "+to_string(line_cnt)+": "+e.message;
            // You can keep feeding data to the gps service...
            // The previous data is ignored and the parser is reset.
        }
        ++line_cnt;
    }

    // Show the final fix information
    //cout << gps.fix.toString() << endl;

    stringstream ss;

    ss << "Parsed " << line_cnt << " lines.\n";

    if (gps_fixes_cnt_)
    {
        ss << "Read " << gps_fixes_cnt_ << " fixes.\n";
        ss << "Skipped " << gps_fixes_zero_datetime_
           << " (" << String::percentToString(100.0*gps_fixes_zero_datetime_/gps_fixes_cnt_) << "%)"
           << " because of zero date and time.\n";

        //        ss << "Skipped " << gps_fixes_skipped_lost_lock_
        //           << " (" << String::percentToString(100.0*gps_fixes_skipped_lost_lock_/gps_fixes_cnt_) << "%)"
        //           << " because of lost GNSS lock.\n";
        ss << "Skipped " << gps_fixes_skipped_quality_cnt_
           << " (" << String::percentToString(100.0*gps_fixes_skipped_quality_cnt_/gps_fixes_cnt_) << "%)"
           << " because of invalid quality.\n";
        ss << "Skipped " << gps_fixes_skipped_time_cnt_
           << " (" << String::percentToString(100.0*gps_fixes_skipped_time_cnt_/gps_fixes_cnt_) << "%)"
           << " because of same time and position.\n";
        ss << "Got " << gps_fixes_.size()
           << " (" << String::percentToString(100.0*gps_fixes_.size()/gps_fixes_cnt_) << "%) fixes.\n";

        if (gps_fixes_without_speedvec_)
            ss << "Got " << gps_fixes_without_speedvec_
               << " (" << String::percentToString(100.0*gps_fixes_without_speedvec_/gps_fixes_cnt_)
               << "%) without speed vector, will be calculated.\n";

        if (gps_fixes_.size())
            ss << "\n Timestamps\n  Begin: " << Time::toString(getTimeFrom(gps_fixes_.begin()->timestamp))
               << "\n  End: " << Time::toString(getTimeFrom(gps_fixes_.rbegin()->timestamp)) << "\n";

        if (quality_counts_.size())
        {
            ss << "\nQuality:\n";

            for (auto& qual_it : quality_counts_)
            {
                if (quality_labels.count(qual_it.first))
                    ss << quality_labels.at(qual_it.first) << ": " << qual_it.second
                       << " (" << String::percentToString(100.0*qual_it.second/gps_fixes_.size()) << "%)\n";
                else
                    ss << "Unknown ("<< qual_it.first << "): " << qual_it.second
                       << " (" << String::percentToString(100.0*qual_it.second/gps_fixes_.size()) << "%)\n";
            }
        }
    }
    else
        ss << "Found 0 fixes.";

    current_text_ = ss.str();

    loginf << "parsed " << gps_fixes_.size() << " fixes in "
           << line_cnt << " lines";

    QApplication::restoreOverrideCursor();
}

/**
*/
bool GPSTrailImportTask::canRun() 
{ 
    return canImportFile(); 
}

/**
*/
void GPSTrailImportTask::run()
{
    loginf << "filename '" << current_filename_ << " fixes " << gps_fixes_.size();

    traced_assert(gps_fixes_.size());
    traced_assert(!buffer_);

    COMPASS::instance().logInfo("GPS Trail NMEA Import") << "started";

    if (currentError().size())
        COMPASS::instance().logWarn("GPS Trail NMEA Import") << "errors:\n" << currentError();

    if (currentText().size())
        COMPASS::instance().logInfo("GPS Trail NMEA Import") << currentText();

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
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acad_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_acid_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_num_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_vx_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_vy_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_track_angle_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_x_stddev_));
    traced_assert(dbcontent_man.metaCanGetVariable(dbcontent_name, DBContent::meta_var_y_stddev_));

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
    Variable& ta_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acad_);
    Variable& ti_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_acid_);
    Variable& tn_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_num_);
    Variable& vx_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vx_);
    Variable& vy_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_vy_);
    Variable& speed_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_ground_speed_);
    Variable& track_angle_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_track_angle_);

    Variable& xstddev_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_x_stddev_);
    Variable& ystddev_var = dbcontent_man.metaGetVariable(dbcontent_name, DBContent::meta_var_y_stddev_);

    PropertyList properties;
    properties.addProperty(sac_var.name(), PropertyDataType::UCHAR);
    properties.addProperty(sic_var.name(), PropertyDataType::UCHAR);
    properties.addProperty(ds_id_var.name(), PropertyDataType::UINT);
    properties.addProperty(line_id_var.name(), PropertyDataType::UINT);
    properties.addProperty(tod_var.name(), PropertyDataType::FLOAT);
    properties.addProperty(ts_var.name(), PropertyDataType::TIMESTAMP);
    properties.addProperty(lat_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(long_var.name(), PropertyDataType::DOUBLE);

    properties.addProperty(xstddev_var.name(), PropertyDataType::DOUBLE);
    properties.addProperty(ystddev_var.name(), PropertyDataType::DOUBLE);

    if (settings_.set_mode_3a_code)
        properties.addProperty(m3a_var.name(), PropertyDataType::UINT);

    if (settings_.set_target_address)
        properties.addProperty(ta_var.name(), PropertyDataType::UINT);

    if (settings_.set_callsign)
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

    NullableVector<unsigned int>& tn_vec = buffer_->get<unsigned int>(tn_var.name());

    NullableVector<double>& vx_vec = buffer_->get<double>(vx_var.name());
    NullableVector<double>& vy_vec = buffer_->get<double>(vy_var.name());
    NullableVector<double>& speed_vec = buffer_->get<double>(speed_var.name());
    NullableVector<double>& track_angle_vec = buffer_->get<double>(track_angle_var.name());

    NullableVector<double>& xstddev_vec = buffer_->get<double>(xstddev_var.name());
    NullableVector<double>& ystddev_vec = buffer_->get<double>(ystddev_var.name());

    // BufferWrapper wrap (buffer_);
    // wrap.init();

    // NullableVector<double>& vx_vec = wrap.getNV<double> (DBContent::meta_var_vx_);

    unsigned int ds_id = Number::dsIdFrom(settings_.ds_sac, settings_.ds_sic);

    traced_assert(dbcontent_man.hasMaxRefTrajTrackNum());
    unsigned int track_num = dbcontent_man.maxRefTrajTrackNum();

    loginf << "max reftraj track num " << track_num;

    // config data source
    {
        DataSourceManager& src_man = COMPASS::instance().dataSourceManager();

        if (!src_man.hasConfigDataSource(ds_id))
        {
            loginf << "creating data source";

            src_man.createConfigDataSource(ds_id);
            traced_assert(src_man.hasConfigDataSource(ds_id));
        }

        dbContent::ConfigurationDataSource& src = src_man.configDataSource(ds_id);

        src.name(settings_.ds_name);
        src.dsType(dbcontent_name); // same as dstype

    }

    float tod, last_tod;
    boost::posix_time::ptime timestamp;
    double speed_ms, track_angle_rad, vx, vy;

    // calc stuff
    Transformation trafo_;
    bool ok;
    double calc_x_pos, calc_y_pos, calc_track_angle_rad, calc_speed_ms;
    float d_t;

    loginf << "filling buffer";

    boost::posix_time::ptime override_date_ts = boost::posix_time::ptime(override_date_);

    last_tod = -1; // impossible first value

    for (unsigned int cnt = 0; cnt < gps_fixes_.size(); ++cnt)
    {
        auto fix_it = gps_fixes_.begin() + cnt;

        // tod
        tod = fix_it->timestamp.hour*3600.0 + fix_it->timestamp.min*60.0 + fix_it->timestamp.sec;

        if (settings_.use_tod_offset)
            tod += settings_.tod_offset;

        // check for out-of-bounds because of midnight-jump
        while (tod < 0.0f)
            tod += tod_24h;
        while (tod > tod_24h)
            tod -= tod_24h;

        traced_assert(tod >= 0.0f);
        traced_assert(tod <= tod_24h);

        // timestamp

        if (settings_.use_override_date)
        {
            if (tod < last_tod) // 24h jump
            {
                loginf << "override date, 24h time detected, increasing date";
                override_date_ts += boost::gregorian::days(1);
            }

            timestamp = override_date_ts;
            timestamp += Time::partialSeconds(tod); // add tod + time offset
        }
        else
        {
            timestamp = getTimeFrom(fix_it->timestamp);
        }

        sac_vec.set(cnt, settings_.ds_sac);
        sic_vec.set(cnt, settings_.ds_sic);
        ds_id_vec.set(cnt, ds_id);
        line_id_vec.set(cnt, line_id_);

        tod_vec.set(cnt, tod);
        ts_vec.set(cnt, timestamp);
        lat_vec.set(cnt, fix_it->latitude);
        long_vec.set(cnt, fix_it->longitude);

        if (settings_.set_mode_3a_code)
            buffer_->get<unsigned int>(m3a_var.name()).set(cnt, settings_.mode_3a_code);

        if (settings_.set_target_address)
            buffer_->get<unsigned int>(ta_var.name()).set(cnt, settings_.target_address);

        if (settings_.set_callsign)
            buffer_->get<string>(ti_var.name()).set(cnt, settings_.callsign);

        tn_vec.set(cnt, track_num);

        // groundspeed, track angle
        if (fix_it->travelAngle != 0.0 && fix_it->speed != 0.0)
        {
            track_angle_rad = DEG2RAD * fix_it->travelAngle;
            speed_ms = fix_it->speed * 0.27778;

            track_angle_vec.set(cnt, fix_it->travelAngle);
            speed_vec.set(cnt, speed_ms * M_S2KNOTS);

            vx = sin(track_angle_rad) * speed_ms;
            vy = cos(track_angle_rad) * speed_ms;

            //loginf << "UGA track_angle_rad " << track_angle_rad << " spd " << speed_ms << " vx " << vx << " vy " << vy;

            vx_vec.set(cnt, vx);
            vy_vec.set(cnt, vy);
        }
        else if (cnt > 0) // derive speed vector from lat long positions
        {
            unsigned int prev_cnt = cnt - 1;

            traced_assert(!ts_vec.isNull(cnt) && !ts_vec.isNull(prev_cnt));

            d_t = Time::partialSeconds(ts_vec.get(cnt) - ts_vec.get(cnt - 1));

            if (d_t < 10.0) // limit time between
            {
                tie(ok, calc_x_pos, calc_y_pos) = trafo_.distanceCart(
                            lat_vec.get(prev_cnt), long_vec.get(prev_cnt), lat_vec.get(cnt), long_vec.get(cnt));

                if (!ok)
                {
                    logerr << "error with latitude " << lat_vec.get(cnt)
                           << " longitude " << long_vec.get(cnt);
                }
                else
                {

                    logdbg << "offsets x " << fixed << calc_x_pos
                           << " y " << fixed << calc_y_pos << " dist " << fixed << sqrt(pow(calc_x_pos,2)+pow(calc_y_pos,2));

                    double calc_v_x = calc_x_pos/d_t;
                    double calc_v_y = calc_y_pos/d_t;

                    logdbg << "calc_v_x " << calc_v_x
                           << " calc_v_y " << calc_v_y << " d_t " << d_t;

                    // x_pos long, y_pos lat

                    logdbg << "interpolated lat " << calc_x_pos << " long " << calc_y_pos;

                    calc_track_angle_rad = atan2(calc_v_x,calc_v_y); // bearing rad
                    calc_speed_ms = sqrt(pow(calc_v_x, 2) + pow(calc_v_y, 2));

                    if (!std::isnan(calc_track_angle_rad) && !std::isinf(calc_track_angle_rad)
                            && !std::isnan(calc_speed_ms) && !std::isinf(calc_speed_ms))
                    {
                        track_angle_vec.set(cnt, calc_track_angle_rad * RAD2DEG);
                        speed_vec.set(cnt, calc_speed_ms * M_S2KNOTS);

                        vx_vec.set(cnt, calc_v_x);
                        vy_vec.set(cnt, calc_v_y);
                    }
                }
            }
        }

        // accuracy
        xstddev_vec.set(cnt, fix_it->horizontalAccuracy());
        ystddev_vec.set(cnt, fix_it->horizontalAccuracy());

        last_tod = tod;

        //++cnt;
    }


    dbcontent_man.maxRefTrajTrackNum(track_num+1); // increment for next

    loginf << "inserting data";

    connect(&dbcontent_man, &DBContentManager::insertDoneSignal,
            this, &GPSTrailImportTask::insertDoneSlot, Qt::UniqueConnection);

    dbcontent_man.insertData({{dbcontent_name, buffer_}});
}

/**
*/
void GPSTrailImportTask::insertDoneSlot()
{
    loginf;

    buffer_ = nullptr;

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();

    disconnect(&dbcontent_man, &DBContentManager::insertDoneSignal,
            this, &GPSTrailImportTask::insertDoneSlot);

    COMPASS::instance().dataSourceManager().saveDBDataSources();
    emit COMPASS::instance().dataSourceManager().dataSourcesChangedSignal();
    emit COMPASS::instance().dbContentManager().dbContentStatusChanged();

    COMPASS::instance().logInfo("GPS Trail NMEA Import") << "done with " << gps_fixes_.size() << " GPS fixes";

    done_ = true;

    //    COMPASS::instance().interface().databaseContentChanged();
    //    object.updateToDatabaseContent();

    QMessageBox msg_box;

    msg_box.setWindowTitle("Import GPS Trail");
    msg_box.setText("Import of "+QString::number(gps_fixes_.size())+" GPS fixes done.");
    msg_box.setStandardButtons(QMessageBox::Ok);

    if (allow_user_interactions_)
        msg_box.exec();

    emit doneSignal();
}

/**
*/
void GPSTrailImportTask::dialogImportSlot()
{
    traced_assert(canRun());

    traced_assert(dialog_);
    dialog_->hide();

    run();
}

/**
*/
void GPSTrailImportTask::dialogCancelSlot()
{
    traced_assert(dialog_);
    dialog_->hide();
}

/**
*/
void GPSTrailImportTask::onConfigurationChanged(const std::vector<std::string>& changed_params)
{
    emit configChanged();
}

//void GPSTrailImportTask::checkParsedData ()
//{
//    loginf;
//}
