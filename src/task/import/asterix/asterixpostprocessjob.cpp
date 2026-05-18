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

#include "dbcontent/dbcontent.h"
#include "asterixpostprocessjob.h"
#include "dbcontent/dbcontentmanager.h"
#include "buffer.h"
#include "compass.h"
#include "files.h"
#include "logger.h"
#include "projectionmanager.h"
#include "util/stringconv.h"
#include "global.h"
#include "json.hpp"

#include <osgEarth/GeoMath>

#include <QThread>

#include <cctype>
#include <fstream>
#include <random>
#include <set>

using namespace std;
using namespace nlohmann;
using namespace Utils;

boost::mutex ASTERIXPostprocessJob::m3a_map_mutex_;
tbb::concurrent_unordered_map<unsigned int, unsigned int> ASTERIXPostprocessJob::obfuscate_m3a_map_;
boost::mutex ASTERIXPostprocessJob::acad_map_mutex_;
tbb::concurrent_unordered_map<unsigned int, unsigned int> ASTERIXPostprocessJob::obfuscate_acad_map_;
boost::mutex ASTERIXPostprocessJob::acid_map_mutex_;
tbb::concurrent_unordered_map<std::string, std::string> ASTERIXPostprocessJob::obfuscate_acid_map_;

namespace
{
// Conspicuity / reserved Mode 3/A codes (octal display -> decimal storage as
// 12-bit value with 3 bits per octal digit). Pass through obfuscation
// unchanged, and excluded from the random pool for newly-generated values.
//   0     -> 0      (no code)
//   01000 -> 512
//   02000 -> 1024
//   07500 -> 3904   (hijack)
//   07600 -> 3968   (radio failure)
//   07700 -> 4032   (general emergency)
//   07776 -> 4094   (reserved)
//   07777 -> 4095   (reserved / military intercept)
const std::set<unsigned int>& m3aConspicuityCodes()
{
    static const std::set<unsigned int> codes = {
        0, 512, 1024, 3904, 3968, 4032, 4094, 4095
    };
    return codes;
}

std::mt19937& obfuscateRng()
{
    static std::mt19937 rng{std::random_device{}()};
    return rng;
}

static const char* kObfuscationFile = "/tmp/compass_obfuscation.json";
}

ASTERIXPostprocessJob::ASTERIXPostprocessJob(
    map<string, shared_ptr<Buffer>> buffers,
    ASTERIXImportTaskSettings settings,
    COMPASS& compass)
    : Job("ASTERIXPostprocessJob"),
    buffers_(std::move(buffers)),
    settings_(settings),
    compass_(compass)
{
}

ASTERIXPostprocessJob::ASTERIXPostprocessJob(map<string, shared_ptr<Buffer>> buffers,
                                             COMPASS& compass)
    : Job("ASTERIXPostprocessJob"),
    buffers_(std::move(buffers)),
    compass_(compass)
{
}

ASTERIXPostprocessJob::~ASTERIXPostprocessJob()
{
    logdbg;
}



void ASTERIXPostprocessJob::run_impl()
{
    logdbg << "start";

    unsigned cnt=0;

    for (auto& buf_it : buffers_)
        cnt += buf_it.second->size();

    logdbg << "num buffers " << buffers_.size() << " size " << cnt;

    started_ = true;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();

    checkARTASHashes();
    doRadarPlotPositionCalculations();
    doXYPositionCalculations();
    doADSBPositionProcessing();
    doGroundSpeedCalculations();

    if (settings_.filter_tod_active_  || settings_.filter_position_rec_active_ 
        || settings_.filter_position_circ_active_ || settings_.filter_modec_active_)
        doFilters();

    if (settings_.obfuscate_secondary_info_)
        doObfuscate();

    auto t_diff = boost::posix_time::microsec_clock::local_time() - start_time;

    unsigned int num_processed = 0;

    for (auto& buf_it : buffers_)
    {
        if (buf_it.second && buf_it.second->size())
            num_processed += buf_it.second->size();
    }

    float num_secs =  t_diff.total_milliseconds() ? t_diff.total_milliseconds() / 1000.0 : 10E-6;

    logdbg << "done: took "
           << String::timeStringFromDouble(num_secs, true)
           << " full " << String::timeStringFromDouble(num_secs, true)
           << " " << ((float) num_processed) / num_secs << " rec/s";

    logdbg << "done";
    done_ = true;
}



void ASTERIXPostprocessJob::checkARTASHashes()
{
    DBContentManager& dbcont_man = compass_.dbContentManager();

    for (auto& buf_it : buffers_)
    {
        const string& dbcontent_name = buf_it.first;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_artas_hash_))
            continue;

        shared_ptr<Buffer> buffer = buf_it.second;
        unsigned int buffer_size = buffer->size();

        if (!buffer_size)
            continue;

        dbContent::Variable& hash_var =
            dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_artas_hash_);

        if (!buffer->has<string>(hash_var.name()))
        {
            logerr << dbcontent_name
                   << " has no ARTAS hash, " << buffer_size << " target reports affected";
            continue;
        }

        NullableVector<string>& hash_vec = buffer->get<string>(hash_var.name());

        unsigned int null_cnt = 0;

        for (unsigned int cnt = 0; cnt < buffer_size; ++cnt)
        {
            if (hash_vec.isNull(cnt))
                ++null_cnt;
        }

        if (null_cnt > 0)
            logerr << dbcontent_name
                   << " " << null_cnt << " of " << buffer_size
                   << " target reports have null ARTAS hash";
    }
}

void ASTERIXPostprocessJob::doRadarPlotPositionCalculations()
{
    // radar calculations
    compass_.projectionManager().doRadarPlotPositionCalculations(buffers_);
}

void ASTERIXPostprocessJob::doXYPositionCalculations()
{
    logdbg;

    // tracked data sources with only x/y coordinates
    compass_.projectionManager().doXYPositionCalculations(buffers_);
}

void ASTERIXPostprocessJob::doADSBPositionProcessing()
{
    DBContentManager& dbcont_man = compass_.dbContentManager();

    string dbcontent_name = "CAT021";

    if (!buffers_.count(dbcontent_name))
        return;

    shared_ptr<Buffer> buffer = buffers_.at(dbcontent_name);
    unsigned int buffer_size = buffer->size();

    if (!buffer_size)
        return;

    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_));
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_));

    traced_assert(dbcont_man.canGetVariable(dbcontent_name, dbcontent_vars::var_cat021_latitude_hr_));
    traced_assert(dbcont_man.canGetVariable(dbcontent_name, dbcontent_vars::var_cat021_longitude_hr_));

    dbContent::Variable& lat_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_);
    dbContent::Variable& lon_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_);

    dbContent::Variable& lat_hr_var = dbcont_man.getVariable(dbcontent_name, dbcontent_vars::var_cat021_latitude_hr_);
    dbContent::Variable& lon_hr_var = dbcont_man.getVariable(dbcontent_name, dbcontent_vars::var_cat021_longitude_hr_);

    traced_assert(lat_var.dataType() == PropertyDataType::DOUBLE);
    traced_assert(lon_var.dataType() == PropertyDataType::DOUBLE);
    traced_assert(lat_hr_var.dataType() == PropertyDataType::DOUBLE);
    traced_assert(lon_hr_var.dataType() == PropertyDataType::DOUBLE);

    string lat_var_name = lat_var.name();
    string lon_var_name = lon_var.name();
    string lat_hr_var_name = lat_hr_var.name();
    string lon_hr_var_name = lon_hr_var.name();

    if (!buffer->has<double>(lat_hr_var_name) || !buffer->has<double>(lon_hr_var_name)) // can not copy
        return;

    if (!buffer->has<double>(lat_var_name))
        buffer->addProperty(lat_var_name, PropertyDataType::DOUBLE); // add if needed

    if (!buffer->has<double>(lon_var_name))
        buffer->addProperty(lon_var_name, PropertyDataType::DOUBLE); // add if needed

    NullableVector<double>& lat_vec = buffer->get<double>(lat_var_name);
    NullableVector<double>& lon_vec = buffer->get<double>(lon_var_name);
    NullableVector<double>& lat_hr_vec = buffer->get<double>(lat_hr_var_name);
    NullableVector<double>& lon_hr_vec = buffer->get<double>(lon_hr_var_name);

    for (unsigned int index=0; index < buffer_size; index++)
    {
        if (!lat_vec.isNull(index) || !lon_vec.isNull(index)) // no need to copy
            continue;

        if (lat_hr_vec.isNull(index) || lon_hr_vec.isNull(index)) // can not copy
            continue;

        lat_vec.set(index, lat_hr_vec.get(index));
        lon_vec.set(index, lon_hr_vec.get(index));
    }
}

void ASTERIXPostprocessJob::doGroundSpeedCalculations()
{
    // general vx/vy to ground speed/track angle conversion

    string dbcontent_name;

    DBContentManager& dbcont_man = compass_.dbContentManager();
    ProjectionManager& proj_man = compass_.projectionManager();

    string vx_var_name;
    string vy_var_name;
    string speed_var_name;
    string track_angle_var_name;

    double speed_ms, track_angle_rad, track_angle_deg;

    for (auto& buf_it : buffers_)
    {
        dbcontent_name = buf_it.first;

        if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_vx_)
            || !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_vy_))
            continue;

        shared_ptr<Buffer> buffer = buf_it.second;
        unsigned int buffer_size = buffer->size();
        //assert(buffer_size);

        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_));
        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_));

        dbContent::Variable& vx_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_vx_);
        dbContent::Variable& vy_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_vy_);
        dbContent::Variable& speed_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_);
        dbContent::Variable& track_angle_var =
            dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_);

        vx_var_name = vx_var.name();
        vy_var_name = vy_var.name();
        speed_var_name = speed_var.name();
        track_angle_var_name = track_angle_var.name();

        traced_assert(vx_var.dataType() == PropertyDataType::DOUBLE);
        traced_assert(vy_var.dataType() == PropertyDataType::DOUBLE);
        traced_assert(speed_var.dataType() == PropertyDataType::DOUBLE);
        traced_assert(track_angle_var.dataType() == PropertyDataType::DOUBLE);

        if (!buffer->has<double>(vx_var_name) || !buffer->has<double>(vy_var_name))
            continue; // cant calculate

        if (buffer->has<double>(speed_var_name) && buffer->has<double>(track_angle_var_name)
            && buffer->get<double>(speed_var_name).isNeverNull()
            && buffer->get<double>(track_angle_var_name).isNeverNull())
        {
            logdbg << "start"
                   << dbcontent_name << " speed and track angle already set";

            continue; // no need for calculation
        }

        if (!buffer->has<double>(speed_var_name))
            buffer->addProperty(speed_var_name, PropertyDataType::DOUBLE); // add if needed

        if (!buffer->has<double>(track_angle_var_name))
            buffer->addProperty(track_angle_var_name, PropertyDataType::DOUBLE); // add if needed

        NullableVector<double>& vx_vec = buffer->get<double>(vx_var_name);
        NullableVector<double>& vy_vec = buffer->get<double>(vy_var_name);
        NullableVector<double>& speed_vec = buffer->get<double>(speed_var_name);
        NullableVector<double>& track_angle_vec = buffer->get<double>(track_angle_var_name);

        unsigned int cnt = 0;

        for (unsigned int index=0; index < buffer_size; index++)
        {
            if (vx_vec.isNull(index) || vy_vec.isNull(index)) // can not calculate
                continue;

            if (!speed_vec.isNull(index) && !track_angle_vec.isNull(index)) // already set
                continue;

            speed_ms = sqrt(pow(vx_vec.get(index), 2)+pow(vy_vec.get(index), 2)) ; // for 1s
            track_angle_rad = atan2(vx_vec.get(index), vy_vec.get(index));

            track_angle_deg = track_angle_rad * RAD2DEG;

            if (track_angle_deg < 0)
                track_angle_deg += 360.0;

            speed_vec.set(index, speed_ms * M_S2KNOTS);
            track_angle_vec.set(index, track_angle_deg);

            ++cnt;
        }

        logdbg << "start"
               << dbcontent_name << " speed and track angle calc " << cnt << " / " << buffer_size;
    }


    // cat021 sgv conversion

    dbcontent_name = "CAT021";

    unsigned int spd_already_set {0}, sgv_spd_no_val {0}, sgv_hgt_no_value {0},
        sgv_is_heading {0}, sgv_is_magnetic {0}, sgv_usable {0};

    logdbg << "got ads-b "
           << (buffers_.count(dbcontent_name) && buffers_.at(dbcontent_name)->size());

    if (buffers_.count(dbcontent_name) && buffers_.at(dbcontent_name)->size())
    {
        auto& buffer = buffers_.at(dbcontent_name);

        unsigned int buffer_size = buffer->size();
        traced_assert(buffer_size);

        if (!buffer->has<float>(dbcontent_vars::var_cat021_sgv_gss_.name()) &&
            buffer->has<bool>(dbcontent_vars::var_cat021_sgv_stp_.name()))
        {  // no speed but sgv stopped bit
            logdbg << "got ads-b no gss but stp";

            unsigned int stp_set {0};

            dbContent::Variable& speed_var =
                dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_);
            traced_assert(speed_var.dataType() == PropertyDataType::DOUBLE);

            NullableVector<double>& speed_vec = buffer->get<double>(speed_var_name);
            NullableVector<bool>& sgv_stp_vec = buffer->get<bool>(dbcontent_vars::var_cat021_sgv_stp_.name());

            for (unsigned int index = 0; index < buffer_size; index++)
            {
                if (!speed_vec.isNull(index))  // already set
                {
                    spd_already_set++;
                    continue;
                }

                if (sgv_stp_vec.isNull(index)) // speed not set
                {
                    ++sgv_spd_no_val;
                    continue;
                }

                if (sgv_stp_vec.get(index))
                {
                    speed_vec.set(index, 0.0);
                    ++stp_set;
                }
            }

            logdbg << "CAT021 spd_already_set " << spd_already_set << " sgv_spd_no_val "
                   << sgv_spd_no_val << " stp_set " << stp_set;

            return;
        }

        logdbg << "got ads-b sgv gss "
               << buffer->has<float>(dbcontent_vars::var_cat021_sgv_gss_.name())
               << " hgt " << buffer->has<double>(dbcontent_vars::var_cat021_sgv_hgt_.name())
               << " htt " << buffer->has<bool>(dbcontent_vars::var_cat021_sgv_htt_.name())
               << " hrd " << buffer->has<bool>(dbcontent_vars::var_cat021_sgv_hrd_.name());

        if (!buffer->has<float>(dbcontent_vars::var_cat021_sgv_gss_.name())
            || !buffer->has<double>(dbcontent_vars::var_cat021_sgv_hgt_.name())
            || !buffer->has<bool>(dbcontent_vars::var_cat021_sgv_htt_.name())
            || !buffer->has<bool>(dbcontent_vars::var_cat021_sgv_hrd_.name()))
            return;

        dbContent::Variable& speed_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_);
        dbContent::Variable& track_angle_var =
            dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_);

        speed_var_name = speed_var.name();
        track_angle_var_name = track_angle_var.name();

        traced_assert(speed_var.dataType() == PropertyDataType::DOUBLE);
        traced_assert(track_angle_var.dataType() == PropertyDataType::DOUBLE);

        NullableVector<double>& speed_vec = buffer->get<double>(speed_var_name);
        NullableVector<double>& track_angle_vec = buffer->get<double>(track_angle_var_name);

        NullableVector<float>& sgv_gss_vec = buffer->get<float>(dbcontent_vars::var_cat021_sgv_gss_.name());
        NullableVector<double>& sgv_hgt_vec = buffer->get<double>(dbcontent_vars::var_cat021_sgv_hgt_.name());
        NullableVector<bool>& sgv_htt_vec = buffer->get<bool>(dbcontent_vars::var_cat021_sgv_htt_.name());
        NullableVector<bool>& sgv_hrd_vec = buffer->get<bool>(dbcontent_vars::var_cat021_sgv_hrd_.name());

        traced_assert(buffer->has<boost::posix_time::ptime>(dbcontent_vars::meta_var_timestamp_.name()));
        NullableVector<boost::posix_time::ptime> ts_vec =
            buffer->get<boost::posix_time::ptime>(dbcontent_vars::meta_var_timestamp_.name());

        NullableVector<double>* lat_vec {nullptr};
        NullableVector<double>* lon_vec {nullptr};
        NullableVector<float>* mode_c_vec {nullptr};

        if(buffer->has<double>(dbcontent_vars::meta_var_latitude_.name())
            && buffer->has<double>(dbcontent_vars::meta_var_longitude_.name()))
        {

            lat_vec = &buffer->get<double>(dbcontent_vars::meta_var_latitude_.name());
            lon_vec = &buffer->get<double>(dbcontent_vars::meta_var_longitude_.name());
        }

        if(buffer->has<float>(dbcontent_vars::meta_var_mc_.name()))
            mode_c_vec = &buffer->get<float>(dbcontent_vars::meta_var_mc_.name());

        // Define position and date parameters
        // double latitude = 37.7749;   // Latitude in degrees (example: San Francisco)
        // double longitude = -122.4194; // Longitude in degrees
        // double altitude = 0;         // Altitude in meters
        // double time = 2024.0;        // Year (decimal format)

        // Magnetic heading angle in degrees (example)
        //double magneticHeading = 45.0;

        for (unsigned int index=0; index < buffer_size; index++)
        {
            if (!speed_vec.isNull(index) && !track_angle_vec.isNull(index)) // already set
            {
                spd_already_set++;
                continue;
            }

            if (sgv_gss_vec.isNull(index)) // speed not set
            {
                ++sgv_spd_no_val;
                continue;
            }

            speed_vec.set(index, sgv_gss_vec.get(index));

            if (sgv_hgt_vec.isNull(index) // heading/track not set or cannot distingush
                || sgv_htt_vec.isNull(index) || sgv_hrd_vec.isNull(index))
            {
                ++sgv_hgt_no_value;
                continue;
            }

            if (sgv_htt_vec.get(index) == 0)
            {
                ++sgv_is_heading;
                continue;
            }

            double true_north_track_angle;

            if (sgv_hrd_vec.get(index) == 1)
            {
                ++sgv_is_magnetic;

                if (lat_vec && lon_vec && !lat_vec->isNull(index) && !lon_vec->isNull(index))
                {
                    traced_assert(!ts_vec.isNull(index));
                    float year = static_cast<float>(ts_vec.get(index).date().year());

                    float altitude_m {0};

                    if (mode_c_vec && !mode_c_vec->isNull(index))
                        altitude_m = mode_c_vec->get(index) * FT2M;

                    double declination = proj_man.declination(year, lat_vec->get(index), lon_vec->get(index), altitude_m);

                    // Calculate the true track by adding declination.
                    true_north_track_angle = sgv_hgt_vec.get(index) + declination;

                    true_north_track_angle = fmod(true_north_track_angle, 360.0);

                    if (true_north_track_angle < 0)
                        true_north_track_angle += 360.0;
                }
                else
                    continue;
            }
            else
                true_north_track_angle = sgv_hgt_vec.get(index);

            track_angle_vec.set(index, true_north_track_angle);

            sgv_usable++; // there
        }

        logdbg << "CAT021 spd_already_set " << spd_already_set
               << " sgv_spd_no_val " << sgv_spd_no_val << " sgv_hgt_no_value " << sgv_hgt_no_value
               << " sgv_is_heading " << sgv_is_heading << " sgv_is_magnetic " << sgv_is_magnetic
               << " sgv_usable " << sgv_usable;
    }
}

void ASTERIXPostprocessJob::doFilters()
{
    string dbcontent_name;

    DBContentManager& dbcont_man = compass_.dbContentManager();

    // do time based filtering first
    if (settings_.filter_tod_active_)
    {
        string tod_var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_time_of_day_));

            dbContent::Variable& tod_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_time_of_day_);

            tod_var_name = tod_var.name();

            traced_assert(buffer->has<float>(tod_var_name));

            NullableVector<float>& tod_vec = buffer->get<float>(tod_var_name);

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                    if (tod_vec.isNull(cnt)
                        || (tod_vec.get(cnt) < settings_.filter_tod_min_ || tod_vec.get(cnt) > settings_.filter_tod_max_))
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // others
    if (settings_.filter_position_rec_active_ || settings_.filter_position_circ_active_ || settings_.filter_modec_active_)
    {
        string lat_var_name;
        string lon_var_name;
        string mc_var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_)
                || !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_)
                || !dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
                continue;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_));
            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_));
            traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_));

            dbContent::Variable& lat_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_);
            dbContent::Variable& lon_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_);
            dbContent::Variable& mc_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_);

            lat_var_name = lat_var.name();
            lon_var_name = lon_var.name();
            mc_var_name = mc_var.name();

            traced_assert(buffer->has<double>(lat_var_name));
            traced_assert(buffer->has<double>(lon_var_name));
            traced_assert(buffer->has<float>(mc_var_name));

            NullableVector<double>& lat_vec = buffer->get<double>(lat_var_name);
            NullableVector<double>& lon_vec = buffer->get<double>(lon_var_name);
            NullableVector<float>& mc_vec = buffer->get<float>(mc_var_name);

            NullableVector<float>* mc_vec2 {nullptr};

            if (dbcontent_name == "CAT062")
            {
                traced_assert(dbcont_man.canGetVariable(dbcontent_name, dbcontent_vars::var_cat062_fl_measured_));
                dbContent::Variable& mc_var2 = dbcont_man.getVariable(dbcontent_name, dbcontent_vars::var_cat062_fl_measured_);

                if (buffer->has<float>(mc_var2.name()))
                    mc_vec2 = &buffer->get<float>(mc_var2.name());
            }

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (settings_.filter_position_rec_active_ && !lat_vec.isNull(cnt) &&
                    !lon_vec.isNull(cnt) &&
                    (lat_vec.get(cnt) < settings_.filter_rec_latitude_min_ ||
                     lat_vec.get(cnt) > settings_.filter_rec_latitude_max_ ||
                     lon_vec.get(cnt) < settings_.filter_rec_longitude_min_ ||
                     lon_vec.get(cnt) > settings_.filter_rec_longitude_max_))
                {
                    to_be_removed.push_back(cnt);
                    continue;
                }

                if (settings_.filter_position_circ_active_ && !lat_vec.isNull(cnt) &&
                    !lon_vec.isNull(cnt))
                {
                    double distance_m = osgEarth::GeoMath::distance(
                        osg::DegreesToRadians(settings_.filter_circ_latitude_),
                        osg::DegreesToRadians(settings_.filter_circ_longitude_),
                        osg::DegreesToRadians(lat_vec.get(cnt)),
                        osg::DegreesToRadians(lon_vec.get(cnt)));

                    logdbg << "distance_m_ " << distance_m << " nm " << distance_m / NM2M
                    << " range " << settings_.filter_circ_range_ << " remove " << (distance_m / NM2M > settings_.filter_circ_range_);

                    if (distance_m / NM2M > settings_.filter_circ_range_)
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
                }

                if (settings_.filter_modec_active_)
                {
                    if (!mc_vec.isNull(cnt)
                        && (mc_vec.get(cnt) < settings_.filter_modec_min_ || mc_vec.get(cnt) > settings_.filter_modec_max_))
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
                    else if (mc_vec2 && !mc_vec2->isNull(cnt)
                             && (mc_vec2->get(cnt) < settings_.filter_modec_min_ || mc_vec2->get(cnt) > settings_.filter_modec_max_))
                    {
                        to_be_removed.push_back(cnt);
                        continue;
                    }
                }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // delete empty ones

    for (auto it = buffers_.cbegin(); it != buffers_.cend() /* not hoisted */; /* no increment */)
    {
        if (!it->second->size())
            buffers_.erase(it++);    // or "it = m.erase(it)" since C++11
        else
            ++it;
    }
}

void ASTERIXPostprocessJob::doObfuscate()
{
    traced_assert(settings_.obfuscate_secondary_info_);

    string dbcontent_name;

    DBContentManager& dbcont_man = compass_.dbContentManager();

    // filter / change mode 3/a codes
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        string var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_m3a_);

            var_name = var.name();

            traced_assert(buffer->has<unsigned int>(var_name));

            NullableVector<unsigned int>& var_vec = buffer->get<unsigned int>(var_name);

            std::vector<unsigned int> to_be_removed;

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (var_vec.isNull(cnt))
                    continue;

                if ((var_vec.get(cnt) >= 832 && var_vec.get(cnt) <= 895) // 1500 - 1577
                    || (var_vec.get(cnt) >= 2560 && var_vec.get(cnt) <= 3071)) // 5000 - 5777
                {
                    to_be_removed.push_back(cnt);
                    continue;
                }
                else // obfuscate
                {
                    unsigned int tmp = var_vec.get(cnt);
                    obfuscateM3A(tmp);
                    var_vec.set(cnt, tmp);
                }
            }

            buffer->removeIndexes(to_be_removed);
        }
    }

    // change acads
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        string var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acad_);

            var_name = var.name();

            traced_assert(buffer->has<unsigned int>(var_name));

            NullableVector<unsigned int>& var_vec = buffer->get<unsigned int>(var_name);

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (var_vec.isNull(cnt))
                    continue;

                // obfuscate
                {
                    unsigned int tmp = var_vec.get(cnt);
                    obfuscateACAD(tmp);
                    var_vec.set(cnt, tmp);
                }
            }
        }
    }

    // change acids
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);
        string var_name;

        for (auto& buf_it : buffers_)
        {
            dbcontent_name = buf_it.first;

            shared_ptr<Buffer> buffer = buf_it.second;
            unsigned int buffer_size = buffer->size();

            if(!buffer_size)
                continue;

            if (!dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_))
                continue;

            dbContent::Variable& var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_acid_);

            var_name = var.name();

            traced_assert(buffer->has<string>(var_name));

            NullableVector<string>& var_vec = buffer->get<string>(var_name);

            for (unsigned int cnt=0; cnt < buffer_size; ++cnt)
            {
                if (var_vec.isNull(cnt))
                    continue;

                // obfuscate
                {
                    std::string tmp = var_vec.get(cnt);
                    obfuscateACID(tmp);
                    var_vec.set(cnt, tmp);
                }
            }
        }
    }
}

void ASTERIXPostprocessJob::obfuscateM3A (unsigned int& value)
{
    // Conspicuity codes pass through unchanged - their operational meaning is
    // part of the data, not the privacy concern.
    const auto& conspicuity = m3aConspicuityCodes();
    if (conspicuity.count(value))
    {
        obfuscate_m3a_map_[value] = value;
        return;
    }

    if (auto it = obfuscate_m3a_map_.find(value); it != obfuscate_m3a_map_.end())
    {
        value = it->second;
        return;
    }

    // Collect already-used target values to avoid collisions.
    std::set<unsigned int> used;
    for (const auto& kv : obfuscate_m3a_map_)
        used.insert(kv.second);

    std::uniform_int_distribution<unsigned int> dist(0, 4095);
    auto& rng = obfuscateRng();

    unsigned int candidate = 0;
    for (int attempts = 0; attempts < 10000; ++attempts)
    {
        candidate = dist(rng);
        if (!conspicuity.count(candidate) && !used.count(candidate))
            break;
    }

    obfuscate_m3a_map_[value] = candidate;
    value = candidate;
}

void ASTERIXPostprocessJob::obfuscateACAD (unsigned int& value)
{
    if (auto it = obfuscate_acad_map_.find(value); it != obfuscate_acad_map_.end())
    {
        value = it->second;
        return;
    }

    std::set<unsigned int> used;
    for (const auto& kv : obfuscate_acad_map_)
        used.insert(kv.second);

    // 24-bit ICAO aircraft address: 0x000001 .. 0xFFFFFF (0 is reserved).
    std::uniform_int_distribution<unsigned int> dist(1, 0xFFFFFF);
    auto& rng = obfuscateRng();

    unsigned int candidate = 0;
    for (int attempts = 0; attempts < 10000; ++attempts)
    {
        candidate = dist(rng);
        if (!used.count(candidate))
            break;
    }

    obfuscate_acad_map_[value] = candidate;
    value = candidate;
}

// Pool of fictional starship names from Star Trek, Star Wars and The Expanse.
// Stored as full uppercase strings; the obfuscator slices each to the
// letter-count of the callsign being replaced. Long names (>= 8 chars)
// guarantee coverage for any plausible ACID letter prefix.
static const std::vector<std::string> ship_names = {
    // Star Trek
    "ENTERPRISE", "VOYAGER", "DEFIANT", "DISCOVERY", "RELIANT", "EXCELSIOR",
    "EQUINOX", "INTREPID", "TITAN", "ODYSSEY", "CONSTELLATION", "STARGAZER",
    "GRISSOM", "SARATOGA", "COCHRANE", "PICARD", "JANEWAY", "SHENZHOU",
    "YORKTOWN", "BURAN", "THUNDERCHILD", "FRANKLIN", "VENGEANCE", "SHRAN",
    "LEXINGTON", "POTEMKIN", "ENDEAVOUR", "COLUMBIA", "KELVIN", "ANTARES",
    "DAEDALUS", "NEBULA", "AKIRA", "PROMETHEUS", "NORWAY", "STEAMRUNNER",
    "EXETER", "MIRANDA", "SOVEREIGN", "AVENGER", "FARRAGUT", "BOZEMAN",
    "CAIRO", "EXCALIBUR", "GALAXY", "HATHAWAY", "MAGELLAN", "YAMAGUCHI",
    "BLACKWELL", "CHALLENGER", "CHARLESTON", "CHEROKEE", "CONCORD",
    "COPERNICUS", "DERBYSHIRE", "DRAKE", "EDISON", "GAGARIN", "GRIFFIN",
    "HANSEN", "MARYLAND", "OBERTH", "OLIVER", "ORION", "PEGASUS",
    "SAGAN", "NIAGARA", "NIMITZ", "GETTYSBURG", "KONGO", "SAREK",
    // Star Wars
    "FALCON", "TANTIVE", "RAZORCREST", "GHOST", "PHANTOM", "EXECUTOR",
    "ECLIPSE", "INTERCEPTOR", "MALEVOLENCE", "INDEPENDENCE", "LIBERTY",
    "PROFUNDITY", "RADDUS", "SUPREMACY", "FINALIZER", "CHIMAERA",
    "ARMAGEDDON", "DEVASTATOR", "INVINCIBLE", "ACCUSER", "RAVAGER",
    "RESOLUTE", "VENATOR", "ACCLAMATOR", "STARDESTROYER", "INVISIBLE",
    // The Expanse
    "ROCINANTE", "TACHI", "DONNAGER", "NAUVOO", "TYNAN", "AGATHAKING",
    "BEHEMOTH", "CONTORTA", "PELLA", "SCOPULI", "BARBAPICCOLA", "RAWESIDE",
    "HYGIEA", "ARBOGHAST", "RAZORBACK", "CHETZEMOKA", "CANTERBURY", "TYCHO",
    "DEWALT", "MAELSTROM", "MEDINA", "MORRIGAN", "BLUEFALCON", "OKIMBO",
    "SCIROCCO", "GUANSHIYIN", "SEUNGUN"
};

namespace
{
inline bool isAsciiLetter(char c) { return std::isalpha(static_cast<unsigned char>(c)); }
inline bool isAsciiDigit (char c) { return std::isdigit(static_cast<unsigned char>(c)); }

// Strip trailing whitespace; uppercase ASCII. Used to canonicalise the
// lookup key so that "XY100", "XY100   " and "xy100" all collapse to one
// entry.
std::string normaliseAcid(const std::string& s)
{
    auto end = s.find_last_not_of(" \t\r\n");
    if (end == std::string::npos)
        return std::string();
    std::string out = s.substr(0, end + 1);
    for (auto& c : out)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

// One contiguous run of input characters of the same class (letter / digit
// / other). Captures the original length and characters; "other" runs are
// passed through unchanged.
struct AcidRun
{
    enum class Kind { Letter, Digit, Other };
    Kind        kind;
    std::string text; // original characters, only meaningful for Other
};

std::vector<AcidRun> splitIntoRuns(const std::string& s)
{
    std::vector<AcidRun> runs;
    std::size_t i = 0;
    while (i < s.size())
    {
        AcidRun::Kind kind = isAsciiLetter(s[i]) ? AcidRun::Kind::Letter
                          : isAsciiDigit (s[i]) ? AcidRun::Kind::Digit
                                               : AcidRun::Kind::Other;
        std::size_t j = i;
        while (j < s.size())
        {
            AcidRun::Kind k = isAsciiLetter(s[j]) ? AcidRun::Kind::Letter
                            : isAsciiDigit (s[j]) ? AcidRun::Kind::Digit
                                                 : AcidRun::Kind::Other;
            if (k != kind) break;
            ++j;
        }
        AcidRun r;
        r.kind = kind;
        r.text = s.substr(i, j - i);
        runs.push_back(std::move(r));
        i = j;
    }
    return runs;
}

// Leading slice of a ship name at index `ship_idx` (wrapping). If the ship
// is shorter than `len`, pad with random A-Z. Subsequent letter runs in
// the same callsign should pass a different `ship_idx` (caller advances)
// so the output reads as multiple ship prefixes rather than one repeated.
std::string letterRunLeading(std::size_t ship_idx, std::size_t len, std::mt19937& rng)
{
    if (len == 0)
        return std::string();
    const std::string& name = ship_names[ship_idx % ship_names.size()];
    if (name.size() >= len)
        return name.substr(0, len);
    std::uniform_int_distribution<int> letter_pick(0, 25);
    std::string out = name;
    while (out.size() < len)
        out += static_cast<char>('A' + letter_pick(rng));
    return out;
}

// 10^n with overflow guard - clamped so callsigns with absurdly long digit
// runs don't blow up the integer space we use for the increment counter.
std::uint64_t pow10Clamped(std::size_t n)
{
    std::uint64_t v = 1;
    for (std::size_t i = 0; i < n && i < 18; ++i)
        v *= 10;
    return v;
}

// Total number of digit characters across all digit runs - the digit slots
// are filled from a single zero-padded counter, so an increment by 1
// monotonically walks the joint digit space.
std::size_t totalDigitSlots(const std::vector<AcidRun>& runs)
{
    std::size_t n = 0;
    for (const auto& r : runs)
        if (r.kind == AcidRun::Kind::Digit)
            n += r.text.size();
    return n;
}

// Materialise a candidate: letter runs use leading slices of ship_names
// starting at `ship_idx` (each successive letter run advances by one);
// digit runs are filled from `digit_value` rendered as a zero-padded
// string of length `totalDigitSlots(runs)`, split across the digit runs.
std::string generateLeadingAcid(const std::vector<AcidRun>& runs,
                                std::size_t ship_idx,
                                std::uint64_t digit_value,
                                std::mt19937& rng)
{
    const std::size_t total_digits = totalDigitSlots(runs);
    std::string digit_str;
    if (total_digits > 0)
    {
        const std::uint64_t mod = pow10Clamped(total_digits);
        if (mod > 0)
            digit_value %= mod;
        digit_str = std::to_string(digit_value);
        if (digit_str.size() < total_digits)
            digit_str = std::string(total_digits - digit_str.size(), '0') + digit_str;
        else if (digit_str.size() > total_digits)
            digit_str = digit_str.substr(digit_str.size() - total_digits);
    }

    std::string out;
    out.reserve(8);
    std::size_t digit_cursor = 0;
    std::size_t letter_run_idx = 0;
    for (const auto& r : runs)
    {
        switch (r.kind)
        {
        case AcidRun::Kind::Letter:
            out += letterRunLeading(ship_idx + letter_run_idx, r.text.size(), rng);
            ++letter_run_idx;
            break;
        case AcidRun::Kind::Digit:
            out.append(digit_str, digit_cursor, r.text.size());
            digit_cursor += r.text.size();
            break;
        case AcidRun::Kind::Other:
            out += r.text;
            break;
        }
    }
    return out;
}

// Pad `s` with trailing spaces up to `target_len`. If `s` is already that
// long or longer, returned unchanged.
std::string padTo(const std::string& s, std::size_t target_len)
{
    if (s.size() >= target_len)
        return s;
    return s + std::string(target_len - s.size(), ' ');
}
}

void ASTERIXPostprocessJob::obfuscateACID (std::string& value)
{
    const std::size_t original_len = value.size();
    const std::string key = normaliseAcid(value);

    // Empty / all-whitespace input: nothing meaningful to obfuscate.
    if (key.empty())
    {
        value = std::string(original_len, ' ');
        return;
    }

    if (auto it = obfuscate_acid_map_.find(key); it != obfuscate_acid_map_.end())
    {
        value = padTo(it->second, original_len);
        return;
    }

    const auto runs = splitIntoRuns(key);
    auto& rng = obfuscateRng();

    const std::size_t total_digits = totalDigitSlots(runs);
    const std::uint64_t digit_mod  = pow10Clamped(total_digits);
    const std::size_t pool_size    = ship_names.size();

    // Initial picks. The ship_idx selects a starting ship for the first
    // letter run; on collision we keep the ship_idx and increment the
    // composite digit value, so the readable "ship prefix" stays stable.
    // Only when the digit space is exhausted do we step to the next ship.
    std::uniform_int_distribution<std::size_t> ship_dist(0, pool_size - 1);
    std::size_t ship_idx = ship_dist(rng);
    std::uint64_t digit_value = total_digits > 0
        ? (std::uniform_int_distribution<std::uint64_t>(0, digit_mod - 1))(rng)
        : 0;

    std::string candidate;
    std::uint64_t digit_attempts = 0;
    std::size_t  ship_attempts   = 0;
    const std::uint64_t digit_budget = total_digits > 0 ? digit_mod : 1;

    while (ship_attempts < pool_size)
    {
        candidate = generateLeadingAcid(runs, ship_idx, digit_value, rng);

        bool collision = (candidate == key);
        if (!collision)
        {
            for (const auto& kv : obfuscate_acid_map_)
                if (kv.second == candidate) { collision = true; break; }
        }
        if (!collision)
            break;

        ++digit_attempts;
        if (total_digits > 0 && digit_attempts < digit_budget)
        {
            digit_value = (digit_value + 1) % digit_mod;
            continue;
        }
        // Digit space exhausted (or none) - advance to the next ship and
        // pick a fresh starting digit value.
        ++ship_attempts;
        ship_idx = (ship_idx + 1) % pool_size;
        digit_attempts = 0;
        if (total_digits > 0)
            digit_value = (std::uniform_int_distribution<std::uint64_t>(0, digit_mod - 1))(rng);
    }

    if (ship_attempts >= pool_size)
    {
        // Pathological saturation - extremely unlikely. Fall back to a
        // random-letters / random-digits rendering, accepting that it may
        // not be a recognisable ship prefix.
        std::uniform_int_distribution<int> letter_pick(0, 25);
        std::uniform_int_distribution<int> digit_pick(0, 9);
        std::string fallback;
        fallback.reserve(key.size());
        for (char c : key)
        {
            if (isAsciiLetter(c))      fallback += static_cast<char>('A' + letter_pick(rng));
            else if (isAsciiDigit(c))  fallback += static_cast<char>('0' + digit_pick(rng));
            else                       fallback += c;
        }
        candidate = std::move(fallback);
    }

    obfuscate_acid_map_[key] = candidate;
    value = padTo(candidate, original_len);
}

void ASTERIXPostprocessJob::loadObfuscationMaps()
{
    static bool loaded = false;
    if (loaded)
        return;

    if (!Utils::Files::fileExists(kObfuscationFile))
    {
        loaded = true;
        return;
    }

    nlohmann::json j;
    try
    {
        std::ifstream in(kObfuscationFile);
        in >> j;
    }
    catch (const std::exception& e)
    {
        logwrn << "could not parse " << kObfuscationFile << ": " << e.what();
        return; // do not flip 'loaded'; allow a retry next time
    }

    if (!j.is_object())
    {
        logwrn << kObfuscationFile << " is not a JSON object, ignoring";
        loaded = true;
        return;
    }

    if (j.contains("m3a") && j.at("m3a").is_object())
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        for (auto& kv : j.at("m3a").items())
        {
            try
            {
                obfuscate_m3a_map_[static_cast<unsigned int>(std::stoul(kv.key()))]
                    = kv.value().get<unsigned int>();
            }
            catch (...) { /* skip malformed entry */ }
        }
    }
    if (j.contains("acad") && j.at("acad").is_object())
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        for (auto& kv : j.at("acad").items())
        {
            try
            {
                obfuscate_acad_map_[static_cast<unsigned int>(std::stoul(kv.key()))]
                    = kv.value().get<unsigned int>();
            }
            catch (...) {}
        }
    }
    if (j.contains("acid") && j.at("acid").is_object())
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);
        for (auto& kv : j.at("acid").items())
        {
            try
            {
                obfuscate_acid_map_[kv.key()] = kv.value().get<std::string>();
            }
            catch (...) {}
        }
    }

    loginf << "loaded obfuscation maps from " << kObfuscationFile
           << ": m3a " << obfuscate_m3a_map_.size()
           << " acad " << obfuscate_acad_map_.size()
           << " acid " << obfuscate_acid_map_.size();
    loaded = true;
}

void ASTERIXPostprocessJob::saveObfuscationMaps()
{
    nlohmann::json j;
    j["m3a"]  = nlohmann::json::object();
    j["acad"] = nlohmann::json::object();
    j["acid"] = nlohmann::json::object();
    {
        boost::mutex::scoped_lock locker(m3a_map_mutex_);
        for (const auto& kv : obfuscate_m3a_map_)
            j["m3a"][std::to_string(kv.first)] = kv.second;
    }
    {
        boost::mutex::scoped_lock locker(acad_map_mutex_);
        for (const auto& kv : obfuscate_acad_map_)
            j["acad"][std::to_string(kv.first)] = kv.second;
    }
    {
        boost::mutex::scoped_lock locker(acid_map_mutex_);
        for (const auto& kv : obfuscate_acid_map_)
            j["acid"][kv.first] = kv.second;
    }
    try
    {
        std::ofstream out(kObfuscationFile);
        out << j.dump(2);
    }
    catch (const std::exception& e)
    {
        logwrn << "could not write " << kObfuscationFile << ": " << e.what();
    }
}

