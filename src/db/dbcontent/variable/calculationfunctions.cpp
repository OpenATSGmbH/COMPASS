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

#include "dbcontent/variable/calculationfunctions.h"
#include "dbcontent/variable/variable.h"
#include "dbcontent/dbcontentmanager.h"
#include "idbvariableresolver.h"
#include "projectionmanager.h"
#include "compass.h"
#include "buffer.h"
#include "global.h"
#include "logger.h"
#include "traced_assert.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include <cmath>

using namespace std;

namespace dbContent
{

namespace calculation_functions
{

void radarProjection(COMPASS& compass, const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer)
{
    compass.projectionManager().doRadarPlotPositionCalculations({{dbcontent_name, buffer}});
}

void adsbSGVSpeedTrack(COMPASS& compass, const std::string& dbcontent_name, const std::shared_ptr<Buffer>& buffer)
{
    DBContentManager& dbcont_man = compass.dbContentManager();
    ProjectionManager& proj_man = compass.projectionManager();

    string speed_var_name;
    string track_angle_var_name;

    if (buffer->size())
    {
        unsigned int buffer_size = buffer->size();

        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_));
        traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_));

        dbContent::Variable& speed_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_ground_speed_);
        dbContent::Variable& track_angle_var =
            dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_track_angle_);

        traced_assert(speed_var.dataType() == PropertyDataType::DOUBLE);
        traced_assert(track_angle_var.dataType() == PropertyDataType::DOUBLE);

        speed_var_name = speed_var.name();
        track_angle_var_name = track_angle_var.name();

        if (!buffer->has<double>(speed_var_name))
            buffer->addProperty(speed_var_name, PropertyDataType::DOUBLE); // add if needed

        if (!buffer->has<double>(track_angle_var_name))
            buffer->addProperty(track_angle_var_name, PropertyDataType::DOUBLE); // add if needed

        NullableVector<double>& speed_vec = buffer->get<double>(speed_var_name);
        NullableVector<double>& track_angle_vec = buffer->get<double>(track_angle_var_name);

        // decoded I021/160 values first

        unsigned int decoded_spd {0}, decoded_ta {0};

        if (buffer->has<double>(dbcontent_vars::var_cat021_ground_speed_.name()))
        {
            NullableVector<double>& decoded_vec =
                buffer->get<double>(dbcontent_vars::var_cat021_ground_speed_.name());

            for (unsigned int index=0; index < buffer_size; index++)
            {
                if (speed_vec.isNull(index) && !decoded_vec.isNull(index))
                {
                    speed_vec.set(index, decoded_vec.get(index));
                    ++decoded_spd;
                }
            }
        }

        if (buffer->has<double>(dbcontent_vars::var_cat021_track_angle_.name()))
        {
            NullableVector<double>& decoded_vec =
                buffer->get<double>(dbcontent_vars::var_cat021_track_angle_.name());

            for (unsigned int index=0; index < buffer_size; index++)
            {
                if (track_angle_vec.isNull(index) && !decoded_vec.isNull(index))
                {
                    track_angle_vec.set(index, decoded_vec.get(index));
                    ++decoded_ta;
                }
            }
        }

        logdbg << "CAT021 decoded speed " << decoded_spd << " track angle " << decoded_ta
               << " of " << buffer_size;

        // then the SGV fields

        unsigned int spd_already_set {0}, sgv_spd_no_val {0}, sgv_hgt_no_value {0},
            sgv_is_heading {0}, sgv_is_magnetic {0}, sgv_usable {0};

        bool has_gss = buffer->has<float>(dbcontent_vars::var_cat021_sgv_gss_.name());
        bool has_stp = buffer->has<bool>(dbcontent_vars::var_cat021_sgv_stp_.name());
        bool has_hgt = buffer->has<double>(dbcontent_vars::var_cat021_sgv_hgt_.name());
        bool has_htt = buffer->has<bool>(dbcontent_vars::var_cat021_sgv_htt_.name());
        bool has_hrd = buffer->has<bool>(dbcontent_vars::var_cat021_sgv_hrd_.name());

        logdbg << "got ads-b sgv gss " << has_gss << " stp " << has_stp << " hgt " << has_hgt
               << " htt " << has_htt << " hrd " << has_hrd;

        if (!has_gss)
        {
            if (has_stp)
            {  // no speed but sgv stopped bit, a stopped target has speed 0
                unsigned int stp_set {0};

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
            }

            return;
        }

        if (!has_hgt || !has_htt || !has_hrd)
            return;

        NullableVector<float>& sgv_gss_vec = buffer->get<float>(dbcontent_vars::var_cat021_sgv_gss_.name());
        NullableVector<double>& sgv_hgt_vec = buffer->get<double>(dbcontent_vars::var_cat021_sgv_hgt_.name());
        NullableVector<bool>& sgv_htt_vec = buffer->get<bool>(dbcontent_vars::var_cat021_sgv_htt_.name());
        NullableVector<bool>& sgv_hrd_vec = buffer->get<bool>(dbcontent_vars::var_cat021_sgv_hrd_.name());

        dbContent::Variable& ts_var = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_timestamp_);

        traced_assert(buffer->has<boost::posix_time::ptime>(ts_var.name()));
        NullableVector<boost::posix_time::ptime> ts_vec = buffer->get<boost::posix_time::ptime>(ts_var.name());

        // position and altitude for the declination, "Latitude*" and "Longitude*" through the Meta

        NullableVector<double>* lat_vec {nullptr};
        NullableVector<double>* lon_vec {nullptr};
        NullableVector<float>* mode_c_vec {nullptr};

        if (dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_)
            && dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_))
        {
            std::string lat_var_name = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_latitude_).name();
            std::string lon_var_name = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_longitude_).name();

            if (buffer->has<double>(lat_var_name) && buffer->has<double>(lon_var_name))
            {
                lat_vec = &buffer->get<double>(lat_var_name);
                lon_vec = &buffer->get<double>(lon_var_name);
            }
        }

        if (dbcont_man.metaCanGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_))
        {
            std::string mode_c_var_name = dbcont_man.metaGetVariable(dbcontent_name, dbcontent_vars::meta_var_mc_).name();

            if (buffer->has<float>(mode_c_var_name))
                mode_c_vec = &buffer->get<float>(mode_c_var_name);
        }

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

}

}
