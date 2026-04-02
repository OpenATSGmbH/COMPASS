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

#include "asteriximportsettings.h"

ASTERIXImportTaskSettings::ASTERIXImportTaskSettings()
    :   reset_date_between_files_ (true)
    ,   ignore_time_jumps_        (false)
    ,   debug_jasterix_           (false)
    ,   current_file_framing_     ("netto")
    ,   num_packets_overload_     (5)
    ,   override_tod_active_      (false)
    ,   override_tod_offset_      (0.0)
    ,   filter_tod_active_        (false)
    ,   filter_tod_min_           (0.0f)
    ,   filter_tod_max_           (86400.0f)
    ,   filter_position_rec_active_   (false)
    ,   filter_rec_latitude_min_      (-90.0)
    ,   filter_rec_latitude_max_      ( 90.0)
    ,   filter_rec_longitude_min_     (-180.0)
    ,   filter_rec_longitude_max_     ( 180.0)
    ,   filter_position_circ_active_  (false)
    ,   filter_circ_latitude_         (0.0)
    ,   filter_circ_longitude_        (0.0)
    ,   filter_circ_range_            (10.0)
    ,   filter_modec_active_      (false)
    ,   filter_modec_min_         (-10000.0f)
    ,   filter_modec_max_         ( 50000.0f)
    ,   file_line_id_             (0)
    ,   date_str_                 ()
    ,   network_ignore_future_ts_ (false)
    ,   obfuscate_secondary_info_ (false)
    ,   date_                     ()
    ,   max_network_lines_        (4)
    ,   chunk_size_jasterix       (20000)
    ,   chunk_size_insert         (50000)
{
}
