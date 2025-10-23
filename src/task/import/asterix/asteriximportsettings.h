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

#include <boost/date_time/posix_time/ptime.hpp>

#include <string>

class ASTERIXImportTaskSettings
{
public:
    ASTERIXImportTaskSettings(); // defines default param values

    // registered
    bool reset_date_between_files_;
    bool ignore_time_jumps_;
    bool debug_jasterix_;
    std::string current_file_framing_;

    unsigned int num_packets_overload_;

    bool override_tod_active_; // not saved
    double override_tod_offset_;

    bool filter_tod_active_; // not saved
    float filter_tod_min_;
    float filter_tod_max_;

    bool filter_position_rec_active_; // not saved
    double filter_rec_latitude_min_;
    double filter_rec_latitude_max_;
    double filter_rec_longitude_min_;
    double filter_rec_longitude_max_;

    bool filter_position_circ_active_; // not saved
    double filter_circ_latitude_;
    double filter_circ_longitude_;
    double filter_circ_range_; // nm

    bool filter_modec_active_; // not saved
    float filter_modec_min_;
    float filter_modec_max_;

    // not saved
    unsigned int file_line_id_;
    std::string date_str_;

    bool network_ignore_future_ts_;

    bool obfuscate_secondary_info_;

    // not in config
    boost::posix_time::ptime date_;
    unsigned int max_network_lines_;

    //import chunk sizes
    unsigned int chunk_size_jasterix;
    unsigned int chunk_size_insert;

    unsigned int max_packets_in_processing_{5};
};