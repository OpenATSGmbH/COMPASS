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

#include "json.hpp"

/**
 */
struct EvaluationSettings
{
    EvaluationSettings();

    unsigned int line_id_ref_;
    nlohmann::json active_sources_ref_; // config var for data_sources_ref_

    unsigned int line_id_tst_;
    nlohmann::json active_sources_tst_; // config var for active_sources_tst_

    std::string current_standard_;
    //std::string current_config_name_;

    nlohmann::json use_grp_in_sector_; //standard_name->sector_layer_name->req_grp_name->bool use
    nlohmann::json use_requirement_; // standard_name->req_grp_name->req_grp_name->bool use

    // load filter
    //bool use_load_filter_ {false};

    //bool use_ref_traj_accuracy_filter_ {false};
    //float ref_traj_minimum_accuracy_ {30};

    // bool use_adsb_filter_ {false};
    // bool use_v0_ {false};
    // bool use_v1_ {false};
    // bool use_v2_ {false};

    // // nucp
    // bool use_min_nucp_ {false};
    // unsigned int min_nucp_ {0};

    // bool use_max_nucp_ {false};
    // unsigned int max_nucp_ {0};

    // // nic
    // bool use_min_nic_ {false};
    // unsigned int min_nic_ {0};

    // bool use_max_nic_ {false};
    // unsigned int max_nic_ {0};

    // // nacp
    // bool use_min_nacp_ {false};
    // unsigned int min_nacp_ {0};

    // bool use_max_nacp_ {false};
    // unsigned int max_nacp_ {0};

    // // sil v1
    // bool use_min_sil_v1_ {false};
    // unsigned int min_sil_v1_ {0};

    // bool use_max_sil_v1_ {false};
    // unsigned int max_sil_v1_ {0};

    // // sil v2
    // bool use_min_sil_v2_ {false};
    // unsigned int min_sil_v2_ {0};

    // bool use_max_sil_v2_ {false};
    // unsigned int max_sil_v2_ {0};

    double result_detail_zoom_ {0.0}; // in WGS84 deg

    std::string min_height_filter_layer_; //layer used as minimum height filter

    // report stuff
    bool report_skip_no_data_details_ {true};
    bool report_split_results_by_mops_ {false};
    bool report_split_results_by_aconly_ms_ {false};

    bool show_ok_joined_target_reports_ {false};

    //grid generation
    unsigned int grid_num_cells_x     = 512;
    unsigned int grid_num_cells_y     = 512;

    //histogram generation
    unsigned int histogram_num_bins = 20;

    //not written to config
    bool load_only_sector_data_ {true};

private:
    friend class EvaluationCalculator;

    // private since specific setter functions
    std::string dbcontent_name_ref_;
    std::string dbcontent_name_tst_;
};
