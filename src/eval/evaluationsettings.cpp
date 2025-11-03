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

#include "evaluationsettings.h"

/**
 */
EvaluationSettings::EvaluationSettings()
:   line_id_ref_                      (0)
,   active_sources_ref_               ()
,   line_id_tst_                      (0)
,   active_sources_tst_               ()
,   current_standard_                 ("")
,   use_grp_in_sector_                ()
,   use_requirement_                  ()
// ,   max_ref_time_diff_                (4.0)
// ,   use_load_filter_                  (false)
// ,   use_ref_traj_accuracy_filter_     (false)
// ,   ref_traj_minimum_accuracy_        (30.0f)
// ,   use_adsb_filter_                  (false)
// ,   use_v0_                           (true)
// ,   use_v1_                           (true)
// ,   use_v2_                           (true)
// ,   use_min_nucp_                     (true)
// ,   min_nucp_                         (4u)
// ,   use_max_nucp_                     (true)
// ,   max_nucp_                         (4u)
// ,   use_min_nic_                      (true)
// ,   min_nic_                          (5u)
// ,   use_max_nic_                      (true)
// ,   max_nic_                          (5u)
// ,   use_min_nacp_                     (true)
// ,   min_nacp_                         (5u)
// ,   use_max_nacp_                     (true)
// ,   max_nacp_                         (5u)
// ,   use_min_sil_v1_                   (true)
// ,   min_sil_v1_                       (2u)
// ,   use_max_sil_v1_                   (true)
// ,   max_sil_v1_                       (2u)
// ,   use_min_sil_v2_                   (true)
// ,   min_sil_v2_                       (4u)
// ,   use_max_sil_v2_                   (true)
// ,   max_sil_v2_                       (4u)
,   result_detail_zoom_               (0.02)
,   min_height_filter_layer_          ("")
,   report_skip_no_data_details_      (true)
,   report_split_results_by_mops_     (false)
,   report_split_results_by_aconly_ms_(false)
,   show_ok_joined_target_reports_    (false)
,   load_only_sector_data_            (true)
,   dbcontent_name_ref_               ("RefTraj")
,   dbcontent_name_tst_               ("CAT062")
{
}
