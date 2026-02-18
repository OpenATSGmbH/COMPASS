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

#include "labelgeneratorconfig.h"

namespace dbContent
{

LabelGeneratorConfig::LabelGeneratorConfig()
    : auto_label_ (true)

    , label_directions_           (nlohmann::json::object())
    , label_lines_                (nlohmann::json::object())
    , label_config_               (nlohmann::json::object())
    , label_ds_ids_               (nlohmann::json::object())
    , declutter_labels_           (true)
    , max_declutter_labels_       (200)
    , filter_mode3a_active_       (false)
    , filter_mode3a_values_       ("7000,7777")
    , filter_modec_min_active_    (false)
    , filter_modec_min_value_     (10.0f)
    , filter_modec_max_active_    (false)
    , filter_modec_max_value_     (400.0f)
    , filter_modec_null_wanted_   (false)
    , filter_ti_active_           (false)
    , filter_ti_values_           ("OE")
    , filter_ta_active_           (false)
    , filter_ta_values_           ("AADDCC")
    , filter_primary_only_active_ (false)
    , label_opacity_              (0.9f)
    , use_utn_as_id_ (false)
{
}

//    registerParameter("auto_label", &auto_label_, auto_label_);
//    registerParameter("label_directions", &label_directions_, label_directions_);
//    registerParameter("label_lines", &label_lines_, label_lines_);
//    registerParameter("label_config", &label_config_, label_config_);
//    registerParameter("label_ds_ids", &label_ds_ids_, label_ds_ids_);
//    registerParameter("declutter_labels", &declutter_labels_, declutter_labels_);
//    registerParameter("max_declutter_labels", &max_declutter_labels_, max_declutter_labels_);

//    registerParameter("filter_mode3a_active", &filter_mode3a_active_, filter_mode3a_active_);
//    registerParameter("filter_mode3a_values", &filter_mode3a_values_, filter_mode3a_values_);
//    updateM3AValuesFromStr(filter_mode3a_values_);

//    registerParameter("filter_modec_min_active", &filter_modec_min_active_, filter_modec_min_active_);
//    registerParameter("filter_modec_min_value", &filter_modec_min_value_, filter_modec_min_value_);
//    registerParameter("filter_modec_max_active", &filter_modec_max_active_, filter_modec_max_active_);
//    registerParameter("filter_modec_max_value", &filter_modec_max_value_, filter_modec_max_value_);
//    registerParameter("filter_modec_null_wanted", &filter_modec_null_wanted_, filter_modec_null_wanted_);

//    registerParameter("filter_ti_active", &filter_ti_active_, filter_ti_active_);
//    registerParameter("filter_ti_values", &filter_ti_values_, filter_ti_values_);
//    updateTIValuesFromStr(filter_ti_values_);

//    registerParameter("filter_ta_active", &filter_ta_active_, filter_ta_active_);
//    registerParameter("filter_ta_values", &filter_ta_values_, filter_ta_values_);
//    updateTAValuesFromStr(filter_ta_values_);

//    registerParameter("filter_primary_only_activ", &filter_primary_only_active_, filter_primary_only_active_);

//    registerParameter("label_opacity", &label_opacity_, label_opacity_);

//    registerParameter("auto_lod", &auto_lod_, auto_lod_);
//    registerParameter("current_lod", &current_lod_, current_lod_);
//    registerParameter("use_utn_as_id", &use_utn_as_id_, use_utn_as_id_);

// do gen.updateFilterValuesFromStrings after register

// do checklabels after init

// add label contents to read set

// do updateAvailableLabelLines after loading done, also in live mode

}
