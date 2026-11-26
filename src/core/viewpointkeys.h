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

#include "json_fwd.hpp"

#include <string>

/**
 * JSON keys and format constants of the view points and view point collections.
 * - in core, since db and filter read and write view point data below the view system
 */
class ViewPointKeys
{
public:
    static const std::string VP_COLLECTION_CONTENT_VERSION;
    static const std::string VP_COLLECTION_CONTENT_TYPE;

    static const std::string VP_COLLECTION_CONTENT_VERSION_KEY;
    static const std::string VP_COLLECTION_CONTENT_TYPE_KEY;
    static const std::string VP_COLLECTION_ARRAY_KEY;

    static const std::string VP_CONTEXT_KEY;
    static const std::string VP_CONTEXT_DATASETS_KEY;
    static const std::string VP_CONTEXT_DATASET_FILENAME_KEY;

    static const std::string VP_ID_KEY;
    static const std::string VP_NAME_KEY;
    static const std::string VP_TYPE_KEY ;
    static const std::string VP_STATUS_KEY;
    static const std::string VP_DESCRIPTION_KEY;
    static const std::string VP_COMMENT_KEY;

    static const std::string VP_LOAD_DATA_KEY;
    static const std::string VP_LOAD_ALL;

    // keys inside the load_data object
    static const std::string VP_DS_TYPES_KEY;
    static const std::string VP_DS_KEY;
    static const std::string VP_FILTERS_KEY;
    static const std::string VP_SELECTED_RECNUMS_KEY;
    static const std::string VP_LABELS_KEY;
    static const std::string VP_TIMESTAMP_KEY;
    static const std::string VP_TIME_WIN_KEY;

    static const std::string VP_POS_LAT_KEY;
    static const std::string VP_POS_LON_KEY;
    static const std::string VP_POS_WIN_LAT_KEY;
    static const std::string VP_POS_WIN_LON_KEY;

    static const std::string VP_ANNOTATION_KEY;
};

/**
 * Load statements of a view point, the "load_data" object.
 * - missing or null: the view point loads no data
 * - object: the view point loads data, data_source_types and data_sources are required
 */
namespace view_point_load
{
/// load_data object of the view point, nullptr if the view point loads no data
const nlohmann::json* loadData(const nlohmann::json& view_point);
bool loadsData(const nlohmann::json& view_point);

/// load_data object that loads all data source types and data sources, without filters
nlohmann::json loadAllData();

/// error text if a required key is missing or has a wrong type, empty if the load_data object is valid
std::string checkLoadData(const nlohmann::json& load_data);
}
