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

#include "viewpointkeys.h"

#include "json.hpp"

const std::string ViewPointKeys::VP_COLLECTION_CONTENT_VERSION {"0.4"};
const std::string ViewPointKeys::VP_COLLECTION_CONTENT_TYPE {"view_points"};

const std::string ViewPointKeys::VP_COLLECTION_CONTENT_VERSION_KEY {"content_version"};
const std::string ViewPointKeys::VP_COLLECTION_CONTENT_TYPE_KEY {"content_type"};
const std::string ViewPointKeys::VP_COLLECTION_ARRAY_KEY {"view_points"};

const std::string ViewPointKeys::VP_CONTEXT_KEY {"view_point_context"};
const std::string ViewPointKeys::VP_CONTEXT_DATASETS_KEY {"datasets"};
const std::string ViewPointKeys::VP_CONTEXT_DATASET_FILENAME_KEY {"filename"};

const std::string ViewPointKeys::VP_ID_KEY {"id"};
const std::string ViewPointKeys::VP_NAME_KEY {"name"};
const std::string ViewPointKeys::VP_TYPE_KEY {"type"};
const std::string ViewPointKeys::VP_STATUS_KEY {"status"};
const std::string ViewPointKeys::VP_COMMENT_KEY {"comment"};
const std::string ViewPointKeys::VP_DESCRIPTION_KEY {"description"};

const std::string ViewPointKeys::VP_LOAD_DATA_KEY {"load_data"};
const std::string ViewPointKeys::VP_LOAD_ALL {"all"};

const std::string ViewPointKeys::VP_DS_TYPES_KEY {"data_source_types"};
const std::string ViewPointKeys::VP_DS_KEY {"data_sources"};
const std::string ViewPointKeys::VP_FILTERS_KEY {"filters"};
const std::string ViewPointKeys::VP_SELECTED_RECNUMS_KEY {"selected_rec_nums"};
const std::string ViewPointKeys::VP_LABELS_KEY {"labels"};
const std::string ViewPointKeys::VP_TIMESTAMP_KEY {"timestamp"};
const std::string ViewPointKeys::VP_TIME_WIN_KEY {"time_window"};

const std::string ViewPointKeys::VP_POS_LAT_KEY {"position_latitude"};
const std::string ViewPointKeys::VP_POS_LON_KEY {"position_longitude"};
const std::string ViewPointKeys::VP_POS_WIN_LAT_KEY {"position_window_latitude"};
const std::string ViewPointKeys::VP_POS_WIN_LON_KEY {"position_window_longitude"};

const std::string ViewPointKeys::VP_ANNOTATION_KEY {"annotations"};

namespace view_point_load
{

const nlohmann::json* loadData(const nlohmann::json& view_point)
{
    if (!view_point.is_object())
        return nullptr;

    auto it = view_point.find(ViewPointKeys::VP_LOAD_DATA_KEY);

    if (it == view_point.end() || !it->is_object())
        return nullptr;

    return &(*it);
}

bool loadsData(const nlohmann::json& view_point)
{
    return loadData(view_point) != nullptr;
}

nlohmann::json loadAllData()
{
    nlohmann::json load_data = nlohmann::json::object();

    load_data[ViewPointKeys::VP_DS_TYPES_KEY] = ViewPointKeys::VP_LOAD_ALL;
    load_data[ViewPointKeys::VP_DS_KEY]       = ViewPointKeys::VP_LOAD_ALL;

    return load_data;
}

std::string checkLoadData(const nlohmann::json& load_data)
{
    auto isAll = [ & ] (const nlohmann::json& value) {
        return value.is_string() && value.get<std::string>() == ViewPointKeys::VP_LOAD_ALL; };

    // data_sources: array of [ds_id, lines] pairs as written, or an object with ds_id strings as keys
    for (const auto& key : { ViewPointKeys::VP_DS_TYPES_KEY, ViewPointKeys::VP_DS_KEY })
    {
        auto it = load_data.find(key);

        if (it == load_data.end())
            return "'" + ViewPointKeys::VP_LOAD_DATA_KEY + "' misses the required key '" + key + "'";

        const bool is_list = it->is_array() || (key == ViewPointKeys::VP_DS_KEY && it->is_object());

        if (!is_list && !isAll(*it))
            return "'" + ViewPointKeys::VP_LOAD_DATA_KEY + "' key '" + key + "' is neither a list nor '"
                   + ViewPointKeys::VP_LOAD_ALL + "'";
    }

    return "";
}

}
