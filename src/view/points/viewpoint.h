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

#include "viewabledataconfig.h"

class ViewManager;
class LatexVisitor;

class ViewPoint : public ViewableDataConfig
{
  public:
    ViewPoint(unsigned int id, const nlohmann::json::object_t& data, ViewManager& view_manager, bool needs_save);
    ViewPoint(unsigned int id, const std::string& json_str, ViewManager& view_manager, bool needs_save);

    unsigned int id() const;

    void setStatus (const std::string& status);
    void setComment (const std::string& comment);

    void print() const;

    virtual void accept(LatexVisitor& v) const;

    const unsigned int id_;

    static bool isValidJSON(nlohmann::json json_obj, 
                            const std::string& json_filename = "", 
                            std::string* err_msg = nullptr,
                            bool verbose = false);

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

    static const std::string VP_DS_TYPES_KEY;
    static const std::string VP_DS_KEY;
    static const std::string VP_FILTERS_KEY;
    static const std::string VP_SELECTED_RECNUMS_KEY;

    static const std::string VP_POS_LAT_KEY;
    static const std::string VP_POS_LON_KEY;
    static const std::string VP_POS_WIN_LAT_KEY;
    static const std::string VP_POS_WIN_LON_KEY;

    static const std::string VP_TIMESTAMP_KEY;
    static const std::string VP_TIME_WIN_KEY;

    static const std::string VP_ANNOTATION_KEY;

protected:
    ViewManager& view_manager_;

    void save();
};
