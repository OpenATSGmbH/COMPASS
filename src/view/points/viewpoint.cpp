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

#include "viewpoint.h"
#include "viewmanager.h"
#include "logger.h"
#include "json.hpp"
#include "dbinterface.h"
#include "latexvisitor.h"
#include "files.h"

using namespace nlohmann;

ViewPoint::ViewPoint(unsigned int id, const nlohmann::json::object_t& data, ViewManager& view_manager, bool needs_save)
    : ViewableDataConfig(data), id_(id), view_manager_(view_manager)
{
    traced_assert(data_.contains(ViewPointKeys::VP_ID_KEY));
    traced_assert(data_.at(ViewPointKeys::VP_ID_KEY) == id_);

    if (!data_.contains(ViewPointKeys::VP_STATUS_KEY))
    {
        data_[ViewPointKeys::VP_STATUS_KEY] = "open";
        needs_save = true;
    }

    if (needs_save)
        save();

}

ViewPoint::ViewPoint(unsigned int id, const std::string& json_str, ViewManager& view_manager, bool needs_save)
    : ViewableDataConfig(json_str), id_(id), view_manager_(view_manager)
{
    traced_assert(data_.contains(ViewPointKeys::VP_ID_KEY));
    traced_assert(data_.at(ViewPointKeys::VP_ID_KEY) == id_);

    if (!data_.contains(ViewPointKeys::VP_STATUS_KEY))
    {
        data_[ViewPointKeys::VP_STATUS_KEY] = "open";
        needs_save = true;
    }

    if (needs_save)
        save();
}

unsigned int ViewPoint::id() const { return id_; }

void ViewPoint::setStatus (const std::string& status)
{
    data_[ViewPointKeys::VP_STATUS_KEY] = status;
    save();
}

void ViewPoint::setComment (const std::string& comment)
{
    data_[ViewPointKeys::VP_COMMENT_KEY] = comment;
    save();
}

void ViewPoint::print() const
{
    loginf << id_ <<": data '" << data_.dump(4) << "'";
}

void ViewPoint::accept(LatexVisitor& v) const
{
    logdbg;
    v.visit(this);
}

void ViewPoint::save()
{
    logdbg << "id " << id_;

    DBInterface& db_interface = view_manager_.dbInterface();
    db_interface.setViewPoint(id_, data_.dump());
}

bool ViewPoint::isValidJSON(nlohmann::json json_obj, 
                            const std::string& json_filename, 
                            std::string* err_msg,
                            bool verbose)
{
    try
    {
        if (verbose)
            loginf << "";

        if (!json_obj.is_object())
            throw std::runtime_error("current data is not an object");

        if (!json_obj.contains(ViewPointKeys::VP_COLLECTION_CONTENT_TYPE_KEY)
                || !json_obj.at(ViewPointKeys::VP_COLLECTION_CONTENT_TYPE_KEY).is_string()
                || json_obj.at(ViewPointKeys::VP_COLLECTION_CONTENT_TYPE_KEY) != ViewPointKeys::VP_COLLECTION_CONTENT_TYPE)
            throw std::runtime_error("current data is not view point content");

        if (!json_obj.contains(ViewPointKeys::VP_COLLECTION_CONTENT_VERSION_KEY)
                || !json_obj.at(ViewPointKeys::VP_COLLECTION_CONTENT_VERSION_KEY).is_string()
                || json_obj.at(ViewPointKeys::VP_COLLECTION_CONTENT_VERSION_KEY) != ViewPointKeys::VP_COLLECTION_CONTENT_VERSION)
            throw std::runtime_error("current data content version is not supported");

        if (json_obj.contains(ViewPointKeys::VP_CONTEXT_KEY))
        {
            json& context = json_obj.at(ViewPointKeys::VP_CONTEXT_KEY);

            if (context.contains(ViewPointKeys::VP_CONTEXT_DATASETS_KEY))
            {
                if (!context.at(ViewPointKeys::VP_CONTEXT_DATASETS_KEY).is_array())
                    throw std::runtime_error("datasets is not an array");

                for (json& ds_it : context.at(ViewPointKeys::VP_CONTEXT_DATASETS_KEY).get<json::array_t>())
                {
                    if (!ds_it.contains(ViewPointKeys::VP_CONTEXT_DATASET_FILENAME_KEY)
                            || !ds_it.at(ViewPointKeys::VP_CONTEXT_DATASET_FILENAME_KEY).is_string())
                        throw std::runtime_error("dataset '"+ds_it.dump()+"' does not contain a valid filename");

                    std::string filename = ds_it.at(ViewPointKeys::VP_CONTEXT_DATASET_FILENAME_KEY);

                    bool found = true;

                    if (!Utils::Files::fileExists(filename))
                    {
                        found = false;

                        std::string file = Utils::Files::getFilenameFromPath(filename);
                        
                        if (verbose)
                            loginf << "filename '" << filename << "' not found";
                        
                        if (!json_filename.empty())
                        {
                            std::string dir = Utils::Files::getDirectoryFromPath(json_filename);
                        
                            if (verbose)
                                loginf << "checking for file '" << file << "' in dir '" << dir << "'";

                            filename = dir+"/"+file;

                            if (Utils::Files::fileExists(filename))
                            {
                                found = true;

                                if (verbose)
                                {
                                    loginf << "filename '" << filename
                                           << "' found at different path";
                                }
                            }
                        }
                    }

                    if (!found)
                        throw std::runtime_error("dataset '"+ds_it.dump()+"' does not contain a usable filename");
                }
            }
        }

        if (!json_obj.contains(ViewPointKeys::VP_COLLECTION_ARRAY_KEY))
            throw std::runtime_error("current data does not contain view points");

        json& view_points = json_obj.at(ViewPointKeys::VP_COLLECTION_ARRAY_KEY);

        if (!view_points.is_array())
            throw std::runtime_error("view_points is not an array");

        if (!view_points.size())
            throw std::runtime_error("view_points is an empty array");

        for (auto& vp_it : view_points.get<json::array_t>())
        {
            if (!vp_it.contains(ViewPointKeys::VP_ID_KEY) || !vp_it.at(ViewPointKeys::VP_ID_KEY).is_number())
                throw std::runtime_error("view point '"+vp_it.dump()+"' does not contain a valid id");

            if (!vp_it.contains(ViewPointKeys::VP_TYPE_KEY) || !vp_it.at(ViewPointKeys::VP_TYPE_KEY).is_string())
                throw std::runtime_error("view point '"+vp_it.dump()+"' does not contain a valid type");
        }

        if (verbose)
        {
            loginf << "current data seems to be valid, contains " << view_points.size()
                << " view points";
        }
    }
    catch(const std::exception& ex)
    {
        if (err_msg)
            *err_msg = ex.what();

        return false;
    }
    catch(...)
    {
        if (err_msg)
            *err_msg = "unknown error";

        return false;
    }
    
    return true;
}
