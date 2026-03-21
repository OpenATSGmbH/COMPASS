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

#include "viewpoint_commands.h"
#include "rtcommand/rtcommand_macros.h"
#include "rtcommand_registry.h"
#include "dbcontentmanager.h"
#include "viewmanager.h"
#include "viewpointgenerator.h"
#include "compass.h"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(RTCommandSetViewPoint)

using namespace std;

namespace
{
    bool validateAnnotationFeatureTypes(const nlohmann::json& anno_json, std::string& error)
    {
        if (anno_json.contains("features") && anno_json.at("features").is_array())
        {
            for (const auto& feat : anno_json.at("features"))
            {
                if (!feat.contains("type"))
                {
                    error = "annotation feature without type";
                    return false;
                }

                std::string type = feat.at("type");
                if (!ViewPointGenFeature::knownFeatureTypes().count(type))
                {
                    error = "unknown annotation feature type '" + type + "'";
                    return false;
                }
            }
        }

        if (anno_json.contains("annotations") && anno_json.at("annotations").is_array())
        {
            for (const auto& child : anno_json.at("annotations"))
            {
                if (!validateAnnotationFeatureTypes(child, error))
                    return false;
            }
        }

        return true;
    }
}

static COMPASS* s_compass = nullptr;

void init_view_point_commands(COMPASS& compass)
{
    s_compass = &compass;
    RTCommandSetViewPoint::init();
}

// set

RTCommandSetViewPoint::RTCommandSetViewPoint()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.dbcontentmanager.loadingDoneSignal()", -1); // think about max duration

    // view point setting triggers load after set, so can wait on that
}


bool RTCommandSetViewPoint::run_impl()
{
    loginf << "vp_json_str_ '" << vp_json_str_ << "'";

    if (!s_compass->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (s_compass->appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+s_compass->appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = s_compass->dbContentManager();
    dbcontent_man.clearData();

    try
    {
        nlohmann::json vp_json_parsed = nlohmann::json::parse(vp_json_str_);

        // validate annotation feature types
        if (vp_json_parsed.contains("annotations") && vp_json_parsed.at("annotations").is_array())
        {
            std::string validation_error;
            for (const auto& anno : vp_json_parsed.at("annotations"))
            {
                if (!validateAnnotationFeatureTypes(anno, validation_error))
                {
                    setResultMessage(validation_error);
                    return false;
                }
            }
        }

        viewable_data_cfg_.reset(new ViewableDataConfig(vp_json_parsed.get<nlohmann::json::object_t>()));

        s_compass->viewManager().setCurrentViewPoint(viewable_data_cfg_.get());
    }
    catch (exception& e)
    {
        logerr << "JSON parse error '" << e.what() << "'";
        setResultMessage(string("JSON parse error '") + e.what() + "'");
        return false;
    }

    return true; // if ok
}

bool RTCommandSetViewPoint::checkResult_impl()
{
    const auto& errors = s_compass->viewManager().viewPointErrors();

    if (!errors.empty())
    {
        nlohmann::json err_json = nlohmann::json::array();
        for (const auto& [component, msg] : errors)
            err_json.push_back({{"component", component}, {"error", msg}});

        setJSONReply({{"view_point_errors", err_json}});
        setResultMessage("view point consumption failed in " + std::to_string(errors.size()) + " component(s)");
        return false;
    }

    return true;
}


/**
 */
void RTCommandSetViewPoint::collectOptions_impl(OptionsDescription &options,
                                                  PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
            ("view_point", po::value<std::string>()->required(), "View point JSON definition");

    ADD_RTCOMMAND_POS_OPTION(positional, "view_point")
}

/**
 */
void RTCommandSetViewPoint::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "view_point", std::string, vp_json_str_)
}
