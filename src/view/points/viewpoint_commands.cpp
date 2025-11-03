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
#include "compass.h"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(RTCommandSetViewPoint)

using namespace std;

void init_view_point_commands()
{
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

    if (!COMPASS::instance().dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (COMPASS::instance().appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+COMPASS::instance().appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = COMPASS::instance().dbContentManager();
    dbcontent_man.clearData();

    try
    {
        nlohmann::json::object_t vp_json = nlohmann::json::parse(vp_json_str_);

        viewable_data_cfg_.reset(new ViewableDataConfig(vp_json));

        COMPASS::instance().viewManager().setCurrentViewPoint(viewable_data_cfg_.get());
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
    return true; // if ok
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
