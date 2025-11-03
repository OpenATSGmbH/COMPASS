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

#include "datasource_commands.h"
#include "rtcommand/rtcommand_macros.h"
#include "rtcommand_registry.h"
#include "datasourcemanager.h"
#include "compass.h"
#include "logger.h"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(dbContent::RTCommandGetConfigDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandGetDBDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandSetDataSources)

using namespace std;

namespace dbContent
{

void init_data_source_commands()
{
    dbContent::RTCommandGetConfigDataSources::init();
    dbContent::RTCommandGetDBDataSources::init();
    dbContent::RTCommandSetDataSources::init();
}

// get from config

RTCommandGetConfigDataSources::RTCommandGetConfigDataSources()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetConfigDataSources::run_impl()
{
    return true;
}

bool RTCommandGetConfigDataSources::checkResult_impl()
{
    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    setJSONReply(ds_man.getSortedConfigDataSourcesAsJSON());

    return true;
}

// get from db

RTCommandGetDBDataSources::RTCommandGetDBDataSources()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetDBDataSources::run_impl()
{
    return true;
}

bool RTCommandGetDBDataSources::checkResult_impl()
{
    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    setJSONReply(ds_man.getSortedDBDataSourcesAsJSON());

    return true;
}

// set

RTCommandSetDataSources::RTCommandSetDataSources()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}


bool RTCommandSetDataSources::run_impl()
{
    loginf << "ds_json_str_ '" << ds_json_str_ << "'";

    DataSourceManager& ds_man = COMPASS::instance().dataSourceManager();

    try
    {
        ds_json_ = nlohmann::json::parse(ds_json_str_);

        ds_man.importDataSourcesJSON(ds_json_);
    }
    catch (exception& e)
    {
        logerr << "JSON parse error '" << e.what() << "'";
        setResultMessage(string("JSON parse error '") + e.what() + "'");
        return false;
    }

    return true; // if ok
}

bool RTCommandSetDataSources::checkResult_impl()
{
    return true; // if ok
}


/**
 */
void RTCommandSetDataSources::collectOptions_impl(OptionsDescription &options,
                                                  PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
            ("data_sources", po::value<std::string>()->required(), "Data sources JSON definition");

    ADD_RTCOMMAND_POS_OPTION(positional, "data_sources")
}

/**
 */
void RTCommandSetDataSources::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "data_sources", std::string, ds_json_str_)
}


} // namespace dbContent
