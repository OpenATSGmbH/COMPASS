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
#include "db_context_manager.h"
#include "dbcontentmanager.h"
#include "dbcontentdeletedbjob.h"
#include "dbinterface.h"
#include "jobmanager.h"
#include "compass.h"
#include "logger.h"
#include "json.hpp"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(dbContent::RTCommandGetConfigDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandGetDBDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandSetDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandDeleteData)

using namespace std;

namespace dbContent
{

void init_data_source_commands()
{
    dbContent::RTCommandGetConfigDataSources::init();
    dbContent::RTCommandGetDBDataSources::init();
    dbContent::RTCommandSetDataSources::init();
    dbContent::RTCommandDeleteData::init();
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
    auto& ctx_man = compass_->dbContextManager();

    nlohmann::json j = nlohmann::json::array();
    for (const auto& ds : ctx_man.activeContext().dataSources())
        j.push_back(ds.toJSON());
    setJSONReply(j);

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
    auto& ctx_man = compass_->dbContextManager();

    nlohmann::json j = nlohmann::json::array();
    for (const auto& ds : ctx_man.activeContext().dataSources())
        j.push_back(ds.toJSON());
    setJSONReply(j);

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

    auto& ctx_man = compass_->dbContextManager();

    try
    {
        ds_json_ = nlohmann::json::parse(ds_json_str_);

        // import sensors from JSON array
        if (ds_json_.is_array())
        {
            for (const auto& ds_j : ds_json_)
                ctx_man.activeContext().addOrReplaceDataSource(context::DataSource::fromJSON(ds_j));
            ctx_man.saveContext(ctx_man.activeContextName());
        }
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


// delete_data

RTCommandDeleteData::RTCommandDeleteData()
    : rtcommand::RTCommand()
{
    condition.setSignal("compass.dbcontentmanager.dataDeletedSignal()", -1);
}

bool RTCommandDeleteData::run_impl()
{
    loginf << "delete_info_str_ '" << delete_info_str_ << "'";

    if (!compass_->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    try
    {
        delete_info_ = nlohmann::json::parse(delete_info_str_);
    }
    catch (exception& e)
    {
        logerr << "JSON parse error '" << e.what() << "'";
        setResultMessage(string("JSON parse error '") + e.what() + "'");
        return false;
    }

    if (!delete_info_.is_array() || delete_info_.empty())
    {
        setResultMessage("delete_info must be a non-empty JSON array");
        return false;
    }

    // clear loaded dataset
    compass_->dbContentManager().clearData();

    // create and submit delete job
    auto job = make_shared<DBContentDeleteDBJob>(compass_->dbInterface());
    job->setDeleteInfo(delete_info_);
    job->cleanupDB(true);

    auto& ctx_man = compass_->dbContextManager();

    QObject::connect(job.get(), &DBContentDeleteDBJob::doneSignal,
            &ctx_man, [&ctx_man, delete_info = delete_info_]()
            {
                ctx_man.applyDeleteInfo(delete_info);
            }, Qt::QueuedConnection);

    compass_->jobManager().addDBJob(job);

    return true;
}

bool RTCommandDeleteData::checkResult_impl()
{
    return true;
}

void RTCommandDeleteData::collectOptions_impl(OptionsDescription& options,
                                              PosOptionsDescription& positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
            ("delete_info", po::value<std::string>()->required(), "Delete info JSON array");

    ADD_RTCOMMAND_POS_OPTION(positional, "delete_info")
}

void RTCommandDeleteData::assignVariables_impl(const VariablesMap& vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "delete_info", std::string, delete_info_str_)
}

} // namespace dbContent
