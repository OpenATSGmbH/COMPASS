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

#include "dbcontent_commands.h"
#include "rtcommand/rtcommand_macros.h"
#include "rtcommand_registry.h"
#include "compass.h"
#include "dbcontentmanager.h"
#include "dbcontentdataengine.h"
#include "loadoperation.h"
#include "dbcontent.h"
#include "view/viewmanager.h"
#include "compass.h"
#include "metavariable.h"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(dbContent::RTCommandGetData)
REGISTER_RTCOMMAND(dbContent::RTCommandGetUTNs)
REGISTER_RTCOMMAND(dbContent::RTCommandGetTarget)
REGISTER_RTCOMMAND(dbContent::RTCommandGetTargetStats)

using namespace std;

namespace dbContent
{

/***************************************************************************************
 * RTCommandGetData
 ***************************************************************************************/

void init_dbcontent_commands()
{
    dbContent::RTCommandGetData::init();
    dbContent::RTCommandGetUTNs::init();
    dbContent::RTCommandGetTarget::init();
    dbContent::RTCommandGetTargetStats::init();
}

RTCommandGetData::RTCommandGetData()
    : rtcommand::RTCommand()
{
    // run_impl loads synchronously (op->wait()); no signal wait needed
}


bool RTCommandGetData::run_impl()
{
    loginf;

    if (!compass_->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (compass_->appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+compass_->appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = compass_->dbContentManager();

    // RT commands bypass UI modality; refuse to start while a load runs so we
    // never nest inside the outer load's event pump
    if (dbcontent_man.dataEngine().isLoading())
    {
        setResultMessage("Load already in progress");
        return false;
    }

    if (!dbcontent_man.existsDBContent(dbcontent_name_))
    {
        setResultMessage("Unknown dbcontent '"+dbcontent_name_+"'");
        return false;
    }

    DBContent& db_content = dbcontent_man.dbContent(dbcontent_name_);

    for (auto& var_it : variables_)
    {
        if (!db_content.hasVariable(var_it))
        {
            setResultMessage("Unknown dbcontent '"+dbcontent_name_+"' variable '"+var_it+"'");
            return false;
        }
    }

    // loading stuff

    VariableSet read_set = getReadSetFor();

    std::string custom_clause;
    if (utn_.has_value())
    {
        dbContent::Variable& var = compass_->dbContentManager().metaVariable(
            dbcontent_vars::meta_var_utn_.name()).getFor(dbcontent_name_);

        custom_clause = var.dbColumnName() + " IN (" + std::to_string(utn_.value()) + ")";
    }

    LoadRequest r = LoadRequest::forContent(dbcontent_name_, read_set, custom_clause);

    // isolated blocking load: harvest the buffers here, read them in
    // checkResult_impl; the view dataset is never touched
    auto op = std::make_shared<LoadOperation>(dbcontent_man, r);
    dbcontent_man.dataEngine().load(op);
    op->wait();
    // @TODO: op state unchecked - a Failed/Cancelled load returns partial data as complete
    buffers_ = op->buffers();

    return true; // if ok
}

bool RTCommandGetData::checkResult_impl()
{
    if (buffers_.empty())
    {
        setResultMessage("No data loaded");
        return false; // error
    }

    const std::map<std::string, std::shared_ptr<Buffer>>& buffers = buffers_;

    if (!buffers.count(dbcontent_name_))
    {
        setResultMessage("No "+dbcontent_name_+" data loaded");
        return false; // error
    }

    // check and set reply

    if (buffers.at(dbcontent_name_)->size() > 10e6)
    {
        setResultMessage(dbcontent_name_+" data too large ("+to_string(buffers.at(dbcontent_name_)->size())+")");
        return false; // error
    }

    nlohmann::json json_reply;

    set<string> variables;

    for (auto& var_name : variables_)
        variables.insert(var_name);

    if (max_size_.has_value())
        json_reply[dbcontent_name_] = buffers.at(dbcontent_name_)->asJSON(variables, max_size_.get());
    else
        json_reply[dbcontent_name_] = buffers.at(dbcontent_name_)->asJSON(variables);

    setJSONReply(json_reply);

    return true; // if ok
}

dbContent::VariableSet RTCommandGetData::getReadSetFor() const
{
    VariableSet read_set;

    DBContentManager& dbcont_man = compass_->dbContentManager();
    DBContent& db_content = dbcont_man.dbContent(dbcontent_name_);

    // ds id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name_, dbcontent_vars::meta_var_ds_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name_, dbcontent_vars::meta_var_ds_id_));

    // line id
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name_, dbcontent_vars::meta_var_line_id_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name_, dbcontent_vars::meta_var_line_id_));

    // timestamp
    traced_assert(dbcont_man.metaCanGetVariable(dbcontent_name_, dbcontent_vars::meta_var_timestamp_));
    read_set.add(dbcont_man.metaGetVariable(dbcontent_name_, dbcontent_vars::meta_var_timestamp_));

    for (auto& var_it : variables_)
    {
        traced_assert(db_content.hasVariable(var_it));

        Variable& var = db_content.variable(var_it);

        if (!read_set.hasVariable(var))
            read_set.add(var);
    }

    return read_set;
}

/**
 */
void RTCommandGetData::collectOptions_impl(OptionsDescription &options,
                                            PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("dbcontent", po::value<std::string>()->required(), "DBContent to retrieve data for")
        ("variables", po::value<std::string>()->required(), "variables to retrieve data for")
        ("utn"      , po::value<unsigned int>()           , "UTN to retrieve data for"      )
        ("max_size" , po::value<unsigned int>()           , "maximum number of results rows");

    ADD_RTCOMMAND_POS_OPTION(positional, "dbcontent")
    ADD_RTCOMMAND_POS_OPTION(positional, "variables")
    ADD_RTCOMMAND_POS_OPTION(positional, "utn"      )
    ADD_RTCOMMAND_POS_OPTION(positional, "max_size" )
}

/**
 */
void RTCommandGetData::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "dbcontent", std::string, dbcontent_name_)
    //@TODO: migrate variables list separator from '|' to ';' to match the project-wide
    //list-to-string convention (see import_asterix_files filenames, reconstruct_references
    //disable_sensors). Likely requires changing RTCommand::ParameterListSeparator.
    RTCOMMAND_GET_STRINGLIST_OR_THROW(vars, "variables", variables_)
    RTCOMMAND_GET_VAR(vars, "utn", unsigned int, utn_)
    RTCOMMAND_GET_VAR(vars, "max_size", unsigned int, max_size_)
}

/***************************************************************************************
 * RTCommandGetUTNs
 ***************************************************************************************/

RTCommandGetUTNs::RTCommandGetUTNs()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetUTNs::run_impl()
{
    if (!compass_->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (compass_->appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+compass_->appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = compass_->dbContentManager();

    if (!dbcontent_man.hasAssociations() || !dbcontent_man.hasTargetsInfo())
    {
        setResultMessage("No target information present");
        return false;
    }

    return true;
}

bool RTCommandGetUTNs::checkResult_impl()
{
    DBContentManager& dbcontent_man = compass_->dbContentManager();

    if (no_desc_)
        setJSONReply(dbcontent_man.utnsAsJSON());
    else
        setJSONReply(dbcontent_man.targetsInfoAsJSON());

    return true;
}

/**
 */
void RTCommandGetUTNs::collectOptions_impl(OptionsDescription &options,
                                           PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("nodesc", "Return a list of existing UTNs without target descriptions");
}

/**
 */
void RTCommandGetUTNs::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_CHECK_VAR(vars, "nodesc", no_desc_)
}

/***************************************************************************************
 * RTCommandGetTarget
 ***************************************************************************************/

RTCommandGetTarget::RTCommandGetTarget()
    : rtcommand::RTCommand()
{
}

bool RTCommandGetTarget::run_impl()
{
    if (!compass_->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (compass_->appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+compass_->appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = compass_->dbContentManager();

    if (!dbcontent_man.hasAssociations() || !dbcontent_man.hasTargetsInfo())
    {
        setResultMessage("No target information present");
        return false;
    }

    return true;
}

bool RTCommandGetTarget::checkResult_impl()
{
    DBContentManager& dbcontent_man = compass_->dbContentManager();

    setJSONReply(dbcontent_man.targetInfoAsJSON(utn_));

    return true;
}

/**
 */
void RTCommandGetTarget::collectOptions_impl(OptionsDescription &options,
                                             PosOptionsDescription &positional)
{
    ADD_RTCOMMAND_OPTIONS(options)
        ("utn", po::value<unsigned int>()->required(), "UTN of the target to retrieve information for");

    ADD_RTCOMMAND_POS_OPTION(positional, "utn")
}

/**
 */
void RTCommandGetTarget::assignVariables_impl(const VariablesMap &vars)
{
    RTCOMMAND_GET_VAR_OR_THROW(vars, "utn", unsigned int, utn_)
}

/***************************************************************************************
 * RTCommandGetTargetStats
 ***************************************************************************************/

RTCommandGetTargetStats::RTCommandGetTargetStats()
    : rtcommand::RTCommand()
{
}

bool RTCommandGetTargetStats::run_impl()
{
    if (!compass_->dbOpened())
    {
        setResultMessage("Database not opened");
        return false;
    }

    if (compass_->appMode() != AppMode::Offline) // to be sure
    {
        setResultMessage("Wrong application mode "+compass_->appModeStr());
        return false;
    }

    DBContentManager& dbcontent_man = compass_->dbContentManager();

    if (!dbcontent_man.hasAssociations() || !dbcontent_man.hasTargetsInfo())
    {
        setResultMessage("No target information present");
        return false;
    }

    return true;
}

bool RTCommandGetTargetStats::checkResult_impl()
{
    DBContentManager& dbcontent_man = compass_->dbContentManager();

    setJSONReply(dbcontent_man.targetStatsAsJSON());

    return true;
}

} // namespace dbContent
