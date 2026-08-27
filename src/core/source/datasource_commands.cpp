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
#include "dbinterface.h"
#include "datasourcesstatustoolwidget.h"
#include "datasourcesstatuswidget.h"
#include "compass.h"
#include "logger.h"
#include "traced_assert.h"
#include "util/timeconv.h"
#include "json.hpp"

#include <boost/program_options.hpp>

REGISTER_RTCOMMAND(dbContent::RTCommandGetDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandGetDataSourceCounts)
REGISTER_RTCOMMAND(dbContent::RTCommandGetSensorStatus)
REGISTER_RTCOMMAND(dbContent::RTCommandSetDataSources)
REGISTER_RTCOMMAND(dbContent::RTCommandDeleteData)

using namespace std;

namespace dbContent
{

void init_data_source_commands()
{
    dbContent::RTCommandGetDataSources::init();
    dbContent::RTCommandGetDataSourceCounts::init();
    dbContent::RTCommandGetSensorStatus::init();
    dbContent::RTCommandSetDataSources::init();
    dbContent::RTCommandDeleteData::init();
}

// get

RTCommandGetDataSources::RTCommandGetDataSources()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetDataSources::run_impl()
{
    if (!compass_->dbContextManager().hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    return true;
}

bool RTCommandGetDataSources::checkResult_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    nlohmann::json j = nlohmann::json::array();
    for (const auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
        j.push_back(ds.toJSON());
    setJSONReply(j);

    return true;
}

// get_data_source_counts

RTCommandGetDataSourceCounts::RTCommandGetDataSourceCounts()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetDataSourceCounts::run_impl()
{
    if (!compass_->dbContextManager().hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    return true;
}

bool RTCommandGetDataSourceCounts::checkResult_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    // inserted_counts_ is ds_id -> dbcontent -> line_id -> count.
    // Serialize keyed by string ds_id / line_id so the JSON object form is stable.
    nlohmann::json reply = nlohmann::json::object();

    for (const auto& [ds_id, dbcont_map] : ctx_man.insertedCounts())
    {
        nlohmann::json& ds_json = reply[std::to_string(ds_id)];
        ds_json = nlohmann::json::object();

        for (const auto& [dbcontent_name, line_map] : dbcont_map)
        {
            nlohmann::json& dbc_json = ds_json[dbcontent_name];
            dbc_json = nlohmann::json::object();

            for (const auto& [line_id, count] : line_map)
                dbc_json[std::to_string(line_id)] = count;
        }
    }

    setJSONReply(reply);
    return true;
}

// sensor status

RTCommandGetSensorStatus::RTCommandGetSensorStatus()
    : rtcommand::RTCommand()
{
    condition.setDelay(10);
}

bool RTCommandGetSensorStatus::run_impl()
{
    if (!compass_->dbContextManager().hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    return true;
}

bool RTCommandGetSensorStatus::checkResult_impl()
{
    auto& ctx_man = compass_->dbContextManager();

    // the tool widget is created lazily and keeps its state updated off the
    // distributed data, independent of the tool being open in the GUI
    auto* tool_widget = ctx_man.statusWidget();
    traced_assert(tool_widget);

    const auto* status_widget = tool_widget->statusWidget();
    traced_assert(status_widget);

    nlohmann::json reply = nlohmann::json::object();

    bool has_tracker = status_widget->hasActiveTracker();
    reply["has_active_tracker"] = has_tracker;

    if (has_tracker)
    {
        const auto& tracker = status_widget->activeTracker().value();

        nlohmann::json tracker_json;
        tracker_json["ds_id"]   = tracker.first;
        tracker_json["line_id"] = tracker.second;

        if (ctx_man.hasDataSource(tracker.first))
            tracker_json["name"] = ctx_man.dataSource(tracker.first)->name();

        reply["active_tracker"] = tracker_json;
    }

    // per data source status as shown in the tool, keyed by string ds id.
    // sources without a state obtained from the active tracker report "Fresh"
    const DataSourcesStatusWidget::SensorStateMap* states = nullptr;
    if (status_widget->hasActiveTrackerStates())
        states = &status_widget->activeTrackerStates().states;

    nlohmann::json sensors = nlohmann::json::object();

    for (const auto& [ds_id, ds] : ctx_man.activeContext().dataSources())
    {
        nlohmann::json sensor_json;

        sensor_json["name"]    = ds.name();
        sensor_json["sac"]     = ds.sac();
        sensor_json["sic"]     = ds.sic();
        sensor_json["ds_type"] = ds.dsType();

        sensor_status::SensorStatus status = sensor_status::SensorStatus::Fresh;
        boost::posix_time::ptime    status_time;

        if (states && states->count(ds.id()))
        {
            const auto& state = states->at(ds.id()).state_current;

            status      = state.status;
            status_time = state.ts_con;
        }

        sensor_json["status"] = sensor_status::stringFromSensorStatus(status);

        if (!status_time.is_not_a_date_time())
            sensor_json["status_time"] = Utils::Time::toString(status_time);

        sensors[std::to_string(ds.id())] = sensor_json;
    }

    reply["sensors"] = sensors;

    setJSONReply(reply);
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

    if (!ctx_man.hasActiveContext())
    {
        setResultMessage("No active context");
        return false;
    }

    try
    {
        ds_json_ = nlohmann::json::parse(ds_json_str_);

        // import sensors from JSON array
        if (ds_json_.is_array())
        {
            for (const auto& ds_j : ds_json_)
            {
                auto ds = context::DataSource::fromJSON(ds_j);
                ctx_man.autoAssignColors(ds);
                ctx_man.activeContext().addOrReplaceDataSource(std::move(ds));
            }
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

    // RT commands bypass UI modality; refuse while a delete is in flight (e.g. a live
    // session's per-tick DB bound) rather than overlapping on the single delete slot
    if (compass_->dbContentManager().hasActiveDeleteJob())
    {
        setResultMessage("Delete already in progress");
        return false;
    }

    compass_->dbContentManager().deleteData(delete_info_);

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
