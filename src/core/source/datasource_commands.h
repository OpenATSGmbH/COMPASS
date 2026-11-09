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

#include "rtcommand/rtcommand.h"

class SensorStatusTracker;
class DBContentManager;

namespace context
{
    class DBContextManager;
}

namespace dbContent
{


extern void init_data_source_commands(rtcommand::RTCommandRegistry& registry,
                                      context::DBContextManager& ctx_man,
                                      DBContentManager& dbcont_man,
                                      SensorStatusTracker& sensor_status_tracker);


// get_data_sources
struct RTCommandGetDataSources : public rtcommand::RTCommand
{
public:
    explicit RTCommandGetDataSources(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(get_data_sources, "retrieves descriptions of data sources in the active context")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// get_data_source_counts
struct RTCommandGetDataSourceCounts : public rtcommand::RTCommand
{
public:
    explicit RTCommandGetDataSourceCounts(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(get_data_source_counts, "retrieves per data source per dbcontent per line inserted record counts")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// get_sensor_status
struct RTCommandGetSensorStatus : public rtcommand::RTCommand
{
public:
    RTCommandGetSensorStatus(context::DBContextManager& ctx_man, SensorStatusTracker& sensor_status_tracker);

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

private:
    context::DBContextManager& ctx_man_;
    SensorStatusTracker&       sensor_status_tracker_;

    DECLARE_RTCOMMAND(get_sensor_status,
                      "retrieves the CAT063 based sensor status of the data sources, as shown in the Sensor Status tool")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// set_data_sources "{\"content_type\":\"data_sources\",\"content_version\":\"0.2\",\"data_sources\":[{\"ds_type\":\"RefTraj\",\"info\":{},\"name\":\"Reconst\",\"sac\":0,\"short_name\":\"Reconst\",\"sic\":0}]}"
struct RTCommandSetDataSources : public rtcommand::RTCommand
{
public:
    std::string ds_json_str_;
    nlohmann::json ds_json_;

    explicit RTCommandSetDataSources(context::DBContextManager& ctx_man);

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

private:
    context::DBContextManager& ctx_man_;

    DECLARE_RTCOMMAND(set_data_sources, "sets data source descriptions")
    DECLARE_RTCOMMAND_OPTIONS
};

// delete_data '[ {"dbcontent": "CAT021", "data_sources": [{"ds_id": 510}]} ]'
struct RTCommandDeleteData : public rtcommand::RTCommand
{
public:
    std::string delete_info_str_;
    nlohmann::json delete_info_;

    RTCommandDeleteData(context::DBContextManager& ctx_man, DBContentManager& dbcont_man);

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

private:
    context::DBContextManager& ctx_man_;
    DBContentManager&          dbcont_man_;

    DECLARE_RTCOMMAND(delete_data, "deletes data from the database")
    DECLARE_RTCOMMAND_OPTIONS
};

}
