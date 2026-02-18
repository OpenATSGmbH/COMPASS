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

namespace dbContent
{


extern void init_data_source_commands();


// get_data_sources
struct RTCommandGetConfigDataSources : public rtcommand::RTCommand
{
public:
    RTCommandGetConfigDataSources();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(get_cfg_data_sources, "retrieves descriptions of data sources present in the configuration")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// get_data_sources_in_db
struct RTCommandGetDBDataSources : public rtcommand::RTCommand
{
public:
    RTCommandGetDBDataSources();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(get_db_data_sources, "retrieves descriptions of data sources present in the database")
    DECLARE_RTCOMMAND_NOOPTIONS
};

// set_data_sources "{\"content_type\":\"data_sources\",\"content_version\":\"0.2\",\"data_sources\":[{\"ds_type\":\"RefTraj\",\"info\":{},\"name\":\"Reconst\",\"sac\":0,\"short_name\":\"Reconst\",\"sic\":0}]}"
struct RTCommandSetDataSources : public rtcommand::RTCommand
{
public:
    std::string ds_json_str_;
    nlohmann::json ds_json_;

    RTCommandSetDataSources();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(set_data_sources, "sets data source descriptions")
    DECLARE_RTCOMMAND_OPTIONS
};

}
