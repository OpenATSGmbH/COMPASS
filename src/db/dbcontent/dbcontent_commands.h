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

#include "buffer/buffer.h"
#include "rtcommand/rtcommand.h"

#include <vector>
#include <memory>

#include <boost/optional.hpp>

namespace dbContent
{

extern void init_dbcontent_commands();

// get_dbcontent_data --dbcontent CAT021 --variables "Timestamp|Time of Day|Latitude|Longitude|Associations" --max_size 100
// get_dbcontent_data --dbcontent CAT021 --variables "Timestamp|Time of Day|Latitude|Longitude|Associations" --utn 0
// get_dbcontent_data --dbcontent CAT062 --variables Timestamp
// get_dbcontent_data --dbcontent CAT062 --variables Timestamp --max_size 100

struct RTCommandGetData : public rtcommand::RTCommand
{
public:
    std::string                   dbcontent_name_;
    std::vector<std::string>      variables_;
    boost::optional<unsigned int> utn_;
    boost::optional<unsigned int> max_size_;

    RTCommandGetData();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    dbContent::VariableSet getReadSetFor() const;

    DECLARE_RTCOMMAND(get_dbcontent_data, "retrieves dbcontent data from the database")
    DECLARE_RTCOMMAND_OPTIONS
};

// get_utns
struct RTCommandGetUTNs : public rtcommand::RTCommand
{
public:
    RTCommandGetUTNs();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    bool no_desc_ = false;

    DECLARE_RTCOMMAND(get_utns, "retrieves UTNS with (optional) target descriptions")
    DECLARE_RTCOMMAND_OPTIONS
};

// get_utn
struct RTCommandGetTarget : public rtcommand::RTCommand
{
public:
    RTCommandGetTarget();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    unsigned int utn_ = 0;

    DECLARE_RTCOMMAND(get_target, "retrieves a target description for the given UTN")
    DECLARE_RTCOMMAND_OPTIONS
};

//get_target_stats
struct RTCommandGetTargetStats : public rtcommand::RTCommand
{
public:
    RTCommandGetTargetStats();

protected:
    virtual bool run_impl() override;
    virtual bool checkResult_impl() override;

    DECLARE_RTCOMMAND(get_target_stats, "retrieves statistics for the current targets")
    DECLARE_RTCOMMAND_NOOPTIONS
};

} // namespace dbContent
