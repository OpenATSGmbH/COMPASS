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

#include "rtcommand_defs.h"

#include "json.hpp"

#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/optional.hpp>

namespace rtcommand
{

/**
 * Keeps track of runtime command execution result and state (including the state of the command's wait condition).
 */
struct RTCommandResult
{
    typedef boost::posix_time::time_duration Duration;

    bool hasError() const
    { 
        return (error.code != CmdErrorCode::NoError);
    }

    void reset()
    {   
        runtime.reset();
        
        json_reply        = {};
        error             = {};
        json_reply_string = "";
    }

    std::string               command;           // command name
    ErrorInfo                 error;             // error information
    boost::optional<Duration> runtime;           // time for execution
    nlohmann::json            json_reply;        // command reply as json
    std::string               json_reply_string; // command reply as string
};

} // namespace rtcommand
