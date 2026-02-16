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

#include <string>

namespace rtcommand
{

struct RTCommandResult;

/**
 * Response to pushing a command to the application.
 */
struct RTCommandResponse
{
    bool isOk() const;

    RTCommandResponse() = default;
    RTCommandResponse(const ErrorInfo& err_info, const std::string& cmd_name);
    RTCommandResponse(const IssueInfo& issue_info);
    RTCommandResponse(const RTCommandResult& result);
    RTCommandResponse(const rtcommand::RTCommand& cmd);

    std::string toJSONString() const;
    
    std::string errorToString() const;
    std::string replyToJSONString(bool format_nicely = false) const;
    std::string stringifiedReply() const;

    static std::string errCode2String(CmdErrorCode code);

    std::string    command;           // command name
    ErrorInfo      error;             // error information
    std::string    execution_time;    // the command's execution time
    nlohmann::json json_reply;        // the command's result data as json
    std::string    json_reply_string; // the command's result data as string
};

} // namespace rtcommand
