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

#include "rtcommand_response.h"
#include "rtcommand_result.h"
#include "rtcommand.h"
#include "rtcommand_registry.h"

#include "timeconv.h"

namespace rtcommand
{

/**
 * Response from error information.
*/
RTCommandResponse::RTCommandResponse(const ErrorInfo& err_info, const std::string& cmd_name)
{
    command           = cmd_name;
    error             = err_info;
    json_reply        = {};
    json_reply_string = "";
    execution_time    = "";
}

/**
 * Response from error from command issue info.
*/
RTCommandResponse::RTCommandResponse(const IssueInfo& issue_info)
{
    command           = issue_info.command;
    error             = issue_info.issued ? ErrorInfo() : issue_info.error;
    json_reply        = {};
    json_reply_string = "";
    execution_time    = "";
}

/**
 * Response from command result.
*/
RTCommandResponse::RTCommandResponse(const RTCommandResult& result)
{
    command           = result.command;
    error             = result.error;
    json_reply        = result.json_reply;
    json_reply_string = result.json_reply_string;
    execution_time    = result.runtime.has_value() ? Utils::Time::toString(result.runtime.value(), 3) : "";
}

/**
 * Response from command.
*/
RTCommandResponse::RTCommandResponse(const rtcommand::RTCommand& cmd)
{
    auto result = cmd.result();

    command           = cmd.name().toStdString();
    error             = cmd.isFinished() ? ErrorInfo()              : result.error;
    json_reply        = cmd.isFinished() ? result.json_reply        : nlohmann::json();
    json_reply_string = cmd.isFinished() ? result.json_reply_string : "";
    execution_time    = result.runtime.has_value() ? Utils::Time::toString(result.runtime.value(), 3) : "";
}

/**
 * Checks if the response is ok (is not in error state).
*/
bool RTCommandResponse::isOk() const
{
    return error.noError();
}

#define STRINGIFY_ERR_CODE(ErrCode, Str) \
    case CmdErrorCode::ErrCode:          \
        return Str;

/**
 * Generates a unique error string from the given error code.
*/
std::string RTCommandResponse::errCode2String(CmdErrorCode code)
{
    switch (code)
    {
        STRINGIFY_ERR_CODE(NoError, "No error")

        STRINGIFY_ERR_CODE(Issue_CommandStringEmpty, "Issue failed: Command string empty")
        STRINGIFY_ERR_CODE(Issue_CommandStringInvalid, "Issue failed: Command string invalid")
        STRINGIFY_ERR_CODE(Issue_CommandStringMismatch, "Issue failed: Command mismatch")
        STRINGIFY_ERR_CODE(Issue_CommandNotFound, "Issue failed: Command not found")
        STRINGIFY_ERR_CODE(Issue_CommandCreationFailed, "Issue failed: Command creation failed")
        STRINGIFY_ERR_CODE(Issue_NotReady, "Issue failed: Application not ready")

        STRINGIFY_ERR_CODE(Config_CommandStringInvalid, "Configuration failed: Command string invalid")
        STRINGIFY_ERR_CODE(Config_CommandStringMismatch, "Configuration failed: Command mismatch")
        STRINGIFY_ERR_CODE(Config_CollectOptionsFailed, "Configuration failed: Collect options failed")
        STRINGIFY_ERR_CODE(Config_ParseOptionsFailed, "Configuration failed: Parse options failed")
        STRINGIFY_ERR_CODE(Config_AssignOptionsFailed, "Configuration failed: Assign options failed")
        STRINGIFY_ERR_CODE(Config_Invalid, "Configuration failed: Invalid or incomplete")

        STRINGIFY_ERR_CODE(Exec_Unconfigured, "Execution failed: Unconfigured command")
        STRINGIFY_ERR_CODE(Exec_InvalidConfig, "Execution failed: Invalid or incomplete configuration")
        STRINGIFY_ERR_CODE(Exec_Crash, "Execution failed: Crash encountered")
        STRINGIFY_ERR_CODE(Exec_Failed, "Execution failed")
        STRINGIFY_ERR_CODE(Exec_InvokeFailed, "Execution failed: Could not invoke")

        STRINGIFY_ERR_CODE(ResultCheck_NotExecuted, "Result check failed: Unexecuted command")
        STRINGIFY_ERR_CODE(ResultCheck_InvalidResult, "Result check failed")
        STRINGIFY_ERR_CODE(ResultCheck_Crash, "Result check failed: Crash encountered")
        STRINGIFY_ERR_CODE(ResultCheck_InvokeFailed, "Result check failed: Could not invoke")
        
        STRINGIFY_ERR_CODE(WaitCond_BadInit, "Wait condition failed: Could not init")
        STRINGIFY_ERR_CODE(WaitCond_Timeout, "Wait condition failed: Timeout")
        STRINGIFY_ERR_CODE(WaitCond_InvokeFailed, "Wait condition failed: Could not invoke")
    }
    return "Unknown error";
}

/**
 * Generate a string from the current error state.
*/
std::string RTCommandResponse::errorToString() const
{
    std::string str;
    if (error.hasError())
        str += std::string("Error") + std::to_string((int)error.code) + " ";
    str += errCode2String(error.code);

    if (!error.message.empty())
        str += " (" + error.message + ")";

    if (!execution_time.empty())
        str += " [" + execution_time + "]";

    return str;
}

/**
 * Returns a json string representation of the response.
*/
std::string RTCommandResponse::toJSONString() const
{
    nlohmann::json root;
    root[ "command"               ] = command;
    root[ "ok"                    ] = error.noError();
    root[ "error_code"            ] = error.code;
    root[ "error"                 ] = errCode2String(error.code);
    root[ "error_additional_info" ] = error.message;
    root[ "execution_time"        ] = execution_time;
    root[ "reply"                 ] = json_reply;

    return root.dump();
}

/**
 * Returns the stored json reply as (formatted) json string. 
*/
std::string RTCommandResponse::replyToJSONString(bool format_nicely) const
{
    if (json_reply.is_null())
        return "";

    return json_reply.dump(format_nicely ? 3 : -1);
}

/**
 * Returns a string represenation of the stored json reply.
*/
std::string RTCommandResponse::stringifiedReply() const
{
    //if a string representation of the reply is set, return this one
    if (!json_reply_string.empty())
        return json_reply_string;

    //otherwise return it as a json string
    return replyToJSONString(true);
}

} // namespace rtcommand
