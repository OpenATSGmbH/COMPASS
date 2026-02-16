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

#include <QString>

namespace rtcommand
{

struct RTCommand;

enum class CmdErrorCode
{
    NoError = 0,

    Issue_CommandStringEmpty    = 100,
    Issue_CommandStringInvalid  = 101,
    Issue_CommandStringMismatch = 102,
    Issue_CommandNotFound       = 103,
    Issue_CommandCreationFailed = 104,
    Issue_NotReady              = 105,

    Config_CommandStringInvalid  = 200,
    Config_CommandStringMismatch = 201,
    Config_CollectOptionsFailed  = 202,
    Config_ParseOptionsFailed    = 203,
    Config_AssignOptionsFailed   = 204,
    Config_Invalid               = 205,

    Exec_Unconfigured            = 300,
    Exec_InvalidConfig           = 301,
    Exec_Crash                   = 302,
    Exec_Failed                  = 303,
    Exec_InvokeFailed            = 304,

    ResultCheck_NotExecuted      = 400,
    ResultCheck_InvalidResult    = 401,
    ResultCheck_Crash            = 402,
    ResultCheck_InvokeFailed     = 403,
    
    WaitCond_BadInit             = 500,
    WaitCond_Timeout             = 501,
    WaitCond_InvokeFailed        = 502
};

/**
 * Error code for object retrieval.
 */
enum class FindObjectErrCode
{
    NoError = 0,
    Invalid,
    NotFound,
    WrongType
};

/**
 * State of a command. 
 */
enum class CmdState
{
    Unconfigured = 0,
    Configured,
    Executed,
    Finished
};

/**
 * Describes a runtime command.
 */
struct RTCommandDescription
{
    QString name;
    QString description;
};

/**
 * Validity struct for RTCommand.
 */
struct IsValid
{
    IsValid() = default;
    IsValid(bool valid, const std::string& msg = "") : is_valid(valid), message(msg) {}

    std::string errorString() const
    {
        if (is_valid)
            return "";
        return (message.empty() ? "Unknown error" : message);
    }

    bool        is_valid = false;
    std::string message;
};

/**
 * Error struct.
*/
struct ErrorInfo
{
    bool hasError() const
    {
        return (code != CmdErrorCode::NoError);
    }
    bool noError() const
    {
        return !hasError();
    }

    CmdErrorCode code = CmdErrorCode::NoError; //error code
    std::string  message;                      //additional string information
};

/**
 * Issue result struct. Obtains the issued command and error information.
*/
struct IssueInfo
{
    std::string command;          // issued command name
    bool        issued   = false; // was command issued successfully?
    ErrorInfo   error;            // error information
};

} // namespace rtcommand
