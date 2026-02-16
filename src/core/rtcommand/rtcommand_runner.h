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

#include "rtcommand_result.h"

#include <memory>
#include <future>
#include <vector>

namespace rtcommand
{

struct RTCommand;
class  RTCommandChain;
class  RTCommandRunnerStash;

/**
 * Class for executing runtime commands in a separate thread.
 * 
 * !This class is supposed to be created inside the main thread to work correctly!
 */
class RTCommandRunner
{
public:
    typedef std::vector<RTCommandResult> Results;

    RTCommandRunner();
    virtual ~RTCommandRunner();

    std::future<Results> runCommand(std::unique_ptr<RTCommand>&& cmd);
    std::future<Results> runCommands(RTCommandChain&& cmds);

private:
    static void runCommand(std::shared_ptr<RTCommand> cmd, RTCommandRunnerStash* stash);
    static bool initWaitCondition(std::shared_ptr<RTCommand>, RTCommandRunnerStash* stash);
    static bool execWaitCondition(std::shared_ptr<RTCommand>, RTCommandRunnerStash* stash);
    static bool cleanupWaitCondition(std::shared_ptr<RTCommand>, RTCommandRunnerStash* stash);
    static bool executeCommand(std::shared_ptr<RTCommand>, RTCommandRunnerStash* stash);
    static bool postCheckCommand(std::shared_ptr<RTCommand>, RTCommandRunnerStash* stash);
    static void logMsg(const std::string& msg, RTCommand* cmd = nullptr);

    std::unique_ptr<RTCommandRunnerStash> stash_;
};

} // namespace rtcommand
