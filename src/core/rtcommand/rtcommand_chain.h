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

#include "rtcommand.h"

#include <memory>
#include <queue>

#include <QObject>

namespace rtcommand
{

/**
 * Chain of runtime commands.
 */
class RTCommandChain
{
public:
    typedef std::unique_ptr<RTCommand> RTCommandPtr;
    typedef std::queue<RTCommandPtr>   RTCommands;

    RTCommandChain() = default;
    RTCommandChain(RTCommandChain&& other);
    virtual ~RTCommandChain() = default;

    void append(std::unique_ptr<RTCommand>&& cmd);
    RTCommandPtr pop();

    const RTCommands& commands() const { return commands_; }

protected:
    void addCommand(RTCommand* cmd);
    void attachWaitCondition(const RTCommandWaitCondition& condition);

private:
    Q_DISABLE_COPY(RTCommandChain)

    RTCommands commands_;
};

} // namespace rtcommand
