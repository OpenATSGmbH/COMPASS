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

#include "rtcommand_chain.h"

namespace rtcommand
{

/**
 */
RTCommandChain::RTCommandChain(RTCommandChain&& other)
{
    commands_ = std::move(other.commands_);
}

/**
*/
void RTCommandChain::append(std::unique_ptr<RTCommand>&& cmd)
{
    addCommand(cmd.release());
}

/**
 */
void RTCommandChain::addCommand(RTCommand* cmd)
{
    commands_.emplace(cmd);
}

/**
 */
void RTCommandChain::attachWaitCondition(const RTCommandWaitCondition& condition)
{
    if (commands_.empty())
        throw std::runtime_error("RTCommandChain::appendWaitCondition: no command to append to");

    auto last = commands_.back().get();
    last->condition = condition;
}

/**
 */
RTCommandChain::RTCommandPtr RTCommandChain::pop()
{
    if (commands_.empty())
        return nullptr;

    auto ptr = std::move(commands_.front());
    commands_.pop();

    return ptr;
}

} // namespace rtcommand
