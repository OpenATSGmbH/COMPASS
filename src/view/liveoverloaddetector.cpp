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

#include "liveoverloaddetector.h"

const unsigned int LiveOverloadDetector::PacketHoldExtraTicks = 4;
const unsigned int LiveOverloadDetector::LatencyHoldTicks     = 5;

/**
 * Called once per tick attempt, returns true if the tick has to be held.
 * Notice: The detecting attempt is held too, so an overload holds hold_ticks attempts in total.
 */
bool LiveOverloadDetector::update(unsigned int packets_in_processing,
                                  const std::optional<double>& latency_s)
{
    //debug: held until the force is switched off, then the hold ends with the next attempt
    if (settings_.force)
    {
        if (overloaded_)
            ++held_ticks_;
        else
            held_ticks_ = 0;

        overloaded_ = true;
        hold_ticks_ = held_ticks_ + 1;

        return true;
    }

    if (overloaded_)
    {
        ++held_ticks_;

        if (held_ticks_ < hold_ticks_)
            return true;
    }

    //check (again)
    overloaded_ = packets_in_processing > settings_.max_packets_in_processing;

    if (overloaded_)
    {
        hold_ticks_ = packets_in_processing + PacketHoldExtraTicks;
    }
    else if (settings_.max_latency_s > 0 && latency_s.has_value() && latency_s.value() > settings_.max_latency_s)
    {
        overloaded_ = true;
        hold_ticks_ = LatencyHoldTicks;
    }

    if (overloaded_)
    {
        held_ticks_ = 0;
        return true;
    }

    return false;
}

/**
 */
void LiveOverloadDetector::reset()
{
    overloaded_ = false;
    hold_ticks_ = 0;
    held_ticks_ = 0;
}
