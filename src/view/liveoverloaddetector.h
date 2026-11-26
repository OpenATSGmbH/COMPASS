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

#include <optional>

/**
 * Decides per live tick attempt whether the tick has to be held because the application is overloaded.
 * - overload = import backlog (packets in processing) or data latency above the configured limits
 * - a detected overload holds a fixed number of tick attempts (backlog + PacketHoldExtraTicks, or
 *   LatencyHoldTicks), then the state is checked again
 * - pure logic, the inputs are passed in
 */
class LiveOverloadDetector
{
public:
    struct Settings
    {
        unsigned int max_packets_in_processing = 4;     // backlog above which ticks are held
        double       max_latency_s             = -1.0;  // latency in seconds above which ticks are held (<= 0 = do not check)
        bool         force                     = false; // debug: overloaded regardless of the inputs, until switched off
    };

    bool update(unsigned int packets_in_processing,
                const std::optional<double>& latency_s);
    void reset();

    bool isOverloaded() const { return overloaded_; }
    unsigned int holdTicks() const { return hold_ticks_; }
    unsigned int heldTicks() const { return held_ticks_; }

    Settings& settings() { return settings_; }
    const Settings& settings() const { return settings_; }

    static const unsigned int PacketHoldExtraTicks;
    static const unsigned int LatencyHoldTicks;

private:
    Settings settings_;

    bool         overloaded_ = false;
    unsigned int hold_ticks_ = 0; // tick attempts held per detected overload
    unsigned int held_ticks_ = 0; // tick attempts held since the detection
};
