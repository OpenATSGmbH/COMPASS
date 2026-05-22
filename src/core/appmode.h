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

#include <string>
#include <stdexcept>

enum AppMode
{
    Offline=0,
    LiveRunning,
    LivePaused
};

inline std::string toString(AppMode app_mode)
{
    if (app_mode == AppMode::Offline)
        return "Offline";
    else if (app_mode == AppMode::LiveRunning)
        return "Live:Running";
    else if (app_mode == AppMode::LivePaused)
        return "Live:Paused";
    else
        throw std::runtime_error("Unkown AppMode "+std::to_string((unsigned int)app_mode));
}

enum class AppState
{
    Starting = 0,
    Running,
    Shutdown
};
