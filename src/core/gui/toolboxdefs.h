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

#include <utility>
#include <string>

namespace toolbox
{

enum class ScreenRatio
{
    Ratio_25Percent = 0,
    Ratio_35Percent,
    Ratio_50Percent,
    RatioMax //limiter
};

/**
 */
inline std::pair<int, int> toParts(ScreenRatio screen_ratio) 
{
    switch(screen_ratio)
    {
        case ScreenRatio::Ratio_25Percent:
            return std::pair<int, int>(1, 3);
        case ScreenRatio::Ratio_35Percent:
            return std::pair<int, int>(1, 2);
        case ScreenRatio::Ratio_50Percent:
            return std::pair<int, int>(1, 1);
        default:
            return std::pair<int, int>(1, 1);
    }
    return std::pair<int, int>(1, 1);
}

/**
 */
inline std::string toString(ScreenRatio screen_ratio) 
{
    switch(screen_ratio)
    {
        case ScreenRatio::Ratio_25Percent:
            return "25%";
        case ScreenRatio::Ratio_35Percent:
            return "35%";
        case ScreenRatio::Ratio_50Percent:
            return "50%";
        default:
        return "";
    }
    return "";
}

}
