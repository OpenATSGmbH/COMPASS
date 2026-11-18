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

#include <algorithm>
#include <optional>

/**
 * Width of a Flight Deck tool, in percent of the width next to the tool button column.
 */
namespace toolbox
{

constexpr int WidthMinPercent  = 10;
constexpr int WidthMaxPercent  = 70;
constexpr int WidthStepPercent = 10;

inline int clampWidthPercent(int percent)
{
    return std::clamp(percent, WidthMinPercent, WidthMaxPercent);
}

/// next lower multiple of the step, within the limits
inline int widthPercentStepDown(int percent)
{
    int p = clampWidthPercent(percent);
    return clampWidthPercent((p + WidthStepPercent - 1) / WidthStepPercent * WidthStepPercent - WidthStepPercent);
}

/// next higher multiple of the step, within the limits
inline int widthPercentStepUp(int percent)
{
    int p = clampWidthPercent(percent);
    return clampWidthPercent(p / WidthStepPercent * WidthStepPercent + WidthStepPercent);
}

/// nearest multiple of the step, within the limits
inline int snapWidthPercent(int percent)
{
    return clampWidthPercent((percent + WidthStepPercent / 2) / WidthStepPercent * WidthStepPercent);
}

inline int widthPercentOrDefault(const std::optional<int>& saved_percent, int default_percent)
{
    return snapWidthPercent(saved_percent.value_or(default_percent));
}

}
