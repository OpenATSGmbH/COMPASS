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

#include "histogram_raw.h"

#include <string>

/**
 * Output of HistogramViewModel, the only data the histogram render widget draws from.
 * Colors and hidden layers are inside the series.
 */
struct HistogramDisplayData
{
    RawHistogramCollection histogram;    // series: name, color, bins, bin labels. Can carry its own log scale.
    std::string            title;
    std::string            x_axis_label;
    bool                   valid = false; // false: the render widget shows the empty chart
};

/**
 * Settings of the histogram drawing that come from the view, not from the data.
 */
struct HistogramRenderOptions
{
    bool use_log_scale = false; // used if the display data carries no log scale
};
