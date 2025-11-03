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

#include "colormap.h"

#include <boost/optional.hpp>

#include "json.hpp"

/**
*/
struct Grid2DRenderSettings
{
    ColorMap                color_map;
    boost::optional<double> min_value;
    boost::optional<double> max_value;
    int                     pixels_per_cell = 1;

    nlohmann::json toJSON() const;
    bool fromJSON(const nlohmann::json& obj);

    bool show_selected = true;
};
