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

#include <vector>

namespace colorscale
{
    enum class ColorScale
    {
        Parula = 0, 
        Parula_Cut,
        Heat, 
        Jet, 
        Turbo, 
        Hot, 
        Gray, 
        Magma, 
        Magma_Cut,
        Inferno, 
        Inferno_Cut,
        Plasma, 
        Plasma_Cut,
        Viridis, 
        Viridis_Cut,
        Cividis, 
        Github, 
        Red,
        Green,
        Blue,
        Cubehelix, 
        HSV, 
        Green2Red,
        Blue2Green2Red,

        Custom,
        NumColorScales
    };

    static const std::vector<ColorScale> UsedColorScales = 
    {
        ColorScale::Parula_Cut,
        ColorScale::Gray,
        ColorScale::Magma_Cut,
        ColorScale::Inferno_Cut,
        ColorScale::Plasma_Cut,
        ColorScale::Viridis_Cut,
        ColorScale::Red,
        ColorScale::Green,
        ColorScale::Blue,
        ColorScale::Green2Red,
        ColorScale::Blue2Green2Red
    }; 

} // namespace colorscale
