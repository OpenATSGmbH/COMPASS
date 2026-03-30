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

namespace dbContent

{

class TargetPosition
{
public:
    TargetPosition() {}
    TargetPosition(double latitude, double longitude, bool has_altitude, bool altitude_calculated, float altitude)
        : latitude_(latitude), longitude_(longitude), has_altitude_(has_altitude),
          altitude_calculated_(altitude_calculated), altitude_(altitude)
    {}

    double latitude_ {0}; // deg
    double longitude_ {0}; // deg
    bool has_altitude_ {false};
    bool altitude_calculated_ {false}; // indicates if secondary or derived
    float altitude_ {0}; // ft

    std::vector<double> asVector() const
    {
        return std::vector<double>({latitude_, longitude_, 0});
        //return std::vector<double>({latitude_, longitude_, has_altitude_ ? altitude_ : 0});
    };
};

}

