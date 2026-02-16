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

#include "rng.h"

#include <string>

namespace rng
{

const double XorShift64::Norm      = 1.0 / 9007199254740992.0;
const double XorShift128Plus::Norm = 1.0 / 9007199254740992.0;
const double PCG32::Norm           = 1.0 / 4294967296.0;

/**
 */
size_t DiscreteSampler::sample(RNG& rng,
                               const double* distribution,
                               size_t n,
                               const std::vector<char>* mask)
{
    double r = rng();
    double cdf = 0.0;
    size_t idx;
    bool assigned = false;
    for (size_t i = 0; i < n; ++i) 
    {
        if (mask && !(*mask)[ i ])
            continue;
        cdf += distribution[ i ];
        if (r <= cdf) { idx = i; assigned = true; break; }
    }

    if (!assigned)
        std::__throw_out_of_range((std::string("random number out of distribution range: r = ") + std::to_string(r) + " cdf = " + std::to_string(cdf)).c_str());

    return idx;
}

} // namespace rng
